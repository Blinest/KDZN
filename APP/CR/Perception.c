/**
 * @file Perception.c
 * @brief 六路肌腱张力感知实现 (比值型)
 *
 * 见 Perception.h 的模型说明。
 *
 * @date 2026-10-03
 */

#include "Perception.h"
#include "SDM.h"
#include "Sensor/Sensor.h"
#include "FreeRTOS.h"
#include "task.h"

#include <math.h>
#include <string.h>

/* ==================== 内部常量 ==================== */
#define PERCEPT_PI  3.14159265358979323846f

/* 丝 i 的截面安装角: 段0={0,2π/3,4π/3}(丝0,2,4), 段1={π/3,π,5π/3}(丝1,3,5)
 * ⇒ 统一写成 α_i = i·π/3, 段号 g = i % 2 (与 SDM.c s_seg_to_wires 一致) */
static const float s_alpha[SDM_WIRES] = {
    0.0f,
    PERCEPT_PI / 3.0f,
    2.0f * PERCEPT_PI / 3.0f,
    PERCEPT_PI,
    4.0f * PERCEPT_PI / 3.0f,
    5.0f * PERCEPT_PI / 3.0f
};

static PerceptState s_state;

/* 绝对残差零点基线 (仅供诊断显示) */
static float s_baseline[SDM_WIRES];

/* 逐通道「无载基线」(预紧力 τ0) 与就绪标志。
 * 判据使用「超额张力 τ-τ0」⇒ 对通道零点偏置/温漂免疫;
 * 松弛丝被外力拉紧会产生大的超额 ⇒ 这正是接触的主信号。
 * 未 capture 时由 _excess() 自适应跟踪(快降慢升)。 */
static float s_tau0[SDM_WIRES];
static bool  s_tau0_init[SDM_WIRES];

/**
 * @brief 逐通道超额张力: τ - τ0, 并在内部自适应维护 τ0
 */
static float _excess(uint8_t i, float tau)
{
    if (!s_tau0_init[i]) {
        s_tau0[i] = tau;
        s_tau0_init[i] = true;
    } else if (tau < s_tau0[i]) {
        s_tau0[i] = tau;                                       /* 立即跟随下降 */
    } else {
        s_tau0[i] += PERCEPT_TAU0_RELEASE * (tau - s_tau0[i]);  /* 缓慢上抬 */
    }
    return tau - s_tau0[i];
}

/* ==================== 对外 API ==================== */

void perception_init(void)
{
    memset(&s_state, 0, sizeof(s_state));
    memset(s_baseline, 0, sizeof(s_baseline));
    memset(s_tau0, 0, sizeof(s_tau0));
    memset(s_tau0_init, 0, sizeof(s_tau0_init));
}

void perception_capture_baseline(void)
{
    for (int i = 0; i < SDM_WIRES; i++) {
        s_tau0[i]      = s_state.force[i];   /* 去皮: 当前张力即无载基线 */
        s_tau0_init[i] = true;
        s_baseline[i]  = s_state.residual[i];
    }
}

void perception_update(const float forces[SDM_WIRES])
{
    if (forces == NULL) return;

    PerceptState *st = &s_state;
    const float R = SDM_DRIVE_RADIUS_M;   /* 与 CR._sdm_run() 同源 (CR.h) */

    /* ---- 0. 扣除「模型重力项」后再做超额基线跟踪 ----
     * 水平安装时重力在各丝上的分布为 0:1:2:0:2:1 (最大 1.27N, 预紧2N时偏差24%),
     * 若把它计入: ① 直接造成误报接触; ② 被 τ0 慢慢吸收 → 变成"瞬态可测、持续丢失"。
     * 显式建模扣除后, τ0 只跟踪预紧力, 与姿态/挂载方向无关。 */
    sdm_gravity_force(CR.joint_space.target_theta,
                      CR.joint_space.target_phi,
                      st->f_grav);

    float ex[SDM_WIRES];
    for (int i = 0; i < SDM_WIRES; i++) {
        st->force[i] = forces[i];
        ex[i] = _excess(i, forces[i] - st->f_grav[i]);   /* τ - G - τ0 */
    }

    /* ---- 1. 期望几何: PCC 基准位移 + 模型目标张力 (诊断用) ---- */
    float base[SDM_WIRES];
    sdm_pcc_deltaL(CR.joint_space.target_theta,
                   CR.joint_space.target_phi,
                   R, base);

    sdm_force_target(CR.joint_space.target_theta,
                     CR.joint_space.target_phi,
                     R, st->f_target);

    float rn = 0.0f;
    float mean_sum[SDM_SEGMENTS] = {0.0f, 0.0f};
    int   mean_cnt[SDM_SEGMENTS] = {0, 0};

    for (int i = 0; i < SDM_WIRES; i++) {
        st->residual[i] = (forces[i] - st->f_target[i]) - s_baseline[i];
        rn += st->residual[i] * st->residual[i];

        const int g = i % 2;
        mean_sum[g] += forces[i];
        mean_cnt[g]++;
    }
    st->residual_norm = sqrtf(rn);
    for (int g = 0; g < SDM_SEGMENTS; g++) {
        st->force_mean[g] = (mean_cnt[g] > 0) ? (mean_sum[g] / (float)mean_cnt[g]) : 0.0f;
    }

    /* ---- 2. 段内份额误差 (比值型判据) ---- */
    float Fx[SDM_SEGMENTS] = {0.0f, 0.0f};
    float Fy[SDM_SEGMENTS] = {0.0f, 0.0f};
    int   geom_ok[SDM_SEGMENTS] = {0, 0};    /* 是否几何分配(用于健康判定) */

    for (int i = 0; i < SDM_WIRES; i++) {
        st->share_meas[i] = 0.0f;
        st->share_des[i]  = 0.0f;
        st->share_err[i]  = 0.0f;
    }

    for (int s = 0; s < SDM_SEGMENTS; s++) {
        float w[SDM_WIRES_PER_SEG];
        const int nact = sdm_segment_shares(s, CR.joint_space.target_phi[s], base, w);
        geom_ok[s] = (nact >= 1) ? 1 : 0;

        /* 段内「超额张力」和: 扣除预紧力, 使份额比较不受零点偏置影响 */
        float tsum = 0.0f;
        for (int j = 0; j < SDM_WIRES_PER_SEG; j++) {
            tsum += ex[s + 2 * j];
        }
        if (tsum < PERCEPT_MIN_SEG_SUM_N) continue;   /* 无有效载荷, 比值无意义 */

        for (int j = 0; j < SDM_WIRES_PER_SEG; j++) {
            const int   wi     = s + 2 * j;
            const float r_meas = ex[wi] / tsum;      /* 超额份额 (无量纲) */
            const float e      = r_meas - w[j];

            st->share_meas[wi] = r_meas;
            st->share_des[wi]  = w[j];
            st->share_err[wi]  = e;

            Fx[s] += e * cosf(s_alpha[wi]);
            Fy[s] += e * sinf(s_alpha[wi]);
        }
    }

    /* ---- 3. 接触判定 ---- */
    float mag[SDM_SEGMENTS];
    for (int g = 0; g < SDM_SEGMENTS; g++) {
        mag[g] = sqrtf(Fx[g] * Fx[g] + Fy[g] * Fy[g]);
    }

    const int seg = (mag[1] > mag[0]) ? 1 : 0;
    st->disturbance      = mag[seg];
    st->contact.seg      = (uint8_t)seg;
    st->contact.F_seg[0] = Fx[seg];
    st->contact.F_seg[1] = Fy[seg];
    st->contact.F_mag    = mag[seg] * PERCEPT_CONTACT_GAIN_N;
    st->contact.detected = (mag[seg] >= PERCEPT_CONTACT_THRESH);

    /* 粗定位: 纯段0→0.5, 纯段1→1.0, 均衡→0.25/0.75 */
    const float tot = mag[0] + mag[1];
    if (tot > SDM_EPS) {
        st->contact.s_norm = (seg == 0)
                           ? 0.5f * (mag[0] / tot)
                           : 0.5f + 0.5f * (mag[1] / tot);
    } else {
        st->contact.s_norm = 0.0f;
    }

    /* ---- 4. 逐路健康 (份额比, 无量纲) ---- */
    st->health_any_bad = false;
    for (int i = 0; i < SDM_WIRES; i++) {
        const int   g   = i % 2;
        const float meas = forces[i];
        PerceptHealth_t h = PERCEPT_HEALTH_OK;

        if (meas >= PERCEPT_OVERLOAD_N) {
            h = PERCEPT_HEALTH_OVERLOAD;
        } else if (geom_ok[g] && st->share_des[i] >= PERCEPT_ACTIVE_SHARE) {
            const float ratio = st->share_meas[i] / st->share_des[i];
            if (ratio <= PERCEPT_BROKEN_RATIO) {
                h = (meas <= PERCEPT_DEAD_N) ? PERCEPT_HEALTH_BROKEN
                                             : PERCEPT_HEALTH_SLACK;
            } else if (ratio >= PERCEPT_OVERTENSION_RATIO) {
                h = PERCEPT_HEALTH_OVERTENSION;
            }
        }

        st->health[i] = h;
        if (h != PERCEPT_HEALTH_OK) {
            st->health_any_bad = true;
        }
    }
}

void perception_update_from_global(void)
{
    float snapshot[SDM_WIRES];

    /* 临界区: 与 DataParserTask 写 global_sensor[] 互斥, 防撕裂 */
    taskENTER_CRITICAL();
    for (int i = 0; i < SDM_WIRES; i++) {
        snapshot[i] = (i < SENSOR_NUM) ? global_sensor[i].x : 0.0f;
    }
    taskEXIT_CRITICAL();

    perception_update(snapshot);
}

const PerceptState *perception_get(void)
{
    return &s_state;
}
