"""
CR_pressure_control 策略可视化
双条件触发 + 动态步长
"""
import matplotlib.pyplot as plt
import numpy as np

# ==================== 参数 ====================
PRESS_HIGH = 200
PRESS_LOW = 0
PRESS_DELTA = 10
PRESS_STEP_MIN = 0.2
PRESS_STEP_MAX = 2.0
PRESS_VEL = 0.5

DT = 0.1  # 控制周期 100ms
TOTAL_TIME = 30  # 仿真时长 30s

# ==================== 模拟传感器输入 ====================
np.random.seed(42)
t = np.arange(0, TOTAL_TIME, DT)
n = len(t)

# 构造模拟压力: 分段信号
val = np.zeros(n)
# 0-5s: 稳定在 100 (正常范围)
val[:50] = 100
# 5-10s: 阶跃到 250 (超出上限)
val[50:100] = 250
# 10-15s: 阶跃到 300 (大幅超出)
val[100:150] = 300 + np.random.randn(50) * 5
# 15-20s: 降到 150 (正常范围)
val[150:200] = 150 + np.random.randn(50) * 3
# 20-25s: 降到 -50 (超出下限)
val[200:250] = -50 + np.random.randn(50) * 5
# 25-30s: 回到 100
val[250:] = 100 + np.random.randn(50) * 2

# ==================== EMA 滤波模拟 ====================
alpha = 0.2
filter_val = np.zeros(n)
filter_val[0] = val[0]
for i in range(1, n):
    filter_val[i] = filter_val[i - 1] * (1 - alpha) + val[i] * alpha

# ==================== 控制策略仿真 ====================
def calc_dynamic_step(delta):
    ratio = (delta - PRESS_DELTA) / (PRESS_HIGH - PRESS_DELTA)
    ratio = max(0.0, min(1.0, ratio))
    return PRESS_STEP_MIN + (PRESS_STEP_MAX - PRESS_STEP_MIN) * ratio

target = np.zeros(6)  # 只模拟一个电机
prev_val = 100
target[0] = 0  # 初始位置

step_history = np.zeros(n)
target_history = np.zeros(n)
triggered = np.zeros(n, dtype=bool)
motor_active = np.zeros(n, dtype=bool)

for i in range(n):
    v = filter_val[i]
    delta = abs(v - prev_val)
    step = calc_dynamic_step(delta)

    out_of_range = (v > PRESS_HIGH) or (v < PRESS_LOW)
    significant_change = (delta >= PRESS_DELTA)

    if out_of_range and significant_change:
        if v > PRESS_HIGH:
            target[0] -= step
        else:
            target[0] += step
        prev_val = v
        triggered[i] = True
        motor_active[i] = True
    else:
        step = 0

    step_history[i] = step
    target_history[i] = target[0]

# ==================== 动态步长映射曲线 ====================
deltas = np.linspace(0, PRESS_HIGH * 1.2, 100)
steps = np.array([calc_dynamic_step(d) for d in deltas])

# ==================== 绘图 ====================
fig, axes = plt.subplots(4, 1, figsize=(14, 12))

# --- 图1: 动态步长映射 ---
ax1 = axes[0]
ax1.plot(deltas, steps, 'b-', linewidth=2)
ax1.axvline(PRESS_DELTA, color='gray', linestyle='--', alpha=0.5, label=f'PRESS_DELTA={PRESS_DELTA}')
ax1.axvline(PRESS_HIGH, color='red', linestyle='--', alpha=0.5, label=f'PRESS_HIGH={PRESS_HIGH}')
ax1.fill_between([PRESS_DELTA, PRESS_HIGH], 0, PRESS_STEP_MAX,
                 alpha=0.1, color='blue')
ax1.set_ylabel('Step (mm)')
ax1.set_xlabel('Pressure Change |Δ|')
ax1.set_title('Dynamic Step Mapping: Δ → Step')
ax1.legend()
ax1.grid(True, alpha=0.3)
ax1.set_xlim(0, PRESS_HIGH * 1.2)

# --- 图2: 原始信号 vs 滤波 ---
ax2 = axes[1]
ax2.plot(t, val, 'gray', alpha=0.4, linewidth=1, label='Raw (simulated)')
ax2.plot(t, filter_val, 'b-', linewidth=1.5, label='EMA filtered')
ax2.axhline(PRESS_HIGH, color='red', linestyle='-', alpha=0.7, label='PRESS_HIGH')
ax2.axhline(PRESS_LOW, color='green', linestyle='-', alpha=0.7, label='PRESS_LOW')
ax2.fill_between(t, PRESS_LOW, PRESS_HIGH, alpha=0.05, color='green')
ax2.set_ylabel('Pressure')
ax2.set_xlabel('Time (s)')
ax2.set_title('Pressure Signal (Raw vs EMA Filtered)')
ax2.legend(fontsize=9)
ax2.grid(True, alpha=0.3)

# --- 图3: 触发与步长 ---
ax3 = axes[2]
ax3.step(t, step_history, 'r-', where='post', linewidth=1.5, label='Step size')
# 标记触发点
trigger_t = t[triggered]
trigger_step = step_history[triggered]
ax3.scatter(trigger_t, trigger_step, s=30, c='red', zorder=5, label='Triggered')
ax3.set_ylabel('Step (mm)')
ax3.set_xlabel('Time (s)')
ax3.set_title('Control Output: Step Size')
ax3.legend(fontsize=9)
ax3.grid(True, alpha=0.3)
ax3.set_ylim(-0.1, max(step_history) * 1.2 + 0.1)

# --- 图4: 电机位置响应 ---
ax4 = axes[3]
ax4.plot(t, target_history, 'b-', linewidth=1.5, label='Motor target position')
# 标注压力状态区域
for i in range(len(t)):
    if filter_val[i] > PRESS_HIGH:
        ax4.axvspan(t[i] - DT / 2, t[i] + DT / 2, alpha=0.06, color='red')
    elif filter_val[i] < PRESS_LOW:
        ax4.axvspan(t[i] - DT / 2, t[i] + DT / 2, alpha=0.06, color='orange')

ax4.set_xlabel('Time (s)')
ax4.set_ylabel('Position (mm)')
ax4.set_title('Motor Response (Target Position, one channel)')
ax4.legend(fontsize=9)
ax4.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('pressure_control_simulation.png', dpi=150)
print("Saved: pressure_control_simulation.png")
