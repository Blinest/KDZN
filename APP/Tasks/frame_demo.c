/**
 * EmM42_V5.0 帧打包/解包 Demo
 * 编译: gcc -o frame_demo frame_demo.c -Wall -Wextra -std=c11
 */
#include <stdio.h>
#include <stdbool.h>
#include "emm42_frame.h"

int main(void) {
    printf("=== Frame 打包/解包 Demo ===\n\n");

    /* =========================================================
     *  1. 用原始测试帧验证解包
     * ========================================================= */
    {
        const uint8_t test_frame[] = {
            0xBB, 0x02, 0x38,
            0x03, 0x04,
            0x03, 0xE8, 0x07, 0xD0, 0x0B, 0xB8, 0x01,
            0x0F, 0xA0, 0x13, 0x88, 0x17, 0x70, 0x00,
            0x1B, 0x58, 0x1F, 0x40, 0x23, 0x28, 0x01,
            0x09, 0xC4, 0x09, 0xC4, 0x09, 0xC4,
            0x09, 0xC4, 0x09, 0xC4, 0x09, 0xC4,
            0x09, 0xC4, 0x09, 0xC4, 0x09, 0xC4,
            0x09, 0xC4, 0x09, 0xC4, 0x09, 0xC4,
            0x13, 0x88, 0x00, 0x00, 0x01,
            0xA9
        };

        printf("── 解包原始测试帧 ──\n");
        printf("  原始: ");
        frame_print_hex(test_frame, sizeof(test_frame));

        FrameData fd;
        int ret = frame_unpack(&fd, test_frame, sizeof(test_frame), true);
        printf("  解包结果: %d (预期 56)\n", ret);
        frame_print(&fd);
    }

    /* =========================================================
     *  2. 从 Motor 结构体打包 → 解包验证 roundtrip
     * ========================================================= */
    {
        printf("\n── Motor → FrameData → 打包 → 解包 roundtrip ──\n");

        /* 模拟3个电机 */
        GlobalMotor motors[3] = {
            { .id=1, .state=true,  .target_pos=0x03E8, .target_vel=0x07D0, .target_acc=0x0BB8 },
            { .id=2, .state=false, .target_pos=0x0FA0, .target_vel=0x1388, .target_acc=0x1770 },
            { .id=3, .state=true,  .target_pos=0x1B58, .target_vel=0x1F40, .target_acc=0x2328 },
        };

        /* 构建 FrameData */
        FrameData fd = {
            .func_code  = FC_MOTOR_STATUS,
            .motor_num  = 3,
            .sensor_num = 4,
            .scale1     = 0x1388,
            .scale2     = 0x0000,
            .sys_state  = 1,
        };

        /* Motor → FrameMotor */
        for (int i = 0; i < 3; i++)
            frame_from_motor(&fd.motors[i], &motors[i]);

        /* 传感器数据 */
        for (int i = 0; i < 4; i++) {
            fd.sensors[i].x = 0x09C4;
            fd.sensors[i].y = 0x09C4;
            fd.sensors[i].z = 0x09C4;
        }

        /* 打包 */
        uint8_t buf[128];
        int packed = frame_pack(buf, sizeof(buf), &fd);
        printf("  打包长度: %d\n", packed);
        printf("  打包帧:   ");
        frame_print_hex(buf, packed);

        /* 解包回去 */
        FrameData fd2;
        int unpacked = frame_unpack(&fd2, buf, packed, true);
        printf("  解包结果: %d\n", unpacked);
        frame_print(&fd2);

        /* 对比 */
        int ok = (packed == unpacked)
              && (fd.motor_num == fd2.motor_num)
              && (fd.sensor_num == fd2.sensor_num)
              && (memcmp(fd.motors, fd2.motors, sizeof(FrameMotor)*3) == 0)
              && (memcmp(fd.sensors, fd2.sensors, sizeof(FrameSensor)*4) == 0)
              && (fd.scale1 == fd2.scale1)
              && (fd.scale2 == fd2.scale2)
              && (fd.sys_state == fd2.sys_state);

        printf("  roundtrip: %s\n\n", ok ? "✓ 一致" : "✗ 不一致");

        /* FrameMotor → Motor 回写 */
        GlobalMotor m_back;
        frame_to_motor(&m_back, &fd2.motors[0]);
        printf("  回写 motor[0]: target_pos=0x%04X vel=0x%04X acc=0x%04X state=%s\n",
               m_back.target_pos, m_back.target_vel, m_back.target_acc,
               m_back.state ? "ON" : "OFF");
    }

    /* =========================================================
     *  3. 扩展: 5个电机 + 2个传感器
     * ========================================================= */
    {
        printf("\n── 扩展: 5电机 + 2传感器 ──\n");

        FrameData fd = {
            .func_code  = FC_MOTOR_CMD,
            .motor_num  = 5,
            .sensor_num = 2,
            .scale1     = 5000,
            .scale2     = 2500,
            .sys_state  = 2,
        };

        for (int i = 0; i < 5; i++) {
            fd.motors[i].displacement = (uint16_t)(1000 + i * 1000);
            fd.motors[i].speed        = (uint16_t)(500  + i * 200);
            fd.motors[i].accel        = (uint16_t)(100  + i * 50);
            fd.motors[i].state        = (i % 2);
        }
        for (int i = 0; i < 2; i++) {
            fd.sensors[i].x = 2500;
            fd.sensors[i].y = 3000;
            fd.sensors[i].z = 3500;
        }

        uint8_t buf[128];
        int packed = frame_pack(buf, sizeof(buf), &fd);
        printf("  打包长度: %d (预期 %d)\n",
               packed, frame_total_len(5, 2));
        printf("  打包帧:   ");
        frame_print_hex(buf, packed);

        FrameData fd2;
        frame_unpack(&fd2, buf, packed, true);
        frame_print(&fd2);
    }

    /* =========================================================
     *  4. 校验和验证
     * ========================================================= */
    {
        printf("\n── 校验和验证 ──\n");

        uint8_t buf[128];
        FrameData fd = {
            .func_code = FC_MOTOR_STATUS,
            .motor_num = 1,
            .sensor_num = 1,
            .motors[0] = { 1000, 500, 100, 1 },
            .sensors[0] = { 2500, 2500, 2500 },
            .scale1 = 5000, .scale2 = 0, .sys_state = 1,
        };

        int len = frame_pack(buf, sizeof(buf), &fd);
        uint8_t orig_cs = buf[len - 1];
        printf("  原始校验: 0x%02X\n", orig_cs);

        /* 篡改数据 → 校验失败 */
        buf[5] ^= 0xFF;
        int ret = frame_unpack(&fd, buf, len, true);
        printf("  篡改后解包: %d (预期 -4 校验失败)\n", ret);

        /* 恢复 → 校验通过 */
        buf[5] ^= 0xFF;
        ret = frame_unpack(&fd, buf, len, true);
        printf("  恢复后解包: %d (预期 %d)\n", ret, len);
    }

    printf("\n=== 全部完成 ✓ ===\n");
    return 0;
}