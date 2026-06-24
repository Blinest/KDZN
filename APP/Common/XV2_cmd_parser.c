/**
 * X_V2_V5.0 指令解析器
 */
#include <stdio.h>
#include "XV2_cmd_parser.h"

#include "usart.h"
#include "Common/can_driver.h"
/* ================================================================
 *  解析: 收到的帧 → Motor
 *  自动区分命令帧(带magic)和回复帧(无magic)
 * ================================================================ */

/**
 * CAN 通信专用的解析函数
 * @param m         电机结构体指针
 * @param can_ext_id CAN 扩展帧 ID（29位）
 * @param buf        数据场缓冲区（不含地址，格式：功能码 + 数据 + 校验）
 * @param len        数据场长度（字节数，包括功能码和校验）
 * @param verify_cs  是否验证校验字节（建议 true）
 * @return 解析结果
 */

X_V2_result_t X_V2_parse_can(GlobalMotor *m, uint32_t can_ext_id,
                               const uint8_t *buf, uint32_t len, bool verify_cs)
{
    if (!m || !buf || len < 2) return X_V2_ERR_TOO_SHORT;  // 至少需要功能码+校验

    // 1. 验证电机地址（从 CAN ID 高8位提取）
    uint8_t recv_addr = (uint8_t)((can_ext_id >> 8) & 0xFF);
    if (recv_addr != m->id && recv_addr != 0) {  // 0 为广播地址，可选处理
        return X_V2_ERR_ADDR;
    }

    // 2. 校验（根据配置的校验方式，这里简化使用 0x6B 示例）
    if (verify_cs && buf[len - 1] != 0x6B) return X_V2_ERR_CHECKSUM;

    uint8_t fc = buf[0];                     // 功能码
    uint32_t data_len = len - 2;             // 数据长度 = 总长 - 功能码 - 校验
    const uint8_t *data = &buf[1];           // 数据起始指针

    switch (fc) {
    case X_V2_FC_MOTOR_ENABLE:              // 0xF3, 回复帧: [F3 status cs]
        if (data_len >= 1) {
            m->state = (data[0] == 0x02) ? 1 : 0;   // 0x02 表示使能成功
        }
        break;

    case X_V2_FC_READ_REALTIME_SPD:         // 0x35, 回复: [35 sign speed(2) cs]
        if (data_len >= 3) {
        	uint8_t dir = data[0];
        	const uint16_t raw = X_V2_be16(&data[1]);
        	// 经过了10倍放大，所以这里需要除 10
            m->current_vel = dir == 0 ? (float)raw / 10: -(float)raw / 10;
        	m->stepper_motor.current_vel = m->current_vel * (float)m->stepper_motor.daocheng / 60.0f;
        	/** //调试用
        	float val = m->current_vel;
        	int int_part = (int)val;
        	int frac_part = (int)((val - int_part) * 100 + 0.5);  // 保留两位小数，四舍五入
        	if (frac_part < 0) frac_part = -frac_part;  // 小数部分取绝对值
        	if (frac_part >= 100) {  // 处理进位，如 1.999 -> 2.00
        		int_part += 1;
        		frac_part -= 100;
        	}

        	char test[32];
        	int len = snprintf(test, sizeof(test), "%d.%02d", int_part, frac_part);
        	if (len > 0 && len < sizeof(test)) {
        		Usart_SendString(&huart1, (uint8_t*)test, len);
        	} else {
        		Usart_SendString(&huart1, (uint8_t*)"ERR_FMT\r\n", 9);
        	}
        	*/

        }
        break;

    case X_V2_FC_READ_REALTIME_POS:         // 0x36, 回复: [36 sign pos(4) cs]
    	if (data_len >= 5) {
    		const uint32_t raw = X_V2_be32(&data[1]);
    		uint8_t dir = data[0];
    		// m->current_pos = dir == 0 ? (float)(360 * raw / 65536): -(float)(360 * raw / 65536);
    		m->current_pos = dir == 0 ? (float) raw / 10.0f : -(float) raw / 10.0f;
    		m->stepper_motor.current_pos = m->current_pos * m->stepper_motor.daocheng / 360.0f;
    	}
    case X_V2_FC_READ_TARGET_POS:           // 0x33
    case X_V2_FC_READ_REALTIME_TPOS:        // 0x34
        if (data_len >= 5) {
            int32_t raw = (int32_t)X_V2_be32(&data[1]);
            m->current_pos = (uint16_t)(raw & 0xFFFF);
        }
        break;

    case X_V2_FC_READ_MOTOR_STATUS:         // 0x3A, 回复: [3A flags cs]
        if (data_len >= 1) {
            m->state = (data[0] & 0x01) != 0;
        }
        break;

    case X_V2_FC_READ_FW_VERSION:           // 0x1F
    case X_V2_FC_READ_HOMING_PARAM:         // 0x22
    case X_V2_FC_READ_DRV_CONFIG:           // 0x42
    case X_V2_FC_READ_SYS_STATUS:           // 0x43
    case X_V2_FC_READ_PHASE_RL:
    case X_V2_FC_READ_POS_PID:
    case X_V2_FC_READ_BUS_VOLTAGE:
    case X_V2_FC_READ_PHASE_CURRENT:
    case X_V2_FC_READ_ENCODER:
    case X_V2_FC_READ_INPUT_PULSES:
    case X_V2_FC_READ_POS_ERROR:
        // 这些回复帧较长，可根据需要解析，暂时忽略
        break;

    case X_V2_FC_WRITE_MICROSTEP:           // 0x84, 回复: [84 status cs]
    case X_V2_FC_WRITE_ID_ADDR:             // 0xAE, 回复: [AE status cs]
    case X_V2_FC_WRITE_LOOP_MODE:           // 0x46
    case X_V2_FC_IMMEDIATE_STOP:            // 0xFE
    case X_V2_FC_SYNC_MOTION:               // 0xFF
    case X_V2_FC_SET_HOME:                  // 0x93
    case X_V2_FC_TRIGGER_HOMING:            // 0x9A
    case X_V2_FC_ABORT_HOMING:              // 0x9C
    case X_V2_FC_TRIGGER_CALIB:             // 0x06
    case X_V2_FC_CLEAR_POSITION:            // 0x0A
    case X_V2_FC_RELEASE_STALL:             // 0x0E
    case X_V2_FC_FACTORY_RESET:             // 0x0F
        // 这些命令的回复通常只包含状态码，可忽略或记录
        break;

    default:
        return X_V2_ERR_UNKNOWN_FC;
    }

    return X_V2_OK;
}
/* ================================================================
 *  构建: Motor → 发送帧写入 Motor.cmd
 * ================================================================ */

X_V2_result_t X_V2_build(GlobalMotor *m, uint8_t fc)
{
    if (!m) return X_V2_ERR_TOO_SHORT;
    uint8_t *c = m->cmd;
    uint8_t addr = (uint8_t)m->id;
	// 进制转换
	uint32_t pos = (uint16_t) m->target_pos;
	uint16_t vel = (uint16_t) m->target_vel;
	uint8_t acc = (uint16_t) m->target_acc;
    switch (fc) {

    case X_V2_FC_MOTOR_ENABLE:
        c[0]=addr; c[1]=0xF3; c[2]=0xAB;
        c[3]=m->state?0x01:0x00; c[4]=0x00; c[5]=0x6B;
        m->size=6; break;

    case X_V2_FC_SPEED_MODE:
        c[0]=addr; c[1]=0xF6;
        c[2]=X_V2_DIR_CW;
        c[3]=(uint8_t)(vel>>8);
        c[4]=(uint8_t)(vel&0xFF);
        c[5]=(uint8_t)acc; c[6]=0x00; c[7]=0x6B;
        m->size=8; break;

    case X_V2_FC_POSITION_MODE:
        c[0]=addr; c[1]=0xFD; c[2]=X_V2_DIR_CW;
        c[3]=(uint8_t)(vel >> 8);
        c[4]=(uint8_t)(vel & 0xFF);
        c[5]=acc;
        c[6]=(uint8_t)((pos>>24)&0xFF);
        c[7]=(uint8_t)((pos>>16)&0xFF);
        c[8]=(uint8_t)((pos>>8)&0xFF);
        c[9]=(uint8_t)(pos&0xFF);
        c[10]=X_V2_POS_REL; c[11]=0x00; c[12]=0x6B;
        m->size=13; break;

    case X_V2_FC_IMMEDIATE_STOP:
        c[0]=addr; c[1]=0xFE; c[2]=0x98; c[3]=0x00; c[4]=0x6B;
        m->size=5; break;

    case X_V2_FC_SYNC_MOTION:
        c[0]=addr; c[1]=0xFF; c[2]=0x66; c[3]=0x6B;
        m->size=4; break;

    case X_V2_FC_SET_HOME:
        c[0]=addr; c[1]=0x93; c[2]=0x88; c[3]=0x01; c[4]=0x6B;
        m->size=5; break;

    case X_V2_FC_TRIGGER_HOMING:
        c[0]=addr; c[1]=0x9A; c[2]=0x00; c[3]=0x00; c[4]=0x6B;
        m->size=5; break;

    case X_V2_FC_ABORT_HOMING:
        c[0]=addr; c[1]=0x9C; c[2]=0x48; c[3]=0x6B;
        m->size=4; break;

    case X_V2_FC_TRIGGER_CALIB:
        c[0]=addr; c[1]=0x06; c[2]=0x45; c[3]=0x6B;
        m->size=4; break;

    case X_V2_FC_CLEAR_POSITION:
        c[0]=addr; c[1]=0x0A; c[2]=0x6D; c[3]=0x6B;
        m->size=4; break;

    case X_V2_FC_RELEASE_STALL:
        c[0]=addr; c[1]=0x0E; c[2]=0x52; c[3]=0x6B;
        m->size=4; break;

    case X_V2_FC_FACTORY_RESET:
        c[0]=addr; c[1]=0x0F; c[2]=0x5F; c[3]=0x6B;
        m->size=4; break;

    /* 读取: 3字节 [addr fc cs] */
    case X_V2_FC_READ_FW_VERSION:
    case X_V2_FC_READ_PHASE_RL:
    case X_V2_FC_READ_POS_PID:
    case X_V2_FC_READ_BUS_VOLTAGE:
    case X_V2_FC_READ_PHASE_CURRENT:
    case X_V2_FC_READ_ENCODER:
    case X_V2_FC_READ_INPUT_PULSES:
    case X_V2_FC_READ_TARGET_POS:
    case X_V2_FC_READ_REALTIME_TPOS:
    case X_V2_FC_READ_REALTIME_SPD:
    case X_V2_FC_READ_REALTIME_POS:
    case X_V2_FC_READ_POS_ERROR:
    case X_V2_FC_READ_MOTOR_STATUS:
    case X_V2_FC_READ_HOMING_STATUS:
        c[0]=addr; c[1]=fc; c[2]=0x6B;
        m->size=3; break;

    /* 读取(带magic): 4字节 */
    case X_V2_FC_READ_DRV_CONFIG:
        c[0]=addr; c[1]=0x42; c[2]=0x6C; c[3]=0x6B;
        m->size=4; break;
    case X_V2_FC_READ_SYS_STATUS:
        c[0]=addr; c[1]=0x43; c[2]=0x7A; c[3]=0x6B;
        m->size=4; break;
    case X_V2_FC_READ_HOMING_PARAM:
        c[0]=addr; c[1]=0x22; c[2]=0x6B;
        m->size=3; break;

    case X_V2_FC_WRITE_MICROSTEP:
        c[0]=addr; c[1]=0x84; c[2]=0x8A;
        c[3]=0x01; c[4]=m->stepper_motor.xifen; c[5]=0x6B;
        m->size=6; break;

    case X_V2_FC_WRITE_ID_ADDR:
        c[0]=addr; c[1]=0xAE; c[2]=0x4B;
        c[3]=0x01; c[4]=(uint8_t)m->id; c[5]=0x6B;
        m->size=6; break;

    case X_V2_FC_WRITE_LOOP_MODE:
        c[0]=addr; c[1]=0x46; c[2]=0x69;
        c[3]=0x01; c[4]=0x02; c[5]=0x6B;
        m->size=6; break;

    default:
        return X_V2_ERR_UNKNOWN_FC;
    }

    return X_V2_OK;
}



