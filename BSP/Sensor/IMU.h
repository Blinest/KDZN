//
// Created by blin on 2026/3/7.
//
#include "stdint.h"

#ifndef CONTROLSYSTEM_IMU_H
#define CONTROLSYSTEM_IMU_H

#define ACC_UPDATE		0x01
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define READ_UPDATE		0x80

void IMU_Init(void);
void IMU_single_read(uint8_t sensor_id);
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
void IMU_Cal(uint8_t sensor_id);
#endif //CONTROLSYSTEM_IMU_H