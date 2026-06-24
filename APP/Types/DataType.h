//
// Created by blin on 2026/3/7.
//

#ifndef CONTROLSYSTEM_DATATYPE_H
#define CONTROLSYSTEM_DATATYPE_H
#include <stdint.h>
typedef struct
{
	uint16_t distance;
	uint16_t vel;
	uint16_t acc;
	uint8_t state;
} MotorData;

typedef struct
{
	uint16_t x;
	uint16_t y;
	uint16_t z;
} SensorData;

typedef struct
{
	MotorData motor_data;
	SensorData sensor_data;
} DataType;

extern DataType data_type;
#endif //CONTROLSYSTEM_DATATYPE_H