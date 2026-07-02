/**
 * @file FlashStorage.h
 * @brief 内部 Flash 参数持久化接口
 *
 * 使用 Bank2 Sector7（0x081E0000, 128KB）存储运行参数，
 * 断电后数据不丢失。首次上电 Flash 为空（全 0xFF），magic 校验失败时跳过恢复。
 *
 * @date 2026-07-02
 * @author blin
 */

#ifndef CONTROLSYSTEM_FLASHSTORAGE_H
#define CONTROLSYSTEM_FLASHSTORAGE_H

#include <stdint.h>
#include "stm32h7xx_hal.h"

#define SENSOR_NUM_LOCAL  6  /* 与 Sensor.h 中 SENSOR_NUM 一致 */

/* Bank2 Sector7: 128KB, 地址 0x081E0000 - 0x081FFFFF */
#define FLASH_STORE_ADDR       0x081E0000U
#define FLASH_STORE_BANK       2U
#define FLASH_STORE_SECTOR     7U
#define FLASH_STORE_MAGIC      0xDEADBEEFU
#define FLASH_VOLTAGE          FLASH_VOLTAGE_RANGE_3   /* 3.3V → 32-bit PSIZE */

/**
 * @brief 持久化数据结构（32 字节对齐，满足 Flash 写入粒度要求）
 *
 * 有效数据约 98 字节，填充到 128 字节（4 × 32 字节 Flash word）
 */
typedef struct {
    uint32_t magic;                              /* 0xDEADBEEF 标识有效 */
    uint8_t  filter_first_sample[SENSOR_NUM_LOCAL]; /* EMA 滤波器首采样标志 */
    uint8_t  reserved0[2];                       /* 对齐填充 */
    int32_t  filter_val[SENSOR_NUM_LOCAL];        /* EMA 滤波器当前输出 */
    float    motor_target[SENSOR_NUM_LOCAL];      /* 压力控制电机目标位置 */
    int32_t  prev_val[SENSOR_NUM_LOCAL];          /* 上次触发时的力值 */
    uint8_t  reserved1[16];                      /* 保留，对齐到 128 字节 */
} __attribute__((aligned(32))) FlashStoreData;

/**
 * @brief 保存参数到内部 Flash
 * @param data 待保存的数据
 * @return HAL_OK 成功，其他失败
 */
HAL_StatusTypeDef FlashStorage_Save(const FlashStoreData *data);

/**
 * @brief 从内部 Flash 加载参数
 * @param data 输出缓冲区
 * @return HAL_OK 数据有效且已加载，HAL_ERROR Flash 为空或数据无效
 */
HAL_StatusTypeDef FlashStorage_Load(FlashStoreData *data);

#endif /* CONTROLSYSTEM_FLASHSTORAGE_H */
