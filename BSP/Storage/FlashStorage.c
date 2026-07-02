/**
 * @file FlashStorage.c
 * @brief 内部 Flash 参数持久化实现
 *
 * STM32H743 Flash 写入约束：
 *   - 最小写入粒度: 256-bit (32 字节, FLASH_TYPEPROGRAM_FLASHWORD)
 *   - 擦除粒度: 整个 Sector (128KB)
 *   - 写入前必须先擦除
 *
 * @date 2026-07-02
 * @author blin
 */

#include "FlashStorage.h"
#include <string.h>

/**
 * @brief 保存参数到内部 Flash
 *
 * 流程：Unlock → 擦除 Sector7 → 按 32 字节粒度写入 → Lock
 * 阻塞操作，约耗时数十毫秒（擦除占大部分时间）
 */
HAL_StatusTypeDef FlashStorage_Save(const FlashStoreData *data)
{
    if (data == NULL) return HAL_ERROR;

    HAL_StatusTypeDef status;

    /* Unlock Flash */
    status = HAL_FLASH_Unlock();
    if (status != HAL_OK) return status;

    /* 擦除 Bank2 Sector7 */
    /* FLASH_Erase_Sector(Sector, Bank, VoltageRange) */
    FLASH_Erase_Sector(FLASH_STORE_SECTOR, FLASH_STORE_BANK, FLASH_VOLTAGE);
    /* 等待擦除完成（HAL_FLASH_Unlock 后硬件自动轮询 BSY） */

    /* 按 32 字节粒度逐 word 写入 */
    const uint32_t *src = (const uint32_t *)data;
    uint32_t addr = FLASH_STORE_ADDR;
    uint32_t words = sizeof(FlashStoreData) / 4;  /* 总 32-bit word 数 */

    for (uint32_t i = 0; i < words; i += 8)  /* 每 8 × 4 = 32 字节一个 flash word */
    {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,
                                   addr + i * 4,
                                   (uint32_t)&src[i]);
        if (status != HAL_OK) {
            HAL_FLASH_Lock();
            return status;
        }
    }

    /* Lock Flash */
    HAL_FLASH_Lock();
    return HAL_OK;
}

/**
 * @brief 从内部 Flash 加载参数
 *
 * Flash 读取无需 Unlock，直接按地址读即可。
 * 校验 magic 字段确认数据有效。
 */
HAL_StatusTypeDef FlashStorage_Load(FlashStoreData *data)
{
    if (data == NULL) return HAL_ERROR;

    /* 直接从 Flash 地址读取（Flash 读取无需 Unlock） */
    const FlashStoreData *stored = (const FlashStoreData *)FLASH_STORE_ADDR;

    /* 校验 magic */
    if (stored->magic != FLASH_STORE_MAGIC) {
        return HAL_ERROR;  /* Flash 为空或数据损坏 */
    }

    /* 复制到 RAM */
    memcpy(data, stored, sizeof(FlashStoreData));
    return HAL_OK;
}
