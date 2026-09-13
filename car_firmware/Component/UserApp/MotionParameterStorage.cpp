#include "MotionParameterStorage.hpp"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

// These symbols are required in both linker scripts, keeping code and data apart.
extern "C" {
extern const unsigned char __motion_params_start__;
extern const unsigned char __motion_params_end__;
}

namespace MotionSettings {
namespace {
struct InternalFlash {
    static constexpr std::uint32_t start_address = 0x08060000U;
    static constexpr std::size_t capacity_bytes = 128U * 1024U;

    static bool layoutValid()
    {
        return reinterpret_cast<std::uintptr_t>(&__motion_params_start__) == start_address &&
            reinterpret_cast<std::uintptr_t>(&__motion_params_end__) == start_address + capacity_bytes;
    }

    std::uint32_t readWord(std::size_t offset)
    {
        return *reinterpret_cast<const volatile std::uint32_t*>(start_address + offset);
    }

    bool programWord(std::size_t offset, std::uint32_t word)
    {
        if (offset % 4 != 0 || offset > capacity_bytes - 4 || readWord(offset) != 0xFFFFFFFFU)
            return false;
        // An all-ones word is already in the desired state.
        if (word == 0xFFFFFFFFU) return true;
        return HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, start_address + offset, word) == HAL_OK;
    }

    bool erase()
    {
        FLASH_EraseInitTypeDef operation{};
        operation.TypeErase = FLASH_TYPEERASE_SECTORS;
        operation.Sector = FLASH_SECTOR_7;
        operation.NbSectors = 1;
        operation.VoltageRange = FLASH_VOLTAGE_RANGE_3; // Board VDD = 3.3 V.
        std::uint32_t error{};
        return HAL_FLASHEx_Erase(&operation, &error) == HAL_OK;
    }

    void yieldAfterProgram()
    {
        // STM32F411 DS10314 table 45: one x32 word takes 16 us typical,
        // 100 us characterized maximum. Never program the entire record in
        // one burst: block for a tick so higher-priority motion runs between
        // words. Erase (up to seconds) remains an exclusive stopped operation.
        vTaskDelay(1U);
    }
};
} // namespace

bool loadFromFlash(Parameters& parameters)
{
    if (!InternalFlash::layoutValid()) return false;
    InternalFlash flash;
    return ParameterJournal<InternalFlash>(flash).load(parameters);
}

SaveResult saveToFlash(const Parameters& parameters, bool recycle)
{
    if (!InternalFlash::layoutValid()) return SaveResult::io_error;
    if (HAL_FLASH_Unlock() != HAL_OK) return SaveResult::io_error;
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                          FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
    InternalFlash flash;
    const auto result = ParameterJournal<InternalFlash>(flash).save(parameters, recycle);
    HAL_FLASH_Lock();
    return result;
}
} // namespace MotionSettings
