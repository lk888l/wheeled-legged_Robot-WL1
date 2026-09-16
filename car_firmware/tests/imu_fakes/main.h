#pragma once
#include <algorithm>
#include <array>
#include <cstdint>

enum HAL_StatusTypeDef { HAL_OK, HAL_ERROR };
struct I2C_HandleTypeDef {
    std::array<uint8_t, 256> registers{};
    bool fail{};
};
inline HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef* device, uint16_t,
    uint16_t reg, uint16_t, uint8_t* out, uint16_t size, uint32_t)
{
    if (device->fail) return HAL_ERROR;
    std::copy_n(device->registers.data() + reg, size, out);
    return HAL_OK;
}
inline HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef* device, uint16_t,
    uint16_t reg, uint16_t, uint8_t* data, uint16_t size, uint32_t)
{
    if (device->fail) return HAL_ERROR;
    std::copy_n(data, size, device->registers.data() + reg);
    return HAL_OK;
}
