#pragma once
#include <cstdint>
struct I2C_HandleTypeDef {};
enum HAL_StatusTypeDef { HAL_OK, HAL_ERROR };
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef*, uint16_t, uint16_t, uint16_t,
                                  uint8_t*, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef*, uint16_t, uint16_t, uint16_t,
                                   uint8_t*, uint16_t, uint32_t);
void HAL_Delay(uint32_t);
