#pragma once
#include "FreeRTOS.h"
inline TickType_t test_tick = 0U;
inline TickType_t xTaskGetTickCount() { return test_tick; }
