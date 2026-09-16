#pragma once
#include "task.h"
struct FakeTimer { void* id{}; unsigned starts{}, stops{}; };
using TimerHandle_t = FakeTimer*;
inline FakeTimer timer_instance;
inline TimerHandle_t xTimerCreate(const char*, TickType_t, BaseType_t, void* id,
                                 void (*)(TimerHandle_t)) {
    timer_instance = {id}; return &timer_instance;
}
inline void* pvTimerGetTimerID(TimerHandle_t timer) { return timer->id; }
inline BaseType_t xTimerStart(TimerHandle_t timer, TickType_t timeout) {
    assert(timeout == 0); ++timer->starts; return pdTRUE;
}
inline BaseType_t xTimerStop(TimerHandle_t timer, TickType_t timeout) {
    assert(timeout == 0); ++timer->stops; return pdTRUE;
}
