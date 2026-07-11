/* 
** RP2XXX Replacement for FreeRTOS critical section mechanism
** Copyright (c) 2026, Sergey Morozyuk <me@sergds.xyz>
*/

#pragma once
#include <Arduino.h>

#undef taskENTER_CRITICAL_FROM_ISR
#undef taskEXIT_CRITICAL_FROM_ISR
#undef portENTER_CRITICAL
#undef portEXIT_CRITICAL

using BaseType_t = int;

// Here we don't actually need interrupt mask, as arduino-pico saves it internally in a stack
inline int taskENTER_CRITICAL_FROM_ISR() {
    noInterrupts();
    return 0;
}

inline void taskEXIT_CRITICAL_FROM_ISR(int mask) {
    interrupts();
    return;
}

#define portENTER_CRITICAL() noInterrupts();
#define portEXIT_CRITICAL()  interrupts();
