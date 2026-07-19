/* 
** RP2XXX Replacement for FreeRTOS Queues
** Copyright (c) 2026, Sergey Morozyuk <me@sergds.xyz>
*/

#pragma once
#include <cstdint>
#include <cstdlib>
#include <pico/util/queue.h>

#define pdPASS  1
#define pdTRUE  1
#define pdFALSE 0

using xQueueHandle = queue_t*;

inline xQueueHandle xQueueCreate(size_t lenght, size_t item_size) {
    queue_t* q = new queue_t;
    queue_init(q, item_size, lenght);
    return q;
}

inline int xQueueSendFromISR(xQueueHandle xQueue, const void *pvItemToQueue, void* /* pxHigherPriorityTaskWoken */) {
    return queue_try_add(xQueue, pvItemToQueue) ? pdTRUE : pdFALSE;
}
inline int xQueueReceive(xQueueHandle xQueue, void* const pvBuffer, uint32_t /* xTicksToWait */ ) {
    return queue_try_remove(xQueue, pvBuffer) ? pdTRUE : pdFALSE;
}