#ifndef TEST_STUB_TASK_H
#define TEST_STUB_TASK_H

#include "FreeRTOS.h"

using TaskHandle_t = void *;

TickType_t xTaskGetTickCount();
BaseType_t xTaskCreatePinnedToCore(void (*task)(void *), const char *, uint32_t, void *, int32_t,
    TaskHandle_t *, int32_t);
void vTaskDelayUntil(TickType_t *, TickType_t);

#endif
