#pragma once
#include "FreeRTOS.h"
using TaskHandle_t = void *;
TickType_t xTaskGetTickCount();
BaseType_t xTaskCreatePinnedToCore(void (*task)(void *), const char *, uint32_t,
    void *, int, TaskHandle_t *, int);
void vTaskDelayUntil(TickType_t *, TickType_t);
