#ifndef TEST_STUB_FREERTOS_H
#define TEST_STUB_FREERTOS_H

#include <stdint.h>

using TickType_t = uint32_t;
using BaseType_t = int32_t;
using portMUX_TYPE = int32_t;

constexpr int32_t pdPASS = 1;
constexpr int32_t portMUX_INITIALIZER_UNLOCKED = 0;

#define portENTER_CRITICAL(lock) ((void)(lock))
#define portEXIT_CRITICAL(lock) ((void)(lock))
#define pdMS_TO_TICKS(value) (value)

#endif
