#pragma once
#include <stdint.h>
using TickType_t = uint32_t;
using BaseType_t = int;
using portMUX_TYPE = int;
constexpr int pdPASS = 1;
constexpr int portMUX_INITIALIZER_UNLOCKED = 0;
#define portENTER_CRITICAL(lock) ((void)(lock))
#define portEXIT_CRITICAL(lock) ((void)(lock))
#define pdMS_TO_TICKS(value) (value)
