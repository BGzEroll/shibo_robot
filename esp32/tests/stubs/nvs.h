#ifndef TEST_STUB_NVS_H
#define TEST_STUB_NVS_H

#include <stddef.h>
#include <stdint.h>

using nvs_handle_t = uint32_t;
using esp_err_t = int32_t;

constexpr int32_t ESP_OK = 0;
constexpr int32_t ESP_ERR_NVS_NOT_FOUND = 1;
constexpr int32_t NVS_READONLY = 0;
constexpr int32_t NVS_READWRITE = 1;

esp_err_t nvs_open(const char *, int32_t, nvs_handle_t *);
esp_err_t nvs_get_str(nvs_handle_t, const char *, char *, size_t *);
esp_err_t nvs_set_str(nvs_handle_t, const char *, const char *);
esp_err_t nvs_commit(nvs_handle_t);
void nvs_close(nvs_handle_t);

#endif
