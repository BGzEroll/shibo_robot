#ifndef WIFI_H
#define WIFI_H

#include <stdint.h>

namespace wifi
{
    struct network
    {
        char ssid[33] = {};
        int8_t rssi = 0;
        bool secured = false;
    };

    struct state
    {
        char ssid[33] = {};
        char ip[16] = {};
        bool connected = false;
        bool ap_active = false;
        bool scanning = false;
    };

    bool init();
    state get_state();
    bool scan();
    uint8_t get_networks(network *out, uint8_t capacity);
    bool connect(const char *ssid, const char *password);
}

#endif
