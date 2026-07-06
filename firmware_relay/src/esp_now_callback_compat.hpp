#ifndef STAMPFLY_ESP_NOW_CALLBACK_COMPAT_HPP
#define STAMPFLY_ESP_NOW_CALLBACK_COMPAT_HPP

#include <esp_now.h>
#include <stdint.h>

#if defined(ESP_IDF_VERSION_MAJOR)
    #if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
        #define STAMPFLY_ESP_NOW_V5_CALLBACKS 1
    #else
        #define STAMPFLY_ESP_NOW_V5_CALLBACKS 0
    #endif
#else
    #define STAMPFLY_ESP_NOW_V5_CALLBACKS 0
#endif

#if STAMPFLY_ESP_NOW_V5_CALLBACKS
typedef esp_now_recv_info_t EspNowRecvInfo;
typedef wifi_tx_info_t EspNowSendInfo;
#else
typedef uint8_t EspNowRecvInfo;
typedef uint8_t EspNowSendInfo;
#endif

inline const uint8_t* espNowRecvSourceAddress(const EspNowRecvInfo* recv_info) {
#if STAMPFLY_ESP_NOW_V5_CALLBACKS
    return recv_info ? recv_info->src_addr : nullptr;
#else
    return reinterpret_cast<const uint8_t*>(recv_info);
#endif
}

#endif
