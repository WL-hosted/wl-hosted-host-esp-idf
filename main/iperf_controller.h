#ifndef WLH_HOST_ESP_IDF_IPERF_CONTROLLER_H
#define WLH_HOST_ESP_IDF_IPERF_CONTROLLER_H

#include "esp_err.h"

#include <stdint.h>

typedef enum wlh_iperf_protocol {
    WLH_IPERF_TCP,
    WLH_IPERF_UDP,
} wlh_iperf_protocol_t;

typedef enum wlh_iperf_role {
    WLH_IPERF_CLIENT,
    WLH_IPERF_SERVER,
} wlh_iperf_role_t;

typedef struct wlh_iperf_request {
    wlh_iperf_protocol_t protocol;
    wlh_iperf_role_t role;
    const char *peer;
    uint32_t duration_sec;
    uint32_t target_mbps;
} wlh_iperf_request_t;

esp_err_t wlh_iperf_init(void);
esp_err_t wlh_iperf_start(const wlh_iperf_request_t *request);
void wlh_iperf_cancel(const char *reason);

#endif
