#include "iperf_controller.h"

#include <stdio.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "iperf.h"
#include "lwip/inet.h"
#include "network.h"

#define WLH_IPERF_MAX_DURATION_SEC 300u
#define WLH_IPERF_MAX_MBPS 100u

typedef struct wlh_iperf_controller {
    SemaphoreHandle_t mutex;
    iperf_id_t active_id;
} wlh_iperf_controller_t;

static const char *TAG = "wlh-iperf";
static wlh_iperf_controller_t controller = {.active_id = -1};

static void state_handler(iperf_id_t instance_id, iperf_state_data_t *data,
                          void *context) {
    wlh_iperf_controller_t *active = context;
    if (data == NULL || active == NULL) return;
    if (data->state == IPERF_STARTED)
        ESP_LOGI(TAG, "iPerf session %d started", instance_id);
    else if (data->state == IPERF_STOPPED)
        ESP_LOGI(TAG, "iPerf session %d stopped", instance_id);
    else if (data->state == IPERF_CLOSED) {
        xSemaphoreTake(active->mutex, portMAX_DELAY);
        if (active->active_id == instance_id) active->active_id = -1;
        xSemaphoreGive(active->mutex);
        ESP_LOGI(TAG, "iPerf session %d closed", instance_id);
    }
}

static bool valid_request(const wlh_iperf_request_t *request) {
    return request != NULL && request->duration_sec > 0u &&
           request->duration_sec <= WLH_IPERF_MAX_DURATION_SEC &&
           request->protocol <= WLH_IPERF_UDP &&
           request->role <= WLH_IPERF_SERVER &&
           (request->role == WLH_IPERF_SERVER || request->peer != NULL) &&
           (request->protocol != WLH_IPERF_UDP ||
            (request->target_mbps > 0u &&
             request->target_mbps <= WLH_IPERF_MAX_MBPS));
}

esp_err_t wlh_iperf_init(void) {
    if (controller.mutex != NULL) return ESP_OK;
    controller.mutex = xSemaphoreCreateMutex();
    return controller.mutex == NULL ? ESP_ERR_NO_MEM : ESP_OK;
}

esp_err_t wlh_iperf_start(const wlh_iperf_request_t *request) {
    esp_ip_addr_t source = {0};
    esp_ip_addr_t destination = {0};
    ip4_addr_t peer;
    iperf_cfg_t config;
    uint32_t protocol_flag;
    iperf_id_t id;

    if (controller.mutex == NULL || !valid_request(request))
        return ESP_ERR_INVALID_ARG;
    if (!wlh_network_get_sta_ipv4(&source.u_addr.ip4))
        return ESP_ERR_INVALID_STATE;
    source.type = ESP_IPADDR_TYPE_V4;
    protocol_flag =
        request->protocol == WLH_IPERF_TCP ? IPERF_FLAG_TCP : IPERF_FLAG_UDP;
    if (request->role == WLH_IPERF_CLIENT) {
        if (!ip4addr_aton(request->peer, &peer)) return ESP_ERR_INVALID_ARG;
        destination.type = ESP_IPADDR_TYPE_V4;
        destination.u_addr.ip4.addr = peer.addr;
        config = (iperf_cfg_t)IPERF_DEFAULT_CONFIG_CLIENT(protocol_flag,
                                                          destination);
        /* UDP supports an explicit source bind; TCP follows the STA route. */
        config.source = source;
        if (request->protocol == WLH_IPERF_UDP)
            config.bw_lim = (int32_t)request->target_mbps * 1000000;
    } else {
        config =
            (iperf_cfg_t)IPERF_DEFAULT_CONFIG_SERVER(protocol_flag, source);
    }
    config.time = request->duration_sec;
    config.state_handler = state_handler;
    config.state_handler_priv = &controller;

    xSemaphoreTake(controller.mutex, portMAX_DELAY);
    if (controller.active_id >= 0) {
        xSemaphoreGive(controller.mutex);
        return ESP_ERR_INVALID_STATE;
    }
    id = iperf_start_instance(&config);
    if (id >= 0) controller.active_id = id;
    xSemaphoreGive(controller.mutex);
    if (id < 0) return ESP_FAIL;

    printf("iperf started: %s %s, duration=%lu sec%s\n",
           request->protocol == WLH_IPERF_TCP ? "tcp" : "udp",
           request->role == WLH_IPERF_CLIENT ? "client" : "server",
           (unsigned long)request->duration_sec,
           request->protocol == WLH_IPERF_UDP &&
                   request->role == WLH_IPERF_CLIENT
               ? " (UDP rate configured)"
               : "");
    return ESP_OK;
}

void wlh_iperf_cancel(const char *reason) {
    iperf_id_t id;
    if (controller.mutex == NULL) return;
    xSemaphoreTake(controller.mutex, portMAX_DELAY);
    id = controller.active_id;
    xSemaphoreGive(controller.mutex);
    if (id >= 0) {
        ESP_LOGW(TAG, "cancelling iPerf session %d: %s", id,
                 reason == NULL ? "requested" : reason);
        (void)iperf_stop_instance(id);
    }
}
