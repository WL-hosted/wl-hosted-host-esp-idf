#include "app.h"

#include <stdlib.h>
#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "pb_decode.h"
#include "sdio_transport.h"
#include "wifi.pb.h"

#include "network.h"

static const char *TAG = "wlh-host";
wlh_app_t g_wlh_app;

static uint8_t *buffer_alloc(void *context, size_t size) {
    (void)context;
    return malloc(size);
}

static void buffer_free(void *context, uint8_t *buffer) {
    (void)context;
    free(buffer);
}

static void executor_task_main(void *argument) {
    wlh_app_t *app = argument;
    wlh_app_task_t task;
    for (;;) {
        if (xQueueReceive(app->executor_queue, &task, portMAX_DELAY) == pdTRUE)
            task.function(task.context);
    }
}

static int executor_post(void *context, wlh_task_fn function,
                         void *task_context) {
    wlh_app_t *app = context;
    wlh_app_task_t task = {function, task_context};
    return xQueueSend(app->executor_queue, &task, 0) == pdTRUE ? 0 : -1;
}

static void initialize_completion(void *context, wlh_host_result_t result,
                                  uint16_t domain, int16_t status,
                                  const uint8_t *payload, size_t payload_size) {
    wlh_app_t *app = context;
    (void)payload;
    (void)payload_size;
    app->wifi_initialized =
        result == WLH_HOST_OK && domain == 0u && status == 0;
    ESP_LOGI(TAG, "Wi-Fi initialize: result=%d domain=%u status=%d", result,
             domain, status);
}

static void log_scan_result(const wlh_host_event_t *event) {
    wlh_protocol_v1_WifiScanResultEvent result =
        wlh_protocol_v1_WifiScanResultEvent_init_zero;
    pb_istream_t input =
        pb_istream_from_buffer(event->payload, event->payload_size);
    pb_size_t index;
    if (!pb_decode(&input, wlh_protocol_v1_WifiScanResultEvent_fields,
                   &result)) {
        ESP_LOGW(TAG, "invalid scan result event");
        return;
    }
    for (index = 0u; index < result.networks_count; ++index) {
        const wlh_protocol_v1_WifiNetwork *network = &result.networks[index];
        char ssid[33] = {0};
        size_t size = network->ssid.size < sizeof(ssid) - 1u
                          ? network->ssid.size
                          : sizeof(ssid) - 1u;
        memcpy(ssid, network->ssid.bytes, size);
        ESP_LOGI(TAG, "scan: ssid=\"%s\" channel=%lu rssi=%ld security=%d",
                 ssid, (unsigned long)network->channel, (long)network->rssi_dbm,
                 network->security);
    }
}

static void handle_connected(const wlh_host_event_t *event) {
    wlh_protocol_v1_WifiConnectedEvent connected =
        wlh_protocol_v1_WifiConnectedEvent_init_zero;
    pb_istream_t input =
        pb_istream_from_buffer(event->payload, event->payload_size);
    if (!pb_decode(&input, wlh_protocol_v1_WifiConnectedEvent_fields,
                   &connected) ||
        !connected.has_link || connected.link.mac.size != 6u) {
        ESP_LOGW(TAG, "connected event omitted interface MAC");
        return;
    }
    if (connected.link.interface ==
        wlh_protocol_v1_WifiInterface_WIFI_INTERFACE_AP) {
        wlh_network_ap_up(connected.link.mac.bytes);
        ESP_LOGI(TAG, "SoftAP link is up at 192.168.4.1");
    } else {
        wlh_network_sta_up(connected.link.mac.bytes);
        ESP_LOGI(TAG, "station link is up; DHCP client started");
    }
}

static void handle_disconnected(const wlh_host_event_t *event) {
    wlh_protocol_v1_WifiDisconnectedEvent disconnected =
        wlh_protocol_v1_WifiDisconnectedEvent_init_zero;
    pb_istream_t input =
        pb_istream_from_buffer(event->payload, event->payload_size);
    if (!pb_decode(&input, wlh_protocol_v1_WifiDisconnectedEvent_fields,
                   &disconnected))
        return;
    if (disconnected.interface ==
        wlh_protocol_v1_WifiInterface_WIFI_INTERFACE_AP)
        wlh_network_ap_down();
    else
        wlh_network_sta_down();
}

void wlh_app_on_event(void *context, const wlh_host_event_t *event) {
    wlh_app_t *app = context;
    switch (event->kind) {
    case WLH_HOST_EVENT_STATE_CHANGED:
        ESP_LOGI(TAG, "Host Core state=%d", event->state);
        if (event->state == WLH_HOST_STATE_READY && !app->wifi_initialized) {
            wlh_host_result_t result = wlh_host_wifi_initialize(
                &app->host, initialize_completion, app);
            if (result != WLH_HOST_OK)
                ESP_LOGW(TAG, "unable to submit Wi-Fi initialize: %d", result);
        }
        break;
    case WLH_HOST_EVENT_WIFI_SCAN_RESULT:
        log_scan_result(event);
        break;
    case WLH_HOST_EVENT_WIFI_SCAN_COMPLETED:
        ESP_LOGI(TAG, "scan complete");
        break;
    case WLH_HOST_EVENT_WIFI_CONNECTED:
        handle_connected(event);
        break;
    case WLH_HOST_EVENT_WIFI_DISCONNECTED:
        handle_disconnected(event);
        break;
    case WLH_HOST_EVENT_ETHERNET_STA_RX:
        wlh_network_input(false, event->payload, event->payload_size);
        break;
    case WLH_HOST_EVENT_ETHERNET_AP_RX:
        wlh_network_input(true, event->payload, event->payload_size);
        break;
    case WLH_HOST_EVENT_WIFI_AP_CLIENT_JOINED:
        ESP_LOGI(TAG, "SoftAP client joined");
        break;
    case WLH_HOST_EVENT_WIFI_AP_CLIENT_LEFT:
        ESP_LOGI(TAG, "SoftAP client left");
        break;
    case WLH_HOST_EVENT_PROTOCOL_FAULT:
        ESP_LOGE(TAG, "protocol fault");
        break;
    default:
        break;
    }
}

void app_main(void) {
    wlh_host_config_t config;
    esp_err_t nvs_result = nvs_flash_init();
    if (nvs_result == ESP_ERR_NVS_NO_FREE_PAGES ||
        nvs_result == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_result = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_result);
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    memset(&g_wlh_app, 0, sizeof(g_wlh_app));
    g_wlh_app.executor_queue = xQueueCreate(32u, sizeof(wlh_app_task_t));
    g_wlh_app.command_lock = xSemaphoreCreateMutex();
    configASSERT(g_wlh_app.executor_queue != NULL);
    configASSERT(g_wlh_app.command_lock != NULL);
    configASSERT(xTaskCreate(executor_task_main, "wlh-executor", 4096u,
                             &g_wlh_app, 5, NULL) == pdPASS);

    wlh_freertos_osal_init(&g_wlh_app.freertos_osal);
    configASSERT(wlh_sdio_transport_init(&g_wlh_app.host) == 0);
    memset(&config, 0, sizeof(config));
    config.transport = wlh_sdio_transport_ops();
    config.buffers = (wlh_buffer_ops_t){NULL, buffer_alloc, buffer_free};
    config.osal = wlh_freertos_osal_ops(&g_wlh_app.freertos_osal);
    config.executor = (wlh_executor_ops_t){&g_wlh_app, executor_post};
    config.on_event = wlh_app_on_event;
    config.event_context = &g_wlh_app;
    config.max_frame_size = WLH_SDIO_MAX_FRAME_SIZE;
    config.rpc_timeout_ms = 10000u;
    config.heartbeat_timeout_ms = 5000u;
    config.max_pending_rpc = 8u;
    config.core_queue_depth = 16u;
    config.stop_timeout_ms = 3000u;
    config.core_task = (wlh_osal_task_attributes_t){"wlh-host-core", 8192u, 6};

    configASSERT(wlh_host_init(&g_wlh_app.host, &config) == WLH_HOST_OK);
    ESP_ERROR_CHECK(wlh_network_init(&g_wlh_app.host));
    wlh_console_start(&g_wlh_app);
    configASSERT(wlh_host_start(&g_wlh_app.host) == WLH_HOST_OK);
    ESP_LOGI(TAG, "ESP32-P4 Host started; use 'help' for console commands");
}
