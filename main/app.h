#ifndef WLH_HOST_ESP_IDF_APP_H
#define WLH_HOST_ESP_IDF_APP_H

#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "wlh/freertos_osal.h"
#include "wlh/host.h"

typedef struct wlh_app_task {
    wlh_task_fn function;
    void *context;
} wlh_app_task_t;

typedef struct wlh_app {
    wlh_host_t host;
    wlh_freertos_osal_t freertos_osal;
    QueueHandle_t executor_queue;
    SemaphoreHandle_t command_lock;
    bool wifi_initialized;
} wlh_app_t;

extern wlh_app_t g_wlh_app;

void wlh_app_on_event(void *context, const wlh_host_event_t *event);
void wlh_console_start(wlh_app_t *app);

#endif
