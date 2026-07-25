#include "app.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_console.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "network.h"
#include "wifi.pb.h"

typedef struct command_wait {
    SemaphoreHandle_t done;
    wlh_host_result_t result;
    uint16_t domain;
    int16_t status;
} command_wait_t;

static const char *TAG = "wlh-console";
static wlh_app_t *console_app;
static esp_console_repl_t *repl;

static void command_completion(void *context, wlh_host_result_t result,
                               uint16_t domain, int16_t status,
                               const uint8_t *payload, size_t payload_size) {
    command_wait_t *wait = context;
    (void)payload;
    (void)payload_size;
    wait->result = result;
    wait->domain = domain;
    wait->status = status;
    xSemaphoreGive(wait->done);
}

static int wait_for_command(command_wait_t *wait, wlh_host_result_t submitted) {
    int result = 1;
    if (submitted != WLH_HOST_OK) {
        printf("request rejected locally: %d\n", submitted);
        return 1;
    }
    if (xSemaphoreTake(wait->done, pdMS_TO_TICKS(15000u)) != pdTRUE) {
        printf("request timed out in console\n");
        return 1;
    }
    printf("result=%d domain=%u status=%d\n", wait->result, wait->domain,
           wait->status);
    if (wait->result == WLH_HOST_OK && wait->domain == 0u && wait->status == 0)
        result = 0;
    return result;
}

static command_wait_t new_wait(void) {
    command_wait_t wait = {0};
    wait.done = xSemaphoreCreateBinary();
    return wait;
}

static void delete_wait(command_wait_t *wait) {
    if (wait->done != NULL) vSemaphoreDelete(wait->done);
}

static int status_command(int argc, char **argv) {
    wlh_host_diagnostics_t diagnostics;
    (void)argc;
    (void)argv;
    wlh_host_get_diagnostics(&console_app->host, &diagnostics);
    printf("core: state=%d session=%lu tx=%lu rx=%lu pending=%lu "
           "timeouts=%lu checksum=%lu seq_gaps=%lu peer_resets=%lu\n",
           diagnostics.state, (unsigned long)diagnostics.session_id,
           (unsigned long)diagnostics.tx_frames,
           (unsigned long)diagnostics.rx_frames,
           (unsigned long)diagnostics.pending_rpc,
           (unsigned long)diagnostics.rpc_timeouts,
           (unsigned long)diagnostics.checksum_errors,
           (unsigned long)diagnostics.sequence_gaps,
           (unsigned long)diagnostics.peer_resets);
    wlh_network_print_status();
    return 0;
}

static int scan_command(int argc, char **argv) {
    command_wait_t wait = new_wait();
    wlh_wifi_scan_params_t params = {
        .scan_id = 1u,
        .include_hidden = true,
        .max_results = 32u,
    };
    wlh_host_result_t submitted;
    int result;
    (void)argc;
    (void)argv;
    if (wait.done == NULL) return 1;
    xSemaphoreTake(console_app->command_lock, portMAX_DELAY);
    submitted = wlh_host_wifi_scan(&console_app->host, &params,
                                   command_completion, &wait);
    result = wait_for_command(&wait, submitted);
    xSemaphoreGive(console_app->command_lock);
    delete_wait(&wait);
    return result;
}

static int sta_connect_command(int argc, char **argv) {
    command_wait_t wait = new_wait();
    wlh_wifi_connect_params_t params;
    wlh_host_result_t submitted;
    int result;
    if (argc < 2 || argc > 3) {
        printf("usage: sta_connect <ssid> [password]\n");
        return 1;
    }
    if (wait.done == NULL) return 1;
    memset(&params, 0, sizeof(params));
    params.ssid = (const uint8_t *)argv[1];
    params.ssid_size = strlen(argv[1]);
    if (argc == 3) {
        params.credential = (const uint8_t *)argv[2];
        params.credential_size = strlen(argv[2]);
        params.security = wlh_protocol_v1_WifiSecurity_WIFI_SECURITY_WPA2_PSK;
    } else {
        params.security = wlh_protocol_v1_WifiSecurity_WIFI_SECURITY_OPEN;
    }
    params.timeout_ms = 15000u;
    xSemaphoreTake(console_app->command_lock, portMAX_DELAY);
    submitted = wlh_host_wifi_connect(&console_app->host, &params,
                                      command_completion, &wait);
    result = wait_for_command(&wait, submitted);
    xSemaphoreGive(console_app->command_lock);
    delete_wait(&wait);
    return result;
}

static int sta_disconnect_command(int argc, char **argv) {
    command_wait_t wait = new_wait();
    wlh_host_result_t submitted;
    int result;
    (void)argc;
    (void)argv;
    if (wait.done == NULL) return 1;
    xSemaphoreTake(console_app->command_lock, portMAX_DELAY);
    submitted =
        wlh_host_wifi_disconnect(&console_app->host, command_completion, &wait);
    result = wait_for_command(&wait, submitted);
    xSemaphoreGive(console_app->command_lock);
    delete_wait(&wait);
    return result;
}

static int ap_start_command(int argc, char **argv) {
    command_wait_t wait = new_wait();
    wlh_wifi_start_ap_params_t params;
    wlh_host_result_t submitted;
    int result;
    if (argc < 2 || argc > 4) {
        printf("usage: ap_start <ssid> [password] [channel]\n");
        return 1;
    }
    if (wait.done == NULL) return 1;
    memset(&params, 0, sizeof(params));
    params.ssid = (const uint8_t *)argv[1];
    params.ssid_size = strlen(argv[1]);
    params.channel = argc == 4 ? (uint32_t)strtoul(argv[3], NULL, 10) : 1u;
    params.max_clients = 4u;
    if (argc >= 3 && argv[2][0] != '\0') {
        params.credential = (const uint8_t *)argv[2];
        params.credential_size = strlen(argv[2]);
        params.security = wlh_protocol_v1_WifiSecurity_WIFI_SECURITY_WPA2_PSK;
    } else {
        params.security = wlh_protocol_v1_WifiSecurity_WIFI_SECURITY_OPEN;
    }
    xSemaphoreTake(console_app->command_lock, portMAX_DELAY);
    submitted = wlh_host_wifi_start_ap(&console_app->host, &params,
                                       command_completion, &wait);
    result = wait_for_command(&wait, submitted);
    xSemaphoreGive(console_app->command_lock);
    delete_wait(&wait);
    return result;
}

static int ap_stop_command(int argc, char **argv) {
    command_wait_t wait = new_wait();
    wlh_host_result_t submitted;
    int result;
    (void)argc;
    (void)argv;
    if (wait.done == NULL) return 1;
    xSemaphoreTake(console_app->command_lock, portMAX_DELAY);
    submitted =
        wlh_host_wifi_stop_ap(&console_app->host, command_completion, &wait);
    result = wait_for_command(&wait, submitted);
    xSemaphoreGive(console_app->command_lock);
    delete_wait(&wait);
    return result;
}

static int ping_command(int argc, char **argv) {
    const char *hostname = argc >= 2 ? argv[1] : "baidu.com";
    esp_err_t result = wlh_network_ping(hostname, 4u);
    if (result != ESP_OK) {
        printf("ping failed: %s\n", esp_err_to_name(result));
        return 1;
    }
    return 0;
}

static void register_command(const char *name, const char *help,
                             esp_console_cmd_func_t function) {
    const esp_console_cmd_t command = {
        .command = name,
        .help = help,
        .func = function,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&command));
}

void wlh_console_start(wlh_app_t *app) {
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    esp_console_dev_uart_config_t uart_config =
        ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    console_app = app;
    repl_config.prompt = "wlh> ";
    repl_config.max_cmdline_length = 256u;
    esp_console_register_help_command();
    register_command("status", "show link and IP diagnostics", status_command);
    register_command("scan", "scan nearby Wi-Fi networks", scan_command);
    register_command("sta_connect", "sta_connect <ssid> [password]",
                     sta_connect_command);
    register_command("sta_disconnect", "disconnect station",
                     sta_disconnect_command);
    register_command("ap_start", "ap_start <ssid> [password] [channel]",
                     ap_start_command);
    register_command("ap_stop", "stop SoftAP", ap_stop_command);
    register_command("ping", "ping [hostname], default baidu.com",
                     ping_command);
    ESP_ERROR_CHECK(
        esp_console_new_repl_uart(&uart_config, &repl_config, &repl));
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
    ESP_LOGI(TAG, "console ready");
}
