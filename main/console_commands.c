#include "app.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_console.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "iperf_controller.h"
#include "network.h"
#include "ota_command.h"
#include "wifi.pb.h"

typedef struct command_wait {
    SemaphoreHandle_t done;
    wlh_host_result_t result;
    uint16_t domain;
    int16_t status;
    /* Typed responses are copied here by their completion so the printing
       happens on the console task rather than the Core task. */
    bool has_io_state;
    wlh_host_io_state_t io_state;
    bool has_adc_sample;
    wlh_host_adc_sample_t adc_sample;
    bool has_kv_value;
    char kv_value[WLH_HOST_MAX_KV_VALUE_SIZE + 1u];
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

static void io_read_completion(void *context, wlh_host_result_t result,
                               uint16_t domain, int16_t status,
                               const wlh_host_io_state_t *state) {
    command_wait_t *wait = context;
    wait->result = result;
    wait->domain = domain;
    wait->status = status;
    wait->has_io_state = state != NULL;
    if (state != NULL) wait->io_state = *state;
    xSemaphoreGive(wait->done);
}

static void adc_read_completion(void *context, wlh_host_result_t result,
                                uint16_t domain, int16_t status,
                                const wlh_host_adc_sample_t *sample) {
    command_wait_t *wait = context;
    wait->result = result;
    wait->domain = domain;
    wait->status = status;
    wait->has_adc_sample = sample != NULL;
    if (sample != NULL) wait->adc_sample = *sample;
    xSemaphoreGive(wait->done);
}

static void kv_read_completion(void *context, wlh_host_result_t result,
                               uint16_t domain, int16_t status,
                               const char *value, size_t value_size) {
    command_wait_t *wait = context;
    wait->result = result;
    wait->domain = domain;
    wait->status = status;
    wait->has_kv_value = value != NULL;
    if (value != NULL && value_size < sizeof(wait->kv_value)) {
        memcpy(wait->kv_value, value, value_size);
        wait->kv_value[value_size] = '\0';
    } else {
        wait->has_kv_value = false;
    }
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

static int wait_for_operation(command_wait_t *wait, wlh_host_result_t submitted,
                              uint32_t timeout_ms) {
    int result = wait_for_command(wait, submitted);
    if (result == 0) {
        if (xSemaphoreTake(console_app->operation_done,
                           pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
            printf("operation did not complete in time\n");
            result = 1;
        }
    }
    console_app->pending_operation = WLH_APP_OP_NONE;
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
    /* Drop any stale completion from a previous scan. */
    (void)xSemaphoreTake(console_app->scan_done, 0);
    submitted = wlh_host_wifi_scan(&console_app->host, &params,
                                   command_completion, &wait);
    result = wait_for_command(&wait, submitted);
    if (result == 0 && xSemaphoreTake(console_app->scan_done,
                                      pdMS_TO_TICKS(30000u)) != pdTRUE) {
        printf("scan did not complete in time\n");
        result = 1;
    }
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
    (void)xSemaphoreTake(console_app->operation_done, 0);
    console_app->pending_operation = WLH_APP_OP_CONNECT;
    submitted = wlh_host_wifi_connect(&console_app->host, &params,
                                      command_completion, &wait);
    result = wait_for_operation(&wait, submitted, 20000u);
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
    (void)xSemaphoreTake(console_app->operation_done, 0);
    console_app->pending_operation = WLH_APP_OP_DISCONNECT;
    submitted =
        wlh_host_wifi_disconnect(&console_app->host, command_completion, &wait);
    result = wait_for_operation(&wait, submitted, 10000u);
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
    (void)xSemaphoreTake(console_app->operation_done, 0);
    console_app->pending_operation = WLH_APP_OP_AP_START;
    submitted = wlh_host_wifi_start_ap(&console_app->host, &params,
                                       command_completion, &wait);
    result = wait_for_operation(&wait, submitted, 10000u);
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
    (void)xSemaphoreTake(console_app->operation_done, 0);
    console_app->pending_operation = WLH_APP_OP_AP_STOP;
    submitted =
        wlh_host_wifi_stop_ap(&console_app->host, command_completion, &wait);
    result = wait_for_operation(&wait, submitted, 10000u);
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

static bool parse_u32_range(const char *text, uint32_t minimum,
                            uint32_t maximum, uint32_t *value) {
    char *end = NULL;
    unsigned long parsed;
    if (text == NULL || value == NULL) return false;
    parsed = strtoul(text, &end, 10);
    if (end == text || *end != '\0' || parsed < minimum || parsed > maximum)
        return false;
    *value = (uint32_t)parsed;
    return true;
}

static int iperf_command(int argc, char **argv) {
    wlh_iperf_request_t request = {
        .duration_sec = 30u,
        .target_mbps = 20u,
    };
    wlh_host_diagnostics_t diagnostics;
    esp_err_t result;
    bool client;

    if (argc < 3 ||
        (strcmp(argv[1], "tcp") != 0 && strcmp(argv[1], "udp") != 0) ||
        (strcmp(argv[2], "client") != 0 && strcmp(argv[2], "server") != 0)) {
        printf("usage: iperf tcp client <IPv4> [duration_sec] | tcp server "
               "[duration_sec] | udp client <IPv4> [duration_sec] [mbps] | "
               "udp server [duration_sec]\n");
        return 1;
    }
    request.protocol =
        strcmp(argv[1], "tcp") == 0 ? WLH_IPERF_TCP : WLH_IPERF_UDP;
    client = strcmp(argv[2], "client") == 0;
    request.role = client ? WLH_IPERF_CLIENT : WLH_IPERF_SERVER;
    if (client) {
        if (argc < 4 || argc > (request.protocol == WLH_IPERF_UDP ? 6 : 5))
            goto usage;
        request.peer = argv[3];
        if ((argc >= 5 &&
             !parse_u32_range(argv[4], 1u, 300u, &request.duration_sec)) ||
            (request.protocol == WLH_IPERF_UDP && argc == 6 &&
             !parse_u32_range(argv[5], 1u, 100u, &request.target_mbps)))
            goto usage;
    } else {
        if (argc > 4 || (argc == 4 && !parse_u32_range(argv[3], 1u, 300u,
                                                       &request.duration_sec)))
            goto usage;
    }
    wlh_host_get_diagnostics(&console_app->host, &diagnostics);
    if (diagnostics.state != WLH_HOST_STATE_READY) {
        printf("iperf unavailable: host is not ready\n");
        return 1;
    }
    result = wlh_iperf_start(&request);
    if (result != ESP_OK) {
        printf("iperf start failed: %s\n", esp_err_to_name(result));
        return 1;
    }
    return 0;

usage:
    printf("invalid iPerf duration or rate\n");
    return 1;
}

static const char *io_mode_name(wlh_host_io_mode_t mode) {
    switch (mode) {
    case WLH_HOST_IO_MODE_INPUT:
        return "in";
    case WLH_HOST_IO_MODE_OUTPUT:
        return "out";
    default:
        return "od";
    }
}

static const char *io_pull_name(wlh_host_io_pull_t pull) {
    switch (pull) {
    case WLH_HOST_IO_PULL_UP:
        return "up";
    case WLH_HOST_IO_PULL_DOWN:
        return "down";
    default:
        return "none";
    }
}

/* Returns false on an unparseable pin so the command can report usage. */
static bool parse_pin(const char *text, uint32_t *pin_id) {
    char *end = NULL;
    unsigned long value = strtoul(text, &end, 10);
    if (end == text || *end != '\0' || value > 0xffffffffUL) return false;
    *pin_id = (uint32_t)value;
    return true;
}

static int io_config_command(int argc, char **argv) {
    command_wait_t wait;
    wlh_host_io_config_t config = {0};
    wlh_host_result_t submitted;
    int result;
    if (argc < 3 || argc > 5) {
        printf("usage: io_config <pin> <in|out|od> [none|up|down] [0|1]\n");
        return 1;
    }
    if (!parse_pin(argv[1], &config.pin_id)) {
        printf("invalid pin: %s\n", argv[1]);
        return 1;
    }
    if (strcmp(argv[2], "in") == 0)
        config.mode = WLH_HOST_IO_MODE_INPUT;
    else if (strcmp(argv[2], "out") == 0)
        config.mode = WLH_HOST_IO_MODE_OUTPUT;
    else if (strcmp(argv[2], "od") == 0)
        config.mode = WLH_HOST_IO_MODE_OPEN_DRAIN;
    else {
        printf("invalid mode: %s (expected in, out or od)\n", argv[2]);
        return 1;
    }
    config.pull = WLH_HOST_IO_PULL_NONE;
    if (argc >= 4) {
        if (strcmp(argv[3], "none") == 0)
            config.pull = WLH_HOST_IO_PULL_NONE;
        else if (strcmp(argv[3], "up") == 0)
            config.pull = WLH_HOST_IO_PULL_UP;
        else if (strcmp(argv[3], "down") == 0)
            config.pull = WLH_HOST_IO_PULL_DOWN;
        else {
            printf("invalid pull: %s (expected none, up or down)\n", argv[3]);
            return 1;
        }
    }
    if (argc == 5) config.initial_level = strcmp(argv[4], "0") != 0;

    wait = new_wait();
    if (wait.done == NULL) return 1;
    xSemaphoreTake(console_app->command_lock, portMAX_DELAY);
    submitted = wlh_host_io_configure(&console_app->host, &config,
                                      command_completion, &wait);
    result = wait_for_command(&wait, submitted);
    xSemaphoreGive(console_app->command_lock);
    delete_wait(&wait);
    return result;
}

static int io_read_command(int argc, char **argv) {
    command_wait_t wait;
    uint32_t pin_id;
    wlh_host_result_t submitted;
    int result;
    if (argc != 2) {
        printf("usage: io_read <pin>\n");
        return 1;
    }
    if (!parse_pin(argv[1], &pin_id)) {
        printf("invalid pin: %s\n", argv[1]);
        return 1;
    }
    wait = new_wait();
    if (wait.done == NULL) return 1;
    xSemaphoreTake(console_app->command_lock, portMAX_DELAY);
    submitted =
        wlh_host_io_read(&console_app->host, pin_id, io_read_completion, &wait);
    result = wait_for_command(&wait, submitted);
    if (result == 0 && wait.has_io_state)
        printf("pin=%lu level=%u mode=%s pull=%s\n",
               (unsigned long)wait.io_state.pin_id,
               wait.io_state.level ? 1u : 0u, io_mode_name(wait.io_state.mode),
               io_pull_name(wait.io_state.pull));
    xSemaphoreGive(console_app->command_lock);
    delete_wait(&wait);
    return result;
}

static int io_write_command(int argc, char **argv) {
    command_wait_t wait;
    uint32_t pin_id;
    wlh_host_result_t submitted;
    int result;
    if (argc != 3) {
        printf("usage: io_write <pin> <0|1>\n");
        return 1;
    }
    if (!parse_pin(argv[1], &pin_id)) {
        printf("invalid pin: %s\n", argv[1]);
        return 1;
    }
    wait = new_wait();
    if (wait.done == NULL) return 1;
    xSemaphoreTake(console_app->command_lock, portMAX_DELAY);
    submitted =
        wlh_host_io_write(&console_app->host, pin_id, strcmp(argv[2], "0") != 0,
                          command_completion, &wait);
    result = wait_for_command(&wait, submitted);
    xSemaphoreGive(console_app->command_lock);
    delete_wait(&wait);
    return result;
}

static int adc_read_command(int argc, char **argv) {
    command_wait_t wait;
    uint32_t pin_id;
    wlh_host_result_t submitted;
    int result;
    if (argc != 2) {
        printf("usage: adc_read <pin>\n");
        return 1;
    }
    if (!parse_pin(argv[1], &pin_id)) {
        printf("invalid pin: %s\n", argv[1]);
        return 1;
    }
    wait = new_wait();
    if (wait.done == NULL) return 1;
    xSemaphoreTake(console_app->command_lock, portMAX_DELAY);
    submitted = wlh_host_adc_read(&console_app->host, pin_id,
                                  adc_read_completion, &wait);
    result = wait_for_command(&wait, submitted);
    if (result == 0 && wait.has_adc_sample)
        printf("pin=%lu mv=%lu\n", (unsigned long)wait.adc_sample.pin_id,
               (unsigned long)wait.adc_sample.millivolts);
    xSemaphoreGive(console_app->command_lock);
    delete_wait(&wait);
    return result;
}

static int kv_read_command(int argc, char **argv) {
    command_wait_t wait;
    wlh_host_result_t submitted;
    int result;
    if (argc != 2) {
        printf("usage: kv_read <key>\n");
        return 1;
    }
    wait = new_wait();
    if (wait.done == NULL) return 1;
    xSemaphoreTake(console_app->command_lock, portMAX_DELAY);
    submitted = wlh_host_kv_read(&console_app->host, argv[1],
                                 kv_read_completion, &wait);
    result = wait_for_command(&wait, submitted);
    if (result == 0 && wait.has_kv_value)
        printf("value=\"%s\"\n", wait.kv_value);
    xSemaphoreGive(console_app->command_lock);
    delete_wait(&wait);
    return result;
}

static int kv_write_command(int argc, char **argv) {
    command_wait_t wait;
    wlh_host_result_t submitted;
    int result;
    if (argc != 3) {
        printf("usage: kv_write <key> <value>\n");
        return 1;
    }
    wait = new_wait();
    if (wait.done == NULL) return 1;
    xSemaphoreTake(console_app->command_lock, portMAX_DELAY);
    submitted = wlh_host_kv_write(&console_app->host, argv[1], argv[2],
                                  command_completion, &wait);
    result = wait_for_command(&wait, submitted);
    xSemaphoreGive(console_app->command_lock);
    delete_wait(&wait);
    return result;
}

static int kv_erase_command(int argc, char **argv) {
    command_wait_t wait;
    wlh_host_result_t submitted;
    int result;
    if (argc != 2) {
        printf("usage: kv_erase <key>\n");
        return 1;
    }
    wait = new_wait();
    if (wait.done == NULL) return 1;
    xSemaphoreTake(console_app->command_lock, portMAX_DELAY);
    submitted = wlh_host_kv_erase(&console_app->host, argv[1],
                                  command_completion, &wait);
    result = wait_for_command(&wait, submitted);
    xSemaphoreGive(console_app->command_lock);
    delete_wait(&wait);
    return result;
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
    console_app = app;
    repl_config.prompt = "wlh> ";
    repl_config.max_cmdline_length = 256u;
    repl_config.task_stack_size = 8192u;
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
    register_command("iperf",
                     "iperf tcp|udp client|server; use help iperf for syntax",
                     iperf_command);
    register_command("io_config",
                     "io_config <pin> <in|out|od> [none|up|down] [0|1]",
                     io_config_command);
    register_command("io_read", "io_read <pin>", io_read_command);
    register_command("io_write", "io_write <pin> <0|1>", io_write_command);
    register_command("adc_read", "adc_read <pin>", adc_read_command);
    register_command("kv_read", "kv_read <key>", kv_read_command);
    register_command("kv_write", "kv_write <key> <value>", kv_write_command);
    register_command("kv_erase", "kv_erase <key>", kv_erase_command);
    wlh_ota_command_register(app);
#if defined(CONFIG_ESP_CONSOLE_UART_DEFAULT) ||                                \
    defined(CONFIG_ESP_CONSOLE_UART_CUSTOM)
    esp_console_dev_uart_config_t uart_config =
        ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(
        esp_console_new_repl_uart(&uart_config, &repl_config, &repl));
#elif defined(CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG)
    esp_console_dev_usb_serial_jtag_config_t usb_config =
        ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(
        esp_console_new_repl_usb_serial_jtag(&usb_config, &repl_config, &repl));
#else
    ESP_LOGE(TAG, "no console backend enabled");
    return;
#endif
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
    ESP_LOGI(TAG, "console ready");
}
