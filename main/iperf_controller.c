#include "iperf_controller.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "iperf.h"
#include "iperf2.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "network.h"

#define WLH_IPERF_MAX_DURATION_SEC 300u
#define WLH_IPERF_MAX_MBPS 100u
#define WLH_IPERF_PORT 5001u
#define WLH_IPERF_REPORT_INTERVAL_MS 3000u
/* iPerf2 servers retransmit the final report for up to ~10 s while they wait
 * for the client to stop re-sending FIN datagrams, and the WL-hosted TX path
 * can hold a multi-second backlog at the moment the data phase ends. The
 * client must outlive that window; each attempt re-sends the FIN, which is
 * also what keeps a server that missed the first FIN retransmitting. */
#define WLH_IPERF_UDP_ACK_ATTEMPTS 20u
#define WLH_IPERF_UDP_ACK_WAIT_MS 500u
#define WLH_IPERF_CUSTOM_INSTANCE_ID 1
/* The UDP session task must outrank the SDIO workers and the executor so it
 * keeps draining the tiny per-socket receive mbox during a firehose burst;
 * otherwise lwIP drops datagrams on a full mbox and the loss reports lie. */
#define WLH_IPERF_UDP_TASK_PRIORITY 12u

typedef struct wlh_iperf_controller {
    SemaphoreHandle_t mutex;
    iperf_id_t active_id;
    TaskHandle_t udp_task;
    int udp_socket;
    bool udp_cancelled;
    uint32_t udp_peer_ipv4;
    wlh_iperf_request_t udp_request;
} wlh_iperf_controller_t;

static const char *TAG = "wlh-iperf";
#define WLH_IPERF_UDP_CATCHUP_BURST 16u
static wlh_iperf_controller_t controller = {
    .active_id = -1,
    .udp_socket = -1,
};

static uint64_t monotonic_ms(void) {
    return (uint64_t)esp_timer_get_time() / 1000u;
}

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

static bool udp_cancelled(void) {
    bool cancelled;
    xSemaphoreTake(controller.mutex, portMAX_DELAY);
    cancelled = controller.udp_cancelled;
    xSemaphoreGive(controller.mutex);
    return cancelled;
}

static bool udp_send_backpressured(void);

static bool send_udp_server_report(int socket, const struct sockaddr_in *peer,
                                   socklen_t peer_size, uint64_t elapsed_ms,
                                   const wlh_iperf2_udp_stats_t *stats,
                                   bool retry_without_terminal) {
    uint8_t report[WLH_IPERF2_DEFAULT_UDP_PACKET_SIZE] = {0};
    uint8_t received[WLH_IPERF2_DEFAULT_UDP_PACKET_SIZE];
    uint32_t attempt;
    if (!wlh_iperf2_udp_encode_server_report(report, sizeof(report),
                                             stats->bytes, elapsed_ms, stats))
        return false;
    for (attempt = 0u; attempt < WLH_IPERF_UDP_ACK_ATTEMPTS && !udp_cancelled();
         ++attempt) {
        int sent = lwip_sendto(socket, report, sizeof(report), 0,
                               (const struct sockaddr *)peer, peer_size);
        if (sent != (int)sizeof(report)) {
            if (!udp_send_backpressured()) {
                ESP_LOGW(TAG, "UDP server report send failed: errno=%d", errno);
                return false;
            }
            /* The client paces its FIN retransmissions, not our backpressure;
               a burst of TX congestion can outlast a 1 ms spin (observed:
               the server deadline report bounced on ENOBUFS for longer and
               the client never got its report). Wait for the next FIN,
               bounded by the socket receive timeout, and retry then. */
            if (lwip_recvfrom(socket, received, sizeof(received), 0, NULL, NULL) <
                0) {
                if (errno != EAGAIN && errno != EWOULDBLOCK) return false;
                return false;
            }
            continue;
        }
        /* iPerf2 retransmits its FIN while it waits for this report.  Drain
         * one such retransmission before sending the next copy. */
        if (lwip_recvfrom(socket, received, sizeof(received), 0, NULL, NULL) <
            0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) return false;
            if (!retry_without_terminal) return true;
        }
    }
    return !udp_cancelled();
}

static void print_udp_summary(uint64_t elapsed_ms,
                              const wlh_iperf2_udp_stats_t *stats) {
    double seconds = (double)elapsed_ms / 1000.0;
    double mbytes = (double)stats->bytes / (1024.0 * 1024.0);
    double mbps = elapsed_ms == 0u ? 0.0
                                   : (double)stats->bytes * 8.0 /
                                         (double)elapsed_ms / 1000.0;
    uint64_t expected = stats->packets + stats->lost;
    double loss =
        expected == 0u ? 0.0 : (double)stats->lost * 100.0 / (double)expected;
    printf("[  1]  0.0-%.1f sec\t%.2f MBytes\t%.2f Mbits/sec\n", seconds,
           mbytes, mbps);
    printf("[  1] %.3f ms %llu/%llu (%.2f%%)\n", stats->jitter_ms,
           (unsigned long long)stats->lost, (unsigned long long)expected, loss);
}

static void finish_udp_task(int socket) {
    if (socket >= 0) lwip_close(socket);
    xSemaphoreTake(controller.mutex, portMAX_DELAY);
    controller.udp_socket = -1;
    controller.udp_task = NULL;
    controller.udp_cancelled = false;
    controller.active_id = -1;
    xSemaphoreGive(controller.mutex);
    ESP_LOGI(TAG, "iPerf session %d stopped", WLH_IPERF_CUSTOM_INSTANCE_ID);
    ESP_LOGI(TAG, "iPerf session %d closed", WLH_IPERF_CUSTOM_INSTANCE_ID);
    vTaskDelete(NULL);
}

static bool udp_send_backpressured(void) {
    return errno == EAGAIN || errno == EWOULDBLOCK || errno == ENOBUFS ||
           errno == ENOMEM || errno < 0;
}

static void udp_pace_callback(void *argument) {
    xTaskNotifyGive((TaskHandle_t)argument);
}

static void udp_client_task_main(void *argument) {
    const wlh_iperf_request_t *request = argument;
    struct sockaddr_in peer = {0};
    struct timeval timeout = {
        .tv_sec = 0,
        .tv_usec = WLH_IPERF_UDP_ACK_WAIT_MS * 1000u,
    };
    uint8_t packet[WLH_IPERF2_DEFAULT_UDP_PACKET_SIZE] = {0};
    uint8_t report[WLH_IPERF2_DEFAULT_UDP_PACKET_SIZE];
    wlh_iperf2_udp_stats_t stats;
    uint64_t started_us = (uint64_t)esp_timer_get_time();
    uint64_t deadline_us =
        started_us + (uint64_t)request->duration_sec * 1000000u;
    uint64_t interval_us =
        (uint64_t)sizeof(packet) * 8u / (uint64_t)request->target_mbps;
    uint64_t next_send_us = started_us;
    uint64_t last_report_ms = started_us / 1000u;
    int socket = -1;
    bool report_received = false;
    unsigned catchup_packets = 0u;
    esp_timer_handle_t pace_timer = NULL;
    const esp_timer_create_args_t pace_timer_args = {
        .callback = udp_pace_callback,
        .arg = xTaskGetCurrentTaskHandle(),
        .name = "wlh-iperf-pace",
    };

    wlh_iperf2_udp_stats_init(&stats);
    peer.sin_family = AF_INET;
    peer.sin_port = htons(WLH_IPERF_PORT);
    peer.sin_addr.s_addr = controller.udp_peer_ipv4;
    socket = lwip_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (socket < 0 ||
        lwip_setsockopt(socket, SOL_SOCKET, SO_SNDTIMEO, &timeout,
                        sizeof(timeout)) != 0 ||
        lwip_setsockopt(socket, SOL_SOCKET, SO_RCVTIMEO, &timeout,
                        sizeof(timeout)) != 0 ||
        lwip_connect(socket, (const struct sockaddr *)&peer, sizeof(peer)) !=
            0 ||
        esp_timer_create(&pace_timer_args, &pace_timer) != ESP_OK ||
        esp_timer_start_periodic(pace_timer, interval_us) != ESP_OK) {
        ESP_LOGE(TAG, "UDP client setup failed: errno=%d", errno);
        goto done;
    }
    xSemaphoreTake(controller.mutex, portMAX_DELAY);
    controller.udp_socket = socket;
    xSemaphoreGive(controller.mutex);
    ESP_LOGI(TAG, "iPerf session %d started", WLH_IPERF_CUSTOM_INSTANCE_ID);
    printf("[  1] local UDP client connected to port %u\n",
           (unsigned)WLH_IPERF_PORT);
    printf("[ ID] Interval\t\tTransfer\tBandwidth\n");

    while (!udp_cancelled() && (uint64_t)esp_timer_get_time() < deadline_us) {
        uint64_t now_us = (uint64_t)esp_timer_get_time();
        wlh_iperf2_udp_header_t header = {
            .sequence = (int32_t)stats.packets,
            .seconds = (uint32_t)(now_us / 1000000u),
            .microseconds = (uint32_t)(now_us % 1000000u),
        };
        int sent;
        wlh_iperf2_udp_encode(packet, &header);
        sent = lwip_send(socket, packet, sizeof(packet), 0);
        if (sent != (int)sizeof(packet)) {
            if (udp_send_backpressured()) {
                vTaskDelay(1u);
                continue;
            }
            ESP_LOGW(TAG, "UDP send failed: errno=%d", errno);
            break;
        }
        stats.packets++;
        stats.bytes += (uint32_t)sent;
        next_send_us += interval_us;
        now_us = (uint64_t)esp_timer_get_time();
        if (now_us / 1000u - last_report_ms >= WLH_IPERF_REPORT_INTERVAL_MS) {
            print_udp_summary((now_us - started_us) / 1000u, &stats);
            last_report_ms = now_us / 1000u;
        }
        /* Pace against an absolute timeline. Clearing a periodic timer's
           accumulated notifications after every packet permanently loses
           send opportunities whenever task scheduling exceeds one interval
           (224 us at 50 Mbps), turning scheduler latency into an artificial
           ~20 Mbps ceiling. If execution fell behind, send the bounded socket
           path again immediately until it catches up; only wait while ahead
           of the requested offered-load timeline. */
        while (now_us < next_send_us && !udp_cancelled()) {
            (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            now_us = (uint64_t)esp_timer_get_time();
        }
        if (now_us >= next_send_us) {
            /* When the requested offered load exceeds the real link rate,
               absolute pacing is permanently behind. Keep catch-up bounded:
               on a single-core host an unbounded ready loop prevents the
               idle task and lower-priority maintenance work from running. */
            if (++catchup_packets == WLH_IPERF_UDP_CATCHUP_BURST) {
                catchup_packets = 0u;
                vTaskDelay(1u);
            }
        } else {
            catchup_packets = 0u;
        }
    }

    if (!udp_cancelled()) {
        uint32_t attempt;
        uint64_t now_us = (uint64_t)esp_timer_get_time();
        wlh_iperf2_udp_header_t final = {
            .sequence = -(int32_t)(stats.packets + 1u),
            .seconds = (uint32_t)(now_us / 1000000u),
            .microseconds = (uint32_t)(now_us % 1000000u),
        };
        wlh_iperf2_udp_encode(packet, &final);
        {
            bool send_failure_logged = false;
            for (attempt = 0u;
                 attempt < WLH_IPERF_UDP_ACK_ATTEMPTS && !udp_cancelled();
                 ++attempt) {
                if (lwip_send(socket, packet, sizeof(packet), 0) ==
                    (int)sizeof(packet)) {
                    if (lwip_recv(socket, report, sizeof(report), 0) > 0) {
                        report_received = true;
                        break;
                    }
                    continue;
                }
                if (!send_failure_logged) {
                    ESP_LOGW(TAG, "UDP FIN send failed: errno=%d", errno);
                    send_failure_logged = true;
                }
                /* A transiently failing send must not spin through the whole
                   window in milliseconds (observed: one FIN at the session
                   end failed and the client gave up 3 ms later, even though
                   the server kept retransmitting its report). Pace the retry
                   with the report wait so the link can recover, and drain
                   the socket in case a report from an earlier FIN is already
                   on its way. */
                if (lwip_recv(socket, report, sizeof(report), 0) > 0) {
                    report_received = true;
                    break;
                }
            }
        }
        if (!report_received)
            ESP_LOGW(TAG, "UDP client did not receive the server report");
    }
    print_udp_summary(((uint64_t)esp_timer_get_time() - started_us) / 1000u,
                      &stats);
done:
    if (pace_timer != NULL) {
        (void)esp_timer_stop(pace_timer);
        (void)esp_timer_delete(pace_timer);
    }
    finish_udp_task(socket);
}

static void udp_server_task_main(void *argument) {
    const wlh_iperf_request_t *request = argument;
    struct sockaddr_in address = {0};
    struct sockaddr_in peer = {0};
    socklen_t peer_size = sizeof(peer);
    struct timeval timeout = {
        .tv_sec = 0,
        .tv_usec = WLH_IPERF_UDP_ACK_WAIT_MS * 1000u,
    };
    wlh_iperf2_udp_stats_t stats;
    uint8_t packet[WLH_IPERF2_DEFAULT_UDP_PACKET_SIZE];
    uint64_t started = monotonic_ms();
    uint64_t first_packet_ms = 0u;
    uint64_t last_report_ms = started;
    uint32_t client_duration_ms = 0u;
    bool fin_stats_finalized = false;
    int socket = -1;
    bool success = false;

    wlh_iperf2_udp_stats_init(&stats);
    address.sin_family = AF_INET;
    address.sin_port = htons(WLH_IPERF_PORT);
    address.sin_addr.s_addr = PP_HTONL(INADDR_ANY);
    socket = lwip_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (socket < 0 ||
        lwip_setsockopt(socket, SOL_SOCKET, SO_RCVTIMEO, &timeout,
                        sizeof(timeout)) != 0 ||
        lwip_bind(socket, (const struct sockaddr *)&address, sizeof(address)) !=
            0) {
        ESP_LOGE(TAG, "UDP server setup failed: errno=%d", errno);
        goto done;
    }
    xSemaphoreTake(controller.mutex, portMAX_DELAY);
    controller.udp_socket = socket;
    xSemaphoreGive(controller.mutex);
    ESP_LOGI(TAG, "iPerf session %d started", WLH_IPERF_CUSTOM_INSTANCE_ID);

    while (!udp_cancelled()) {
        int count;
        uint64_t now = monotonic_ms();
        uint64_t elapsed = first_packet_ms == 0u ? 0u : now - first_packet_ms;
        uint64_t deadline = (uint64_t)request->duration_sec * 1000u;
        if (first_packet_ms != 0u && client_duration_ms != 0u &&
            elapsed >= client_duration_ms) {
            /* The client keeps retransmitting its FIN while it waits for the
               report; drain those so a congested TX burst cannot cost the
               client its final report. */
            success = send_udp_server_report(socket, &peer, peer_size, elapsed,
                                             &stats, false);
            break;
        }
        if (first_packet_ms == 0u && now - started >= deadline) break;
        if (first_packet_ms != 0u && now - first_packet_ms >= deadline) {
            success = send_udp_server_report(socket, &peer, peer_size, elapsed,
                                             &stats, false);
            break;
        }
        count = lwip_recvfrom(socket, packet, sizeof(packet), 0,
                              (struct sockaddr *)&peer, &peer_size);
        if (count < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
            if (!udp_cancelled())
                ESP_LOGW(TAG, "UDP receive failed: errno=%d", errno);
            break;
        }
        {
            wlh_iperf2_udp_header_t header;
            if (!wlh_iperf2_udp_decode(packet, (uint32_t)count, &header))
                continue;
            if (first_packet_ms == 0u) {
                first_packet_ms = monotonic_ms();
                (void)wlh_iperf2_udp_decode_client_duration_ms(
                    packet, (uint32_t)count, &client_duration_ms);
            }
            if (header.sequence < 0) {
                /* stats_finish accumulates from next_sequence, so a
                   retransmitted FIN must only finalize once. */
                if (!fin_stats_finalized) {
                    wlh_iperf2_udp_stats_finish(&stats, header.sequence);
                    fin_stats_finalized = true;
                }
                if (send_udp_server_report(socket, &peer, peer_size,
                                           monotonic_ms() - first_packet_ms,
                                           &stats, false)) {
                    success = true;
                    break;
                }
                /* The client retransmits its FIN for ~10 s; a single lost
                   report must not end the session, try again on the next
                   copy. */
                continue;
            }
            wlh_iperf2_udp_stats_add(&stats, &header, (uint32_t)count,
                                     (uint64_t)esp_timer_get_time());
        }
        now = monotonic_ms();
        if (first_packet_ms != 0u &&
            now - last_report_ms >= WLH_IPERF_REPORT_INTERVAL_MS) {
            print_udp_summary(now - first_packet_ms, &stats);
            last_report_ms = now;
        }
    }
    if (first_packet_ms != 0u)
        print_udp_summary(monotonic_ms() - first_packet_ms, &stats);
    if (!success && !udp_cancelled())
        ESP_LOGW(TAG, "UDP iPerf session ended without a server report");
done:
    finish_udp_task(socket);
}

esp_err_t wlh_iperf_init(void) {
    if (controller.mutex != NULL) return ESP_OK;
    controller.mutex = xSemaphoreCreateMutex();
    return controller.mutex == NULL ? ESP_ERR_NO_MEM : ESP_OK;
}

static esp_err_t start_udp(const wlh_iperf_request_t *request) {
    ip4_addr_t peer;
    if (request->role == WLH_IPERF_CLIENT) {
        if (!ip4addr_aton(request->peer, &peer)) return ESP_ERR_INVALID_ARG;
    } else {
        peer.addr = 0u;
    }
    xSemaphoreTake(controller.mutex, portMAX_DELAY);
    if (controller.active_id >= 0) {
        xSemaphoreGive(controller.mutex);
        return ESP_ERR_INVALID_STATE;
    }
    controller.active_id = WLH_IPERF_CUSTOM_INSTANCE_ID;
    controller.udp_request = *request;
    controller.udp_request.peer = NULL;
    controller.udp_peer_ipv4 = peer.addr;
    controller.udp_cancelled = false;
    if (xTaskCreate(request->role == WLH_IPERF_SERVER ? udp_server_task_main
                                                      : udp_client_task_main,
                    "wlh-iperf-udp", 6144u, &controller.udp_request,
                    WLH_IPERF_UDP_TASK_PRIORITY,
                    &controller.udp_task) != pdPASS) {
        controller.active_id = -1;
        xSemaphoreGive(controller.mutex);
        return ESP_ERR_NO_MEM;
    }
    xSemaphoreGive(controller.mutex);
    return ESP_OK;
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
    if (request->protocol == WLH_IPERF_UDP) {
        esp_err_t result = start_udp(request);
        if (result == ESP_OK)
            printf("iperf started: udp %s, duration=%lu sec\n",
                   request->role == WLH_IPERF_SERVER ? "server" : "client",
                   (unsigned long)request->duration_sec);
        return result;
    }
    source.type = ESP_IPADDR_TYPE_V4;
    protocol_flag =
        request->protocol == WLH_IPERF_TCP ? IPERF_FLAG_TCP : IPERF_FLAG_UDP;
    if (request->role == WLH_IPERF_CLIENT) {
        if (!ip4addr_aton(request->peer, &peer)) return ESP_ERR_INVALID_ARG;
        destination.type = ESP_IPADDR_TYPE_V4;
        destination.u_addr.ip4.addr = peer.addr;
        config = (iperf_cfg_t)IPERF_DEFAULT_CONFIG_CLIENT(protocol_flag,
                                                          destination);
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
    int socket;
    if (controller.mutex == NULL) return;
    xSemaphoreTake(controller.mutex, portMAX_DELAY);
    id = controller.active_id;
    socket = controller.udp_socket;
    if (controller.udp_task != NULL) controller.udp_cancelled = true;
    xSemaphoreGive(controller.mutex);
    if (socket >= 0) (void)lwip_shutdown(socket, SHUT_RDWR);
    if (id >= 0 && socket < 0) {
        ESP_LOGW(TAG, "cancelling iPerf session %d: %s", id,
                 reason == NULL ? "requested" : reason);
        (void)iperf_stop_instance(id);
    }
}
