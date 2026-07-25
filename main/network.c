#include "network.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_netif_defaults.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "lwip/inet.h"
#include "lwip/netdb.h"
#include "ping/ping_sock.h"

typedef struct netif_driver {
    wlh_host_t *host;
    bool ap;
} netif_driver_t;

typedef struct ping_context {
    SemaphoreHandle_t done;
    esp_ping_handle_t handle;
} ping_context_t;

static const char *TAG = "wlh-netif";
static esp_netif_t *sta_netif;
static esp_netif_t *ap_netif;
static netif_driver_t sta_driver;
static netif_driver_t ap_driver;

static esp_err_t transmit(void *handle, void *buffer, size_t size) {
    netif_driver_t *driver = handle;
    wlh_host_result_t result =
        driver->ap ? wlh_host_ethernet_ap_send(driver->host, buffer, size)
                   : wlh_host_ethernet_sta_send(driver->host, buffer, size);
    return result == WLH_HOST_OK ? ESP_OK : ESP_FAIL;
}

static void free_rx_buffer(void *handle, void *buffer) {
    (void)handle;
    free(buffer);
}

static esp_netif_t *create_netif(netif_driver_t *driver, const char *key,
                                 const char *description,
                                 esp_netif_flags_t flags, int route_priority,
                                 const esp_netif_ip_info_t *ip_info) {
    esp_netif_inherent_config_t inherent = {
        .flags = flags,
        .ip_info = ip_info,
        .if_key = key,
        .if_desc = description,
        .route_prio = route_priority,
    };
    esp_netif_driver_ifconfig_t driver_config = {
        .handle = driver,
        .transmit = transmit,
        .driver_free_rx_buffer = free_rx_buffer,
    };
    esp_netif_config_t config = {
        .base = &inherent,
        .driver = &driver_config,
        .stack = ESP_NETIF_NETSTACK_DEFAULT_ETH,
    };
    return esp_netif_new(&config);
}

esp_err_t wlh_network_init(wlh_host_t *host) {
    esp_netif_ip_info_t ap_ip;
    if (host == NULL) return ESP_ERR_INVALID_ARG;
    sta_driver = (netif_driver_t){host, false};
    ap_driver = (netif_driver_t){host, true};

    memset(&ap_ip, 0, sizeof(ap_ip));
    IP4_ADDR(&ap_ip.ip, 192, 168, 4, 1);
    IP4_ADDR(&ap_ip.gw, 192, 168, 4, 1);
    IP4_ADDR(&ap_ip.netmask, 255, 255, 255, 0);

    sta_netif = create_netif(&sta_driver, "WLH_STA", "WL-hosted station",
                             ESP_NETIF_DHCP_CLIENT, 100, NULL);
    ap_netif =
        create_netif(&ap_driver, "WLH_AP", "WL-hosted SoftAP",
                     ESP_NETIF_DHCP_SERVER | ESP_NETIF_FLAG_AUTOUP, 10, &ap_ip);
    if (sta_netif == NULL || ap_netif == NULL) return ESP_ERR_NO_MEM;
    esp_netif_action_start(sta_netif, NULL, 0, NULL);
    esp_netif_action_start(ap_netif, NULL, 0, NULL);
    esp_netif_action_disconnected(sta_netif, NULL, 0, NULL);
    esp_netif_action_disconnected(ap_netif, NULL, 0, NULL);
    ESP_LOGI(TAG, "native esp_netif/lwIP STA and AP interfaces created");
    return ESP_OK;
}

static void link_up(esp_netif_t *netif, const uint8_t mac[6]) {
    uint8_t mutable_mac[6];
    memcpy(mutable_mac, mac, sizeof(mutable_mac));
    ESP_ERROR_CHECK(esp_netif_set_mac(netif, mutable_mac));
    esp_netif_action_connected(netif, NULL, 0, NULL);
}

void wlh_network_sta_up(const uint8_t mac[6]) {
    link_up(sta_netif, mac);
}

void wlh_network_sta_down(void) {
    esp_netif_action_disconnected(sta_netif, NULL, 0, NULL);
}

void wlh_network_ap_up(const uint8_t mac[6]) {
    link_up(ap_netif, mac);
}

void wlh_network_ap_down(void) {
    esp_netif_action_disconnected(ap_netif, NULL, 0, NULL);
}

void wlh_network_input(bool ap, const uint8_t *frame, size_t size) {
    void *copy;
    esp_netif_t *netif = ap ? ap_netif : sta_netif;
    if (frame == NULL || size < 14u || size > 1518u) return;
    copy = malloc(size);
    if (copy == NULL) return;
    memcpy(copy, frame, size);
    if (esp_netif_receive(netif, copy, size, copy) != ESP_OK) free(copy);
}

static void print_netif(const char *name, esp_netif_t *netif) {
    esp_netif_ip_info_t info;
    uint8_t mac[6] = {0};
    esp_netif_get_ip_info(netif, &info);
    esp_netif_get_mac(netif, mac);
    printf("%s: %s mac=%02x:%02x:%02x:%02x:%02x:%02x ip=" IPSTR
           " gateway=" IPSTR "\n",
           name, esp_netif_is_netif_up(netif) ? "up" : "down", mac[0], mac[1],
           mac[2], mac[3], mac[4], mac[5], IP2STR(&info.ip), IP2STR(&info.gw));
}

void wlh_network_print_status(void) {
    print_netif("sta", sta_netif);
    print_netif("ap ", ap_netif);
}

static void ping_success(esp_ping_handle_t handle, void *context) {
    uint32_t sequence;
    uint32_t time_ms;
    uint32_t ttl;
    ip_addr_t address;
    esp_ping_get_profile(handle, ESP_PING_PROF_SEQNO, &sequence,
                         sizeof(sequence));
    esp_ping_get_profile(handle, ESP_PING_PROF_TIMEGAP, &time_ms,
                         sizeof(time_ms));
    esp_ping_get_profile(handle, ESP_PING_PROF_TTL, &ttl, sizeof(ttl));
    esp_ping_get_profile(handle, ESP_PING_PROF_IPADDR, &address,
                         sizeof(address));
    printf("%u bytes from %s: icmp_seq=%lu ttl=%lu time=%lu ms\n", 64u,
           ipaddr_ntoa(&address), (unsigned long)sequence, (unsigned long)ttl,
           (unsigned long)time_ms);
    (void)context;
}

static void ping_timeout(esp_ping_handle_t handle, void *context) {
    uint32_t sequence;
    esp_ping_get_profile(handle, ESP_PING_PROF_SEQNO, &sequence,
                         sizeof(sequence));
    printf("timeout icmp_seq=%lu\n", (unsigned long)sequence);
    (void)context;
}

static void ping_end(esp_ping_handle_t handle, void *context) {
    ping_context_t *ping = context;
    uint32_t transmitted;
    uint32_t received;
    esp_ping_get_profile(handle, ESP_PING_PROF_REQUEST, &transmitted,
                         sizeof(transmitted));
    esp_ping_get_profile(handle, ESP_PING_PROF_REPLY, &received,
                         sizeof(received));
    printf("%lu packets transmitted, %lu received\n",
           (unsigned long)transmitted, (unsigned long)received);
    xSemaphoreGive(ping->done);
}

esp_err_t wlh_network_ping(const char *hostname, uint32_t count) {
    struct addrinfo hints = {.ai_family = AF_INET, .ai_socktype = SOCK_RAW};
    struct addrinfo *address_info = NULL;
    esp_ping_config_t config = ESP_PING_DEFAULT_CONFIG();
    esp_ping_callbacks_t callbacks = {
        .on_ping_success = ping_success,
        .on_ping_timeout = ping_timeout,
        .on_ping_end = ping_end,
    };
    ping_context_t ping = {0};
    esp_err_t result;

    if (hostname == NULL || count == 0u) return ESP_ERR_INVALID_ARG;
    if (getaddrinfo(hostname, NULL, &hints, &address_info) != 0 ||
        address_info == NULL)
        return ESP_ERR_NOT_FOUND;
    inet_addr_to_ip4addr(
        ip_2_ip4(&config.target_addr),
        &((struct sockaddr_in *)address_info->ai_addr)->sin_addr);
    freeaddrinfo(address_info);
    config.count = count;
    config.timeout_ms = 1000u;
    config.interval_ms = 1000u;
    ping.done = xSemaphoreCreateBinary();
    if (ping.done == NULL) return ESP_ERR_NO_MEM;
    callbacks.cb_args = &ping;
    result = esp_ping_new_session(&config, &callbacks, &ping.handle);
    if (result == ESP_OK) result = esp_ping_start(ping.handle);
    if (result == ESP_OK &&
        xSemaphoreTake(ping.done, pdMS_TO_TICKS((count + 2u) * 1000u)) !=
            pdTRUE)
        result = ESP_ERR_TIMEOUT;
    if (ping.handle != NULL) esp_ping_delete_session(ping.handle);
    vSemaphoreDelete(ping.done);
    return result;
}
