#ifndef WLH_HOST_ESP_IDF_NETWORK_H
#define WLH_HOST_ESP_IDF_NETWORK_H

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_netif.h"

#include "wlh/host.h"

esp_err_t wlh_network_init(wlh_host_t *host);
void wlh_network_sta_up(const uint8_t mac[6]);
void wlh_network_sta_down(void);
void wlh_network_ap_up(const uint8_t mac[6]);
void wlh_network_ap_down(void);
void wlh_network_input(bool ap, const uint8_t *frame, size_t size);
void wlh_network_print_status(void);
esp_err_t wlh_network_ping(const char *hostname, uint32_t count);
bool wlh_network_get_sta_ipv4(esp_ip4_addr_t *address);

#endif
