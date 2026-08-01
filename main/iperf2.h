#ifndef WLH_HOST_ESP_IDF_IPERF2_H
#define WLH_HOST_ESP_IDF_IPERF2_H

#include <stdbool.h>
#include <stdint.h>

#define WLH_IPERF2_UDP_HEADER_SIZE 12u
#define WLH_IPERF2_UDP_SERVER_UDP_HEADER_SIZE 16u
#define WLH_IPERF2_UDP_SERVER_HEADER_SIZE 112u
#define WLH_IPERF2_UDP_SERVER_REPORT_SIZE                                      \
    (WLH_IPERF2_UDP_SERVER_UDP_HEADER_SIZE + WLH_IPERF2_UDP_SERVER_HEADER_SIZE)
#define WLH_IPERF2_DEFAULT_UDP_PACKET_SIZE 1470u

typedef struct wlh_iperf2_udp_header {
    int32_t sequence;
    uint32_t seconds;
    uint32_t microseconds;
} wlh_iperf2_udp_header_t;

typedef struct wlh_iperf2_udp_stats {
    uint64_t packets;
    uint64_t lost;
    uint64_t out_of_order;
    uint64_t bytes;
    int32_t next_sequence;
    double jitter_ms;
    double last_transit_ms;
    bool have_transit;
} wlh_iperf2_udp_stats_t;

bool wlh_iperf2_udp_decode(const uint8_t *data, uint32_t size,
                           wlh_iperf2_udp_header_t *header);
bool wlh_iperf2_udp_decode_client_duration_ms(const uint8_t *data,
                                              uint32_t size,
                                              uint32_t *duration_ms);
void wlh_iperf2_udp_encode(uint8_t data[WLH_IPERF2_UDP_HEADER_SIZE],
                           const wlh_iperf2_udp_header_t *header);
void wlh_iperf2_udp_stats_init(wlh_iperf2_udp_stats_t *stats);
void wlh_iperf2_udp_stats_add(wlh_iperf2_udp_stats_t *stats,
                              const wlh_iperf2_udp_header_t *header,
                              uint32_t bytes, uint64_t arrival_us);
void wlh_iperf2_udp_stats_finish(wlh_iperf2_udp_stats_t *stats,
                                 int32_t terminal_sequence);
bool wlh_iperf2_udp_encode_server_report(uint8_t *data, uint32_t size,
                                         uint64_t bytes, uint64_t elapsed_ms,
                                         const wlh_iperf2_udp_stats_t *stats);

#endif
