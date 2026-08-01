#include "iperf2.h"

#include <arpa/inet.h>
#include <string.h>

bool wlh_iperf2_udp_decode(const uint8_t *data, uint32_t size,
                           wlh_iperf2_udp_header_t *header) {
    uint32_t value;
    if (data == NULL || header == NULL || size < WLH_IPERF2_UDP_HEADER_SIZE)
        return false;
    memcpy(&value, data, sizeof(value));
    header->sequence = (int32_t)ntohl(value);
    memcpy(&value, data + 4u, sizeof(value));
    header->seconds = ntohl(value);
    memcpy(&value, data + 8u, sizeof(value));
    header->microseconds = ntohl(value);
    return header->microseconds < 1000000u;
}

bool wlh_iperf2_udp_decode_client_duration_ms(const uint8_t *data,
                                              uint32_t size,
                                              uint32_t *duration_ms) {
    const uint32_t required_flags = 0x48010000u;
    uint32_t value;
    int32_t amount;
    uint64_t decoded_ms;
    if (data == NULL || duration_ms == NULL || size < 40u) return false;
    memcpy(&value, data + 16u, sizeof(value));
    if ((ntohl(value) & required_flags) != required_flags) return false;
    memcpy(&value, data + 36u, sizeof(value));
    amount = (int32_t)ntohl(value);
    if (amount >= 0) return false;
    decoded_ms = (uint64_t)(-(int64_t)amount) * 10u;
    if (decoded_ms == 0u || decoded_ms > UINT32_MAX) return false;
    *duration_ms = (uint32_t)decoded_ms;
    return true;
}

void wlh_iperf2_udp_encode(uint8_t data[WLH_IPERF2_UDP_HEADER_SIZE],
                           const wlh_iperf2_udp_header_t *header) {
    uint32_t value;
    value = htonl((uint32_t)header->sequence);
    memcpy(data, &value, sizeof(value));
    value = htonl(header->seconds);
    memcpy(data + 4u, &value, sizeof(value));
    value = htonl(header->microseconds);
    memcpy(data + 8u, &value, sizeof(value));
}

void wlh_iperf2_udp_stats_init(wlh_iperf2_udp_stats_t *stats) {
    memset(stats, 0, sizeof(*stats));
}

void wlh_iperf2_udp_stats_add(wlh_iperf2_udp_stats_t *stats,
                              const wlh_iperf2_udp_header_t *header,
                              uint32_t bytes, uint64_t arrival_us) {
    double sent_ms = (double)header->seconds * 1000.0 +
                     (double)header->microseconds / 1000.0;
    double transit = (double)arrival_us / 1000.0 - sent_ms;
    if (header->sequence < 0) return;
    stats->packets++;
    stats->bytes += bytes;
    if (header->sequence == stats->next_sequence) {
        stats->next_sequence++;
    } else if (header->sequence > stats->next_sequence) {
        stats->lost += (uint64_t)(header->sequence - stats->next_sequence);
        stats->next_sequence = header->sequence + 1;
    } else {
        stats->out_of_order++;
    }
    if (stats->have_transit) {
        double delta = transit - stats->last_transit_ms;
        if (delta < 0.0) delta = -delta;
        stats->jitter_ms += (delta - stats->jitter_ms) / 16.0;
    } else {
        stats->have_transit = true;
    }
    stats->last_transit_ms = transit;
}

void wlh_iperf2_udp_stats_finish(wlh_iperf2_udp_stats_t *stats,
                                 int32_t terminal_sequence) {
    int64_t sent_packets;
    if (stats == NULL || terminal_sequence >= 0) return;
    sent_packets = -(int64_t)terminal_sequence - 1;
    if (sent_packets > stats->next_sequence)
        stats->lost += (uint64_t)(sent_packets - stats->next_sequence);
}

bool wlh_iperf2_udp_encode_server_report(uint8_t *data, uint32_t size,
                                         uint64_t bytes, uint64_t elapsed_ms,
                                         const wlh_iperf2_udp_stats_t *stats) {
    uint32_t values[28] = {0};
    uint64_t elapsed_us;
    uint64_t expected_datagrams;
    uint64_t jitter_us;
    size_t index;
    if (data == NULL || stats == NULL ||
        size < WLH_IPERF2_UDP_SERVER_REPORT_SIZE)
        return false;
    elapsed_us = elapsed_ms * 1000u;
    expected_datagrams = stats->packets + stats->lost;
    jitter_us = (uint64_t)(stats->jitter_ms * 1000.0);
    memset(data, 0, WLH_IPERF2_UDP_SERVER_UDP_HEADER_SIZE);
    values[0] = htonl(0x88000000u);
    values[1] = htonl((uint32_t)(bytes >> 32u));
    values[2] = htonl((uint32_t)bytes);
    values[3] = htonl((uint32_t)(elapsed_us / 1000000u));
    values[4] = htonl((uint32_t)(elapsed_us % 1000000u));
    values[5] = htonl((uint32_t)stats->lost);
    values[6] = htonl((uint32_t)stats->out_of_order);
    values[7] = htonl((uint32_t)expected_datagrams);
    values[8] = htonl((uint32_t)(jitter_us / 1000000u));
    values[9] = htonl((uint32_t)(jitter_us % 1000000u));
    values[25] = htonl((uint32_t)(stats->lost >> 32u));
    values[26] = htonl((uint32_t)(stats->out_of_order >> 32u));
    values[27] = htonl((uint32_t)(expected_datagrams >> 32u));
    for (index = 0u; index < sizeof(values) / sizeof(values[0]); ++index) {
        memcpy(data + WLH_IPERF2_UDP_SERVER_UDP_HEADER_SIZE +
                   index * sizeof(uint32_t),
               &values[index], sizeof(uint32_t));
    }
    return true;
}
