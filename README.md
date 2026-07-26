# WL-hosted ESP-IDF Host

ESP32-P4 Host Adapter for WL-hosted. An ESP32-C6 running
[`wl-hosted-coproc-esp-idf`](https://github.com/WL-hosted/wl-hosted-coproc-esp-idf)
provides Wi-Fi over a 4-bit SDIO link. The P4 uses ESP-IDF's native
`esp_netif` and lwIP components; no private or third-party lwIP fork is used.

## Hardware

The default board profile is fixed to the requested ESP32-P4 wiring:

| Signal | P4 GPIO |
|---|---:|
| SDIO CLK | 54 |
| SDIO CMD | 53 |
| SDIO D3 | 52 |
| SDIO D2 | 51 |
| SDIO D1 | 50 |
| SDIO D0 | 49 |
| C6 EN | 19 |

Use external pull-ups on CMD and DAT0-DAT3. The Host configures SDMMC Slot 1,
4-bit mode, 512-byte blocks, and a 40 MHz maximum clock. GPIO 19 is pulsed low
at startup to reset the C6.

## Build

ESP-IDF 5.5.2 is the validated version.

```sh
git submodule update --init --recursive
idf.py set-target esp32p4
idf.py build
```

Flashing is intentionally left to the board owner:

```sh
idf.py -p /dev/your-port flash monitor
```

## Console MVP

The UART console runs at 115200 baud. Type `help` to list commands:

```text
status
scan
sta_connect <ssid> [password]
sta_disconnect
ap_start <ssid> [password] [channel]
ap_stop
ping [hostname]
io_config <pin> <in|out|od> [none|up|down] [0|1]
io_read <pin>
io_write <pin> <0|1>
adc_read <pin>
kv_read <key>
kv_write <key> <value>
kv_erase <key>
```

The `io_*` and `adc_read` pin numbers are logical profile pins, not GPIO
numbers; the coprocessor README lists the mapping for each target. `io_write`
requires the pin to have been configured as `out` or `od` first, and `kv_write`
persists the value on the coprocessor across resets.

`ping` defaults to `baidu.com` and sends four ICMP echo requests. Station mode
uses an lwIP DHCP client. SoftAP creates a separate Ethernet netif at
`192.168.4.1/24` with the ESP-IDF DHCP server. NAPT is not enabled.

Typical MVP flow:

```text
wlh> status
wlh> scan
wlh> sta_connect MyWifi my-password
wlh> ping baidu.com
wlh> ap_start WLH-Test test12345 1
```

The SDIO transport and Core calls are asynchronous and bounded. The C6 Adapter
uses a single hardware TX window so the Host can safely derive one wire frame
from each cumulative SDIO packet-length update.
