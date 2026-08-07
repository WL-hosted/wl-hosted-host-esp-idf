#include "sdio_transport.h"

#include <stdatomic.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/sdmmc_host.h"
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdmmc_cmd.h"
#include "wlh/protocol/endian.h"
#include "wlh/protocol/wire.h"

#define WLH_SDIO_CLK_GPIO 54
#define WLH_SDIO_CMD_GPIO 53
#define WLH_SDIO_D3_GPIO 52
#define WLH_SDIO_D2_GPIO 51
#define WLH_SDIO_D1_GPIO 50
#define WLH_SDIO_D0_GPIO 49
#define WLH_C6_ENABLE_GPIO 19

#define WLH_SDIO_FUNCTION 1u
#define WLH_SDIO_BLOCK_SIZE 512u
#define WLH_SDIO_END_ADDRESS 0x1f800u
#define WLH_SDIO_ADDRESS_MASK 0x3ffu
#define WLH_SDIO_LENGTH_MASK 0xfffffu
#define WLH_SDIO_LENGTH_MODULUS 0x100000u
#define WLH_SDIO_TOKEN_MASK 0xfffu
#define WLH_SDIO_TOKEN_MODULUS 0x1000u
#define WLH_SDIO_BUFFER_SIZE 4092u
#define WLH_SDIO_DMA_FRAME_SIZE 4096u
#define WLH_SDIO_DMA_ALIGNMENT 64u
#define WLH_SDIO_NEW_PACKET_BIT (1u << 23)
#define WLH_SDIO_FUNC1_BIT (1u << 1)

#define WLH_SDIO_REG_TOKEN 0x44u
#define WLH_SDIO_REG_INT_RAW 0x50u
#define WLH_SDIO_REG_PACKET_LEN 0x60u
#define WLH_SDIO_REG_INT_CLEAR 0xd4u
#define WLH_SDIO_REG_HOST_INT_ENABLE 0xdcu
#define WLH_SDIO_REG_HOST_TO_SLAVE 0x8cu

#define WLH_CCCR_FN_ENABLE 0x02u
#define WLH_CCCR_FN_READY 0x03u
#define WLH_CCCR_INT_ENABLE 0x04u
#define WLH_CCCR_BLOCK_SIZE_LOW 0x10u
#define WLH_CCCR_BLOCK_SIZE_HIGH 0x11u
#define WLH_SDIO_FBR_FUNCTION1 0x100u

#define WLH_SDIO_TX_QUEUE_DEPTH 16u
#define WLH_SDIO_IO_TASK_STACK_SIZE 6144u
#define WLH_SDIO_INIT_RETRIES 3u
#define WLH_SDIO_TX_BURST_LIMIT 8u
/* Consecutive invalid length reads before the RX task declares the link
 * corrupt and triggers the transport recovery path. Transient stale/torn
 * samples resolve in one or two reads; 10 keeps the delay under ~20 ms. */
#define WLH_SDIO_RX_INVALID_LIMIT 10u
#define WLH_SDIO_RX_BURST_LIMIT 8u
#define WLH_SDIO_IO_IDLE_BIT (1u << 0)

_Static_assert(WLH_SDIO_DMA_FRAME_SIZE % WLH_SDIO_DMA_ALIGNMENT == 0u,
               "SDIO TX DMA buffer must cover whole cache lines");
_Static_assert((WLH_SDIO_MAX_FRAME_SIZE + 4u) % WLH_SDIO_DMA_ALIGNMENT == 0u,
               "SDIO RX DMA buffer must cover whole cache lines");

typedef struct tx_job {
    uint8_t *frame;
    size_t size;
    wlh_transport_tx_complete_fn completion;
    void *completion_context;
} tx_job_t;

typedef struct lifecycle_job {
    bool start;
    wlh_transport_lifecycle_complete_fn completion;
    void *completion_context;
} lifecycle_job_t;

typedef struct sdio_transport {
    wlh_host_t *host;
    sdmmc_card_t card;
    sdmmc_host_t sdmmc;
    QueueHandle_t tx_queue;
    QueueHandle_t lifecycle_queue;
    SemaphoreHandle_t bus_lock;
    SemaphoreHandle_t io_state_lock;
    EventGroupHandle_t io_events;
    atomic_bool running;
    uint32_t active_io;
    bool hardware_initialized;
    uint32_t rx_byte_count;
    uint32_t tx_buffer_count;
    /* Consecutive invalid length reads while NEW_PACKET stays asserted. A
     * stale or torn sample resolves on the next read; a stable garbage
     * length (stuck slave counter or SDIO bus fault) never does, and after
     * this many retries the RX task hands the link to the recovery path. */
    unsigned rx_invalid_streak;
} sdio_transport_t;

static const char *TAG = "wlh-sdio-host";
static sdio_transport_t transport;

static bool begin_io(void) {
    bool admitted = false;
    xSemaphoreTake(transport.io_state_lock, portMAX_DELAY);
    if (atomic_load(&transport.running)) {
        if (transport.active_io++ == 0u)
            xEventGroupClearBits(transport.io_events, WLH_SDIO_IO_IDLE_BIT);
        admitted = true;
    }
    xSemaphoreGive(transport.io_state_lock);
    return admitted;
}

static void end_io(void) {
    xSemaphoreTake(transport.io_state_lock, portMAX_DELAY);
    configASSERT(transport.active_io > 0u);
    if (--transport.active_io == 0u)
        xEventGroupSetBits(transport.io_events, WLH_SDIO_IO_IDLE_BIT);
    xSemaphoreGive(transport.io_state_lock);
}

static esp_err_t read_register(uint32_t address, void *data, size_t size) {
    return sdmmc_io_read_bytes(&transport.card, WLH_SDIO_FUNCTION,
                               address & WLH_SDIO_ADDRESS_MASK, data, size);
}

static esp_err_t write_register(uint32_t address, const void *data,
                                size_t size) {
    return sdmmc_io_write_bytes(&transport.card, WLH_SDIO_FUNCTION,
                                address & WLH_SDIO_ADDRESS_MASK, data, size);
}

static esp_err_t set_block_size(uint8_t function, uint16_t size) {
    uint32_t base = function == 0u ? 0u : WLH_SDIO_FBR_FUNCTION1;
    esp_err_t result =
        sdmmc_io_write_byte(&transport.card, 0, base + WLH_CCCR_BLOCK_SIZE_LOW,
                            (uint8_t)size, NULL);
    if (result != ESP_OK) return result;
    return sdmmc_io_write_byte(&transport.card, 0,
                               base + WLH_CCCR_BLOCK_SIZE_HIGH,
                               (uint8_t)(size >> 8), NULL);
}

static esp_err_t enable_function(void) {
    uint8_t value = 0u;
    unsigned retry;
    esp_err_t result =
        sdmmc_io_read_byte(&transport.card, 0, WLH_CCCR_FN_ENABLE, &value);
    if (result != ESP_OK) return result;
    value |= WLH_SDIO_FUNC1_BIT;
    result = sdmmc_io_write_byte(&transport.card, 0, WLH_CCCR_FN_ENABLE, value,
                                 NULL);
    if (result != ESP_OK) return result;
    for (retry = 0u; retry < 100u; ++retry) {
        result =
            sdmmc_io_read_byte(&transport.card, 0, WLH_CCCR_FN_READY, &value);
        if (result == ESP_OK && (value & WLH_SDIO_FUNC1_BIT) != 0u) break;
        vTaskDelay(pdMS_TO_TICKS(10u));
    }
    if (retry == 100u) return ESP_ERR_TIMEOUT;

    result =
        sdmmc_io_read_byte(&transport.card, 0, WLH_CCCR_INT_ENABLE, &value);
    if (result != ESP_OK) return result;
    value |= 1u | WLH_SDIO_FUNC1_BIT;
    result = sdmmc_io_write_byte(&transport.card, 0, WLH_CCCR_INT_ENABLE, value,
                                 NULL);
    if (result != ESP_OK) return result;
    result = set_block_size(0u, WLH_SDIO_BLOCK_SIZE);
    if (result != ESP_OK) return result;
    return set_block_size(1u, WLH_SDIO_BLOCK_SIZE);
}

static void c6_enable_pulse(void) {
    gpio_config_t config = {
        .pin_bit_mask = 1ULL << WLH_C6_ENABLE_GPIO,
        .mode = GPIO_MODE_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&config));
    gpio_set_level(WLH_C6_ENABLE_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(100u));
    gpio_set_level(WLH_C6_ENABLE_GPIO, 1);
    /* Wait for the C6 to boot and start its SDIO slave (~600 ms worst case)
       so the first sdmmc_card_init attempt succeeds; retries are only a
       fallback for genuine line trouble. */
    vTaskDelay(pdMS_TO_TICKS(1500u));
}

static esp_err_t hardware_start(void) {
    sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
    unsigned retry;
    esp_err_t result;

    c6_enable_pulse();
    transport.sdmmc = (sdmmc_host_t)SDMMC_HOST_DEFAULT();
    transport.sdmmc.slot = SDMMC_HOST_SLOT_1;
    transport.sdmmc.max_freq_khz = SDMMC_FREQ_HIGHSPEED;
    transport.sdmmc.flags |= SDMMC_HOST_FLAG_ALLOC_ALIGNED_BUF;

    slot.width = 4;
    slot.clk = WLH_SDIO_CLK_GPIO;
    slot.cmd = WLH_SDIO_CMD_GPIO;
    slot.d0 = WLH_SDIO_D0_GPIO;
    slot.d1 = WLH_SDIO_D1_GPIO;
    slot.d2 = WLH_SDIO_D2_GPIO;
    slot.d3 = WLH_SDIO_D3_GPIO;

    result = sdmmc_host_init();
    if (result != ESP_OK) return result;
    result = sdmmc_host_init_slot(transport.sdmmc.slot, &slot);
    if (result != ESP_OK) goto fail;

    for (retry = 0u; retry < WLH_SDIO_INIT_RETRIES; ++retry) {
        memset(&transport.card, 0, sizeof(transport.card));
        result = sdmmc_card_init(&transport.sdmmc, &transport.card);
        if (result == ESP_OK && enable_function() == ESP_OK) break;
        ESP_LOGW(TAG, "waiting for ESP32-C6 SDIO function (%u)", retry + 1u);
        vTaskDelay(pdMS_TO_TICKS(100u));
    }
    if (retry == WLH_SDIO_INIT_RETRIES) {
        result = ESP_ERR_TIMEOUT;
        goto fail_slot;
    }

    transport.rx_byte_count = 0u;
    transport.tx_buffer_count = 0u;
    {
        const uint32_t interrupt_enable = WLH_SDIO_NEW_PACKET_BIT;
        result = write_register(WLH_SDIO_REG_HOST_INT_ENABLE, &interrupt_enable,
                                sizeof(interrupt_enable));
    }
    /* Do not write a soft link reset (WLH_SDIO_REG_HOST_TO_SLAVE) here:
       hardware_start always cold-boots the C6 via the EN pulse, so the slave
       already starts with a fresh link core in WAITING_FOR_HELLO. The slave
       handles a soft reset asynchronously (SDIO stop/reset/start, then a core
       restart ~150 ms later); triggering it after card init races with Hello
       negotiation and tears down the freshly negotiated session, leaving the
       host stuck in READY until heartbeat timeout — an endless recover loop. */
    if (result != ESP_OK) goto fail_slot;
    transport.hardware_initialized = true;
    ESP_LOGI(TAG, "SDIO ready: %u kHz 4-bit CLK=%d CMD=%d D0..D3=%d,%d,%d,%d",
             (unsigned)transport.sdmmc.max_freq_khz, WLH_SDIO_CLK_GPIO,
             WLH_SDIO_CMD_GPIO, WLH_SDIO_D0_GPIO, WLH_SDIO_D1_GPIO,
             WLH_SDIO_D2_GPIO, WLH_SDIO_D3_GPIO);
    return ESP_OK;

fail_slot:
    sdmmc_host_deinit_slot(transport.sdmmc.slot);
    return result;
fail:
    sdmmc_host_deinit();
    return result;
}

static void hardware_stop(void) {
    xSemaphoreTake(transport.io_state_lock, portMAX_DELAY);
    atomic_store(&transport.running, false);
    xSemaphoreGive(transport.io_state_lock);
    if (!transport.hardware_initialized) return;
    /* sdmmc_host_deinit_slot is not safe while another task is blocked in
       sdmmc_io_wait_int or executing CMD52/CMD53. Prevent new operations and
       wait for every admitted operation to leave the driver first. */
    xEventGroupWaitBits(transport.io_events, WLH_SDIO_IO_IDLE_BIT, pdFALSE,
                        pdTRUE, portMAX_DELAY);
    sdmmc_host_deinit_slot(transport.sdmmc.slot);
    transport.hardware_initialized = false;
}

static esp_err_t wait_for_tx_buffer(void) {
    unsigned retry;
    for (retry = 0u; retry < 200u; ++retry) {
        uint32_t token = 0u;
        uint32_t available;
        esp_err_t result =
            read_register(WLH_SDIO_REG_TOKEN, &token, sizeof(token));
        if (result != ESP_OK) return result;
        token = (token >> 16) & WLH_SDIO_TOKEN_MASK;
        available =
            (token + WLH_SDIO_TOKEN_MODULUS - transport.tx_buffer_count) %
            WLH_SDIO_TOKEN_MODULUS;
        if (available > 0u) {
            transport.tx_buffer_count =
                (transport.tx_buffer_count + 1u) % WLH_SDIO_TOKEN_MODULUS;
            return ESP_OK;
        }
        /* Keep the poll granularity under the frame cadence of a ~20 Mbps
         * stream: at 2 ms each round-trip the stall latency alone caps the
         * TCP sender (the C6 pool stays full, so every frame waits here). */
        vTaskDelay(1u);
    }
    return ESP_ERR_TIMEOUT;
}

static esp_err_t read_data(uint32_t address, uint8_t *data, size_t size) {
    size_t remaining = size;
    uint8_t *position = data;

    while (remaining >= WLH_SDIO_BLOCK_SIZE) {
        size_t block_size =
            (remaining / WLH_SDIO_BLOCK_SIZE) * WLH_SDIO_BLOCK_SIZE;
        esp_err_t result = sdmmc_io_read_blocks(
            &transport.card, WLH_SDIO_FUNCTION, address, position, block_size);
        if (result != ESP_OK) return result;
        remaining -= block_size;
        position += block_size;
        address += block_size;
    }
    if (remaining > 0u)
        return sdmmc_io_read_bytes(&transport.card, WLH_SDIO_FUNCTION, address,
                                   position, remaining);
    return ESP_OK;
}

static esp_err_t write_data(uint32_t address, const uint8_t *data,
                            size_t size) {
    size_t remaining = size;
    const uint8_t *position = data;

    while (remaining >= WLH_SDIO_BLOCK_SIZE) {
        size_t block_size =
            (remaining / WLH_SDIO_BLOCK_SIZE) * WLH_SDIO_BLOCK_SIZE;
        esp_err_t result = sdmmc_io_write_blocks(
            &transport.card, WLH_SDIO_FUNCTION, address, position, block_size);
        if (result != ESP_OK) return result;
        remaining -= block_size;
        position += block_size;
        address += block_size;
    }
    if (remaining > 0u)
        return sdmmc_io_write_bytes(&transport.card, WLH_SDIO_FUNCTION, address,
                                    position, remaining);
    return ESP_OK;
}

static esp_err_t write_frame(uint8_t *dma_frame, const uint8_t *frame,
                             size_t size) {
    uint32_t transfer_size = (uint32_t)((size + WLH_SDIO_BLOCK_SIZE - 1u) &
                                        ~(WLH_SDIO_BLOCK_SIZE - 1u));
    esp_err_t result;
    configASSERT(transfer_size <= WLH_SDIO_DMA_FRAME_SIZE);
    memset(dma_frame, 0, transfer_size);
    memcpy(dma_frame, frame, size);
    if (!begin_io()) return ESP_ERR_INVALID_STATE;
    /* Wait for a slave RX buffer WITHOUT holding the bus lock. The token poll
     * sleeps up to 2 ms per round; while it held the lock, the RX task could
     * not drain frames and the slave's buffers stayed full, so the wait could
     * wedge the whole datapath for up to its 400 ms timeout under load. The
     * sdmmc driver serializes transactions internally. */
    result = wait_for_tx_buffer();
    if (result == ESP_OK) {
        xSemaphoreTake(transport.bus_lock, portMAX_DELAY);
        result =
            write_data(WLH_SDIO_END_ADDRESS - size, dma_frame, transfer_size);
        xSemaphoreGive(transport.bus_lock);
    }
    end_io();
    return result;
}

static void tx_task_main(void *argument) {
    /* ESP32-P4 SDMMC DMA operates on cache-backed internal L2 RAM. Both the
       address and transfer extent must cover complete 64-byte cache lines so
       the driver's C2M/M2C cache synchronization cannot touch neighboring
       allocations. */
    uint8_t *dma_frame = heap_caps_aligned_alloc(
        WLH_SDIO_DMA_ALIGNMENT, WLH_SDIO_DMA_FRAME_SIZE,
        MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    tx_job_t job;
    unsigned burst_frames = 0u;
    (void)argument;
    configASSERT(dma_frame != NULL);
    for (;;) {
        if (xQueueReceive(transport.tx_queue, &job, portMAX_DELAY) != pdTRUE)
            continue;
        if (!atomic_load(&transport.running)) {
            job.completion(job.completion_context, job.frame, job.size,
                           ESP_FAIL);
            continue;
        }
        esp_err_t result = write_frame(dma_frame, job.frame, job.size);
        if (result != ESP_OK)
            ESP_LOGW(TAG, "CMD53 TX failed: %s", esp_err_to_name(result));
        job.completion(job.completion_context, job.frame, job.size,
                       (int)result);
        /* taskYIELD() only hands the CPU to an equal-priority task. Under a
           sustained stream that leaves both SDIO workers permanently ready
           and starves the single-core idle task. Bound the work quantum and
           really block at its boundary; an empty queue resets the quantum. */
        if (uxQueueMessagesWaiting(transport.tx_queue) == 0u) {
            burst_frames = 0u;
        } else if (++burst_frames == WLH_SDIO_TX_BURST_LIMIT) {
            burst_frames = 0u;
            vTaskDelay(1u);
        } else {
            taskYIELD();
        }
    }
}

static esp_err_t read_pending_frame(uint8_t *frame, size_t *frame_size) {
    uint32_t packet_length = 0u;
    uint32_t raw_packet_length;
    uint32_t interrupt_status = 0u;
    uint32_t size;
    uint32_t transfer_size;
    esp_err_t result;

    result = read_register(WLH_SDIO_REG_INT_RAW, &interrupt_status,
                           sizeof(interrupt_status));
    if (result != ESP_OK) return result;
    if ((interrupt_status & WLH_SDIO_NEW_PACKET_BIT) == 0u)
        return ESP_ERR_NOT_FOUND;
    result = read_register(WLH_SDIO_REG_PACKET_LEN, &packet_length,
                           sizeof(packet_length));
    if (result != ESP_OK) return result;
    raw_packet_length = packet_length;
    packet_length &= WLH_SDIO_LENGTH_MASK;
    size = (packet_length + WLH_SDIO_LENGTH_MODULUS - transport.rx_byte_count) %
           WLH_SDIO_LENGTH_MODULUS;
    /* The slave asserts NEW_PACKET when it starts a send and only re-asserts
     * after the host has read the data, so the interrupt must never be
     * cleared unless this packet is consumed. A length read that has not
     * moved (delta 0) or is otherwise invalid is a stale/torn sample from
     * the slave's cumulative counter register; leaving the interrupt
     * asserted and retrying is the only safe action: clearing it here would
     * strand the packet at the slave TX queue head until the heartbeat
     * recovery power-cycles the coprocessor, and resynchronizing to garbage
     * would corrupt every later delta. The 0xffffffff sample is the SDIO
     * bus-fault signature esp-hosted also detects. The caller bounds the
     * retry: a persistent garbage length is a link fault, not a race. */
    if (raw_packet_length == UINT32_MAX) {
        ESP_LOGW(TAG, "SDIO PACKET_LEN read fault (0xffffffff)");
        return ESP_ERR_INVALID_SIZE;
    }
    if (packet_length == WLH_SDIO_LENGTH_MASK || size < WLH_FRAME_HEADER_SIZE ||
        size > WLH_SDIO_MAX_FRAME_SIZE) {
        ESP_LOGW(TAG,
                 "invalid SDIO RX length: raw=0x%08lx previous=%lu "
                 "delta=%lu max=%u",
                 (unsigned long)raw_packet_length,
                 (unsigned long)transport.rx_byte_count, (unsigned long)size,
                 (unsigned)WLH_SDIO_MAX_FRAME_SIZE);
        return ESP_ERR_INVALID_SIZE;
    }
    /* Length validated: the interrupt can be cleared now. The slave's next
     * NEW_PACKET assert follows this packet's CMD53 data read, so it cannot
     * be lost to the stale status written here. */
    esp_err_t clear_result = write_register(
        WLH_SDIO_REG_INT_CLEAR, &interrupt_status, sizeof(interrupt_status));
    if (clear_result != ESP_OK) return clear_result;
    transfer_size = (uint32_t)((size + WLH_SDIO_BLOCK_SIZE - 1u) &
                               ~(WLH_SDIO_BLOCK_SIZE - 1u));
    result = read_data(WLH_SDIO_END_ADDRESS - size, frame, transfer_size);
    if (result != ESP_OK) {
        /* The frame is lost; resynchronize so the next interrupt starts
           from a clean byte-count baseline. */
        transport.rx_byte_count = packet_length;
        return result;
    }
    transport.rx_byte_count = packet_length;
    *frame_size = size;
    return ESP_OK;
}

static void rx_task_main(void *argument) {
    /* Keep RX under the same cache-line ownership contract as TX. */
    uint8_t *frame = heap_caps_aligned_alloc(
        WLH_SDIO_DMA_ALIGNMENT, WLH_SDIO_MAX_FRAME_SIZE + 4u,
        MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    bool pending = false;
    (void)argument;
    configASSERT(frame != NULL);
    for (;;) {
        esp_err_t result;
        unsigned burst_frames = 0u;
        size_t size = 0u;
        if (!atomic_load(&transport.running)) {
            pending = false;
            vTaskDelay(pdMS_TO_TICKS(20u));
            continue;
        }
        if (!pending) {
            if (!begin_io()) continue;
            result = sdmmc_io_wait_int(&transport.card, pdMS_TO_TICKS(1000u));
            end_io();
            if (result == ESP_ERR_TIMEOUT) continue;
            if (result != ESP_OK) {
                ESP_LOGW(TAG, "SDIO interrupt wait failed: %s",
                         esp_err_to_name(result));
                continue;
            }
        }
        pending = false;
        for (;;) {
            /* Drain every queued frame; the slave may have posted more
               than one transaction per interrupt. The packet-length register
               is cumulative, so one read may contain several complete WLH
               wire frames. */
            if (!begin_io()) break;
            xSemaphoreTake(transport.bus_lock, portMAX_DELAY);
            result = read_pending_frame(frame, &size);
            xSemaphoreGive(transport.bus_lock);
            end_io();
            if (result == ESP_ERR_NOT_FOUND) {
                /* The asserted interrupt cleared on its own (slave advanced
                   or was reset); any corruption episode is over. */
                transport.rx_invalid_streak = 0u;
                break;
            }
            if (result == ESP_ERR_INVALID_SIZE) {
                /* Stale/torn length: retry without clearing the interrupt
                   (see read_pending_frame). A stable garbage length never
                   fixes itself; hand the link to the recovery path instead
                   of spinning here or waiting out the 5 s heartbeat. */
                if (++transport.rx_invalid_streak >=
                    WLH_SDIO_RX_INVALID_LIMIT) {
                    ESP_LOGE(TAG,
                             "SDIO RX length corrupt for %u reads; "
                             "triggering transport recovery",
                             transport.rx_invalid_streak);
                    transport.rx_invalid_streak = 0u;
                    wlh_host_transport_lost(transport.host);
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(1u));
                continue;
            }
            if (result != ESP_OK) {
                ESP_LOGW(TAG, "SDIO RX read failed: %s",
                         esp_err_to_name(result));
                break;
            }
            if (wlh_frame_validate(frame, size, WLH_SDIO_MAX_FRAME_SIZE) !=
                WLH_WIRE_OK) {
                ESP_LOGW(TAG, "dropping invalid SDIO RX transaction");
                continue;
            }
            transport.rx_invalid_streak = 0u;
            {
                wlh_host_result_t host_result;
                do {
                    host_result =
                        wlh_host_on_frame(transport.host, frame, size);
                    if (host_result == WLH_HOST_PENDING_FULL)
                        vTaskDelay(pdMS_TO_TICKS(1u));
                } while (host_result == WLH_HOST_PENDING_FULL &&
                         atomic_load(&transport.running));
                if (host_result != WLH_HOST_OK)
                    ESP_LOGW(TAG, "Host Core rejected SDIO frame: %d",
                             (int)host_result);
            }
            /* A continuously asserted slave queue must not let this high
               priority worker monopolize a single-core host. Preserve the
               pending state after a bounded burst, block for one scheduler
               tick, then resume draining without relying on a new edge. */
            if (++burst_frames == WLH_SDIO_RX_BURST_LIMIT) {
                pending = true;
                vTaskDelay(1u);
                break;
            }
        }
    }
}

static void lifecycle_task_main(void *argument) {
    lifecycle_job_t job;
    (void)argument;
    for (;;) {
        esp_err_t result;
        if (xQueueReceive(transport.lifecycle_queue, &job, portMAX_DELAY) !=
            pdTRUE)
            continue;
        if (job.start) {
            result = hardware_start();
            atomic_store(&transport.running, result == ESP_OK);
        } else {
            hardware_stop();
            result = ESP_OK;
        }
        job.completion(job.completion_context, (int)result);
    }
}

static int submit_lifecycle(bool start,
                            wlh_transport_lifecycle_complete_fn completion,
                            void *completion_context) {
    lifecycle_job_t job = {start, completion, completion_context};
    if (completion == NULL ||
        xQueueSend(transport.lifecycle_queue, &job, 0) != pdTRUE)
        return -1;
    return 0;
}

static int transport_start(void *context,
                           wlh_transport_lifecycle_complete_fn completion,
                           void *completion_context) {
    (void)context;
    return submit_lifecycle(true, completion, completion_context);
}

static int transport_stop(void *context,
                          wlh_transport_lifecycle_complete_fn completion,
                          void *completion_context) {
    (void)context;
    return submit_lifecycle(false, completion, completion_context);
}

static int transport_submit_tx(void *context, uint8_t *frame, size_t size,
                               wlh_transport_tx_complete_fn completion,
                               void *completion_context) {
    tx_job_t job = {frame, size, completion, completion_context};
    (void)context;
    if (!atomic_load(&transport.running) || frame == NULL ||
        size < WLH_FRAME_HEADER_SIZE || size > WLH_SDIO_MAX_FRAME_SIZE ||
        completion == NULL)
        return -1;
    return xQueueSend(transport.tx_queue, &job, 0) == pdTRUE ? 0 : -1;
}

int wlh_sdio_transport_init(wlh_host_t *host) {
    memset(&transport, 0, sizeof(transport));
    transport.host = host;
    transport.tx_queue =
        xQueueCreate(WLH_SDIO_TX_QUEUE_DEPTH, sizeof(tx_job_t));
    transport.lifecycle_queue = xQueueCreate(2u, sizeof(lifecycle_job_t));
    transport.bus_lock = xSemaphoreCreateMutex();
    transport.io_state_lock = xSemaphoreCreateMutex();
    transport.io_events = xEventGroupCreate();
    if (transport.tx_queue == NULL || transport.lifecycle_queue == NULL ||
        transport.bus_lock == NULL || transport.io_state_lock == NULL ||
        transport.io_events == NULL)
        return -1;
    xEventGroupSetBits(transport.io_events, WLH_SDIO_IO_IDLE_BIT);
    if (xTaskCreate(lifecycle_task_main, "wlh-sdio-life", 6144u, NULL, 7,
                    NULL) != pdPASS ||
        /* Keep TX and RX at the same priority. Either direction may carry the
           CreditUpdate required for its peer to continue, so permanently
           preferring one side can deadlock a saturated bidirectional link. */
        /* CMD52/CMD53 traverse the ESP-IDF SDMMC command and cache/DMA
           preparation layers on this task's stack.  The former 4 KiB budget
           overflows under sustained full-size Ethernet TX; RX executes the
           same driver stack, so give both bounded I/O workers the measured
           safe budget rather than relying on the idle-traffic call depth. */
        xTaskCreate(tx_task_main, "wlh-sdio-tx", WLH_SDIO_IO_TASK_STACK_SIZE,
                    NULL, 9, NULL) != pdPASS ||
        xTaskCreate(rx_task_main, "wlh-sdio-rx", WLH_SDIO_IO_TASK_STACK_SIZE,
                    NULL, 9, NULL) != pdPASS)
        return -1;
    return 0;
}

wlh_transport_ops_t wlh_sdio_transport_ops(void) {
    return (wlh_transport_ops_t){
        .context = &transport,
        .start = transport_start,
        .stop = transport_stop,
        .submit_tx = transport_submit_tx,
    };
}
