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
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdmmc_cmd.h"
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
#define WLH_SDIO_INIT_RETRIES 3u

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
    atomic_bool running;
    bool hardware_initialized;
    uint32_t rx_byte_count;
    uint32_t tx_buffer_count;
} sdio_transport_t;

static const char *TAG = "wlh-sdio-host";
static sdio_transport_t transport;

static uint32_t round_up_4(size_t size) {
    return (uint32_t)((size + 3u) & ~3u);
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
    atomic_store(&transport.running, false);
    if (!transport.hardware_initialized) return;
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
        vTaskDelay(pdMS_TO_TICKS(2u));
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

static esp_err_t write_frame(const uint8_t *frame, size_t size) {
    uint32_t transfer_size = round_up_4(size);
    uint8_t *dma_frame =
        heap_caps_calloc(1u, transfer_size, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    esp_err_t result;
    if (dma_frame == NULL) return ESP_ERR_NO_MEM;
    memcpy(dma_frame, frame, size);
    xSemaphoreTake(transport.bus_lock, portMAX_DELAY);
    result = wait_for_tx_buffer();
    if (result == ESP_OK) {
        result =
            write_data(WLH_SDIO_END_ADDRESS - size, dma_frame, transfer_size);
    }
    xSemaphoreGive(transport.bus_lock);
    heap_caps_free(dma_frame);
    return result;
}

static void tx_task_main(void *argument) {
    tx_job_t job;
    (void)argument;
    for (;;) {
        if (xQueueReceive(transport.tx_queue, &job, portMAX_DELAY) != pdTRUE)
            continue;
        if (!atomic_load(&transport.running)) {
            job.completion(job.completion_context, job.frame, job.size,
                           ESP_FAIL);
            continue;
        }
        esp_err_t result = write_frame(job.frame, job.size);
        if (result != ESP_OK)
            ESP_LOGW(TAG, "CMD53 TX failed: %s", esp_err_to_name(result));
        job.completion(job.completion_context, job.frame, job.size,
                       (int)result);
    }
}

static esp_err_t read_pending_frame(uint8_t *frame, size_t *frame_size) {
    uint32_t packet_length = 0u;
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
    /* Clear the interrupt on every path: a failed or garbage transaction
       must never leave the bit asserted and wedge the RX task. */
    esp_err_t clear_result = write_register(
        WLH_SDIO_REG_INT_CLEAR, &interrupt_status, sizeof(interrupt_status));
    if (result != ESP_OK) return result;
    if (clear_result != ESP_OK) return clear_result;
    packet_length &= WLH_SDIO_LENGTH_MASK;
    size = (packet_length + WLH_SDIO_LENGTH_MODULUS - transport.rx_byte_count) %
           WLH_SDIO_LENGTH_MODULUS;
    if (packet_length == WLH_SDIO_LENGTH_MASK || size < WLH_FRAME_HEADER_SIZE ||
        size > WLH_SDIO_MAX_FRAME_SIZE) {
        /* Floating bus or lost framing (e.g. slave mid-reset): drop this
           transaction and resynchronize to the slave's counter. */
        transport.rx_byte_count = packet_length;
        return ESP_ERR_INVALID_SIZE;
    }
    transfer_size = round_up_4(size);
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
    uint8_t *frame = heap_caps_malloc(WLH_SDIO_MAX_FRAME_SIZE + 4u,
                                      MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    (void)argument;
    configASSERT(frame != NULL);
    for (;;) {
        esp_err_t result;
        size_t size = 0u;
        if (!atomic_load(&transport.running)) {
            vTaskDelay(pdMS_TO_TICKS(20u));
            continue;
        }
        result = sdmmc_io_wait_int(&transport.card, pdMS_TO_TICKS(1000u));
        if (result == ESP_ERR_TIMEOUT) continue;
        if (result != ESP_OK) {
            ESP_LOGW(TAG, "SDIO interrupt wait failed: %s",
                     esp_err_to_name(result));
            continue;
        }
        for (;;) {
            /* Drain every queued frame; the slave may have posted more
               than one transaction per interrupt. Hold the SDIO bus only
               for one CMD53 transaction so the TX task can return credits
               between received Ethernet frames. */
            xSemaphoreTake(transport.bus_lock, portMAX_DELAY);
            result = read_pending_frame(frame, &size);
            xSemaphoreGive(transport.bus_lock);
            if (result == ESP_ERR_NOT_FOUND) break;
            if (result == ESP_ERR_INVALID_SIZE) {
                ESP_LOGW(TAG, "dropping invalid SDIO RX transaction");
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
    if (transport.tx_queue == NULL || transport.lifecycle_queue == NULL ||
        transport.bus_lock == NULL)
        return -1;
    if (xTaskCreate(lifecycle_task_main, "wlh-sdio-life", 6144u, NULL, 7,
                    NULL) != pdPASS ||
        /* TX must preempt RX after Host Core queues a CreditUpdate. Otherwise
           a continuous RX burst can fill the bounded TX queue and permanently
           lose credits until the coprocessor reaches CONGESTED. */
        xTaskCreate(tx_task_main, "wlh-sdio-tx", 4096u, NULL, 9, NULL) !=
            pdPASS ||
        xTaskCreate(rx_task_main, "wlh-sdio-rx", 4096u, NULL, 6, NULL) !=
            pdPASS)
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
