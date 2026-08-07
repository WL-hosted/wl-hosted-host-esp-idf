#include "ota_command.h"

#include <stdio.h>
#include <string.h>

#include "esp_console.h"
#include "esp_http_client.h"
#include "esp_littlefs.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "mbedtls/sha256.h"
#include "network.h"
#include "ota.pb.h"
#include "sdio_transport.h"
#include "wlh/protocol/raw_record.h"
#include "wlh/protocol/wire.h"

static const char *TAG = "wlh-ota";

#define OTA_STORAGE_LABEL "storage"
#define OTA_STORAGE_BASE "/storage"
#define OTA_IMAGE_PATH OTA_STORAGE_BASE "/coproc.bin"
#define OTA_HTTP_TIMEOUT_MS 15000
#define OTA_RPC_TIMEOUT_MS 15000u
#define OTA_ABORT_TIMEOUT_MS 5000u
#define OTA_CREDIT_TIMEOUT_MS 10000u
#define OTA_DOWNLOAD_BUFFER 4096u
#define OTA_PROGRESS_STEP 65536u
#define OTA_IMAGE_MAGIC 0xE9u

/* Largest OTA record payload the host link admits: SDIO frame minus the frame
 * header, the raw-record header, and the 16-byte OTA stream sub-header. Mirrors
 * the check in host_ota.c (size + 16 <= ota_max_record). */
#define OTA_HOST_CHUNK_LIMIT                                                   \
    ((uint32_t)(WLH_SDIO_MAX_FRAME_SIZE - WLH_FRAME_HEADER_SIZE -              \
                WLH_RAW_RECORD_HEADER_SIZE - 16u))

static wlh_app_t *ota_app;

typedef struct ota_wait {
    SemaphoreHandle_t done;
    wlh_host_result_t result;
    uint16_t domain;
    int16_t status;
    bool has_begin;
    wlh_host_ota_begin_response_t begin;
} ota_wait_t;

static void ota_rpc_completion(void *context, wlh_host_result_t result,
                               uint16_t domain, int16_t status,
                               const uint8_t *payload, size_t payload_size) {
    ota_wait_t *wait = context;
    (void)payload;
    (void)payload_size;
    wait->result = result;
    wait->domain = domain;
    wait->status = status;
    xSemaphoreGive(wait->done);
}

static void
ota_begin_completion(void *context, wlh_host_result_t result, uint16_t domain,
                     int16_t status,
                     const wlh_host_ota_begin_response_t *response) {
    ota_wait_t *wait = context;
    wait->result = result;
    wait->domain = domain;
    wait->status = status;
    wait->has_begin = response != NULL;
    if (response != NULL) wait->begin = *response;
    xSemaphoreGive(wait->done);
}

/* Returns 0 when the operation completed with a success status. */
static int ota_wait(ota_wait_t *wait, wlh_host_result_t submitted,
                    uint32_t timeout_ms, const char *what) {
    if (submitted != WLH_HOST_OK) {
        printf("%s rejected locally: %d\n", what, submitted);
        return 1;
    }
    if (xSemaphoreTake(wait->done, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        printf("%s timed out\n", what);
        return 1;
    }
    if (wait->result != WLH_HOST_OK || wait->domain != 0u ||
        wait->status != 0) {
        printf("%s failed: result=%d domain=%u status=%d\n", what, wait->result,
               wait->domain, wait->status);
        return 1;
    }
    return 0;
}

static esp_err_t ota_mount_storage(void) {
    if (esp_littlefs_mounted(OTA_STORAGE_LABEL)) return ESP_OK;
    esp_vfs_littlefs_conf_t conf = {
        .base_path = OTA_STORAGE_BASE,
        .partition_label = OTA_STORAGE_LABEL,
        .format_if_mount_failed = true,
        .dont_mount = false,
    };
    esp_err_t err = esp_vfs_littlefs_register(&conf);
    if (err != ESP_OK)
        ESP_LOGE(TAG,
                 "littlefs mount failed: %s (is 'storage' partition flashed?)",
                 esp_err_to_name(err));
    return err;
}

/* Downloads url into OTA_IMAGE_PATH, returning the byte count and the SHA-256
 * of the full image. Plain HTTP only. */
static esp_err_t ota_download(const char *url, uint64_t *image_size,
                              uint8_t sha256[32]) {
    if (strncmp(url, "http://", 7) != 0) {
        printf("only plain http:// URLs are supported\n");
        return ESP_ERR_INVALID_ARG;
    }

    esp_http_client_config_t config = {
        .url = url,
        .timeout_ms = OTA_HTTP_TIMEOUT_MS,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
        printf("failed to init http client\n");
        return ESP_FAIL;
    }

    esp_err_t err = ESP_FAIL;
    FILE *file = NULL;
    uint8_t *buffer = NULL;
    mbedtls_sha256_context sha;
    bool sha_started = false;

    err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        printf("http open failed: %s\n", esp_err_to_name(err));
        goto cleanup;
    }
    int64_t content_length = esp_http_client_fetch_headers(client);
    int status = esp_http_client_get_status_code(client);
    if (status != 200) {
        printf("http status %d\n", status);
        err = ESP_FAIL;
        goto cleanup;
    }

    file = fopen(OTA_IMAGE_PATH, "wb");
    if (file == NULL) {
        printf("cannot open %s for write\n", OTA_IMAGE_PATH);
        err = ESP_FAIL;
        goto cleanup;
    }
    buffer = malloc(OTA_DOWNLOAD_BUFFER);
    if (buffer == NULL) {
        printf("out of memory for download buffer\n");
        err = ESP_ERR_NO_MEM;
        goto cleanup;
    }

    mbedtls_sha256_init(&sha);
    if (mbedtls_sha256_starts(&sha, 0) != 0) {
        err = ESP_FAIL;
        goto cleanup;
    }
    sha_started = true;

    uint64_t total = 0u;
    uint64_t last_print = 0u;
    bool magic_checked = false;
    for (;;) {
        int read =
            esp_http_client_read(client, (char *)buffer, OTA_DOWNLOAD_BUFFER);
        if (read < 0) {
            printf("http read error after %llu bytes\n",
                   (unsigned long long)total);
            err = ESP_FAIL;
            goto cleanup;
        }
        if (read == 0) {
            if (esp_http_client_is_complete_data_received(client)) break;
            /* Socket closed early relative to Content-Length. */
            printf("http stream ended prematurely at %llu bytes\n",
                   (unsigned long long)total);
            err = ESP_FAIL;
            goto cleanup;
        }
        /* LittleFS flash erases stall the single CPU (UNICORE) inside the
         * driver; the console task running this download rarely blocks while
         * HTTP data is buffered, so the IDLE task starves and the Task WDT
         * panics mid-download. Yield around the flash write so IDLE can feed
         * the watchdog. */
        vTaskDelay(pdMS_TO_TICKS(2u));
        if (!magic_checked) {
            if (buffer[0] != OTA_IMAGE_MAGIC) {
                printf("not an ESP firmware image (magic 0x%02x)\n", buffer[0]);
                err = ESP_ERR_INVALID_ARG;
                goto cleanup;
            }
            magic_checked = true;
        }
        if (mbedtls_sha256_update(&sha, buffer, (size_t)read) != 0) {
            err = ESP_FAIL;
            goto cleanup;
        }
        if (fwrite(buffer, 1u, (size_t)read, file) != (size_t)read) {
            printf("write to %s failed\n", OTA_IMAGE_PATH);
            err = ESP_FAIL;
            goto cleanup;
        }
        /* LittleFS flash erases stall the single CPU (UNICORE) inside the
         * driver; the console task running this download never blocks while
         * HTTP data is buffered, so the IDLE task starves and the Task WDT
         * panics mid-download. Yield every chunk so IDLE can feed the WDT. */
        vTaskDelay(pdMS_TO_TICKS(1u));
        total += (uint64_t)read;
        if (total - last_print >= OTA_PROGRESS_STEP) {
            printf("downloaded %llu bytes\n", (unsigned long long)total);
            last_print = total;
        }
    }

    if (total == 0u) {
        printf("downloaded image is empty\n");
        err = ESP_ERR_INVALID_SIZE;
        goto cleanup;
    }
    if (content_length > 0 && (uint64_t)content_length != total) {
        printf("size mismatch: header %lld, received %llu\n",
               (long long)content_length, (unsigned long long)total);
        err = ESP_ERR_INVALID_SIZE;
        goto cleanup;
    }
    if (mbedtls_sha256_finish(&sha, sha256) != 0) {
        err = ESP_FAIL;
        goto cleanup;
    }

    *image_size = total;
    printf("download complete: %llu bytes\n", (unsigned long long)total);
    err = ESP_OK;

cleanup:
    if (sha_started) mbedtls_sha256_free(&sha);
    if (buffer != NULL) free(buffer);
    if (file != NULL) fclose(file);
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    return err;
}

/* Streams OTA_IMAGE_PATH to the coprocessor in chunk-sized records with credit
 * backpressure. Returns 0 on success. */
static int ota_stream_file(uint32_t transfer_id, uint64_t image_size,
                           uint32_t chunk) {
    FILE *file = fopen(OTA_IMAGE_PATH, "rb");
    if (file == NULL) {
        printf("cannot reopen %s for streaming\n", OTA_IMAGE_PATH);
        return 1;
    }
    uint8_t *buffer = malloc(chunk);
    if (buffer == NULL) {
        fclose(file);
        printf("out of memory for stream buffer\n");
        return 1;
    }

    int result = 0;
    uint64_t offset = 0u;
    uint64_t last_print = 0u;
    while (offset < image_size) {
        size_t want = chunk;
        if (image_size - offset < (uint64_t)chunk)
            want = (size_t)(image_size - offset);
        size_t got = fread(buffer, 1u, want, file);
        if (got != want) {
            printf("read %zu of %zu bytes at offset %llu\n", got, want,
                   (unsigned long long)offset);
            result = 1;
            break;
        }
        for (;;) {
            wlh_host_result_t sent = wlh_host_ota_stream_send(
                &ota_app->host, transfer_id, offset, buffer, got);
            if (sent == WLH_HOST_OK) break;
            if (sent == WLH_HOST_NO_CREDIT) {
                if (xSemaphoreTake(ota_app->ota_credit_ready,
                                   pdMS_TO_TICKS(OTA_CREDIT_TIMEOUT_MS)) !=
                    pdTRUE) {
                    printf("no OTA credit within %ums at offset %llu\n",
                           OTA_CREDIT_TIMEOUT_MS, (unsigned long long)offset);
                    result = 1;
                    break;
                }
            } else if (sent == WLH_HOST_PENDING_FULL) {
                vTaskDelay(pdMS_TO_TICKS(5));
            } else {
                printf("stream_send failed: %d at offset %llu\n", sent,
                       (unsigned long long)offset);
                result = 1;
                break;
            }
        }
        if (result != 0) break;
        offset += got;
        if (offset - last_print >= OTA_PROGRESS_STEP || offset == image_size) {
            printf("sent %llu/%llu bytes\n", (unsigned long long)offset,
                   (unsigned long long)image_size);
            last_print = offset;
        }
    }

    free(buffer);
    fclose(file);
    return result;
}

static void ota_abort_transfer(uint32_t transfer_id) {
    ota_wait_t wait = {0};
    wait.done = xSemaphoreCreateBinary();
    if (wait.done == NULL) return;
    wlh_host_result_t submitted = wlh_host_ota_abort(
        &ota_app->host, transfer_id, ota_rpc_completion, &wait);
    if (ota_wait(&wait, submitted, OTA_ABORT_TIMEOUT_MS, "ota abort") != 0)
        ESP_LOGW(TAG, "abort of transfer %lu not confirmed",
                 (unsigned long)transfer_id);
    vSemaphoreDelete(wait.done);
}

static int ota_command(int argc, char **argv) {
    if (argc != 2) {
        printf("usage: ota <http-url>\n");
        return 1;
    }

    wlh_host_diagnostics_t diagnostics;
    wlh_host_get_diagnostics(&ota_app->host, &diagnostics);
    if (diagnostics.state != WLH_HOST_STATE_READY) {
        printf("link not ready (state=%d); connect the coprocessor first\n",
               diagnostics.state);
        return 1;
    }
    esp_ip4_addr_t address;
    if (!wlh_network_get_sta_ipv4(&address)) {
        printf("no station IP; run sta_connect before ota\n");
        return 1;
    }

    xSemaphoreTake(ota_app->command_lock, portMAX_DELAY);

    int result = 1;
    uint32_t transfer_id = 0u;
    bool transfer_open = false;
    ota_wait_t wait = {0};
    wait.done = xSemaphoreCreateBinary();
    if (wait.done == NULL) {
        printf("out of memory\n");
        goto unlock;
    }

    if (ota_mount_storage() != ESP_OK) goto cleanup;

    uint64_t image_size = 0u;
    uint8_t sha256[32];
    if (ota_download(argv[1], &image_size, sha256) != ESP_OK) goto cleanup;

    /* Discard a stale credit signal from any previous aborted run. */
    (void)xSemaphoreTake(ota_app->ota_credit_ready, 0);

    /* BEGIN: the coprocessor erases its target partition synchronously here,
     * which can take several seconds for a multi-megabyte image. */
    wlh_host_ota_begin_params_t params = {0};
    params.image_size = image_size;
    memcpy(params.sha256, sha256, sizeof(params.sha256));
    wait.has_begin = false;
    wlh_host_result_t submitted = wlh_host_ota_begin(
        &ota_app->host, &params, ota_begin_completion, &wait);
    if (ota_wait(&wait, submitted, OTA_RPC_TIMEOUT_MS, "ota begin") != 0)
        goto cleanup;
    if (!wait.has_begin) {
        printf("ota begin returned no response\n");
        goto cleanup;
    }
    transfer_id = wait.begin.transfer_id;
    transfer_open = true;

    uint32_t chunk = wait.begin.stream_chunk_size;
    if (chunk > OTA_HOST_CHUNK_LIMIT) chunk = OTA_HOST_CHUNK_LIMIT;
    uint32_t alignment = wait.begin.stream_alignment;
    if (alignment > 1u) chunk -= chunk % alignment;
    if (chunk == 0u) {
        printf("negotiated chunk size is zero\n");
        goto cleanup;
    }
    printf("transfer %lu: negotiated chunk %lu, alignment %lu, using %lu\n",
           (unsigned long)transfer_id,
           (unsigned long)wait.begin.stream_chunk_size,
           (unsigned long)wait.begin.stream_alignment, (unsigned long)chunk);

    if (ota_stream_file(transfer_id, image_size, chunk) != 0) goto cleanup;

    /* FINALIZE: coprocessor drains its flash queue, then verifies length and
     * SHA-256. A non-zero status means the image was rejected and the
     * coprocessor already released its handle. */
    wait.result =
        WLH_HOST_TIMEOUT; /* sentinel: overwritten iff a completion arrives */
    submitted = wlh_host_ota_finalize(&ota_app->host, transfer_id, image_size,
                                      ota_rpc_completion, &wait);
    if (ota_wait(&wait, submitted, OTA_RPC_TIMEOUT_MS, "ota finalize") != 0) {
        /* Only a delivered completion (result == OK, bad status) means the
         * coprocessor self-cleaned; local reject or timeout leaves the transfer
         * open, so fall through to the abort in cleanup. */
        if (wait.result == WLH_HOST_OK) transfer_open = false;
        goto cleanup;
    }
    transfer_open = false; /* image verified; activate takes over ownership */

    /* ACTIVATE with reboot: the coprocessor replies before restarting, then the
     * link drops and host-core re-handshakes on its own. */
    submitted = wlh_host_ota_activate(&ota_app->host, transfer_id, true,
                                      ota_rpc_completion, &wait);
    if (ota_wait(&wait, submitted, OTA_RPC_TIMEOUT_MS, "ota activate") != 0)
        goto cleanup;

    printf("activated; coprocessor rebooting, link will re-handshake\n");
    result = 0;

cleanup:
    if (result != 0 && transfer_open) ota_abort_transfer(transfer_id);
    if (wait.done != NULL) vSemaphoreDelete(wait.done);
unlock:
    xSemaphoreGive(ota_app->command_lock);
    return result;
}

void wlh_ota_command_register(wlh_app_t *app) {
    ota_app = app;
    const esp_console_cmd_t command = {
        .command = "ota",
        .help = "ota <http-url>: download coprocessor firmware and update it",
        .func = ota_command,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&command));
}
