#ifndef WLH_HOST_ESP_IDF_SDIO_TRANSPORT_H
#define WLH_HOST_ESP_IDF_SDIO_TRANSPORT_H

#include "wlh/host.h"

#define WLH_SDIO_MAX_FRAME_SIZE 4092u

int wlh_sdio_transport_init(wlh_host_t *host);
wlh_transport_ops_t wlh_sdio_transport_ops(void);

#endif
