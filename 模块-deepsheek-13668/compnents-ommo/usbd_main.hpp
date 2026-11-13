#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "USB_CDC.h"

USB_CDC_HANDLE usbd_main_add_cdc(uint8_t *rx_buffer, const uint16_t rx_buffer_len, const uint16_t max_tx_buffer_len);
void usbd_main_init();
void usbd_main_start();
void usbd_main_uninit();

#ifdef OMMO_USB_SOF_TIMESTAMP_CAPTURE
void usbd_main_get_last_sof_frame_id_and_timestamp(uint16_t *frame_id, uint32_t *timestamp);
#endif

#ifdef __cplusplus
}
#endif
