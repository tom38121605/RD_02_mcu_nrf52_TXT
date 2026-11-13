/**
 * Copyright (c) 2019, Ommo Technologies
 *
 * All rights reserved.
 */

#include <cstring>
#include "sdk_macros.h"

#include "ommo_fw.pb.h"
#include "sdk_config.h"
#include "ommo_config.h"
#include "usb_serial_num.h"
#include "nrfx_swi.h"
#include "nrf_usbd.h"

#include "timestamp_timer.hpp"

#include "usbd_main.hpp"

//  Information that is used during enumeration.
static const USB_DEVICE_INFO usb_device_info = {
    .VendorId      = 0x3334,                   // VendorId - Ommo's USB VID
    .ProductId     = APP_USBD_PID,             // ProductId
    .sVendorName   = "Ommo Technologies Inc.", // VendorName
    .sProductName  = APP_USBD_STRINGS_PRODUCT, // ProductName
    .sSerialNumber = g_extern_serial_number // SerialNumber - Declared in usb_serial_num.c and
                                            // filled in at runtime
};

#ifdef OMMO_USB_SOF_TIMESTAMP_CAPTURE
static nrfx_swi_t sof_evt_swi;
static nrf_ppi_channel_t sof_timestamp_capture_ppi;
static nrf_timer_cc_channel_t sof_timestamp_capture_ch;
static uint16_t sof_frame_id;
static uint32_t sof_frame_timestamp;

static void sof_event_handler(nrfx_swi_t swi, nrfx_swi_flags_t flags)
{
    sof_frame_id = (uint16_t)(NRF_USBD->FRAMECNTR & 0x7FF);
    sof_frame_timestamp = timestamp_read_capture(sof_timestamp_capture_ch);
}

void usbd_main_get_last_sof_frame_id_and_timestamp(uint16_t *frame_id, uint32_t *timestamp)
{
    NRFX_CRITICAL_SECTION_ENTER();
    *frame_id = sof_frame_id;
    *timestamp = sof_frame_timestamp;
    NRFX_CRITICAL_SECTION_EXIT();
}

#endif

/**
 * @brief Add communication device class to USB stack.
 */
USB_CDC_HANDLE usbd_main_add_cdc(uint8_t *rx_buffer, const uint16_t rx_buffer_len, const uint16_t max_tx_buffer_len)
{
    static const USB_ADD_EP_INFO EPBulkIn = {
        .MaxPacketSize = max_tx_buffer_len,       // Maximum packet size
        .Interval      = 0,                       // Interval not used for Bulk endpoints.
        .Flags         = 0,                       // Flags not used.
        .InDir         = USB_DIR_IN,              // IN direction (Device to Host)
        .TransferType  = USB_TRANSFER_TYPE_BULK,  // Endpoint type - Bulk.
        .ISO_Type      = USB_ISO_SYNC_TYPE_NONE
    };

    const USB_ADD_EP_INFO EPBulkOut = {
        .MaxPacketSize = rx_buffer_len,            // Maximum packet size
        .Interval      = 0,                     // Interval not used for Bulk endpoints.
        .Flags         = 0,                     // Flags not used.
        .InDir         = USB_DIR_OUT,           // OUT direction (Host to Device)
        .TransferType  = USB_TRANSFER_TYPE_BULK,// Endpoint type - Bulk.
        .ISO_Type      = USB_ISO_SYNC_TYPE_NONE
    };

    USB_CDC_INIT_DATA InitData = {};
    InitData.EPIn              = USBD_AddEPEx(&EPBulkIn, nullptr, 0);
    InitData.EPOut             = USBD_AddEPEx(&EPBulkOut, rx_buffer, rx_buffer_len);
#ifdef OMMO_USB_CDC_NO_NOTIFICATION_EP
    InitData.EPInt             = 0;  // no interrupt endpoint -- eliminates periodic schedule consumption
#else
    static const USB_ADD_EP_INFO EPIntIn = {
        .MaxPacketSize = 16,
#ifdef OMMO_USB_CDC_NOTIFICATION_INTERVAL_MS
        .Interval      = (OMMO_USB_CDC_NOTIFICATION_INTERVAL_MS * 8),  // ms -> emUSBD 125us units
#else
        .Interval      = 64,                   // 8 ms
#endif
        .Flags         = 0,
        .InDir         = USB_DIR_IN,
        .TransferType  = USB_TRANSFER_TYPE_INT,
        .ISO_Type      = USB_ISO_SYNC_TYPE_NONE
    };
    InitData.EPInt             = USBD_AddEPEx(&EPIntIn, nullptr, 0);
#endif
    return USBD_CDC_Add(&InitData);
}

void usbd_main_init()
{
    // Fill in serial number
    ommo_usbd_serial_num_generate();

    // Start USB
    USBD_Init();
    USBD_EnableIAD();
    USBD_SetDeviceInfo(&usb_device_info);


#ifdef OMMO_USB_SOF_TIMESTAMP_CAPTURE
    APP_ERROR_CHECK(nrfx_swi_alloc(&sof_evt_swi, sof_event_handler, APP_IRQ_PRIORITY_MID));
    sof_timestamp_capture_ch = timestamp_add_capture_event(nrf_usbd_event_address_get(NRF_USBD_EVENT_SOF), &sof_timestamp_capture_ppi);
    nrf_ppi_fork_endpoint_setup(sof_timestamp_capture_ppi, nrfx_swi_task_trigger_address_get(sof_evt_swi, 0));
#endif
}

void usbd_main_start()
{
    USBD_Start();
}

void usbd_main_uninit()
{
    USBD_Stop();
    USBD_DeInit();
}
