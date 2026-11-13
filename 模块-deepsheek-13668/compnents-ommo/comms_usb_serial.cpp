/**
 * Copyright (c) 2019, Ommo Technologies
 *
 * All rights reserved.
 */

#include "comms_usb_serial.hpp"

#include "app_error.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "sdk_macros.h"
#include "USB_CDC.h"

#include "ommo_config.h"
#include "rtc_timeout_gen.hpp"
#include "timestamp_timer.hpp"
#include "trace.h"
#include "usbd_main.hpp"

#ifdef OMMO_USB_STATE_TRACE_ENABLED
#define USB_TRACE_STATE_CHANGE(STATE_MASK) TRACE_WITH_ARGS_2(TRACE_TYPE_TIME_U26_U16, TRACE_USB_STATE_CHANGE, (STATE_MASK) | (usb_current_state << 16) | (usb_target_state << 8), usb_cdc_dtr_state)
#define USB_TRACE_STATE_CHANGE_INIT() TRACE_WITH_ARGS_2(TRACE_TYPE_TIME_U26_U16, TRACE_USB_STATE_CHANGE, 0xFFFFFF, 0xFF)
#else
#define USB_TRACE_STATE_CHANGE(STATE_MASK)
#define USB_TRACE_STATE_CHANGE_INIT()
#endif
#ifdef OMMO_USB_DATA_TRACE_ENABLED
#define USB_TRACE(LOCATION) TRACE(TRACE_TYPE_TIME_U26_U16, LOCATION)
#define USB_TRACE_WITH_DATA(LOCATION, SIZE, BUFFER) TRACE_WITH_ARGS_2(TRACE_TYPE_TIME_DATA, LOCATION, SIZE, BUFFER)
#else
#define USB_TRACE(LOCATION)
#define USB_TRACE_WITH_DATA(LOCATION, SIZE, BUFFER)
#endif

void comms_usb_serial::static_usb_on_state_change(void * pContext, uint8_t new_state)
{
    ((comms_usb_serial*)pContext)->determine_usb_state(new_state);
}

void comms_usb_serial::static_usb_cdc_on_line_state_change(USB_CDC_CONTROL_LINE_STATE * state, const void *pContext)
{
    ((comms_usb_serial*)pContext)->usb_cdc_dtr_state = (usb_serial_dtr_state_t)state->DTR;
    ((comms_usb_serial*)pContext)->determine_usb_state(USBD_GetState());
}

void comms_usb_serial::static_on_reconnect_timeout(void * pContext)
{
    // SUSPENDED events may occur due to intermittent signal loss.  After one second, assume the
    // host has actually gone into hibernation and close the port.  The host may want to keep the
    // port open and idle during hibernation (by leaving DTR on), but Ommo devices do not support
    // that.  Evidence suggests that it takes about 2.7 seconds for comms_packetizer to start
    // dropping packets on a base station.
    ((comms_usb_serial*)pContext)->determine_usb_state(OMMO_USB_STAT_CONCLUSIVELY_SUSPENDED);
}

void comms_usb_serial::determine_usb_state(uint8_t usb_state_mask)
{
    //  ATTACHED  READY + CONFIGURED  SUSPENDED   usb_cdc_dtr_state   new usb_target_state  Meaning
    //  0         d/c                 d/c         d/c                 DETACHED              no host connected
    //  1         d/c                 d/c         !ON                 ATTACHED              host connected, but port closed
    //  1         1                   0           ON                  OPENED                host connected, port open
    //  1         1                   1           ON                  RECONNECTING          host connected, port was open, but signal lost
    //  1         0                   n/a         ON                  RECONNECTING          host connected, port was open, but device is reconnecting to host

    usb_serial_state_t prev_target_state = usb_target_state;
    if (usb_state_mask & USB_STAT_ATTACHED)
    {
        if (usb_cdc_dtr_state != USB_SERIAL_DTR_STATE_ON)
        {
            usb_target_state = USB_SERIAL_STATE_ATTACHED;
        }
        else if ( (usb_state_mask & (USB_STAT_CONFIGURED | USB_STAT_SUSPENDED | USB_STAT_READY)) == (USB_STAT_CONFIGURED | USB_STAT_READY) )
        {
            usb_target_state = USB_SERIAL_STATE_OPENED;
        }
        else
        {
            usb_target_state = USB_SERIAL_INTERNAL_STATE_RECONNECTING;
        }
    }
    else
    {
        usb_target_state = USB_SERIAL_STATE_DETACHED;
        usb_cdc_dtr_state = USB_SERIAL_DTR_STATE_DETACHED;
    }

    if (usb_target_state == USB_SERIAL_INTERNAL_STATE_RECONNECTING)
    {
        if (prev_target_state == USB_SERIAL_INTERNAL_STATE_RECONNECTING)
            APP_ERROR_CHECK(rtc_timer_reset_timeout(reconnect_timeout_channel));
        else
            APP_ERROR_CHECK(rtc_timer_set_timeout(reconnect_timeout_channel, MS_TO_RTC_TIMEOUT_GEN(1000)));
    }
    else
    {
        APP_ERROR_CHECK(rtc_timer_disable_timeout(reconnect_timeout_channel));
    }

    USB_TRACE_STATE_CHANGE(usb_state_mask);
}

ret_code_t comms_usb_serial::add_tasks_to_event_queue(event_queue_manager *event_queue)
{
    // Add our packetizer's functions
    VERIFY_SUCCESS(comms_packetizer::add_tasks_to_event_queue(event_queue));

    // Add our functions
    VERIFY_SUCCESS(event_queue->register_task(this, event_queue_state_change_pending, event_queue_state_change_process));

    return NRF_SUCCESS;
}

bool  comms_usb_serial::event_queue_state_change_pending()
{
    return usb_current_state != usb_target_state;
}

void comms_usb_serial::event_queue_state_change_process()
{
    // This should NOT be called from an ISR
    APP_ERROR_CHECK_BOOL(current_int_priority_get() == APP_IRQ_PRIORITY_THREAD);

    const usb_serial_state_t old_state = usb_current_state;
    usb_current_state                  = usb_target_state;
    usb_serial_event_t event           = USB_SERIAL_EVENT_NONE;
    bool cancel_io = false;

    //  old_state     usb_target_state  event     Meaning
    //  DETACHED      ATTACHED          ATTACHED  host connected
    //  DETACHED      OPENED            OPENED    host connected and port opened before event queue could process ATTACHED state
    //  DETACHED      RECONNECTING      ATTACHED  host connected, port opened, and signal lost before event queue could process ATTACHED state
    //  ATTACHED      DETACHED          DETACHED  host disconnected
    //  ATTACHED      OPENED            OPENED    port opened
    //  ATTACHED      RECONNECTING      ATTACHED  port opened and signal lost before event queue could process ATTACHED state
    //  OPENED        DETACHED          CLOSED    host disconnected (no DETACHED event is sent)
    //  OPENED        ATTACHED          CLOSED    port closed
    //  OPENED        RECONNECTING      none      signal lost for less than 1 second
    //  RECONNECTING  DETACHED          CLOSED    host disconnected after previously losing signal (no DETACHED event is sent)
    //  RECONNECTING  ATTACHED          CLOSED    signal reacquired and port closed, or host disconnected and reconnected before event queue could process OPENED or DETACHED states
    //  RECONNECTING  OPENED            none      signal reacquired

    switch (usb_current_state)
    {
        case USB_SERIAL_STATE_OPENED:
            enable_packet_reading();
            if (old_state < USB_SERIAL_STATE_FIRST_OPENED)
                event = USB_SERIAL_EVENT_OPENED;
            break;

        case USB_SERIAL_STATE_ATTACHED:
            event = (old_state >= USB_SERIAL_STATE_FIRST_OPENED)
                          ? USB_SERIAL_EVENT_CLOSED
                          : USB_SERIAL_EVENT_ATTACHED;
            break;

        case USB_SERIAL_STATE_DETACHED:
            event = (old_state >= USB_SERIAL_STATE_FIRST_OPENED)
                          ? USB_SERIAL_EVENT_CLOSED
                          : USB_SERIAL_EVENT_DETACHED;
            break;

        case USB_SERIAL_INTERNAL_STATE_RECONNECTING:
            if (old_state < USB_SERIAL_STATE_FIRST_OPENED)
                event = USB_SERIAL_EVENT_ATTACHED;
            cancel_io = true;
            break;

        default:
            break;
    }

    //If the port was closed, clear out all reads/writes
    if (event == USB_SERIAL_EVENT_CLOSED)
        cancel_io = true;

    if (cancel_io)
    {
        USBD_CDC_CancelWrite(usb_cdc_handle);
        USBD_CDC_CancelRead(usb_cdc_handle);
        disable_packet_reading();
    }

    //Pass event on to handler
    if (event != USB_SERIAL_EVENT_NONE && usb_cdc_event_handler)
    {
        (*usb_cdc_event_handler)(event);
    }
}

usb_serial_state_t comms_usb_serial::get_state(void)
{
    if (usb_current_state == USB_SERIAL_INTERNAL_STATE_RECONNECTING)
        return USB_SERIAL_STATE_OPENED;
    return usb_current_state;
}

void comms_usb_serial::init(const comms_packetizer_packet_ready_handler packet_ready_handler, const usb_serial_event_handler_t ev_callback)
{
    // Save callback function, can be NULL
    usb_cdc_event_handler  = ev_callback;

    // Init port state vars
    usb_cdc_dtr_state = USB_SERIAL_DTR_STATE_OFF;
    usb_current_state = USB_SERIAL_STATE_DETACHED;
    usb_target_state  = USB_SERIAL_STATE_DETACHED;
    USB_TRACE_STATE_CHANGE_INIT();

    //Register cdc and callbacks with usbd
    USBD_RegisterSCHook(&usb_state_hook, static_usb_on_state_change, this);
    usb_cdc_handle = usbd_main_add_cdc(usb_cdc_raw_buffer, USB_SERIAL_RX_BUFFER_SIZE, USB_SERIAL_TX_BUFFER_SIZE);
    USBD_CDC_SetOnControlLineState(usb_cdc_handle, static_usb_cdc_on_line_state_change, this);

    // Initialize our transport layer variables
    write_io_ctx.pfOnComplete = static_usb_cdc_on_write_cplt;
    write_io_ctx.pContext = this;
    read_io_ctx.pfOnComplete = static_usb_cdc_on_read_cplt;
    read_io_ctx.pContext = this;

    //Initialize the packetizer
    comms_packetizer::init(this, packet_ready_handler, this);

    // Allocate timeout channel.  On USB-powered devices, power is typically interrupted during
    // host hibernation, causing the connection to close.  On battery-powered devices, a
    // momentarily power loss will not cause the connection to close.  A one second timeout ensures
    // that a true USB SUSPEND operation eventually closes the connection, regardless of how the
    // device is powered or how the host controls power.
    APP_ERROR_CHECK(rtc_timer_get_timeout_channel(&reconnect_timeout_channel, static_on_reconnect_timeout, this));
}

// **********************************
// * Transport layer implementation *
// **********************************
void comms_usb_serial::static_usb_cdc_on_write_cplt(USB_ASYNC_IO_CONTEXT * pIOContext)
{
    comms_usb_serial *class_ptr = ((comms_usb_serial*)pIOContext->pContext);
    USB_TRACE(TRACE_USB_SERIAL_WRITE_COMPLETE);

    ret_code_t err_code;
    switch(pIOContext->Status)
    {
    case 0:
        err_code = NRF_SUCCESS;
        break;
    
    case USB_STATUS_EP_HALTED:
    case USB_STATUS_EP_BUSY:
        err_code = NRF_ERROR_BUSY;
        break;
    
    case USB_STATUS_ERROR:  // typically means suspended
    default:
        err_code = NRF_ERROR_NOT_FOUND;
        break;
    }
    
    class_ptr->tx_complete_callback(pIOContext->NumBytesTransferred, err_code, class_ptr->p_context);
}

void comms_usb_serial::static_usb_cdc_on_read_cplt(USB_ASYNC_IO_CONTEXT * pIOContext)
{
    comms_usb_serial *class_ptr = ((comms_usb_serial*)pIOContext->pContext);
    USB_TRACE_WITH_DATA(TRACE_USB_SERIAL_READ_COMPLETE, pIOContext->NumBytesTransferred, pIOContext->pData);
    class_ptr->rx_complete_callback(pIOContext->NumBytesTransferred, pIOContext->Status != 0, class_ptr->p_context);
}

ret_code_t comms_usb_serial::comms_transport_tx(uint8_t *p_tx_buffer, size_t tx_length, bool expedite_tx)
{
    //Setup context for usb_cdc
    write_io_ctx.NumBytesToTransfer = tx_length;
    write_io_ctx.pData = p_tx_buffer;

    USB_TRACE_WITH_DATA(TRACE_USB_SERIAL_WRITE, tx_length, p_tx_buffer);
    USBD_CDC_WriteAsync(usb_cdc_handle, &write_io_ctx, expedite_tx);

    return NRF_SUCCESS;
}

ret_code_t comms_usb_serial::comms_transport_rx(uint8_t *p_rx_buffer, size_t max_rx_length)
{
    //Setup context for usb_cdc
    read_io_ctx.NumBytesToTransfer = max_rx_length;
    read_io_ctx.pData = p_rx_buffer;

    USB_TRACE(TRACE_USB_SERIAL_READ);
    USBD_CDC_ReadAsync(usb_cdc_handle, &read_io_ctx, true);

    return NRF_SUCCESS;
}

bool comms_usb_serial::is_connected()
{
    return (usb_current_state == USB_SERIAL_STATE_OPENED && !(USBD_GetState() & USB_STAT_SUSPENDED));
}