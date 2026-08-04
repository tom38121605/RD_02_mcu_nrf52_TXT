/**
 * Copyright (c) 2025, Ommo Technologies
 *
 * All rights reserved.
 */

#include "sdk_macros.h"

#include "app_error.h"
#include "cobs.h"
#include "comms_transport.hpp"
#include "ommo_app_error.h"
#include "ommo_config.h"
#include "rtc_timeout_gen.hpp"

#include "comms_packetizer.hpp"

#pragma GCC push_options
#pragma GCC optimize ("O3") // The following functions are called from time-critical ISRs

void comms_packetizer::static_transport_read_complete(size_t actual_rx_length, bool failed, const void *p_context)
{
    ((comms_packetizer*)p_context)->transport_read_complete(actual_rx_length, failed);
}

void comms_packetizer::static_transport_write_complete(size_t actual_tx_length, ret_code_t err_code, const void *p_context)
{
    ((comms_packetizer*)p_context)->transport_write_complete(actual_tx_length, err_code);
}

void comms_packetizer::static_on_flush_timeout(void * p_context)
{
    // Force a write of buffer data
    ((comms_packetizer*)p_context)->flush_flag = true;
}

void comms_packetizer::transport_read_complete(size_t actual_rx_length, bool failed)
{
    uint16_t i_raw;
    uint16_t i_cobs;

    // Note that errors may be reported synchronously
    if (failed)
    {
        // Retry from the main event queue
        rx_failed = true;
        return;
    }

    OMMO_APP_ERROR_CHECK(in_comms_transport_rx ? NRF_ERROR_INTERNAL : NRF_SUCCESS, STRING("unexpected synchronous call of transport_read_complete"), 0);

    if (rx_cobs_buffer_top + actual_rx_length > sizeof(rx_cobs_buffer))
    {
        rx_lost_packets++;
        rx_cobs_buffer_top = rx_cobs_buffer_in_use;
    }
    else
    {
        memcpy(&rx_cobs_buffer[rx_cobs_buffer_top], rx_raw_buffer, actual_rx_length);
        rx_cobs_buffer_top += actual_rx_length;

        rx_data_ready = true;
    }

    // Set up next transfer (if opened and not suspended)
    if (transport_device->is_connected())
        setup_next_rx();
}

void comms_packetizer::transport_write_complete(size_t reported_tx_length, ret_code_t err_code)
{
    // Note that errors may be reported synchronously
    if (err_code != NRF_SUCCESS)
    {
        // Retry up to 3 times from the main event queue
        if (tx_retry_count >= 3)
            on_tx_complete(err_code);
        else
            tx_failed = true;
        return;
    }

    OMMO_APP_ERROR_CHECK(in_comms_transport_tx ? NRF_ERROR_INTERNAL : NRF_SUCCESS, STRING("unexpected synchronous call of transport_write_complete"), 0);

    tx_buffer_transport += reported_tx_length;
    if (tx_buffer_transport >= tx_buffer_transport_end)
    {
        on_tx_complete(NRF_SUCCESS);
    }
    else
    {
        // Set up next transfer
        setup_next_tx();
    }
}

void comms_packetizer::setup_next_rx()
{
    in_comms_transport_rx = true;
    APP_ERROR_CHECK(transport_device->comms_transport_rx(rx_raw_buffer, sizeof(rx_raw_buffer)));
    in_comms_transport_rx = false;
}

void comms_packetizer::setup_next_tx()
{
    in_comms_transport_tx = true;
    APP_ERROR_CHECK(transport_device->comms_transport_tx(tx_buffer_transport,
                                                         tx_buffer_transport_end - tx_buffer_transport,
                                                         tx_buffer_transport_flags & TRANSPORT_FLAG_FLUSH));
    in_comms_transport_tx = false;
}

void comms_packetizer::on_tx_complete(ret_code_t err_code)
{
    if (err_code != NRF_SUCCESS)
        tx_lost_packets++;
    tx_buffer_transport_flags = TRANSPORT_FLAG_NONE;
    tx_buffer_transport = NULL;
    tx_err_code = err_code;
    tx_retry_count = 0;
}

#pragma GCC pop_options

void comms_packetizer::initiate_rx()
{
    NRFX_CRITICAL_SECTION_ENTER();
    setup_next_rx();
    NRFX_CRITICAL_SECTION_EXIT();
}

void comms_packetizer::initiate_tx()
{
    NRFX_CRITICAL_SECTION_ENTER();
    setup_next_tx();
    NRFX_CRITICAL_SECTION_EXIT();
}

ret_code_t comms_packetizer::add_tasks_to_event_queue(event_queue_manager *event_queue)
{
    // main_loop_only: flush_tx_buffers busy-waits calling execute_once(); re-submitting TX/RX mid-flush corrupts TX state
    VERIFY_SUCCESS(event_queue->register_task(this, event_queue_start_rx_pending, event_queue_start_rx_process, nullptr, nullptr, true));
    VERIFY_SUCCESS(event_queue->register_task(this, event_queue_start_tx_pending, event_queue_start_tx_process, nullptr, nullptr, true));
    VERIFY_SUCCESS(event_queue->register_task(this, event_queue_rx_error_recovery_pending, event_queue_rx_error_recovery_process, nullptr, nullptr, true));
    VERIFY_SUCCESS(event_queue->register_task(this, event_queue_tx_error_recovery_pending, event_queue_tx_error_recovery_process, nullptr, nullptr, true));
    return NRF_SUCCESS;
}

bool comms_packetizer::event_queue_start_tx_pending()
{
    return transport_device->is_connected() &&
           tx_buffer_transport_flags == TRANSPORT_FLAG_NONE &&
           (tx_buffer_active_top > OMMO_USB_SERIAL_MIN_PACKET_SIZE
            || (flush_flag && tx_buffer_active_top != tx_buffer_base));
}

void comms_packetizer::event_queue_start_tx_process()
{
    // Don't need to send anything again for at least 500ms
    if (RTC_TIMER)
    {
        APP_ERROR_CHECK(rtc_timer_reset_timeout(flush_timeout_channel));
    }

    // Send the buffer if it is above the minimum size or if we are forcing a flush
    NRFX_CRITICAL_SECTION_ENTER();
    tx_buffer_transport_flags = TRANSPORT_FLAG_IN_PROGRESS;
    if(flush_flag) tx_buffer_transport_flags = (transport_flags_t)(tx_buffer_transport_flags | TRANSPORT_FLAG_FLUSH);
    tx_buffer_transport = tx_buffer_active + tx_buffer_base;
    tx_buffer_transport_end = tx_buffer_active + tx_buffer_active_top;

    // Reset local state
    flush_flag = 0;
    tx_buffer_active_top = tx_buffer_base;

    // Switch buffers while transport layer uses tx buffer
    if (tx_buffer_active == tx_buffer_primary)
    {
        tx_buffer_active = tx_buffer_secondary;
    }
    else
    {
        tx_buffer_active = tx_buffer_primary;
    }
    NRFX_CRITICAL_SECTION_EXIT();

    // Buffer ready, start the write
    initiate_tx();
}

bool comms_packetizer::event_queue_start_rx_pending()
{
    return rx_data_ready;
}

void comms_packetizer::event_queue_start_rx_process()
{
    uint8_t crc;
    size_t  cobs_size, current_top, packet_size, not_used;

    // This function is not reentrant.
    OMMO_APP_ERROR_CHECK(rx_cobs_buffer_in_use ? NRF_ERROR_INTERNAL : NRF_SUCCESS, L(rx_cobs_buffer_in_use), L(rx_cobs_buffer_bottom), L(rx_cobs_buffer_top), L(rx_lost_packets), L(tx_lost_packets), 0);

    // Latch current end of valid data, and prevent ISR from overwriting the data
    NRFX_CRITICAL_SECTION_ENTER();
    current_top = rx_cobs_buffer_top;
    rx_cobs_buffer_in_use = current_top;
    NRFX_CRITICAL_SECTION_EXIT();

    // Attempt to find the end of a packet
    for (cobs_size = rx_cobs_buffer_bottom; cobs_size < current_top; cobs_size++)
    {
        if (rx_cobs_buffer[cobs_size] == '\0')
            break;
    }

    // Bail if this is a partial packet
    if (cobs_size >= current_top)
    {
        rx_cobs_buffer_bottom = current_top;
        rx_cobs_buffer_in_use = 0;
        return;
    }
  
    // Decode our rx cobs buffer, in-place
    if (!cobs_decode_crc(
          rx_cobs_buffer, cobs_size,
          rx_cobs_buffer, cobs_size, &crc, &not_used,
          &packet_size))
    {
        packet_size = 0;
    }

    // \0 is excluded for decoding the COBS data, but included in subsequent pointer math
    cobs_size++;

    // Prevent ISR from updating the decoded packet.  If it needs to overwrite data, anything
    // after this point is disposable.
    rx_cobs_buffer_in_use = cobs_size;

    // Is this a real packet?
    if (packet_size > 0x00)
    {
        if (crc == 0x00)
        {
            if (packet_handler)
                (*packet_handler)(rx_cobs_buffer, packet_size, packet_handler_context);
        }
        else
        {
            //Report a 0x00 length packet, indicating an invalid packet was received
            if (packet_handler)
                (*packet_handler)(nullptr, 0x00, packet_handler_context);
        }
    }

    // Shift the processed packet to the start of the cobs buffer, and release use of the buffer
    // to transport_read_complete/ISR.  Note that there may be another packet between cobs_size
    // and current_top.
    NRFX_CRITICAL_SECTION_ENTER();
    // disable_packet_reading() may have been called by the handler
    if (rx_cobs_buffer_in_use != 0)
    {
        NRFX_ASSERT(rx_cobs_buffer_top >= cobs_size);
        memmove(rx_cobs_buffer, &rx_cobs_buffer[cobs_size], rx_cobs_buffer_top - cobs_size);
        rx_cobs_buffer_top -= cobs_size;
        rx_cobs_buffer_in_use = 0;
        rx_cobs_buffer_bottom = 0;
        if (rx_cobs_buffer_top == 0)
            rx_data_ready = false;
    }
    NRFX_CRITICAL_SECTION_EXIT();
}

bool comms_packetizer::event_queue_rx_error_recovery_pending()
{
    return rx_failed;
}

void comms_packetizer::event_queue_rx_error_recovery_process()
{
    rx_failed = false;
    if (transport_device->is_connected())
        initiate_rx();
}
        
bool comms_packetizer::event_queue_tx_error_recovery_pending()
{
    return tx_failed;
}

void comms_packetizer::event_queue_tx_error_recovery_process()
{
    tx_failed = false;
    if (transport_device->is_connected())
    {
        tx_retry_count++;
        initiate_tx();
    }
    else
    {
        on_tx_complete(NRF_ERROR_INVALID_STATE);
    }
}

ret_code_t comms_packetizer::send_ack_blocking(const uint8_t return_code, const bool block)
{
    const uint8_t buffer[] = { OMMO_COMMAND_ACK, return_code };
    return send_command_packet_blocking(buffer, sizeof(buffer), block);
}

ret_code_t comms_packetizer::send_ack(const uint8_t return_code)
{
    return send_ack_blocking(return_code, false);
}

ret_code_t comms_packetizer::send_command_packet_blocking(const uint8_t * buffer, const size_t length, const bool block)
{
    uint16_t tx_write_location;
    ret_code_t err_code = NRF_SUCCESS;

    // Ignore write requests when port is not open
    if (transport_device->is_connected())
    {
        // Reserve space in buffer if USB packet fits in USB buffer
        const size_t max_possible_output_length = COBS_CMD_CRC_POST0_MAX_LEN(length);

        CRITICAL_REGION_ENTER();
        if (tx_buffer_active_top + max_possible_output_length < USB_SERIAL_TX_BUFFER_SIZE)
        {
            tx_write_location = tx_buffer_active_top;
            tx_buffer_active_top += max_possible_output_length;
        }
        else
        {
            // Buffer full
            tx_lost_packets++;
            err_code = NRF_ERROR_BUSY;
        }
        CRITICAL_REGION_EXIT();

        // If data fit into the buffer, copy into reserved buffer location
        // Note: usb event queue processing is done at the lowest priority level so this process
        // cannot be interrupted
        if (err_code == NRF_SUCCESS)
        {
            cobs_encode_usb_command_crc_post0_zero_pad(buffer, length, tx_buffer_active + tx_write_location, max_possible_output_length);

            err_code = flush_tx_buffers(block);
        }
    }
    else
    {
        tx_lost_packets++;
    }

    return err_code;
}

ret_code_t comms_packetizer::flush_tx_buffers(bool block)
{
    ret_code_t err_code = NRF_SUCCESS;

    flush_flag = true;
    if (block)
    {
        while (flush_flag || tx_buffer_transport_flags)
        {
            main_event_queue.execute_once_as_main();
        }

        err_code = tx_err_code;
    }

    return err_code;
}

ret_code_t comms_packetizer::send_command_packet(const uint8_t * buffer, size_t length)
{
    return send_command_packet_blocking(buffer, length, false);
}

ret_code_t comms_packetizer::write(const uint8_t * buffer, const uint16_t length)
{
    uint16_t tx_write_location;
    ret_code_t err_code = NRF_ERROR_BUSY;

    // Ignore write requests when port is not open
    if (transport_device->is_connected())
    {
        CRITICAL_REGION_ENTER();
        if (tx_buffer_active_top + length < USB_SERIAL_TX_BUFFER_SIZE)
        {
            // Reserve space in buffer if USB packet fits in USB buffer
            tx_write_location = tx_buffer_active_top;
            tx_buffer_active_top += length;
            err_code = NRF_SUCCESS;
        }
        else
        {
            // Buffer full
            tx_lost_packets++;
            err_code = NRF_ERROR_BUSY;
        }
        CRITICAL_REGION_EXIT();

        // If data fit into the buffer, copy into reserved buffer location
        // Note: usb event queue processing is done at the lowest priority level so this process
        // cannot be interrupted
        if (err_code == NRF_SUCCESS)
        {
            memcpy(tx_buffer_active + tx_write_location, buffer, length);
        }
    }
    else
    {
        tx_lost_packets++;
    }

    return err_code;
}

ret_code_t comms_packetizer::write_data_packet_vargslist(const size_t data_length, va_list valist)
{
    uint16_t tx_write_location;
    ret_code_t err_code = NRF_ERROR_BUSY;

    // Ignore write requests when port is not open
    if (transport_device->is_connected())
    {
        // Reserve space in buffer if USB packet fits in USB buffer
        const size_t output_length = COBS_POST0_MAX_LEN(data_length);

        CRITICAL_REGION_ENTER();
        if (tx_buffer_active_top + output_length < USB_SERIAL_TX_BUFFER_SIZE)
        {
            tx_write_location = tx_buffer_active_top;
            tx_buffer_active_top += output_length;
            if (RTC_TIMER)
            {
                VERIFY_SUCCESS(rtc_timer_reset_timeout(flush_timeout_channel));
            }
            err_code = NRF_SUCCESS;
        }
        else
        {
            // Buffer full
            tx_lost_packets++;
            err_code = NRF_ERROR_BUSY;
        }
        CRITICAL_REGION_EXIT();

        // If data fit into the buffer, copy into reserved buffer location
        // Note: usb event queue processing is done at the lowest priority level so this process
        // cannot be interrupted
        if (err_code == NRF_SUCCESS)
        {
            cobs_encode_post_zero_pad_vargs(
                  tx_buffer_active + tx_write_location,
                  output_length, valist);
        }
    }
    else
    {
        tx_lost_packets++;
    }

    return err_code;
}

ret_code_t comms_packetizer::write_now(const uint8_t * buffer, const uint16_t length, const bool block)
{
    ret_code_t err_code = NRF_SUCCESS;
    // Ignore write requests when port is not open
    if (transport_device->is_connected())
    {
        err_code = write(buffer, length);
        if (err_code == NRF_SUCCESS)
            err_code = flush_tx_buffers(block);
    }

    return err_code;
}

ret_code_t comms_packetizer::enable_flush_timeout()
{
    return rtc_timer_get_timeout_channel(&flush_timeout_channel, static_on_flush_timeout, this);
}

void comms_packetizer::enable_packet_reading()
{
    if (RTC_TIMER)
    {
        APP_ERROR_CHECK(rtc_timer_set_timeout(flush_timeout_channel, MS_TO_RTC_TIMEOUT_GEN(500)));
    }

    initiate_rx();
}

void comms_packetizer::disable_packet_reading()
{
    clear_tx_buffer();
    clear_cobs_rx_buffer();

    if (RTC_TIMER)
    {
        APP_ERROR_CHECK(rtc_timer_disable_timeout(flush_timeout_channel));
    }
}

void comms_packetizer::init(comms_transport_interface *transport, const comms_packetizer_packet_ready_handler packet_ready_callback, const void *p_context)
{
    // Save callback function, can be NULL
    transport_device = transport;
    packet_handler = packet_ready_callback;
    packet_handler_context = p_context;

    // Init buffers
    rx_cobs_buffer_in_use = 0;
    rx_cobs_buffer_top    = 0;
    rx_cobs_buffer_bottom = 0;
    in_comms_transport_rx = false;
    rx_data_ready         = false;
    rx_failed             = false;

    flush_flag             = false;
    tx_retry_count         = 0;
    in_comms_transport_tx  = false;
    tx_buffer_transport_flags  = TRANSPORT_FLAG_NONE;
    tx_buffer_base         = 0;
    tx_buffer_active_top   = 0;
    tx_buffer_active       = tx_buffer_primary;
    tx_buffer_transport    = nullptr; 
    tx_buffer_transport_end= nullptr;

    // Misc
    rx_lost_packets = 0;
    tx_lost_packets = 0;
    flush_timeout_channel = RTC_CHANNEL_UNINIT_VALUE;

    // Set transport up to call us back on rx/tx complete
    transport_device->set_callbacks(static_transport_read_complete, static_transport_write_complete, this);
}

void comms_packetizer::set_tx_header_size(uint16_t reserved_size)
{
    tx_buffer_active_top = reserved_size;
    tx_buffer_base = reserved_size;
}
