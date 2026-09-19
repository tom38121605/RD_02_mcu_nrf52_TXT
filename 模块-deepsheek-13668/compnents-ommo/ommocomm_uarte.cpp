/**
 * Copyright (c) 2019, Ommo Technologies
 *
 * All rights reserved.
 *
 *
 */

#include "nrf_gpio.h"
#include "nrf_ppi.h"
#include "nrfx_gpiote.h"

#include "ommo_config.h"
#include "dfu_container.h"
#include "ommo_app_error.h"
#include "port_master.hpp"
#include "stm32_dfu.hpp"
#include "timestamp_timer.hpp"
#include "utils.hpp"
#include "uarte_basic.hpp"

#include "ommocomm_automode.h"
#include "ommocomm_sampler_fw.h"
#include "ommocomm_uarte.hpp"
#include "event_queue_manager.hpp"
#include "trace.h"

#ifndef OMMOCOMM_IRQ_PRIORITY
#define OMMOCOMM_IRQ_PRIORITY NRFX_UARTE_DEFAULT_CONFIG_IRQ_PRIORITY
#endif

#ifdef OMMOCOMM_ENABLED

void ommocomm_uarte::static_uarte_handler(nrfx_uarte_event_t const * p_event, void * p_context)
{
    OMMO_TRACE_ISR_ENTER(TRACE_ISR_UARTE_OMMOCOMM);
    ((ommocomm_uarte *)p_context)->uarte_handler(p_event);
    OMMO_TRACE_ISR_EXIT(TRACE_ISR_UARTE_OMMOCOMM);
}

void ommocomm_uarte::static_timeout_handler(void * p_context)
{
    ((ommocomm_uarte*)p_context)->timeout_handler();
}

void ommocomm_uarte::timeout_handler()
{
    //Only execute if we haven't already entered the uarte rx function
    if(nrfx_atomic_flag_set_fetch(&timeout_executed))
        return;

    switch(current_state)
    {
        //does not start timer on tx cases below, so there will be no timeout handler
        //case OMMOCOMM_STATE_MOSI_HEADER:
        //case OMMOCOMM_STATE_MOSI_PAYLOAD:
        //case OMMOCOMM_STATE_AUTO_MODE_TX:
        default:
            OMMO_APP_ERROR_CHECK(NRF_ERROR_INTERNAL, STRING("Unexpected state in ommo comm RX handler"), 0);

        case OMMOCOMM_STATE_MOSI_HEADER_TO_PAYLOAD_DELAY:
            //TX Payload
            current_state = OMMOCOMM_STATE_MOSI_PAYLOAD;
            APP_ERROR_CHECK(uarte_tx_data(mosi_payload, mosi_header.payload_length));
            break;

        case OMMOCOMM_STATE_MISO_PAYLOAD_TO_IDLE_DELAY:
            current_state = OMMOCOMM_STATE_IDLE;
            break;

        case OMMOCOMM_STATE_AUTO_MODE_STOPPING:
            //Delay finished, send stop command
            mosi_payload[0] = OMMOCOMM_CMD_AUTOMODE_CMD_STOP;
            APP_ERROR_CHECK(uarte_tx_data(mosi_payload, 1));
            break;

        case OMMOCOMM_STATE_MISO_HEADER:
        case OMMOCOMM_STATE_MISO_PAYLOAD:
            uarte_rx_aborted = true;
            uarte_instance->rx_abort();
            miso_header->ack_code = OMMOCOMM_ACK_TIMEOUT;
            current_state = OMMOCOMM_STATE_IDLE;
            break;

        case OMMOCOMM_STATE_AUTO_MODE_RX:
            //Wait for a synch TX to be initiated (from sampler class) and to finish
            current_state = OMMOCOMM_STATE_AUTO_MODE_TX;
            uarte_rx_aborted = true;
            uarte_instance->rx_abort();

            //Execute callback, data is not available this time
            auto_seq_executing = false;
            (*auto_evt_handler)(OMMOCOMM_EVENT_TIMEOUT, auto_evt_ctx);
            break;
    }
}

void ommocomm_uarte::uarte_handler(nrfx_uarte_event_t const *p_event)
{
    if(p_event->type == NRFX_UARTE_EVT_TX_DONE)
    {
        switch(current_state)
        {
            //case OMMOCOMM_STATE_MOSI_HEADER_TO_PAYLOAD_DELAY: timeout
            //case OMMOCOMM_STATE_MISO_PAYLOAD_TO_IDLE_DELAY: timeout
            //case OMMOCOMM_STATE_MISO_HEADER: rx
            //case OMMOCOMM_STATE_MISO_PAYLOAD: rx
            //case OMMOCOMM_STATE_AUTO_MODE_RX: rx
            //case OMMOCOMM_STATE_IDLE:
            default:
                OMMO_APP_ERROR_CHECK(NRF_ERROR_INTERNAL, STRING("Unexpected state in ommo comm TX handler"), 0);

            case OMMOCOMM_STATE_MOSI_HEADER:
                if(mosi_header.payload_length > 0)
                {
                    //Delay to give coprocessor time to setup rx
                    setup_delay_state(OMMOCOMM_STATE_MOSI_HEADER_TO_PAYLOAD_DELAY);
                }
                else //No MOSI payload
                {
                    current_state = OMMOCOMM_STATE_MISO_HEADER;
                    APP_ERROR_CHECK(uarte_rx_data((uint8_t*)miso_header, sizeof(miso_header_t)));
                }
                break;

            case OMMOCOMM_STATE_MOSI_PAYLOAD:
                current_state = OMMOCOMM_STATE_MISO_HEADER;
                APP_ERROR_CHECK(uarte_rx_data((uint8_t*)miso_header, sizeof(miso_header_t)));
                break;

            case OMMOCOMM_STATE_AUTO_MODE_TX:
                if(auto_mode_payload_length > 0)
                {
                    //This needs to happen at high priority because the time between SYNCH TX and sample RX
                    // has been minimized.

                    //Finished sending automode command, now rx data
                    current_state = OMMOCOMM_STATE_AUTO_MODE_RX;
                    uarte_rx_data(miso_payload, auto_mode_payload_length);
                }
                else
                {
                    //Execute callback to let client know auto command is finished
                    auto_seq_executing = false;
                    (*auto_evt_handler)(OMMOCOMM_EVENT_SUCCESS, auto_evt_ctx);

                    //Stay in OMMOCOMM_STATE_AUTO_MODE_TX state waiting to send another automode command
                    break;
                }
                break;

            case OMMOCOMM_STATE_AUTO_MODE_STOPPING:
                    //Execute callback to let client know command sent
                    auto_seq_executing = false;
                    (*auto_evt_handler)(OMMOCOMM_EVENT_SUCCESS, auto_evt_ctx);

                    //Go back to idle state
                    current_state = OMMOCOMM_STATE_IDLE;
                    break;

        }
    }
    else if(p_event->type == NRFX_UARTE_EVT_RX_DONE)
    {
        //Ignore RX event if we aborted it part way through
        if(uarte_rx_aborted)
        {
            uarte_rx_aborted = false;
            return;
        }

        //Only execute if we haven't already entered the timeout function
        if(nrfx_atomic_flag_set_fetch(&timeout_executed))
            return;

        //Stop timeout timer
        APP_ERROR_CHECK(rtc_timer_disable_timeout(rtc_timeout_channel));

        switch(current_state)
        {
            //case OMMOCOMM_STATE_IDLE:
            //case OMMOCOMM_STATE_MOSI_HEADER: tx
            //case OMMOCOMM_STATE_MOSI_PAYLOAD: tx
            //case OMMOCOMM_STATE_MOSI_HEADER_TO_PAYLOAD_DELAY: timeout
            //case OMMOCOMM_STATE_MISO_PAYLOAD_TO_IDLE_DELAY: timeout
            //case OMMOCOMM_STATE_AUTO_MODE_TX: tx
            //case OMMOCOMM_STATE_AUTO_MODE_STOPPING: tx/timeout
            default:
                OMMO_APP_ERROR_CHECK(NRF_ERROR_INTERNAL, STRING("Unexpected state in ommo comm RX handler"), 0);

            case OMMOCOMM_STATE_MISO_HEADER:
                if(miso_header->payload_length > 0)
                {
                    uint8_t *rx_buffer = miso_payload;

                    //Check user buffer size
                    if(miso_header->payload_length > miso_payload_buffer_max_length)
                    {
                        //Flag error and rx data into trash buffer
                        miso_header->ack_code = OMMOCOMM_ACK_NO_MEM;
                        rx_buffer = mosi_payload;
                    }

                    //Start rx
                    current_state = OMMOCOMM_STATE_MISO_PAYLOAD;
                    APP_ERROR_CHECK(uarte_rx_data(rx_buffer, miso_header->payload_length));
                }
                else //No MISO payload
                {
                    //Delay to give coprocessor time to setup rx
                    setup_delay_state(OMMOCOMM_STATE_MISO_PAYLOAD_TO_IDLE_DELAY);
                }
                break;

            case OMMOCOMM_STATE_MISO_PAYLOAD:
                //Delay to give coprocessor time to setup rx
                setup_delay_state(OMMOCOMM_STATE_MISO_PAYLOAD_TO_IDLE_DELAY);
                break;

            case OMMOCOMM_STATE_AUTO_MODE_RX:
                //Wait for a synch TX to be initiated (from sampler class) and to finish
                current_state = OMMOCOMM_STATE_AUTO_MODE_TX;

                //Execute callback to let client know new data is available
                auto_seq_executing = false;
                (*auto_evt_handler)(OMMOCOMM_EVENT_SUCCESS, auto_evt_ctx);
                break;
        }
    }
    else if(p_event->type == NRFX_UARTE_EVT_ERROR)
    {
        switch(current_state)
        {
            case OMMOCOMM_STATE_IDLE:
            case OMMOCOMM_STATE_MISO_PAYLOAD_TO_IDLE_DELAY:
            case OMMOCOMM_STATE_MOSI_HEADER:
            case OMMOCOMM_STATE_MOSI_HEADER_TO_PAYLOAD_DELAY:
            case OMMOCOMM_STATE_MOSI_PAYLOAD:
            case OMMOCOMM_STATE_AUTO_MODE_TX:
                // Ignore RX errors caused by noise while in idle or TX states.
                return;
        }

        //Only execute if we haven't already entered the timeout function
        if(nrfx_atomic_flag_set_fetch(&timeout_executed))
            return;

        //Stop timeout timer
        APP_ERROR_CHECK(rtc_timer_disable_timeout(rtc_timeout_channel));

        switch(current_state)
        {
            default:
                OMMO_APP_ERROR_CHECK(NRF_ERROR_INTERNAL, STRING("Unexpected state in ommo comm uart handler"), 0);
                break;

            //need a way to test this
            //slave change baud rate when sending bk to test?
            case OMMOCOMM_STATE_MISO_PAYLOAD:
            case OMMOCOMM_STATE_MISO_HEADER:
                miso_header->ack_code = OMMOCOMM_ACK_INTERNAL_ERROR;
                //Delay to give coprocessor time to setup rx
                setup_delay_state(OMMOCOMM_STATE_MISO_PAYLOAD_TO_IDLE_DELAY);
                break;

            case OMMOCOMM_STATE_AUTO_MODE_RX:
                //Wait for a synch TX to be initiated (from sampler class) and to finish
                current_state = OMMOCOMM_STATE_AUTO_MODE_TX;
                uarte_rx_aborted = true;
                uarte_instance->rx_abort();

                //Execute callback, data is not available this time
                auto_seq_executing = false;
                (*auto_evt_handler)(OMMOCOMM_EVENT_ERROR, auto_evt_ctx);
                break;
        }
    }
}

void ommocomm_uarte::setup_delay_state(ommocomm_state_t new_state)
{
    //Delay to give coprocessor time to setup rx
    current_state = new_state;

    timeout_executed = false;
    // RTC clock resolution is ~30.5 us, yields timeout of 183-213 us
    APP_ERROR_CHECK(rtc_timer_set_timeout(rtc_timeout_channel, US_TO_RTC_TIMEOUT_GEN(214)));
}

uint32_t ommocomm_uarte::uarte_rx_data(uint8_t* data, uint16_t size)
{
    //create a rx timer
    //max delay by practice: ~20ms (ERASE_MEM)
    timeout_executed = false;

    //NOTE OMMOCOMM_CMD_ACTIVATE_MEM uses sw crc on ommocomm side, causing extended time
    uint8_t rtc_timeout_ms = 1; //make default value 1ms
    if (current_state == OMMOCOMM_STATE_MISO_HEADER &&
        (mosi_header.command == OMMOCOMM_CMD_ERASE_MEM || mosi_header.command == OMMOCOMM_CMD_WRITE_MEM ||
         mosi_header.command == OMMOCOMM_CMD_ACTIVATE_MEM))
    {
        rtc_timeout_ms = 50;
    }
    else if(current_state == OMMOCOMM_STATE_MISO_PAYLOAD)
    {
        rtc_timeout_ms = 10;
    }

    VERIFY_SUCCESS(rtc_timer_set_timeout(rtc_timeout_channel, MS_TO_RTC_TIMEOUT_GEN(rtc_timeout_ms)));

    //Make sure pullup is enabled
    if(data_pull_enable_pin != 0xFF)
        nrfx_gpiote_clr_task_trigger(data_pull_enable_pin);

    //Start the RX
    uarte_instance->switch_to_rx_only();
    VERIFY_SUCCESS(nrfx_uarte_rx(uarte_instance->get_nrfx_uarte_instance(), data, size));

    return NRF_SUCCESS;
}

uint32_t ommocomm_uarte::uarte_tx_data(uint8_t* data, uint16_t size, bool hold_xfer) const
{
    //Enable pushpull
    uarte_instance->switch_to_tx_only();

    //Deactivate pullup
    if(data_pull_enable_pin != 0xFF)
        nrfx_gpiote_set_task_trigger(data_pull_enable_pin);

    VERIFY_SUCCESS(nrfx_uarte_tx_with_hold(uarte_instance->get_nrfx_uarte_instance(), data, size, hold_xfer));

    return NRF_SUCCESS;
}

ommocomm_uarte::~ommocomm_uarte()
{
    ommocomm_uarte::release();
}

uint32_t ommocomm_uarte::send_command(mosi_header_t mosi_header_g, uint8_t *mosi_payload_g, miso_header_t *miso_header_g, uint8_t *miso_payload_g, uint8_t miso_payload_buffer_size_g)
{
    VERIFY_TRUE(is_acquired() && current_state == OMMOCOMM_STATE_IDLE, NRF_ERROR_INVALID_STATE);

    //Save mosi payload (if present)
    mosi_header = mosi_header_g;
    if(mosi_header.payload_length > 0)
        memmove(mosi_payload, mosi_payload_g, mosi_header.payload_length);

    //Save slave response buffer pointers
    miso_header = miso_header_g;
    miso_payload = miso_payload_g;
    miso_payload_buffer_max_length = miso_payload_buffer_size_g;

    //send out tx command header
    current_state = OMMOCOMM_STATE_MOSI_HEADER;
    VERIFY_SUCCESS(uarte_tx_data((uint8_t*)&mosi_header, sizeof(mosi_header_t)));

    //wait until ack comes back (with rx timeout)
    //if the command fails, current_state can be set to idle before the triggered ISR clears rx_aborted
    while(current_state != OMMOCOMM_STATE_IDLE || uarte_rx_aborted == true)
    {
        main_event_queue.execute_once_with_sleep();
    }

    return NRF_SUCCESS;
}

uint32_t ommocomm_uarte::spi_trx(uint8_t port_ss_index, const uint8_t* tx_data, uint8_t tx_len, uint8_t *rx_data, uint8_t rx_len)
{
    mosi_header_t mosi_header;
    miso_header_t miso_header;

    mosi_header.command = OMMOCOMM_CMD_SINGLE_SPI_TX_RX;
    mosi_header.addr = 0xFF;
    mosi_header.payload_length = tx_len + sizeof(ommocomm_mosi_payload_spi_trx_t);

    ommocomm_mosi_payload_spi_trx_t *single_spi_trx = (ommocomm_mosi_payload_spi_trx_t*)mosi_payload;
    single_spi_trx->spi_id = 0; //coprocessor has only 1 spi peripheral
    single_spi_trx->cs_mask = (0x01<<port_ss_index); //Switch to a mask
    single_spi_trx->rx_len = rx_len;
    memcpy(single_spi_trx->tx_data, tx_data, tx_len);

    VERIFY_SUCCESS(send_command(mosi_header, mosi_payload, &miso_header, rx_data, rx_len));

    return convert_ommo_ack_code_to_nrf_error_code(miso_header.ack_code);
}

uint32_t ommocomm_uarte::i2c_trx(uint8_t i2c_bus_id, const uint8_t* tx_data, uint8_t tx_len, uint8_t *rx_data, uint8_t rx_len)
{
    if (!tx_len)
    {
        return NRF_ERROR_INVALID_LENGTH;
    }

    mosi_header_t mosi_header =
    {
        .command = OMMOCOMM_CMD_SINGLE_I2C_TX_RX,
        .addr = 0xFF,
        .payload_length = sizeof(ommocomm_mosi_payload_i2c_trx_t)
    };

    // Decrease TX amount since we are treating first byte as address.
    mosi_header.payload_length += --tx_len;

    ommocomm_mosi_payload_i2c_trx_t *single_i2c_trx = reinterpret_cast<ommocomm_mosi_payload_i2c_trx_t*>(mosi_payload);
    single_i2c_trx->i2c_id = i2c_bus_id;
    single_i2c_trx->addr = tx_data[0];
    single_i2c_trx->rx_len = rx_len;
    memcpy(single_i2c_trx->tx_data, &tx_data[1], tx_len);

    miso_header_t miso_header;
    VERIFY_SUCCESS(send_command(mosi_header, mosi_payload, &miso_header, rx_data, rx_len));

    return convert_ommo_ack_code_to_nrf_error_code(miso_header.ack_code);
}


uint32_t ommocomm_uarte::automode_enable()
{
    VERIFY_TRUE(is_acquired() && current_state == OMMOCOMM_STATE_IDLE, NRF_ERROR_INVALID_STATE);

    //Flag auto_mode and wait for a synch TX to be initiated and to finish
    current_state = OMMOCOMM_STATE_AUTO_MODE_TX;
    return NRF_SUCCESS;
}

uint32_t ommocomm_uarte::automode_setup_synch_tx_and_hold(uint8_t sequence, uint8_t *data, uint8_t data_len, ommocomm_auto_evt_handler_t auto_seq_complete_cb, void *context)
{
    VERIFY_TRUE(is_acquired() && current_state == OMMOCOMM_STATE_AUTO_MODE_TX, NRF_ERROR_INVALID_STATE);

    //save callback func
    auto_evt_handler = auto_seq_complete_cb;
    auto_evt_ctx = context;

    //Save miso buffer
    miso_payload = data;
    auto_mode_payload_length = data_len;

    mosi_payload[0] = OMMOCOMM_CMD_AUTOMODE_CMD_SYNCH | (sequence & OMMOCOMM_AUTOMODE_COMMAND_SEQ_NUM_MASK);
    VERIFY_SUCCESS(uarte_tx_data(mosi_payload, 1, true));
    auto_seq_executing = true;

    return NRF_SUCCESS;
}

uint32_t ommocomm_uarte::automode_send_stop_tx(ommocomm_auto_evt_handler_t stop_complete_cb, void *context)
{
    VERIFY_TRUE(is_acquired() && current_state == OMMOCOMM_STATE_AUTO_MODE_TX, NRF_ERROR_INVALID_STATE);

    //save callback func
    auto_evt_handler = stop_complete_cb;
    auto_evt_ctx = context;

    //Delay to give coprocessor time to setup rx
    setup_delay_state(OMMOCOMM_STATE_AUTO_MODE_STOPPING);
    auto_seq_executing = true;

    return NRF_SUCCESS;
}

uint32_t ommocomm_uarte::automode_seq_clear(uint8_t seq_num)
{
    mosi_header_t mosi_header;
    mosi_header.command = OMMOCOMM_CMD_AUTO_SEQUENCE_CLEAR;
    mosi_header.addr = 0xFF;
    mosi_header.payload_length = 1;

    miso_header_t miso_header;

    VERIFY_SUCCESS(send_command(mosi_header, &seq_num, &miso_header, NULL, 0));

    return convert_ommo_ack_code_to_nrf_error_code(miso_header.ack_code);
}

uint32_t ommocomm_uarte::automode_seq_add(uint8_t seq_num, uint8_t *auto_command, uint8_t auto_command_len)
{
    ommocomm_mosi_payload_auto_sequence_add_t *auto_seq_payload = (ommocomm_mosi_payload_auto_sequence_add_t*)mosi_payload;
    auto_seq_payload->auto_sequence_num = seq_num;
    memcpy(auto_seq_payload->auto_command_data, auto_command, auto_command_len+1);

    mosi_header_t mosi_header;
    mosi_header.command = OMMOCOMM_CMD_AUTO_SEQUENCE_ADD;
    mosi_header.addr = 0xFF;
    mosi_header.payload_length = auto_command_len + 1;

    miso_header_t miso_header;

    VERIFY_SUCCESS(send_command(mosi_header, mosi_payload, &miso_header, NULL, 0));

    return convert_ommo_ack_code_to_nrf_error_code(miso_header.ack_code);
}

uint32_t ommocomm_uarte::get_synch_to_spi_tx_delay(uint32_t *delay)
{
    mosi_header_t mosi_header;
    mosi_header.command = OMMOCOMM_CMD_GET_SYCH_TO_SPI_DELAY;
    mosi_header.addr = 0xFF;
    mosi_header.payload_length = 0;

    miso_header_t miso_header;
    miso_header.ack_code = (ommocomm_ack_t)0x55;

    VERIFY_SUCCESS(send_command(mosi_header, NULL, &miso_header, (uint8_t*)delay, 4));

    return convert_ommo_ack_code_to_nrf_error_code(miso_header.ack_code);
}

uint32_t ommocomm_uarte::get_version(uint16_t *fw_version, ommocomm_device_type_t *device_type, uint16_t *device_id)
{
    ommocomm_miso_payload_get_version_t payload;

    mosi_header_t mosi_header;
    mosi_header.command = OMMOCOMM_CMD_GET_VERSION;
    mosi_header.addr = 0xFF;
    mosi_header.payload_length = 0;

    miso_header_t miso_header;

    VERIFY_SUCCESS(send_command(mosi_header, NULL, &miso_header, (uint8_t*)&payload, sizeof(ommocomm_miso_payload_get_version_t)));
    *fw_version = payload.fw_version;
    *device_type = (ommocomm_device_type_t)payload.device_type;
    *device_id = payload.device_id;
    return convert_ommo_ack_code_to_nrf_error_code(miso_header.ack_code);
}

uint32_t ommocomm_uarte::get_wai(uint8_t *wai)
{
    mosi_header_t mosi_header;
    mosi_header.command = OMMOCOMM_CMD_WHO_AM_I;
    mosi_header.addr = 0xFF;
    mosi_header.payload_length = 0;

    miso_header_t miso_header;

    VERIFY_SUCCESS(send_command(mosi_header, NULL, &miso_header, wai, 1));

    return convert_ommo_ack_code_to_nrf_error_code(miso_header.ack_code);
}

uint32_t ommocomm_uarte::enter_bootloader_mode()
{
    mosi_header_t mosi_header;
    mosi_header.command        = OMMOCOMM_CMD_BOOTLOADER;
    mosi_header.addr           = 0xFF;
    mosi_header.payload_length = 0;

    miso_header_t miso_header;

    VERIFY_SUCCESS(send_command(mosi_header, NULL, &miso_header, NULL, 0));

    return convert_ommo_ack_code_to_nrf_error_code(miso_header.ack_code);
}

uint32_t ommocomm_uarte::check_fw_and_update(prog_stage_cb_t progress_callback, void (*wdt_feed_ptr)())
{
    // Check for slave present and make sure coprocessor firmware is correct
    // Grab version number from ommocomm interface
    uint16_t ommocomm_uarte_version;
    ommocomm_device_type_t ommocomm_uarte_device_type;
    uint16_t ommocomm_uarte_device_id;
    bool needs_update = false;

    ret_code_t result = get_version(&ommocomm_uarte_version, &ommocomm_uarte_device_type, &ommocomm_uarte_device_id);
    if (result == NRF_SUCCESS)
    {
        if (ommocomm_uarte_device_type == OMMOCOMM_DEVICE_TYPE_SENSOR_SAMPLER)
        {
            // Check firmware (We can downgrade in case if version greater than 3)
            needs_update = ommocomm_uarte_device_id == ommocomm_sampler_fw.device_id &&
                           (ommocomm_uarte_version < ommocomm_sampler_fw.version || (ommocomm_uarte_version != ommocomm_sampler_fw.version && ommocomm_uarte_version > 3));

            if (needs_update)
            {
                // Reboot into bootloader mode
                enter_bootloader_mode();
                nrf_delay_ms(20);  // Wait for boot loader to start up
            }
        }
    }
    else //Device "could" be in bootloader mode already
    {
        nrf_delay_ms(20);  // Wait for boot loader to flush the incomplete "get version" packet
        needs_update = true;
    }

    if (needs_update)
    {
        // Initiate bootloader mode, this will break our uarte instance
        stm32_dfu fw_updater((nrfx_uarte_t*)uarte_instance->get_nrfx_uarte_instance(), (uint8_t)uart_config.pselrxd, wdt_feed_ptr);
        result = fw_updater.initiate(5);  // Each attempt is >20ms, should only require 1 attempt

        // Attempt the update
        if (result == NRF_SUCCESS)
        {
            result = fw_updater.fw_update_encrypted(&ommocomm_sampler_fw, progress_callback);

            // Give time for device to reboot
            if (result == NRF_SUCCESS)
            {
                nrf_delay_ms(OMMOCOMM_REBOOT_TIME);
            }
        }

        // Switch our uarte_basic back to DIRECT mode
        uarte_instance->reconfigure(&uart_config, static_uarte_handler, this);
    }

    return result;
}

ret_code_t ommocomm_uarte::init()
{
    return NRF_SUCCESS;
}

ret_code_t ommocomm_uarte::acquire(uarte_basic *uarte, ommocomm_acqmode_t mode, uint32_t uarte_trx_pin_g, uint8_t cs_index_twi_location, uint8_t data_pull_enable_pin_g)
{
    ret_code_t rvalue;

    VERIFY_TRUE(acquired_mode == OMMOCOMM_ACQMODE_IDLE, NRF_ERROR_INVALID_STATE);
    VERIFY_TRUE(current_state == OMMOCOMM_STATE_IDLE, NRF_ERROR_INVALID_STATE);
    VERIFY_TRUE(mode != OMMOCOMM_ACQMODE_IDLE, NRF_ERROR_INVALID_PARAM);

    //Save connection status
    uarte_instance = uarte;
    acquired_ss_index_bus_location = cs_index_twi_location;
    data_pull_enable_pin = data_pull_enable_pin_g;

    //Configure uarte
    uart_config.pseltxd            = OMMOCOMM_UNUSED_PIN,
    uart_config.pselrxd            = uarte_trx_pin_g,
    uart_config.hwfc               = NRF_UARTE_HWFC_DISABLED,
    uart_config.parity             = NRF_UARTE_PARITY_EXCLUDED,
    uart_config.baudrate           = NRF_UARTE_BAUDRATE_1000000,
    uart_config.interrupt_priority = OMMOCOMM_IRQ_PRIORITY;
    rvalue = uarte_instance->acquire(&uart_config, static_uarte_handler, this);
    if (rvalue != NRF_SUCCESS)
        return rvalue;

    //Grab our timeout channel
    APP_ERROR_CHECK(rtc_timer_get_timeout_channel(&rtc_timeout_channel, static_timeout_handler, this));

    //Flag as acquired
    acquired_mode = mode;

    //Check for a pull up enable line
    if(data_pull_enable_pin != 0xFF)
    {
        if (!nrfx_gpiote_is_init())
        {
            APP_ERROR_CHECK(nrfx_gpiote_init());
        }

        nrfx_gpiote_out_config_t gpiote_config_out = NRFX_GPIOTE_CONFIG_OUT_TASK_HIGH; //pull enabled
        APP_ERROR_CHECK(nrfx_gpiote_out_init(data_pull_enable_pin, &gpiote_config_out));
        nrfx_gpiote_out_task_enable(data_pull_enable_pin);

        APP_ERROR_CHECK(nrfx_ppi_channel_alloc(&pull_disable_ppi));
        APP_ERROR_CHECK(nrfx_ppi_channel_alloc(&pull_enable_ppi));

        //Deactivate pullup after first byte is complete and reactivate after ENDRX
        nrf_ppi_channel_endpoint_setup(pull_disable_ppi, nrfx_uarte_event_address_get(uarte_instance->get_nrfx_uarte_instance(), NRF_UARTE_EVENT_RXDRDY), nrfx_gpiote_set_task_addr_get(data_pull_enable_pin));
        nrf_ppi_channel_endpoint_setup(pull_enable_ppi, nrfx_uarte_event_address_get(uarte_instance->get_nrfx_uarte_instance(), NRF_UARTE_EVENT_ENDRX), nrfx_gpiote_clr_task_addr_get(data_pull_enable_pin));

        APP_ERROR_CHECK(nrfx_ppi_channel_enable(pull_disable_ppi));
        APP_ERROR_CHECK(nrfx_ppi_channel_enable(pull_enable_ppi));
    }

    // TODO : Delay to allow the UARTE to settle
    ommo_delay_ms(10);

    return NRF_SUCCESS;
}

uint32_t ommocomm_uarte::get_start_task() const
{
    VERIFY_TRUE(acquired_mode != OMMOCOMM_ACQMODE_IDLE, NRF_ERROR_INVALID_STATE);
    return uarte_instance->get_starttx_task();
}

ret_code_t ommocomm_uarte::release()
{
    // Check if we are in the right state
    VERIFY_TRUE(acquired_mode != OMMOCOMM_ACQMODE_IDLE, NRF_ERROR_INVALID_STATE);

    //Stop the timeout timer and release channel
    APP_ERROR_CHECK(rtc_timer_disable_timeout(rtc_timeout_channel));
    rtc_timer_release_timeout_channel(rtc_timeout_channel);

    //Uninit uarte
    uarte_instance->release();
    uarte_instance = nullptr;
    current_state = OMMOCOMM_STATE_IDLE;
    acquired_mode = OMMOCOMM_ACQMODE_IDLE;

    //Check for a pull up enable line
    if(data_pull_enable_pin != 0xFF)
    {
        APP_ERROR_CHECK(nrfx_ppi_channel_free(pull_enable_ppi));
        APP_ERROR_CHECK(nrfx_ppi_channel_free(pull_disable_ppi));
        nrfx_gpiote_out_uninit(data_pull_enable_pin);
        data_pull_enable_pin = 0xFF;
    }

    return NRF_SUCCESS;
}

ret_code_t ommocomm_uarte::trx(const uint8_t * p_tx_buffer, size_t tx_length, uint8_t * p_rx_buffer, size_t rx_length, comm_device_trx_flags_t flags)
{
    if (flags == COMM_DEVICE_FLAGS_NONE)
    {
        switch (acquired_mode)
        {
            case OMMOCOMM_ACQMODE_IDLE:
                return NRF_ERROR_INVALID_STATE;

            case OMMOCOMM_ACQMODE_SPI:
                return spi_trx(acquired_ss_index_bus_location, p_tx_buffer, tx_length, p_rx_buffer, rx_length);

            case OMMOCOMM_ACQMODE_TWI:
                return i2c_trx(acquired_ss_index_bus_location, p_tx_buffer, tx_length, p_rx_buffer, rx_length);

            default:
                break;
        }
    }

    return NRF_ERROR_NOT_SUPPORTED;
}

ret_code_t ommocomm_uarte::trx(const uint8_t *p_tx_buffer, size_t tx_length, uint8_t *p_rx_buffer, size_t rx_length, comm_device_event_handler_t handler, void * p_context,
                       comm_device_trx_flags_t flags)
{
   ret_code_t ret = trx(p_tx_buffer, tx_length, p_rx_buffer, rx_length, flags);

   if (handler != nullptr)
   {
       handler(this, ret == NRF_SUCCESS ? COMM_DEVICE_TRX_RESULT_SUCCESS : COMM_DEVICE_TRX_RESULT_ERROR, p_context);
   }
   return ret;
}

[[nodiscard]] bool ommocomm_uarte::is_acquired() const
{
    return acquired_mode != OMMOCOMM_ACQMODE_IDLE;
}

[[nodiscard]] bool ommocomm_uarte::is_trx_completed() const
{
    return auto_seq_executing==false;
}

#endif // OMMOCOMM_STATE_IDLE