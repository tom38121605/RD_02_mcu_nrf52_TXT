/**
 * Copyright (c) 2019, Ommo Technologies
 *
 * All rights reserved.
 *
 *
 */

#include <string.h>
#include <inttypes.h>

#include "nrf.h"
#include "nrf_drv_clock.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_nvmc.h"

#include "main.hpp"
#include "ommo_config.h"
#include "ommo_fw.pb.h"
#include "utils.hpp"
#include "ommo_app_error.h"
#include "usb_serial.hpp"
#include "nrf_power.h"
#include "git_describe.h"
#include "ommo_errors.h"
#include "timestamp_timer.hpp"

#ifdef OMMO_APP_DFU_COMMANDS_ENABLED
#include "ommo_app_dfu_results.hpp"
#include "ommo_app_dfu_writer.hpp"
#endif
#ifdef OMMOCOMM_1WIRE
#include "comms_uarte_siu_1wire.hpp"
#endif
#ifdef OMMO_BOOTLOADER_PRESENT
#include "nrf_bootloader_info.h"
#include "ommo_app_dfu_common.hpp"
#endif

// Ensure that the largest possible crash report can be transmitted
STATIC_ASSERT(PACKET_RESPONSE_BUFFER_SIZE >= ALIGN_NUM(1024, 1 + OMMO_LOG_BUF_SIZE));

extern uint32_t __vectors_start__;

static bool in_crash_handler = false;

__WEAK void advanced_wdt_feed(void)
{
    // do nothing
}

__WEAK ret_code_t disrupt_realtime_operation(uint16_t timeout_ms)
{
    return NRF_SUCCESS;
}

__WEAK void resume_realtime_operation()
{
    // do nothing
}

__WEAK bool realtime_operation_requested()
{
    return false;
}

#ifdef OMMO_BOOTLOADER_PRESENT

static const uint32_t *static_vectors_ptr = &__vectors_start__;

#pragma GCC push_options
#pragma GCC optimize("O0")

bool bootloader_is_programmed()
{
    return (size_t)static_vectors_ptr != 0;
}

#pragma GCC pop_options

void reboot_into_bootloader()
{
    //Disable data line disabled (this is sometimes used to keep power on)
    #ifdef OMMO_DATA_EN_PIN
    nrf_gpio_pin_write(OMMO_DATA_EN_PIN, 0);
    nrf_delay_ms(1);
    #endif

    //Feed the dog before reset
    advanced_wdt_feed();

    //Reboot into bootloader mode
    nrf_power_gpregret_set(BOOTLOADER_DFU_START);
    NVIC_SystemReset();
    //Unreachable
}

#endif // OMMO_BOOTLOADER_PRESENT

void get_firmare_versions(uint32_t *bootloader_version_ptr, uint32_t *application_version_ptr)
{
#ifdef OMMO_BOOTLOADER_PRESENT
    if(bootloader_is_programmed())
    {
        if(bootloader_version_ptr != nullptr)
            *bootloader_version_ptr = ((nrf_dfu_settings_t*)dfu_settings)->bootloader_version;
        if(application_version_ptr != nullptr)
            *application_version_ptr = ((nrf_dfu_settings_t*)dfu_settings)->app_version;
    }
    else
#endif
    {
        if(bootloader_version_ptr != nullptr)
            *bootloader_version_ptr = 12345; //This BL version number is used to detect direct programmed units
        if(application_version_ptr != nullptr)
            *application_version_ptr = GIT_DESCRIBE_VER;
    }
}


#pragma GCC diagnostic push
//#pragma GCC diagnostic ignored "-Wanalyzer-infinite-loop"
#pragma diagnostic ignored "-Wanalyzer-infinite-loop"
void infinite_loop()
{
    for(;;);
}

#pragma GCC diagnostic pop

uint16_t general_process_packet_received(uint8_t data[], uint16_t packet_size, uint8_t response_buffer[], uint16_t response_buffer_size)
{
    uint16_t response_length = 0;

    switch(data[0])
    {
#ifdef OMMO_BOOTLOADER_PRESENT
        case OMMO_COMMAND_BOOTLOAD:
            if(packet_size == 1)
            {
                if(bootloader_is_programmed())
                {
                    reboot_into_bootloader();  // does not return
                }
                else
                {
                    return fill_in_ack_packet(response_buffer, OMMO_ACK_INVALID_MODE);
                }
            }
            else
            {
                return fill_in_ack_packet(response_buffer, OMMO_ACK_INVALID_DATA);
            }
            break;

        case OMMO_COMMAND_FW_VERSION:
            if(packet_size == 1)
            {
                response_buffer[0] = OMMO_COMMAND_FW_VERSION;
                get_firmare_versions((uint32_t*)(response_buffer+1), (uint32_t*)(response_buffer+5));
                return 9;
            }
            else
            {
                return fill_in_ack_packet(response_buffer, OMMO_ACK_INVALID_DATA);
            }
            break;
#endif
        case OMMO_COMMAND_IS_CRASHED:
            if(packet_size == 1)
            {
                response_buffer[0] = OMMO_COMMAND_IS_CRASHED;
                response_buffer[1] = in_crash_handler;
                return 2;
            }
            else
            {
                return fill_in_ack_packet(response_buffer, OMMO_ACK_INVALID_DATA);
            }
            break;

        case OMMO_COMMAND_GET_UUID:
            if(packet_size == 1)
            {
                response_buffer[0] = OMMO_COMMAND_GET_UUID;
                memcpy(response_buffer+1, (const void *)&NRF_FICR->DEVICEID[0], 4);
                return 5;
            }
            else
            {
                return fill_in_ack_packet(response_buffer, OMMO_ACK_INVALID_DATA);
            }
            break;

        case OMMO_COMMAND_PING:
            if(packet_size < response_buffer_size)
            {
                response_buffer[0] = OMMO_COMMAND_PING;
                memcpy(response_buffer+1, data+1, packet_size-1);
                return packet_size;
            }
            else
            {
                return fill_in_ack_packet(response_buffer, OMMO_ACK_INVALID_DATA);
            }
            break;

#ifdef OMMO_TIMESTAMP_SYNCH_IN
        case OMMO_COMMAND_SET_ESB_ENABLED:
            if(packet_size == 2)
            {
                timestamp_set_esb_synch_enabled(data[1]);
                return fill_in_ack_packet(response_buffer, OMMO_ACK_SUCCESS);
            }
            else
            {
                return fill_in_ack_packet(response_buffer, OMMO_ACK_INVALID_DATA);
            }
            break;
#endif

#ifndef OMMO_CRASH_REPORT_DISABLED
        case OMMO_COMMAND_GET_CRASH_REPORT:
            size_t resp_buf_length;
            if(packet_size == 1)
            {
                response_buffer[0] = OMMO_COMMAND_GET_CRASH_REPORT;
                ret_code_t err_code = ommo_app_error_flash_read_decode(response_buffer + 1, response_buffer_size - 1, &resp_buf_length);
                if (err_code != NRF_SUCCESS)
                {
                    return fill_in_ack_packet(response_buffer, OMMO_ACK_NOT_FOUND);
                }
                else
                {
                    return (resp_buf_length + 1);
                }
            }
            else
            {
                return fill_in_ack_packet(response_buffer, OMMO_ACK_INVALID_DATA);
            }
            break;

        case OMMO_COMMAND_CLEAR_CRASH_REPORT:
            if(packet_size == 1)
            {
                nrf_nvmc_page_erase((uint32_t)dfu_crash_report_buffer);
                return fill_in_ack_packet(response_buffer, OMMO_ACK_SUCCESS);
            }
            else
            {
                return fill_in_ack_packet(response_buffer, OMMO_ACK_INVALID_DATA);
            }
            break;
#endif

        case OMMO_COMMAND_FW_VERSION_STRING:
            if(packet_size == 1)
            {
                const char git_describe[] = GIT_DESCRIBE_STR;
                response_buffer[0] = OMMO_COMMAND_FW_VERSION_STRING;
                memcpy(response_buffer+1, git_describe, sizeof(git_describe));
                return (1 + sizeof(git_describe));  // includes \0 at end of git_describe
            }
            else
            {
                return fill_in_ack_packet(response_buffer, OMMO_ACK_INVALID_DATA);
            }
            break;

#ifndef OMMO_USB_DISABLED
        case OMMO_COMMAND_SIU_COMMS_INFO:
            if(packet_size == 1)
            {
                const char usb_string[] = APP_USBD_STRINGS_PRODUCT;
                uint16_t pid = APP_USBD_PID;
                response_buffer[0] = OMMO_COMMAND_SIU_COMMS_INFO;
                response_buffer[1] = OMMO_USB_COMMUNICATION_CLASS;
                memcpy(response_buffer+2, &pid, sizeof(pid));
                memcpy(response_buffer+4, usb_string, sizeof(usb_string));
                return (3 + sizeof(usb_string));  // excludes \0 at end of usb_string
            }
            else
            {
                return fill_in_ack_packet(response_buffer, OMMO_ACK_INVALID_DATA);
            }
            break;
#endif

#ifdef OMMO_APP_DFU_COMMANDS_ENABLED
        case OMMO_COMMAND_APP_DFU_ERASE:
            if (packet_size != 1)
            {
                return fill_in_ack_packet(response_buffer, OMMO_ACK_INVALID_DATA);
            }
            else
            {
                ret_code_t result = ommo_app_dfu_erase();
                return fill_in_ack_packet(response_buffer, convert_nrf_error_code_to_ommo_ack_code(result));
            }
            break;
        
        case OMMO_COMMAND_APP_DFU_WRITE:
            if (packet_size < 6)
            {
                return fill_in_ack_packet(response_buffer, OMMO_ACK_INVALID_DATA);
            }
            else
            {
                uint32_t offset;
                memcpy(&offset, &data[1], sizeof(offset));
                ret_code_t result = ommo_app_dfu_write(offset, &data[5], packet_size - 5);
                if (result == NRF_SUCCESS)
                {
                    response_buffer[0] = OMMO_COMMAND_APP_DFU_WRITE;
                    return 1;
                }
                else
                {
                    return fill_in_ack_packet(response_buffer, convert_nrf_error_code_to_ommo_ack_code(result));
                }
            }
            break;

        case OMMO_COMMAND_APP_DFU_VERIFY:
            if (packet_size != 5)
            {
                return fill_in_ack_packet(response_buffer, OMMO_ACK_INVALID_DATA);
            }
            else
            {
                uint32_t crc;
                memcpy(&crc, &data[1], sizeof(crc));
                ommo_app_dfu_verify_results_t results = {0};
                ret_code_t result = ommo_app_dfu_verify(crc, &results);
                if (result == NRF_SUCCESS)
                {
                    copyUint8(response_buffer, response_length, OMMO_COMMAND_APP_DFU_VERIFY);
                    copyUint32_LE(response_buffer, response_length, results.previous_bootloader_version);
                    copyUint32_LE(response_buffer, response_length, results.new_bootloader_version);
                    copyUint32_LE(response_buffer, response_length, results.previous_application_version);
                    copyUint32_LE(response_buffer, response_length, results.new_application_version);
                    return response_length;
                }
                else
                {
                    return fill_in_ack_packet(response_buffer, convert_nrf_error_code_to_ommo_ack_code(result));
                }
            }
            break;

        case OMMO_COMMAND_APP_DFU_STATUS:
            {
                ommo_app_dfu_results_t dfu_results;
                ret_code_t result = ommo_app_dfu_validate_results(&dfu_results, [](uint32_t) { ::advanced_wdt_feed(); });
                if (result == NRF_SUCCESS)
                {
                    if (dfu_results.signature == 0)
                    {
                        // no DFU installed since last full flash erase
                        response_buffer[0] = OMMO_COMMAND_APP_DFU_STATUS;
                        return 1;
                    }
                    else
                    {
                        ommo_app_dfu_storage_location_results_t *storage_results = ommo_app_dfu_get_last_dfu_results(&dfu_results);
                        copyUint8(response_buffer, response_length, OMMO_COMMAND_APP_DFU_STATUS);
                        copyUint32_LE(response_buffer, response_length, storage_results->dfu_crc);
                        copyUint32_LE(response_buffer, response_length, storage_results->fault_id);
                        return response_length;
                    }
                }
                else
                {
                    return fill_in_ack_packet(response_buffer, convert_nrf_error_code_to_ommo_ack_code(result));
                }
            }
            break;
        
        case OMMO_COMMAND_APP_DFU_INSTALL:
            if (packet_size != 5)
            {
                return fill_in_ack_packet(response_buffer, OMMO_ACK_INVALID_DATA);
            }
#ifdef OMMO_BOOTLOADER_PRESENT
            else if(!bootloader_is_programmed())
            {
                return fill_in_ack_packet(response_buffer, OMMO_ACK_INVALID_MODE);
            }
#endif
            else
            {
                uint32_t crc;
                memcpy(&crc, &data[1], sizeof(crc));

                bool pending;
                ret_code_t result = ommo_app_dfu_check_pending(crc, &pending, [](uint32_t) { ::advanced_wdt_feed(); });
                if (result == NRF_SUCCESS)
                {
                    if (pending)
                        reboot_into_bootloader();  // does not return
                    response_buffer[0] = OMMO_COMMAND_APP_DFU_INSTALL;
                    return 1;
                }
                else
                {
                    return fill_in_ack_packet(response_buffer, convert_nrf_error_code_to_ommo_ack_code(result));
                }
            }
            break;
#endif // OMMO_APP_DFU_COMMANDS_ENABLED

#ifdef OMMO_USB_SOF_TIMESTAMP_CAPTURE
        case OMMO_COMMAND_USB_GET_SOF_FRAME_ID_AND_TIMESTAMP:
            if(packet_size != 1)
            {
                return fill_in_ack_packet(response_buffer, OMMO_ACK_INVALID_DATA);
            }
            else
            {
                copyUint8(response_buffer, response_length, OMMO_COMMAND_USB_GET_SOF_FRAME_ID_AND_TIMESTAMP);
                uint16_t frame;
                uint32_t timestamp;
                usbd_main_get_last_sof_frame_id_and_timestamp(&frame, &timestamp);
                copyUint16_LE(response_buffer, response_length, frame);
                copyUint32_LE(response_buffer, response_length, timestamp);
                return response_length;
            }
#endif

        case OMMO_COMMAND_TEST_CRASH:
            if(packet_size == 2)
            {
                switch(data[1])
                {
                    case TEST_CRASH_MODE_BUS_FAULT:
                        *(uint32_t*)(0xdead0000) = 0xdead;
                        break;

                    case TEST_CRASH_MODE_WATCHDOG_TIMEOUT:
                        // WDT is not running while in the crash handler
                        if(in_crash_handler)
                            return fill_in_ack_packet(response_buffer, OMMO_ACK_INVALID_MODE);
                        infinite_loop();
                        break;

                    case TEST_CRASH_MODE_API_ERROR_1:
                        APP_ERROR_CHECK(NRF_ERROR_INTERNAL);
                        break;

                    case TEST_CRASH_MODE_API_ERROR_2:
                        uint16_t test_uint16_array[] = { 1, 2 };
                        uint32_t test_uint32_array[] = { 3, 4 };
                        int8_t test_int8_array[] = { 5, 6 };
                        int16_t test_int16_array[] = { 7, 8 };
                        int32_t test_int32_array[] = { 9, 10, 11 };
                        // TODO not all possible combinations could be tested in a single error
                        // when OMMO_LOG_BUF_SIZE was only 1024, but that has been increased to a
                        // full page now.  The types tested here exercise unique encoding cases.
                        OMMO_APP_ERROR_CHECK(NRF_ERROR_INTERNAL, L(true),
                                                                 L('{'), L((unsigned char)'{'), // L((signed char)'{'),
                                                                 L((short)12345), // L((unsigned short)12345),
                                                                 L((int)1234567890), // L(1234567890U),
                                                                 L(1234567890L), // L(1234567890UL),
                                                                 L(0x123456789abcdef0LL), // L(0x123456789abcdef0ULL),
                                                                 L(123.456f), L(123.456), L((long double)123.456),
                                                                 STRING("test"),
                                                                 L2(test_uint16_array, 2),
                                                                 CRASH_DATA_TYPE_INTERNAL_TEST,
                                                                 L2(test_uint32_array, 2),
                                                                 L2(test_int8_array, 2),
                                                                 L2(test_int16_array, 2),
                                                                 L2(test_int32_array, 3),
                                                                 0);
                        break;
                }
            }

            // Should not get here
            return fill_in_ack_packet(response_buffer, OMMO_ACK_INVALID_DATA);
    }

    //No one handled it
    return 0;
}

#ifndef NO_USB_SERIAL

static bool packet_received = false;

#define CRASH_HANDLER_TX_BUFFER_SIZE ALIGN_NUM(1024, OMMO_LOG_BUF_SIZE)

void usb_serial_general_process_and_respond(uint8_t data[], uint16_t length, const void *p_context)
{
    uint16_t response_length = 0;
    uint8_t response_buffer[CRASH_HANDLER_TX_BUFFER_SIZE];

    packet_received = true;

    //Check for invalid packets
    if(length == 0x00)
        response_length = fill_in_ack_packet(response_buffer, OMMO_ACK_INVALID_DATA);

    //Try general processor
    if(response_length == 0) 
        response_length = general_process_packet_received(data, length, response_buffer, sizeof(response_buffer));

    if(response_length == 0) //No response, must be an unknown command
        response_length = fill_in_ack_packet(response_buffer, OMMO_ACK_UNKNOWN_COMMAND);

    usb_serial_send_command_packet(response_buffer, response_length);
}

#ifdef OMMOCOMM_1WIRE

void crash_handler_1wire_packet_received(uint8_t *buffer, uint16_t length, const void *context)
{
    uint16_t response_length = 0;
    uint8_t response_buffer[CRASH_HANDLER_TX_BUFFER_SIZE];

    packet_received = true;

    //Check for invalid packets
    if(length == 0x00)
        response_length = fill_in_ack_packet(response_buffer, OMMO_ACK_INVALID_DATA);

    // The 1-wire interface is intended to emulate a USB connection.  The device is connected to an
    // adapter with a USB CDC instance that simply relays USB packets over 1-wire to the device.
    if(response_length == 0) 
        response_length = general_process_packet_received(buffer, length, response_buffer, sizeof(response_buffer));

    if(response_length == 0) //No response, must be an unknown command
        response_length = fill_in_ack_packet(response_buffer, OMMO_ACK_UNKNOWN_COMMAND);

    ((comms_uarte_siu_1wire*)context)->send_command_packet_async(response_buffer, response_length);
}

#endif // OMMOCOMM_1WIRE

void crash_reboot_check()
{
#ifdef OMMO_POWER_ON_PIN
    uint32_t power_off_delay_ms = 0;
    nrf_gpio_pin_write(OMMO_POWER_ON_PIN, 1);
    nrf_gpio_cfg_output(OMMO_POWER_ON_PIN);
#endif

#ifdef OMMO_DATA_EN_PIN
    nrf_gpio_pin_write(OMMO_DATA_EN_PIN, 1);
    //ATTENTION Needs strong drive for higher current load
    nrf_gpio_cfg(OMMO_DATA_EN_PIN, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_H0H1, NRF_GPIO_PIN_NOSENSE);
#endif

    if(nrf_power_gpregret_get() == FIRMWARE_CRASH_REBOOT)
    {
        //Set flag so that we can return crash state
        in_crash_handler = true;

        // Clear fault handler mark in GPREGRET register.
        nrf_power_gpregret_set(nrf_power_gpregret_get() & ~FIRMWARE_CRASH_REBOOT);

#ifdef OMMO_CRASH_OUTPUT_PULSE_PIN
        nrf_gpio_pin_set(OMMO_CRASH_OUTPUT_PULSE_PIN);
        nrf_gpio_cfg_output(OMMO_CRASH_OUTPUT_PULSE_PIN);
        nrf_delay_ms(2);
        nrf_gpio_pin_clear(OMMO_CRASH_OUTPUT_PULSE_PIN);
#endif

#ifdef OMMO_CRASH_OUTPUT_LOW_PIN
        nrf_gpio_pin_clear(OMMO_CRASH_OUTPUT_LOW_PIN);
        nrf_gpio_cfg_output(OMMO_CRASH_OUTPUT_LOW_PIN);
#endif

        //Init USB serial
        usbd_main_init();
        usb_serial_init(usb_serial_general_process_and_respond, nullptr);
        usbd_main_start();

#ifdef OMMOCOMM_1WIRE
        APP_TIMER_DEF(comms_1wire_timer_timer_id);
        nrfx_timer_t comms_1wire_timer = OMMOCOMM_1WIRE_TIMER;
        comms_1wire.init(&comms_1wire_timer, &comms_1wire_timer_timer_id, &crash_handler_1wire_packet_received, &comms_1wire, OMMOCOMM_UARTE_DATA_PIN, 1);
#endif

#ifndef OMMO_NO_LED
        change_led_state(APP_STATE_CRASH);
#endif

        //Clear and fill in our main event queue
        main_event_queue.reset();
        APP_ERROR_CHECK(main_event_queue.register_task(advanced_wdt_feed)); //Feed the watchdog
        APP_ERROR_CHECK(usb_serial_add_tasks_to_event_queue(&main_event_queue));
#ifdef OMMOCOMM_1WIRE
        APP_ERROR_CHECK(comms_1wire.add_tasks_to_event_queue(&main_event_queue));
#endif

        //Loop forever
        while(true)
        {
#ifdef OMMO_POWER_ON_PIN
            if(packet_received)
            {
                packet_received = false;
                power_off_delay_ms = 0;
            }

            // Power off at least 10 seconds after last USB or 1-wire message.  With all of the
            // other logic/interrupts during the loop, this delay is typically 30-40 seconds.
            nrf_delay_us(1000);
            power_off_delay_ms++;
            if(power_off_delay_ms > 10000)
            {
                // This should have no effect on production units.  This is helpful for development
                // units that may not actually be able to turn off, to signal that it is no longer
                // possible to connect to the crash handler.
#ifndef OMMO_NO_LED
                change_led_state(APP_STATE_POWER_DOWN);
#endif

                nrf_gpio_pin_write(OMMO_POWER_ON_PIN, 0);
                #ifdef OMMO_DATA_EN_PIN
                  nrf_gpio_pin_write(OMMO_DATA_EN_PIN, 0);
                #endif

                //Wait for death
                while(true)
                {
                    advanced_wdt_feed();
                    __WFI();
                }
            }
#endif // OMMO_POWER_ON_PIN

            //Execute event queue items
            main_event_queue.execute_once_with_sleep();
        }
        //Unreachable
    }
}

#endif // !NO_USB_SERIAL

void clocks_start()
{
    uint16_t i;

    //Init clocks
    OMMO_APP_ERROR_CHECK(nrf_drv_clock_init(), STRING("Unable to start clock"), 0);
    nrf_drv_clock_lfclk_request(NULL);
    for(i=0; i<500 && !nrf_drv_clock_lfclk_is_running(); i++) nrf_delay_ms(1);
    if(!nrf_drv_clock_lfclk_is_running()) OMMO_APP_ERROR_CHECK(NRF_ERROR_TIMEOUT, STRING("Unable to start clock"), 0);
    nrf_drv_clock_hfclk_request(NULL);
    for(i=0; i<500 && !nrf_drv_clock_hfclk_is_running(); i++) nrf_delay_ms(1);
    if(!nrf_drv_clock_hfclk_is_running()) OMMO_APP_ERROR_CHECK(NRF_ERROR_TIMEOUT, STRING("Unable to start clock"), 0);
}

bool is_in_array(uint32_t val, uint32_t *array, uint16_t array_len)
{
    for(uint16_t i=0; i<array_len; i++)
        if(array[i] == val)
            return true;

    return false;
}

bool is_in_array(uint8_t val, uint8_t *array, uint16_t array_len)
{
    for(uint16_t i=0; i<array_len; i++)
        if(array[i] == val)
            return true;

    return false;
}

uint16_t remove_duplicates(uint8_t *array, uint16_t array_len)
{
    uint16_t i, j, output_array_len = 1;

    for(i=1; i<array_len; i++) //Next element index
    {
        for(j=0; j<output_array_len; j++) //Existing element index
        {
            if(array[i] == array[j])
                break;
        }
        if(j==output_array_len) //Not found
        {
            array[output_array_len++] = array[i];
        }
    }
    return output_array_len;
}

uint32_t find_max(uint32_t *array, uint16_t array_len)
{
    uint32_t max_count = 0;
    for(uint16_t i=0; i<array_len; i++)
        if(array[i] > max_count)
            max_count = array[i];

    return max_count;
}

uint32_t interpolate_table(const uint32_t *x_values, const uint32_t *y_values, const uint32_t table_len, uint32_t x)
{
    // Check input bounds and saturate if out-of-bounds
    if (x > x_values[table_len-1])
        return y_values[table_len-1];
    if (x < x_values[0])
        return y_values[0];

    // Find the segment that holds x
    for (uint32_t segment = 1; segment<table_len; segment++)
    {
        if (x_values[segment] >= x)
        {
            // Found the correct segment, interpolate
            // y-y0 = (y1-y0)/(x1-x0)*(x-x0)
            uint32_t x0 = x_values[segment-1];
            uint32_t y0 = y_values[segment-1];
            uint32_t x1 = x_values[segment];
            uint32_t y1 = y_values[segment];
            uint32_t y = y0 + (uint64_t)(x-x0)*(y1-y0)/(x1-x0);
            return y;
        }
    }

    // Something with the data was wrong if we get here
    return -1;
}

uint32_t convert_ommo_ack_code_to_nrf_error_code(uint8_t ack_code)
{
    switch(ack_code)
    {
        case OMMO_ACK_SUCCESS:
            return NRF_SUCCESS;

        case OMMO_ACK_INVALID_MODE:
            return NRF_ERROR_INVALID_STATE;

        case OMMO_ACK_INVALID_DATA:
            return NRF_ERROR_INVALID_PARAM;

        case OMMO_ACK_UNKNOWN_COMMAND:
            return NRF_ERROR_NOT_SUPPORTED;

        case OMMO_ACK_NOT_FOUND:
            return NRF_ERROR_NOT_FOUND;

        case OMMO_ACK_BUSY:
            return NRF_ERROR_BUSY;

        case OMMO_ACK_INTERNAL_ERROR:
            return NRF_ERROR_INVALID_STATE;

        case OMMO_ACK_INVALID_SENSOR:
            return NRF_ERROR_INTERNAL;

        case OMMO_ACK_NO_MEM:
            return NRF_ERROR_NO_MEM;

        case OMMO_ACK_TIMEOUT:
            return NRF_ERROR_TIMEOUT;

        case OMMO_ACK_FORBIDDEN:
            return NRF_ERROR_FORBIDDEN;

        case OMMO_ACK_CORRUPT:
            return OMMO_ERROR_CORRUPT;

        case OMMO_ACK_ALREADY_COMPLETED:
            return OMMO_ERROR_ALREADY_COMPLETED;

        case OMMO_ACK_COMMIT_FAILED:
            return OMMO_ERROR_COMMIT_FAILED;

        case OMMO_ACK_ROLLBACK_FAILED:
            return OMMO_ERROR_ROLLBACK_FAILED;

        default:
            return NRF_ERROR_INTERNAL;
    }
}

OmmoAck convert_nrf_error_code_to_ommo_ack_code(uint32_t error_code)
{
    switch(error_code)
    {
        case NRF_SUCCESS:
            return OMMO_ACK_SUCCESS;

        case NRF_ERROR_INVALID_STATE:
            return OMMO_ACK_INVALID_MODE;

        case NRF_ERROR_INVALID_DATA:
        case NRF_ERROR_INVALID_PARAM:
            return OMMO_ACK_INVALID_DATA;

        case NRF_ERROR_NOT_SUPPORTED:
            return OMMO_ACK_UNKNOWN_COMMAND;

        case NRF_ERROR_NOT_FOUND:
            return OMMO_ACK_NOT_FOUND;

        case NRF_ERROR_BUSY:
            return OMMO_ACK_BUSY;

        case NRF_ERROR_INTERNAL:
            return OMMO_ACK_INVALID_SENSOR;

        case NRF_ERROR_NO_MEM:
            return OMMO_ACK_NO_MEM;

        case NRF_ERROR_TIMEOUT:
            return OMMO_ACK_TIMEOUT;
        
        case NRF_ERROR_FORBIDDEN:
            return OMMO_ACK_FORBIDDEN;

        case OMMO_ERROR_CORRUPT:
            return OMMO_ACK_CORRUPT;

        case OMMO_ERROR_ALREADY_COMPLETED:
            return OMMO_ACK_ALREADY_COMPLETED;

        case OMMO_ERROR_COMMIT_FAILED:
            return OMMO_ACK_COMMIT_FAILED;

        case OMMO_ERROR_ROLLBACK_FAILED:
            return OMMO_ACK_ROLLBACK_FAILED;

        default:
            return OMMO_ACK_INTERNAL_ERROR;
    }
}

/* checks if all bytes in a memory area equal a specific value */
bool memallset(const void *ptr, uint8_t value, size_t len)
{
    const uint8_t *p = (const uint8_t *)ptr;

    // Create a 32-bit word filled with the desired value
    uint32_t pattern = (uint32_t)value;
    pattern |= pattern << 8;
    pattern |= pattern << 16;

    // Align to 4 bytes
    while (len > 0 && ((uintptr_t)p & 3)) {
        if (*p++ != value) {
            return false;
        }
        len--;
    }

    // Check 32-bit words
    const uint32_t *p32 = (const uint32_t *)p;
    while (len >= 4) {
        if (*p32++ != pattern) {
            return false;
        }
        len -= 4;
    }

    // Check any leftover bytes
    p = (const uint8_t *)p32;
    while (len > 0) {
        if (*p++ != value) {
            return false;
        }
        len--;
    }

    return true;
}

/**
 * @brief Blocking delay with optional chaining for drift-free sequential delays.
 *
 * @param delay_ms        Delay duration in milliseconds (max ~134,000 ms)
 * @param continue_from_time  NULL for immediate start, or pointer to previous 
 *                            return value to chain delays without accumulated drift
 * @return End timestamp for use in subsequent chained calls
 *
 * @example
 *   // Single delay:
 *   ommo_delay_ms(100, NULL);
 *
 *   // Chained delays (drift-free, 200ms):
 *   uint32_t t = ommo_delay_ms(100, NULL);
 *   ommo_delay_ms(100, &t);
 *
 *   // Chained delays example 2 (drift-free, 200ms):
 *   uint32_t t = timestamp_get_current_timestamp();
 *   ommo_delay_ms(100, &t);
 *   ommo_delay_ms(100, &t);
 */
uint32_t ommo_delay_ms(uint32_t delay_ms, uint32_t *continue_from_time)
{
    uint32_t delay_ticks = delay_ms * 16000; // Convert ms to ticks (16MHz timer)
    uint32_t end_time;

    //Prevents wraparound comparison issue
    ASSERT(delay_ticks < INT32_MAX);
    
    if (continue_from_time != NULL)
        end_time = *continue_from_time + delay_ticks;
    else
        end_time = timestamp_get_current_timestamp() + delay_ticks;
    
    while ((int32_t)(end_time - timestamp_get_current_timestamp()) > 0)
        main_event_queue.execute_once();
    
    return end_time;
}


class FaultHandlerBootCheck
{
public:
    FaultHandlerBootCheck()
    {
        if (nrf_power_gpregret_get() == FAULT_HANDLER_REBOOT)
        {
            nrf_power_gpregret_set(0);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wuninitialized"
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
            char msg[140] = { 0 };
            // To interpret the SCB register values, see https://developer.arm.com/documentation/dui0552/a/cortex-m3-peripherals/system-control-block
            // Of particular note: if CFSR & 0x400 (IMPRECISERR), then it is an asynchronous fault, and the PC is not the precise fault location
            snprintf(msg, sizeof(msg), "HARD FAULT: R0 %08X, R1 %08X, LR %08X, PC %08X, HFSR %08X, CFSR %08X, MMAR %08X, BFAR %08X, AFSR %08X",
                     this->stack_frame.r0, this->stack_frame.r1, this->stack_frame.lr, this->stack_frame.pc, this->hfsr, this->cfsr, this->mmar, this->bfar, this->afsr);
            OMMO_APP_ERROR_CHECK(NRF_ERROR_INTERNAL, STRING(msg), 0);
#pragma GCC diagnostic pop
        }
    }

    void on_fault(const cortex_basic_stack_frame_t *stack_frame)
    {
        this->stack_frame = *stack_frame;
        this->hfsr = SCB->HFSR;
        this->cfsr = SCB->CFSR;
        this->mmar = SCB->MMFAR;
        this->bfar = SCB->BFAR;
        this->afsr = SCB->AFSR;
    }

private:
    cortex_basic_stack_frame_t stack_frame;
    uint32_t hfsr;
    uint32_t cfsr;
    uint32_t mmar;
    uint32_t bfar;
    uint32_t afsr;
};

FaultHandlerBootCheck faultHandlerBootCheck __attribute__((section(".non_init")));

void handle_hard_fault(const cortex_basic_stack_frame_t *stack_frame)
{
    faultHandlerBootCheck.on_fault(stack_frame);
    nrf_power_gpregret_set(FAULT_HANDLER_REBOOT);
    NVIC_SystemReset();
}

extern "C" __attribute__((naked)) void HardFault_Handler()
{
    // Call the handler with the stack frame.
    IRQ_STACK_FRAME_EXTRACT(handle_hard_fault);
}
