/**
 * Copyright (c) 2019, Ommo Technologies
 *
 * All rights reserved.
 *
 *
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "nrf_gpio.h"
#include "nrfx_gpiote.h"
#include "nrf_delay.h"
#include "nrf_power.h"
#include "app_timer.h"

#include "event_queue_manager.hpp"
#include "port_master.hpp"
#include "ommo_config.h"
#include "ommo_app_error.h"
#include "usb_serial.hpp"
#include "utils.hpp"
#include "timestamp_timer.hpp"
#include "led_driver.hpp"
#include "comms_uarte_siu.hpp"
#include "comms_esb_siu_wireless.hpp"
#include "trace.h"

#include "main.hpp"

#ifdef OMMO_ADC_127L01
#include "ADS127L01_spi_sampler.hpp"
#elif defined(OMMO_ADC_127L11)
#include "ADS127L11_mux_sampler.hpp"
#endif

// Interrupt priorities
//
// Component      Symbol                                  Value
// advanced_wdt                                           0
// ADC timer      OMMO_ADC_SAMPLING_TIMER_PRIORITY        0
// ESB            RADIO_IRQHandler                        0
// ESB            NRF_ESB_RADIO_EVT_PRIORITY              1
// sensors        OMMO_PM_SPI_IRQ_PRIORITY                1
// timestamp      see main                                3
// ESB            COMMS_ESB_PACKET_BUILDING_ISR_PRI       3
// UARTE default  NRFX_UARTE_DEFAULT_CONFIG_IRQ_PRIORITY  6
// RTC default    NRFX_RTC_DEFAULT_CONFIG_IRQ_PRIORITY    6
// timer default  TIMER_DEFAULT_CONFIG_IRQ_PRIORITY       6
// USB            see BSP_USB_InstallISR                  6

//Global variables
volatile data_mode_t data_mode;
volatile uint8_t packet_id;
static bool in_fault_handler = false;

//Debug variables
uint32_t tx_count;
uint32_t skipped_tx_count;
uint8_t tx_buffer[PACKET_RESPONSE_BUFFER_SIZE];

//Led driver
static led_driver main_led;

//Set regulator for 3.3V, constant will be loaded when hex file is downloaded
#ifdef OMMO_UICR_REG0
  volatile const uint32_t REGOUT0 __attribute__((section(".uicr_regout0"))) = OMMO_UICR_REG0;
#endif

//volatile const uint32_t APPROTECT_REG __attribute__((section(".uicr_approtect"))) = 0x5A;

static void set_leds();

void ommo_app_error_fault_handler(uint32_t id, uint32_t pc, ret_code_t error_code, uint32_t line_num, const uint8_t *p_file_name)
{
    if (!in_fault_handler)
    {
        in_fault_handler = true;
        ESB_TRACE(FAULT_HANDLER);

        __disable_irq();
    }

    NRF_BREAKPOINT_COND;
    // On assert, the system can only recover with a reset.

    //Reboot into bootloader mode
    nrf_power_gpregret_set(FIRMWARE_CRASH_REBOOT);
    NVIC_SystemReset();
    //Unreachable
}

//Link ADS127L11_mux_sampler to comms esb
bool comms_esb_is_sampling_state_machine_configured()
{
    return adc_sensors_is_state_machine_configured();
}

void sample_set_ready(uint8_t *data, uint16_t data_size, uint32_t timestamp, uint32_t timestamp_offset)
{
uint8_t adc_channel;
uint16_t index;

    //Wait until we are enabled and have synch
    if(data_mode == DATA_MODE_DISABLED || timestamp_is_synch_lost())
        return;

    if(data_mode == DATA_MODE_USB)
    {
        //Calculate decoded packet size
        size_t packet_size = 1 + 4 + data_size; // pkt_id(1) + ts(4)
#ifdef OMMO_INCLUDE_TIMESTAMP_OFFSET
        packet_size += 4; // tso(4)
#endif

        //Do COBS encoding, add packet delimiter(s) (0x00), and transmit via USB
        uint32_t code = usb_serial_write_data_packet_vargs(packet_size,
            (uint32_t)1, &packet_id,
            (uint32_t)4, &timestamp,
#ifdef OMMO_INCLUDE_TIMESTAMP_OFFSET
            (uint32_t)4, &timestamp_offset,
#endif
            (uint32_t)data_size, data,
            (uint32_t)0);

        if(code == NRF_ERROR_BUSY)
            skipped_tx_count++;
        else
            OMMO_APP_ERROR_CHECK(code, STRING("USB data transfer error"), 0);
    }
    else //DATA_MODE_WIRELESS
    {
        //Compose our data packet
        uint16_t index = 0;
        copyUint8(tx_buffer, index, packet_id); //PACKET_ID
        copyUint32_LE(tx_buffer, index, timestamp); //TIME_STAMP
#ifdef OMMO_INCLUDE_TIMESTAMP_OFFSET
        copyUint32_LE(data_tx_buffer, index, timestamp_offset); //TIME_STAMP_OFFSET
#endif
        memcpy(tx_buffer+index, data, data_size); index += data_size; //SENSOR DATA

        uint32_t code = comms_esb_send_sensor_data(tx_buffer, index);
        if(code == NRF_ERROR_BUSY)
            skipped_tx_count++;
    }

    //Update count
    tx_count++;
}

void sensors_disable()
{
    adc_sensors_disable();
    packet_id = 0xFF;
    data_mode = DATA_MODE_DISABLED;
}

void comms_esb_event(comms_esb_event_type type)
{
    switch(type)
    {
        case COMMS_ESB_EVENT_SD_DISCONNECT:
            sensors_disable();
            break;

        case COMMS_ESB_EVENT_PAUSE_DATA:
        case COMMS_ESB_EVENT_PAUSE_DATA_SLEEP_IMU:
            adc_sensors_disable();
            data_mode = DATA_MODE_DISABLED;
            //Save packet id for resume
            break;

        case COMMS_ESB_EVENT_RESUME_DATA:
            //packet id is already set
            data_mode = DATA_MODE_WIRELESS;
            adc_sensors_enable();
            break;

        default:
            break;
    }

    set_leds();
}

void comms_esb_direct_comms_packet_received(uint8_t data[], uint16_t length)
{
uint16_t response_length;
uint8_t response_buffer[OMMO_ESB_DC_MAX_MULTISYNCH_TRANSFER_PACKET_LENGTH];

    response_length = 0;

    switch(data[0])
    {
        case OMMO_COMMAND_WIRELESS_DATA_ENABLE:
        case OMMO_COMMAND_WIRELESS_DATA_ENABLE_W_SLEEP:
        {
            bool enable_sleep = (data[0] == OMMO_COMMAND_WIRELESS_DATA_ENABLE_W_SLEEP);

            if(data_mode != DATA_MODE_DISABLED || !adc_sensors_is_state_machine_configured())
            {
                response_length = fill_in_ack_packet(response_buffer, OMMO_ACK_INVALID_MODE);
            }
            else if(length != 6)
            {
                response_length = fill_in_ack_packet(response_buffer, OMMO_ACK_INVALID_DATA);
            }
            else
            {
                packet_id = data[1];
                uint8_t data_channel = data[2];
                uint8_t data_good_id = data[3];
                uint16_t data_tx_time;
                memcpy(&data_tx_time, &data[4], 2);

                uint32_t rvalue = comms_esb_enter_sd_mode(data_channel, data_good_id, data_tx_time, enable_sleep);
                if(rvalue == NRF_SUCCESS)
                {
                    data_mode = DATA_MODE_WIRELESS;

                    if(!enable_sleep)
                        adc_sensors_enable();

                    return; //No response
                }
                else
                {
                    packet_id = 0xFF;
                    response_length = fill_in_ack_packet(response_buffer, OMMO_ACK_INVALID_DATA);
                }
            }
            break;
        }
    }

    if(response_length == 0) //No response, try protocol agnostic commands
        response_length = process_packet_received(data, length, response_buffer, OMMO_ESB_DC_MAX_MULTISYNCH_TRANSFER_PACKET_LENGTH);

    if(response_length == 0) //No response, must be an unknown command
        response_length = fill_in_ack_packet(response_buffer, OMMO_ACK_UNKNOWN_COMMAND);
    
    comms_esb_dc_send_packet_to_pc(response_buffer, response_length);
}

void usb_serial_packet_received(uint8_t data[], uint16_t length, const void*)
{
uint16_t response_length = 0;
uint8_t response_buffer[1024];

    //Check for enable packet
    switch(data[0])
    {
        case OMMO_COMMAND_USB_DATA_ENABLE:
            if(data_mode != DATA_MODE_DISABLED || !adc_sensors_is_state_machine_configured())
            {
                response_length = fill_in_ack_packet(response_buffer, OMMO_ACK_INVALID_MODE);
            }
            else if(length != 2)
            {
                response_length = fill_in_ack_packet(response_buffer, OMMO_ACK_INVALID_DATA);
            }
            else
            {
                ret_code_t error_code = adc_sensors_enable();
                if(error_code == NRF_SUCCESS)
                {
                    packet_id = data[1];
                    data_mode = DATA_MODE_USB;
                }
                response_length = fill_in_ack_packet(response_buffer, convert_nrf_error_code_to_ommo_ack_code(error_code));

                set_leds();
            }
            break;

        case OMMO_COMMAND_USB_SET_SYNCH_CH_ID:
            if(length != 2)
            {
                response_length = fill_in_ack_packet(response_buffer, OMMO_ACK_INVALID_DATA);
            }
            else
            {
                response_length = fill_in_ack_packet(response_buffer, convert_nrf_error_code_to_ommo_ack_code(comms_esb_set_synch_rx_channel(data[1])));
            }
            break;
    }

    if(response_length == 0) //No response, try protocol agnostic commands
        response_length = process_packet_received(data, length, response_buffer, sizeof(response_buffer));

    if(response_length == 0) //No response, must be an unknown command
        response_length = fill_in_ack_packet(response_buffer, OMMO_ACK_UNKNOWN_COMMAND);

    usb_serial_send_command_packet(response_buffer, response_length);
}

uint16_t process_packet_received(uint8_t data[], uint16_t packet_size, uint8_t response_buffer[], uint16_t response_buffer_size)
{
uint16_t response_length;

    response_length = 0;

    switch(data[0])
    {
        case OMMO_COMMAND_DATA_DISABLE:
            if(data_mode == DATA_MODE_DISABLED)
            {
                response_length = fill_in_ack_packet(response_buffer, OMMO_ACK_INVALID_MODE);
            }
            else if(packet_size != 1)
            {
                response_length = fill_in_ack_packet(response_buffer, OMMO_ACK_INVALID_DATA);
            }
            else
            {
                sensors_disable();
                set_leds();

                response_length = fill_in_ack_packet(response_buffer, OMMO_ACK_SUCCESS);
            }
            break;
    }

    if(response_length == 0) //Try general processor
        response_length = general_process_packet_received(data, packet_size, response_buffer, response_buffer_size);

    if(response_length == 0) //No response, try device info processor
        response_length = adc_process_packet_received(data, packet_size, response_buffer, response_buffer_size);

    //No one handled it
    return response_length;
}

#ifdef OMMO_ADC_MONITOR_USBC
void usbc_debug_mode_change(bool debug_mode, bool orientation)
{
    if(debug_mode)
    {
        if(orientation)
        {
            comms_uarte_set_rx_pin(OMMO_DEBUG_SYNCH_NORM);
        }
        else
        {
            comms_uarte_set_rx_pin(OMMO_DEBUG_SYNCH_REV);
        }
    }
    else
    {
        comms_uarte_set_rx_pin(OMMO_COMMS_UARTE_SYNCH_PIN);
    }
}
#endif

static void set_leds()
{
    application_state new_state;

    if(comms_esb_get_current_state() == COMMS_ESB_EXTERNAL_STATE_IDLE ||
       comms_esb_get_current_state() == COMMS_ESB_EXTERNAL_STATE_SEARCHING)
    {
        new_state = APP_STATE_IDLE;
    }
    else if(comms_esb_get_current_state() == COMMS_ESB_EXTERNAL_STATE_SD_SLEEP)
    {
        new_state = APP_STATE_SLEEP;
    }
    else if(comms_esb_get_current_state() == COMMS_ESB_EXTERNAL_STATE_DC)
    {
        new_state = APP_STATE_DC;
    }
    else if(usb_serial_get_state() == USB_SERIAL_STATE_OPENED && data_mode == DATA_MODE_DISABLED)
    {
        new_state = APP_STATE_CONNECTED;
    }
    else if(timestamp_is_synch_lost())
    {
        new_state = APP_STATE_SYNCH_LOST;
    }
    else if(!timestamp_is_synch_wired())
    {
        new_state = APP_STATE_WIRELESS_SYNCH;
    }
    else
    {
        new_state = APP_STATE_WIRED_SYNCH;
    }

    change_led_state(new_state, false);
}

led_driver::set_led_fn_t map_state_to_led_indication[] = {
    /* APP_STATE_CRASH          */ &led_driver::set_orange_flash,
    /* APP_STATE_OMMOCOMM_DFU   */ &led_driver::set_purple,
    /* APP_STATE_DC             */ &led_driver::set_cyan,
    /* APP_STATE_IDLE           */ &led_driver::set_white_fade,
    /* APP_STATE_CONNECTED      */ &led_driver::set_blue_fade,
    /* APP_STATE_SYNCH_LOST     */ &led_driver::set_blue_flash,
    /* APP_STATE_SLEEP          */ &led_driver::set_dim_blue_lp_flash,
    /* APP_STATE_WIRELESS_SYNCH */ &led_driver::set_green_fade,
    /* APP_STATE_WIRED_SYNCH    */ &led_driver::set_green,
};

void change_led_state(application_state new_state, bool immediate)
{
    APP_ERROR_CHECK_BOOL(new_state < std::size(map_state_to_led_indication));
    (&main_led->*map_state_to_led_indication[new_state])();
    if (immediate)
        main_led.flush_pending_tasks();
}

void push_led_state(application_state new_state)
{
    main_led.push_state();
    change_led_state(new_state);
}

void pop_led_state()
{
    main_led.pop_state();
}

void gpio_init( void )
{
    nrf_gpio_pin_write(OMMO_LED_RED, 0);
    nrf_gpio_cfg_output(OMMO_LED_RED);

    nrf_gpio_pin_write(OMMO_LED_GREEN, 0);
    nrf_gpio_cfg_output(OMMO_LED_GREEN);

    nrf_gpio_pin_write(OMMO_LED_BLUE, 0);
    nrf_gpio_cfg_output(OMMO_LED_BLUE);

    //No external pull-up, ensure line stays high so that ST micro can avoid framing errors and
    //respond quickly when ommocomm communication is required
#ifdef OMMOCOMM_EEPROM_PIN
    nrf_gpio_cfg_input(OMMOCOMM_EEPROM_PIN, NRF_GPIO_PIN_PULLUP);
#endif

//    nrf_gpio_pin_write(DEBUG, 0);
//    nrf_gpio_cfg_output(DEBUG);

#ifdef OMMO_DEBUG_IO
    nrf_gpio_pin_clear(OMMO_DEBUG_IO);
    nrf_gpio_cfg_output(OMMO_DEBUG_IO);
#endif

#ifdef OMMO_DEBUG_RX
    nrf_gpio_pin_clear(OMMO_DEBUG_RX);
    nrf_gpio_cfg_output(OMMO_DEBUG_RX);
#endif

    #ifdef OMMO_CHRG_CHARGE_PIN
    nrf_gpio_cfg_input(OMMO_CHRG_CHARGE_PIN, NRF_GPIO_PIN_PULLUP);
    #endif

    #ifdef OMMO_10V_ENABLE_PIN
    nrf_gpio_pin_write(OMMO_10V_ENABLE_PIN, !OMMO_10V_ENABLE);   //active low
    nrf_gpio_cfg_output(OMMO_10V_ENABLE_PIN);
    #endif
}

void timestamp_synch_lost_found(bool found, bool wired)
{
    //Update LED state
    set_leds();
}

void usb_serial_event_handler(usb_serial_event_t event)
{
    switch(event)
    {
        case USB_SERIAL_EVENT_OPENED:
            sensors_disable();
            OMMO_APP_ERROR_CHECK(comms_esb_enter_synch_only_mode(), STRING("Wireless communication error"), 0);
            break;

        case USB_SERIAL_EVENT_CLOSED:
            sensors_disable();
            OMMO_APP_ERROR_CHECK(comms_esb_enter_direct_comm_mode(), STRING("Wireless communication error"), 0);
            break;

        default:
            break;
    }

    //Update LED state
    set_leds();
}

int main(void)
{
uint32_t i;

    //Init globals
    data_mode = DATA_MODE_DISABLED;
    tx_count = 0;
    skipped_tx_count = 0;
    packet_id = 0xFF;

    //LEDs (must be BEFORE crash reboot check)
    main_led.init(OMMO_LED_PWM, OMMO_LED_RED, OMMO_LED_GREEN, OMMO_LED_BLUE, NRFX_PWM_PIN_NOT_USED, OMMO_LED_ACTIVE_LOW);

    //Init app timer (must be BEFORE crash reboot check)
    app_timer_init();

    // Crash reboot check
    crash_reboot_check();

    clocks_start();

    //Init gpio
    gpio_init();

    //Init rtc timer
    rtc_timer_init();

    //Init port master
    port_master_init();

    //Timestamp timer.  ADC_127L01 does not need sample events, but ESB does.
    timestamp_init_with_internal_sample_event(3);
    timestamp_set_synch_lost_found_callback(timestamp_synch_lost_found);
    comms_uarte_init(OMMO_COMMS_UARTE_SYNCH_PIN);

    //Init ESB
    comms_esb_init(comms_esb_direct_comms_packet_received, comms_esb_event);
    comms_esb_enter_direct_comm_mode(); //Start looking for a BS

    //Init USB serial
    usbd_main_init();
    usb_serial_init(usb_serial_packet_received, usb_serial_event_handler);
    usbd_main_start();

    //Init sensor sampler
#if defined(OMMO_ADC_127L01) || defined(OMMO_ADC_127L11)
    adc_sensors_init(&sample_set_ready);
#endif

#ifdef OMMO_ADC_MONITOR
    //Init adc monitor
    adc_monitor_init();
#ifdef OMMO_ADC_MONITOR_USBC
    adc_monitor_set_usbc_callback(usbc_debug_mode_change);
#endif
#endif

    //Update LED state
    set_leds();

    //Clear and fill in our main event queue
    main_event_queue.reset();
    APP_ERROR_CHECK(usb_serial_add_tasks_to_event_queue(&main_event_queue));
    APP_ERROR_CHECK(comms_esb_add_tasks_to_event_queue(&main_event_queue));
    APP_ERROR_CHECK(main_led.add_tasks_to_event_queue(&main_event_queue));

    //Execute event queue (without sleep)
    main_event_queue.execute_forever();
}
