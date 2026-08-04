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

#include "sdk_config.h"

#include "nrfx_timer.h"
#include "nrf_timer.h"
#include "nrf_gpio.h"
#include "nrfx_gpiote.h"
#include "nrf_delay.h"
#include "nrfx_uarte.h"
#include "nrfx_power.h"
#include "app_timer.h"
#include "advanced_wdt.h"
#include "nrf_delay.h"
#include "random.h"

#include "ommo_config.h"
#include "trace.h"
#include "usb_serial.hpp"
#include "akm_st_spi_sampler.hpp"
#include "utils.hpp"
#include "timestamp_timer.hpp"
#include "led_driver.hpp"
#include "adc_monitor.hpp"
#ifdef OMMOCOMM_1WIRE
#include "comms_uarte_siu_1wire.hpp"
#endif
#include "comms_uarte_siu.hpp"
#include "comms_esb_siu_wireless.hpp"
#include "ommo_fw.pb.h"
#include "ommo_esb.h"
#include "ommo_app_error.h"
#include "port_master.hpp"
#include "device_info_reader_flash.hpp"
#include "main.hpp"
#include "ommo_power.hpp"
#include "port_master.hpp"
#include "trace.h"

#ifdef OMMOCOMM_ENABLED
#include "dfu_container.h"
#include "ommocomm_types.h"
#include "ommocomm_uarte.hpp"
#endif

#ifdef OMMO_BQ25188
#include "serial_led_spi.hpp"
#include "bq25188_twi_driver.hpp"
#endif

#ifdef OMMO_BQ27Z558
#include "bq27z558_twi_driver.hpp"
    #if defined(GOLDEN_IMAGE_ZWD_CCGAIN_1P38)
        #include "zwd_100p_health_ccgain1p38x.gm.h"
        #define GM_BIN          zwd_100p_health_ccgain1p38x_gm_bin
        #define GM_BIN_SIZE     ZWD_100P_HEALTH_CCGAIN1P38X_GM_BIN_SIZE
    #elif defined(GOLDEN_IMAGE_ZWD)
        #include "zwd_100p_health.gm.h"
        #define GM_BIN          zwd_100p_health_gm_bin
        #define GM_BIN_SIZE     ZWD_100P_HEALTH_GM_BIN_SIZE
    #else
        #error "No golden image selected."
    #endif
#endif

#ifdef OMMO_CHARGE_LED
#include "led_animator.hpp"
#endif

// Interrupt priorities
//
// Component      Symbol                                  Value   Notes
// advanced_wdt                                           0
// ESB hw ISR     NRF_ESB_DEFAULT_CONFIG                  0
// ESB sw ISR     NRF_ESB_RADIO_EVT_PRIORITY              1
// 1-wire         see main                                1       ESB and 1-wire are exclusive
// Ommocomm       OMMOCOMM_IRQ_PRIORITY                   2       Has set-up time requirements
// I2C sensors    OMMO_PM_TWI_IRQ_PRIORITY                3
// SPI sensors    OMMO_PM_SPI_IRQ_PRIORITY                3
// timestamp      see main                                3
// ESB sw ISR     COMMS_ESB_PACKET_BUILDING_ISR_PRI       3
// UARTE synch    NRFX_UARTE_DEFAULT_CONFIG_IRQ_PRIORITY  6
// RTC default    NRFX_RTC_DEFAULT_CONFIG_IRQ_PRIORITY    6       ex. timeouts, and time-boxed wireless state transitions
// timer default  TIMER_DEFAULT_CONFIG_IRQ_PRIORITY       6
// USB            see BSP_USB_InstallISR                  6       ESB operates in synch-only mode while USB is open
// GPIOTE ISR     NRFX_GPIOTE_CONFIG_IRQ_PRIORITY         6       ex. OMMO_IMU_IRQ_PIN to wake from sleep mode
// app_timer2     APP_TIMER_CONFIG_IRQ_PRIORITY           6       ex. ADC reads

// Enables use of 13517 test board without the 12359 battery charger tester.  12359 can be used to
// simulate battery power/provide VBAT, which enables using 13517 with the VBUS switch in the OFF
// position.  Without 12359, the switch is the only way to power the MSIU, which connects VBUS to
// VCON, which would normally enable charging mode.  The final 13891 calibration board leaves VCON
// disconnected.
//#define SIMULATE_BATTERY_POWER

// Similar to BOOTLOADER_DFU_GPREGRET2* defines in nrf_bootloader_info.h
#define OMMO_GPREGRET2_MAGIC_NUMBER_MASK    0xF8
#define OMMO_GPREGRET2_SHUTDOWN_COUNT_MASK  0x07
#define OMMO_GPREGRET2_MAGIC_NUMBER         0x50  // boot loader value is 0xA8

//Power on/shutdown logic
power_req_t power_status;

//Global variables
data_mode_t data_mode;
uint8_t packet_id;
static bool in_fault_handler = false;
static bool in_charge_loop = false;

//Event queues
static event_queue_manager one_hundred_ms_event_queue;

//Debug variables
uint32_t tx_count;
uint32_t skipped_tx_count;
uint8_t tx_buffer[PACKET_RESPONSE_BUFFER_SIZE];

//Led driver
static led_driver main_led;

//Task loop variables
APP_TIMER_DEF(one_hundred_ms_timer_id);
bool one_hundred_ms_timer_event = false;

uint8_t hot_plug_pkt_counter;

#ifdef OMMO_BUTTON_PIN
nrfx_timer_t button_down_timer = OMMO_BUTTON_DOWN_TIMER;
bool button_down;
bool button_initialized;
#endif

#ifdef OMMO_BQ25188
bq25188_twi_driver bq25188_charger_instance(OMMO_BQ25188_INT);
#endif

#ifdef OMMO_BQ27Z558
BQ27Z558 fuel_gauge_instance;
bool low_battery_state = false;
#ifdef DEBUG_NRF
uint16_t low_battery_toggle_timer_ms = 0;
#endif
#endif

//1-wire
#ifdef OMMOCOMM_1WIRE
APP_TIMER_DEF(comms_1wire_timer_timer_id);
nrfx_timer_t comms_1wire_timer = OMMOCOMM_1WIRE_TIMER;
#endif

#ifdef OMMO_CHARGE_LED
APP_TIMER_DEF(led_strip_timer_id);

serial_led_spi charge_led_serial_instance(OMMO_CHARGE_LED_INVERTED, OMMO_CHARGE_LED_RGB_BYTE_ORDER);
led_animator led_strip_instance;
#endif

//Set regulator for 3.3V(UICR_REGOUT0_VOUT_3V3) or other values, constant will be loaded when hex file is downloaded
#ifdef OMMO_UICR_REG0
  volatile const uint32_t REGOUT0 __attribute__((section(".uicr_regout0"))) = OMMO_UICR_REG0;
#endif

//volatile const uint32_t APPROTECT_REG __attribute__((section(".uicr_approtect"))) = 0x5A;

uint32_t disrupt_realtime_operation_active_count = 0;

static void set_leds();

#pragma GCC push_options
#pragma GCC optimize("O0")  //Disable optimizations to ensure parameters are available to a debugger
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
#pragma GCC pop_options

void modify_power_status(bool add_item, power_req_t req)
{
    if(add_item)
        power_status = (power_req_t)(power_status | req);
    else
        power_status = (power_req_t)(power_status & ~(req));
}

void check_power_requirements()
{
    //If we lost our reason for being alive, reboot and recheck
    //If there isn't another reason to be alive, shutdown will be called after reboot
    if(!(power_status & OMMO_POWER_REQUIREMENTS))
    {
        //Flush current time to flash
        device_info_reader_flash_save_hours_counter_sum();

        //Reboot
        NVIC_SystemReset();
    }
}

void nrfx_power_usb_event_handler(nrfx_power_usb_evt_t event)
{
    bool usb_power_detected = (nrfx_power_usbstatus_get() != NRFX_POWER_USB_STATE_DISCONNECTED);
    modify_power_status(usb_power_detected, POWER_REQ_USB);
    check_power_requirements();
}

//If we need more efficiency, we could send this data from sensor sampler with the SPI TX read data in between the samples and then specify
//  the parameters to send to usb_serial_write_data_packet_vargs to skip them.  This would eliminate another sensor data copy.
void sample_set_ready(uint8_t *data, uint16_t data_size, uint32_t timestamp, uint32_t timestamp_offset)
{
#ifdef EMI_AUTO_SAMPLE_SENSOR
    static uint16_t delay = 0;
    static bool led_on = true;
    if(delay++ > 250)
    {
        delay = 0;
        if(led_on) main_led.set_off(); else main_led.set_on(RGB_RED(RGB_MAX_VALUE));
        led_on = !led_on;
    }
    return;
#endif

    //Wait until we are enabled and have synch
    if(data_mode == DATA_MODE_DISABLED || timestamp_is_synch_lost())
        return;

#ifdef EMI_RUN_WITHOUT_SENSOR
    static uint8_t dummy_data = 0;
    for(int i=0; i<data_size; i++)
        data[i] = dummy_data++;
#endif

    if(data_mode == DATA_MODE_USB)
    {
        //Calculate decoded packet size
        size_t packet_size = 1 + 4 + data_size;
#ifdef OMMO_INCLUDE_TIMESTAMP_OFFSET
        packet_size += 4;
#endif

#ifdef OMMOCOMM_1WIRE
        if(power_status & POWER_REQ_1WIRE)
        {
            //Do COBS encoding, add packet delimiter(s) (0x00), and transmit via 1-wire
            ret_code_t code = comms_1wire.send_data_packet_async_vargs(packet_size,
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
                OMMO_APP_ERROR_CHECK(code, STRING("1-wire data transfer error"), 0);
        }
#endif

        if(usb_serial_get_state() == USB_SERIAL_STATE_OPENED)
        {
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

#ifndef RUN_WITHOUT_SYNC
        //Transmit synch signal alert 
        uint32_t num_misses = timestamp_synch_get_num_misses();
        if(num_misses >  USB_DATA_SYNC_LOST_COUNT_TO_SYSTEM_ALERT)
        {
            tx_buffer[0] = OMMO_COMMAND_ALERT_SYNCH_FAIL;
            memcpy(tx_buffer+1, &num_misses, 4);
#ifdef OMMOCOMM_1WIRE
            if(power_status & POWER_REQ_1WIRE)
                comms_1wire.send_command_packet_async(tx_buffer, 5);
#endif
            if(usb_serial_get_state() == USB_SERIAL_STATE_OPENED)
                usb_serial_send_command_packet(tx_buffer, 5);
        }
#endif
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

static void usb_send_hotplug_detect_event(uint8_t event)
{
    uint8_t hot_plug_cmd_buf[2] = { 0 };
    hot_plug_cmd_buf[0]         = OMMO_COMMAND_HOT_PLUG_DETECT;
    hot_plug_cmd_buf[1]         = event;

    if (usb_serial_get_state() == USB_SERIAL_STATE_OPENED)
        usb_serial_send_command_packet(hot_plug_cmd_buf, 2);

#ifdef OMMOCOMM_1WIRE
    if (power_status & POWER_REQ_1WIRE)
        comms_1wire.send_command_packet_async(hot_plug_cmd_buf, 2);
#endif
}

void sensor_event_handler(sensors_event_t event)
{
    switch (event)
    {
        case SENSORS_EVENT_HOTPLUG_REMOVE:
#ifdef EMI_AUTO_SAMPLE_SENSOR
            sensors_stop_state_machine();
#elif !defined(EMI_RUN_WITHOUT_SENSOR)
            // Stop state machine
            sensors_stop_state_machine();
            data_mode = DATA_MODE_DISABLED;
            packet_id = 0xFF;

            set_leds();

            // Make sure we have a reason to be "alive"
            check_power_requirements();
#endif
            usb_send_hotplug_detect_event(1);
            comms_esb_on_hot_unplug();
            break;

        case SENSORS_EVENT_HOTPLUG_INSERT:
            usb_send_hotplug_detect_event(0);
            comms_esb_on_hot_plugin();
            break;

        case SENSORS_EVENT_WAKE_ON_IMU:
            (void) comms_esb_wakeup();
            break;

        case SENSORS_EVENT_DFU_STARTED:
            change_led_state(APP_STATE_OMMOCOMM_DFU, APP_SUBSTATE_NORMAL, true);
            break;

        case SENSORS_EVENT_DFU_FINISHED:
            set_leds();
            break;

        case SENSORS_STATE_MACHINE_CONFIGURED:
            comms_esb_state_machine_configured();
            break;

        case SENSORS_STATE_MACHINE_UNCONFIGURED:
            comms_esb_state_machine_unconfigured();
            break;

        default:
            break;
    }
}

void comms_esb_event(comms_esb_event_type type)
{
    switch(type)
    {
        case COMMS_ESB_EVENT_SD_DISCONNECT:
            sensors_stop_state_machine(SENSORS_STOP_CMD_SUSPEND, true);
            data_mode = DATA_MODE_DISABLED;
            packet_id = 0xFF;
            break;

        case COMMS_ESB_EVENT_PAUSE_DATA:
            sensors_stop_state_machine(SENSORS_STOP_CMD_SUSPEND, true);
            // TODO sensors_start_idle_hot_plug_check()?
            data_mode = DATA_MODE_DISABLED;
            //Save packet id for resume
            break;

        case COMMS_ESB_EVENT_PAUSE_DATA_SLEEP_IMU:
            sensors_stop_state_machine(SENSORS_STOP_CMD_WAKE_ON_IMU, true);
            // TODO sensors_start_idle_hot_plug_check()?
            data_mode = DATA_MODE_DISABLED;
            //Save packet id for resume
            break;

        case COMMS_ESB_EVENT_RESUME_DATA:
            //packet id is already set
            data_mode = DATA_MODE_WIRELESS;
            sensors_start_state_machine();
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

            if(data_mode != DATA_MODE_DISABLED || !sensors_is_state_machine_ready_to_run())
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
                        sensors_start_state_machine();

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
        response_length = process_packet_received(data, length, response_buffer, sizeof(response_buffer));

    if(response_length == 0) //No response, must be an unknown command
        response_length = fill_in_ack_packet(response_buffer, OMMO_ACK_UNKNOWN_COMMAND);
    
    comms_esb_dc_send_packet_to_pc(response_buffer, response_length);
}

uint16_t process_usb_serial_packet_received(uint8_t data[], uint16_t length, uint8_t response_buffer[], uint16_t response_buffer_size)
{
uint16_t response_length = 0;

    //Check for invalid packets
    if(length == 0x00)
        return fill_in_ack_packet(response_buffer, OMMO_ACK_INVALID_DATA);

    switch(data[0])
    {
        case OMMO_COMMAND_USB_DATA_ENABLE:
            if(data_mode != DATA_MODE_DISABLED || !sensors_is_state_machine_ready_to_run())
            {
                response_length = fill_in_ack_packet(response_buffer, OMMO_ACK_INVALID_MODE);
            }
            else if(length != 2)
            {
                response_length = fill_in_ack_packet(response_buffer, OMMO_ACK_INVALID_DATA);
            }
            else
            {
                //Try to start state machine
                uint32_t rvalue = sensors_start_state_machine();

                if(rvalue == NRF_SUCCESS)
                {
                    packet_id = data[1];
                    data_mode = DATA_MODE_USB;
                }

                //Return success/failure
                set_leds();
                response_length = fill_in_ack_packet(response_buffer, convert_nrf_error_code_to_ommo_ack_code(rvalue));
            }
            break;

        case OMMO_COMMAND_USB_SET_SYNCH_CH_ID:
            if(length != 2)
            {
                response_length = fill_in_ack_packet(response_buffer, OMMO_ACK_INVALID_DATA);
            }
            else
            {
                response_length = fill_in_ack_packet(response_buffer, comms_esb_set_synch_rx_channel(data[1]));
            }
            break;
    }

    if(response_length == 0) //No response, try protocol agnostic commands
        response_length = process_packet_received(data, length, response_buffer, response_buffer_size);

    return response_length;
}

void usb_serial_packet_received(uint8_t data[], uint16_t length, const void *context)
{
uint16_t response_length = 0;

    response_length = process_usb_serial_packet_received(data, length, tx_buffer, PACKET_RESPONSE_BUFFER_SIZE);

    if(response_length == 0) //No response, must be an unknown command
        response_length = fill_in_ack_packet(tx_buffer, OMMO_ACK_UNKNOWN_COMMAND);

    usb_serial_send_command_packet(tx_buffer, response_length);
}

void disable_data_mode(bool disable_hot_plug)
{
    //Stop state machine
    sensors_stop_state_machine();

#ifndef HOT_PLUG_OFF
    if(!disable_hot_plug)
    {
        sensors_start_idle_hot_plug_check();
    }
#endif

    data_mode = DATA_MODE_DISABLED;
    packet_id = 0xFF;
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
                disable_data_mode(false);
                set_leds();

                response_length = fill_in_ack_packet(response_buffer, OMMO_ACK_SUCCESS);
            }
            break;

        #ifdef OMMO_BQ25188
        case OMMO_COMMAND_ENTER_SHIP_MODE:
            if(packet_size != 1)
            {
                response_length = fill_in_ack_packet(response_buffer, OMMO_ACK_INVALID_DATA);
            }
            else
            {
                enter_ship_mode(); //does not return
            }
            break;
        #endif
    }

    if(response_length == 0) //Try general processor
        response_length = general_process_packet_received(data, packet_size, response_buffer, response_buffer_size);

#ifndef EMI_AUTO_SAMPLE_SENSOR
    //If auto sampling, ignore all sensor commands
    if(response_length == 0) //Let sensors take a crack at the packet
        response_length = sensors_process_packet_received(data, packet_size, response_buffer, response_buffer_size);
#endif

    //No one handled it
    return response_length;
}

#ifdef OMMO_ADC_MONITOR_USBC
void usbc_debug_mode_change(bool debug_mode, bool orientation)
{
    if(debug_mode)
    {
#ifdef OMMO_DEBUG_SYNCH_REV
        if(orientation)
        {
            comms_uarte_set_rx_pin(OMMO_DEBUG_SYNCH_NORM);
        }
        else
        {
            comms_uarte_set_rx_pin(OMMO_DEBUG_SYNCH_REV);
        }
#elif defined(OMMO_DEBUG_SYNCH)
        comms_uarte_set_rx_pin(OMMO_DEBUG_SYNCH);
#endif //OMMO_DEBUG_SYNCH_REV
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
    application_substates substate;

    if(in_charge_loop)
    {
        new_state = APP_STATE_CHARGING;
    }
#ifndef EMI_AUTO_SAMPLE_SENSOR
#ifdef OMMOCOMM_1WIRE
    else if(!(power_status & POWER_REQ_1WIRE) && //1 wire keeps esb disabled, usb connected moves esb to synch only mode
       (comms_esb_get_current_state() == COMMS_ESB_EXTERNAL_STATE_IDLE ||
       comms_esb_get_current_state() == COMMS_ESB_EXTERNAL_STATE_SEARCHING))
#else
    if(comms_esb_get_current_state() == COMMS_ESB_EXTERNAL_STATE_IDLE ||
       comms_esb_get_current_state() == COMMS_ESB_EXTERNAL_STATE_SEARCHING)
#endif
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
#ifdef OMMOCOMM_1WIRE
    else if((usb_serial_get_state() == USB_SERIAL_STATE_OPENED || (power_status & POWER_REQ_1WIRE))
             && data_mode == DATA_MODE_DISABLED)
#else
    else if(usb_serial_get_state() == USB_SERIAL_STATE_OPENED && data_mode == DATA_MODE_DISABLED)
#endif
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
#endif

    substate = APP_SUBSTATE_NORMAL;
#ifdef OMMO_BQ27Z558
    if (low_battery_state)
        substate = APP_SUBSTATE_LOW_BATTERY;
#endif
    // TODO passive charging overrides low battery

    change_led_state(new_state, substate, false);
}

led_driver::set_led_fn_t map_state_to_led_indication[][3] = {
    //                             Normal                                      Passive Charging                          Low Battery
    /* APP_STATE_CRASH          */ { &led_driver::set_orange_flash,            &led_driver::set_orange_flash,            &led_driver::set_orange_flash             },
    /* APP_STATE_POWER_DOWN     */ { &led_driver::set_orange_reboot_flash,     &led_driver::set_orange_reboot_flash,     &led_driver::set_orange_reboot_flash      },
    /* APP_STATE_FUEL_GAUGE_DFU */ { &led_driver::set_pink,                    &led_driver::set_pink,                    &led_driver::set_pink                     },
    /* APP_STATE_CHARGING       */ { &led_driver::set_very_dim_white,          &led_driver::set_very_dim_white,          &led_driver::set_very_dim_white           },
    /* APP_STATE_OMMOCOMM_DFU   */ { &led_driver::set_dim_white,               &led_driver::set_dim_white,               &led_driver::set_dim_white                },
    /* APP_STATE_DC             */ { &led_driver::set_cyan,                    &led_driver::set_cyan_purple_mssolid,     &led_driver::set_cyan_magenta_mssolid     },
    /* APP_STATE_IDLE           */ { &led_driver::set_white_fade,              &led_driver::set_white_purple_msfade,     &led_driver::set_white_magenta_msfade     },
    /* APP_STATE_CONNECTED      */ { &led_driver::set_blue_fade,               &led_driver::set_blue_purple_msfade,      &led_driver::set_blue_dim_magenta_msfade  },
    /* APP_STATE_SYNCH_LOST     */ { &led_driver::set_blue_flash,              &led_driver::set_blue_purple_msflash,     &led_driver::set_blue_dim_magenta_msflash },
    /* APP_STATE_SLEEP          */ { &led_driver::set_dim_blue_lp_flash,       &led_driver::set_dim_blue_lp_flash,       &led_driver::set_dim_magenta_lp_flash     },
    /* APP_STATE_WIRELESS_SYNCH */ { &led_driver::set_green_fade,              &led_driver::set_green_purple_msfade,     &led_driver::set_green_magenta_msfade     },
    /* APP_STATE_WIRED_SYNCH    */ { &led_driver::set_green,                   &led_driver::set_green_purple_mssolid,    &led_driver::set_green_magenta_mssolid    },
};

void change_led_state(application_state new_state, application_substates substate, bool immediate)
{
    APP_ERROR_CHECK_BOOL(new_state < std::size(map_state_to_led_indication));
    APP_ERROR_CHECK_BOOL(substate  < std::size(map_state_to_led_indication[0]));
    (main_led.*map_state_to_led_indication[new_state][substate])();
    if (immediate)
        main_led.flush_pending_tasks();
}

#ifdef OMMO_BQ25188
void enter_ship_mode()
{
    //Shutdown via ship mode
    if(data_mode != DATA_MODE_DISABLED)
        sensors_stop_state_machine();

    // Charger, go to ship mode
    comm_device *comm_dev;
    OMMO_APP_ERROR_CHECK(port_master_acquire(OMMO_BQ25188_PORT, IC_BUS_TYPE_I2C, OMMO_BQ25188_BUS_LOCATION, &comm_dev),
                         STRING("Battery charger comms initialization failure"), 0);

    OMMO_APP_ERROR_CHECK(bq25188_charger_instance.enable_ship_mode(comm_dev), STRING("Battery charger comms initialization failure"), 0);
    comm_dev->release();

    //We should shut down quickly, but wait before rebooting
    for (int i = 0; i < 1000; i++)
    {
        advanced_wdt_feed();
        nrf_delay_ms(1);
    }

    //Unreachable
    NVIC_SystemReset();
}
#endif

void low_vin_handler()
{
    //Shutdown via ship mode
    #ifdef OMMO_BQ25188
    enter_ship_mode();
    #endif

    //Power down
    #if defined OMMO_POWER_ON_PIN && defined OMMO_BUTTON_PIN
    nrfx_gpiote_out_task_trigger(OMMO_POWER_ON_PIN);
    #endif
}

#ifdef OMMO_BUTTON_PIN
void button_transition_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action, void *context)
{
    //Button pressed
    button_down = true;
}

void button_down_timer_handler(nrf_timer_event_t event_type, void* p_context)
{
    //Button has been down for 1/2s
#ifdef OMMO_POWER_ON_PIN
    if(button_initialized && !(nrfx_gpiote_in_is_set(OMMO_BUTTON_PIN)^OMMO_BUTTON_ACTIVE_STATE)) //Make sure button is still down
    {
        // Provide a visual indication to the operator to acknowledge the button press
        main_led.set_off();
        main_led.flush_pending_tasks();

        // Do not shut down via check_power_requirements().  See button_init: a back-up timer
        // will kill power if this function does not successfully shut down.
        shutdown();
    }
#endif
}
#endif

void gpio_init( void )
{
    #ifdef OMMO_CHRG_CHARGE_PIN
    nrf_gpio_cfg_input(OMMO_CHRG_CHARGE_PIN, NRF_GPIO_PIN_PULLUP);
    #endif

    #ifdef OMMO_CHARGE_ISET2 //Set to ISET
    nrf_gpio_pin_write(OMMO_CHARGE_ISET2, 0);
    nrf_gpio_cfg_output(OMMO_CHARGE_ISET2);
    #endif

    #ifdef OMMO_SENSOR_TO_VCON_PIN
    nrf_gpio_pin_write(OMMO_SENSOR_TO_VCON_PIN, 1); //ON
    nrf_gpio_cfg_output(OMMO_SENSOR_TO_VCON_PIN);
    #endif

    #ifdef OMMO_CHRG_TO_VCON_PIN
    nrf_gpio_pin_write(OMMO_CHRG_TO_VCON_PIN, 0); //OFF
    nrf_gpio_cfg_output(OMMO_CHRG_TO_VCON_PIN);
    #endif

    #ifdef OMMO_POWER_ON_PIN
    nrf_gpio_pin_write(OMMO_POWER_ON_PIN, 1);
    nrf_gpio_cfg_output(OMMO_POWER_ON_PIN);
    #endif

    #ifdef OMMO_SENSOR_ON_PIN
    nrf_gpio_pin_write(OMMO_SENSOR_ON_PIN, 1);
    nrf_gpio_cfg_output(OMMO_SENSOR_ON_PIN);
    nrf_delay_ms(10);
    #endif

    #ifdef OMMO_BUTTON_PIN
    nrf_gpio_cfg_input(OMMO_BUTTON_PIN, NRF_GPIO_PIN_PULLUP);
    #endif

    #ifdef OMMO_DEBUG_TX
    nrf_gpio_pin_write(OMMO_DEBUG_TX, 0);
    nrf_gpio_cfg_output(OMMO_DEBUG_TX);
    #endif

#ifdef OMMO_PINS_TO_GROUND
    const uint32_t pins_g[] = OMMO_PINS_TO_GROUND;
    for(uint8_t i = 0; i < sizeof(pins_g)/sizeof(uint32_t); i++)
    {
      nrf_gpio_pin_clear(pins_g[i]);
      nrf_gpio_cfg(pins_g[i], NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_H0H1, NRF_GPIO_PIN_NOSENSE);
    }
#endif

#ifdef OMMO_PINS_TO_VCC
    const uint32_t pins_v[] = OMMO_PINS_TO_VCC;
    for(uint8_t i = 0; i < sizeof(pins_v)/sizeof(uint32_t); i++)
    {
      nrf_gpio_pin_set(pins_v[i]);
      nrf_gpio_cfg(pins_v[i], NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_H0H1, NRF_GPIO_PIN_NOSENSE);
    }
#endif

#ifdef OMMO_DATA_EN_PIN
    nrf_gpio_pin_write(OMMO_DATA_EN_PIN, 1);
    //ATTENTION Needs strong drive for higher current load
    nrf_gpio_cfg(OMMO_DATA_EN_PIN, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_H0H1, NRF_GPIO_PIN_NOSENSE);
#endif

#ifdef OMMO_DEBUG_IO
    nrf_gpio_pin_write(OMMO_DEBUG_IO, 0);
    nrf_gpio_cfg_output(OMMO_DEBUG_IO);
#endif

#ifdef OMMO_EEPROM_AT25_CS
    nrf_gpio_pin_write(OMMO_EEPROM_AT25_CS, 1);
    nrf_gpio_cfg(OMMO_EEPROM_AT25_CS, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE);
#endif

#ifdef OMMO_DEBUG_1_PIN
    nrf_gpio_pin_clear(OMMO_DEBUG_1_PIN);
    nrf_gpio_cfg_output(OMMO_DEBUG_1_PIN);
#endif
#ifdef OMMO_DEBUG_2_PIN
    nrf_gpio_pin_clear(OMMO_DEBUG_2_PIN);
    nrf_gpio_cfg_output(OMMO_DEBUG_2_PIN);
#endif
#ifdef OMMO_DEBUG_3_PIN
    nrf_gpio_pin_clear(OMMO_DEBUG_3_PIN);
    nrf_gpio_cfg_output(OMMO_DEBUG_3_PIN);
#endif
#ifdef OMMO_DEBUG_4_PIN
    nrf_gpio_pin_clear(OMMO_DEBUG_4_PIN);
    nrf_gpio_cfg_output(OMMO_DEBUG_4_PIN);
#endif
#ifdef OMMO_DEBUG_5_PIN
    nrf_gpio_pin_clear(OMMO_DEBUG_5_PIN);
    nrf_gpio_cfg_output(OMMO_DEBUG_5_PIN);
#endif
#ifdef OMMO_DEBUG_6_PIN
    nrf_gpio_pin_clear(OMMO_DEBUG_6_PIN);
    nrf_gpio_cfg_output(OMMO_DEBUG_6_PIN);
#endif
}

#ifdef OMMO_BUTTON_PIN
void button_init()
{
    nrf_ppi_channel_t ppi_channel_button_pressed;

    if (!nrfx_gpiote_is_init())
    {
        APP_ERROR_CHECK(nrfx_gpiote_init());
    }

    //Setup button handler on press of button pin
    nrfx_gpiote_in_config_t button_in_config;
    button_in_config.is_watcher = false;
    button_in_config.hi_accuracy = true;
    button_in_config.pull = NRF_GPIO_PIN_PULLUP;
    button_in_config.sense = (OMMO_BUTTON_ACTIVE_STATE?NRF_GPIOTE_POLARITY_LOTOHI:NRF_GPIOTE_POLARITY_HITOLO);
    button_in_config.skip_gpio_setup = false;
    APP_ERROR_CHECK(nrfx_gpiote_in_init(OMMO_BUTTON_PIN, &button_in_config, button_transition_handler));
    nrfx_gpiote_in_event_enable(OMMO_BUTTON_PIN, true);

    #ifdef OMMO_POWER_ON_PIN
    nrfx_gpiote_out_config_t power_out_config = NRFX_GPIOTE_CONFIG_OUT_TASK_LOW;
    APP_ERROR_CHECK(nrfx_gpiote_out_init(OMMO_POWER_ON_PIN, &power_out_config));
    nrfx_gpiote_out_task_enable(OMMO_POWER_ON_PIN);
    #endif

    //Setup timmer for measuring down time
    nrfx_timer_config_t timer_cfg = NRFX_TIMER_DEFAULT_CONFIG;
    APP_ERROR_CHECK(nrfx_timer_init(&button_down_timer, &timer_cfg, button_down_timer_handler));
    nrfx_timer_compare(&button_down_timer, NRF_TIMER_CC_CHANNEL0, US_TO_TICKS_16MHZ(500000), true); //Long press, turn off leds and shutdown
    nrfx_timer_compare(&button_down_timer, NRF_TIMER_CC_CHANNEL1, US_TO_TICKS_16MHZ(1000000), false); //Super long press, hardware power kill

    //Start timer on button press
    APP_ERROR_CHECK(nrfx_ppi_channel_alloc(&ppi_channel_button_pressed));
    APP_ERROR_CHECK(nrfx_ppi_channel_assign(ppi_channel_button_pressed, nrfx_gpiote_in_event_addr_get(OMMO_BUTTON_PIN), nrfx_timer_task_address_get(&button_down_timer, NRF_TIMER_TASK_START)));
    APP_ERROR_CHECK(nrfx_ppi_channel_enable(ppi_channel_button_pressed));

    #ifdef OMMO_POWER_ON_PIN
    //Kill power on super long press
    APP_ERROR_CHECK(nrfx_ppi_channel_alloc(&ppi_channel_button_pressed));
    APP_ERROR_CHECK(nrfx_ppi_channel_assign(ppi_channel_button_pressed, nrfx_timer_compare_event_address_get(&button_down_timer, NRF_TIMER_CC_CHANNEL1), nrfx_gpiote_out_task_addr_get(OMMO_POWER_ON_PIN)));
    APP_ERROR_CHECK(nrfx_ppi_channel_enable(ppi_channel_button_pressed));
    #endif

    //Set power status - the button is most likely down but we will not indicate that in the power status register
    //  until the button is released for the first time
    button_initialized = false;
    power_status = (power_req_t)(power_status | POWER_REQ_NOT_BUTTON_SIGNAL);

    //Initialize with button down, it will be 'released' and 'initialized' in the main loop
    button_down = true;
    nrfx_timer_enable(&button_down_timer);
}
#endif

void timestamp_synch_lost_found(bool found, bool wired)
{
    //Update LED state
    set_leds();
}

void one_hundred_ms_timer_handler(void *context)
{
    one_hundred_ms_timer_event = true;
}
bool one_hundred_ms_event_pending()
{
    return one_hundred_ms_timer_event;
}
void one_hundred_ms_event_process()
{
    // Clear pending flag
    one_hundred_ms_timer_event = false;

    //Execute event queue one time
    one_hundred_ms_event_queue.execute_once();
}

// Increment hours counter sum every 1s
void one_hundred_ms_event_increment_hours_count()
{
static uint8_t increment_hours_count = 0;

    increment_hours_count++;
    if(increment_hours_count >= 10)
    {
        increment_hours_count = 0;

        device_info_reader_flash_increment_hours_counter_sum();
    }
}

#ifdef OMMO_BUTTON_PIN
void one_hundred_ms_event_check_button()
{
    //Watch for button to be released
    if(button_down && (nrfx_gpiote_in_is_set(OMMO_BUTTON_PIN)^OMMO_BUTTON_ACTIVE_STATE))
    {
        nrfx_timer_pause(&button_down_timer);
        nrfx_timer_clear(&button_down_timer);
        button_down = false;
        button_initialized = true;
    }
}
#endif

#ifdef OMMOCOMM_ENABLED

static uint32_t ommocomm_ping_count = 0;
static ommocomm_uarte *ommocomm_ping_dev = nullptr;

bool one_hundred_ms_event_ommocomm_ping_pending()
{
    ommocomm_ping_count++;
    if(ommocomm_ping_count < OMMOCOMM_IDLE_PING_INTERVAL) return false;
    return (power_status & POWER_REQ_OMMOCOMM_SAMPLER);
}

bool one_hundred_ms_event_ommocomm_ping_acquire()
{
    return port_master_acquire_ommocomm_direct(1, true, &ommocomm_ping_dev) == NRF_SUCCESS;
}

void one_hundred_ms_event_ommocomm_ping_process()
{
    ommocomm_ping_count = 0;
    if(!ping_ommocomm_uarte(ommocomm_ping_dev))
    {
        modify_power_status(false, POWER_REQ_OMMOCOMM_SAMPLER);
        check_power_requirements();
    }
}

void one_hundred_ms_event_ommocomm_ping_release()
{
    ommocomm_ping_dev->release();
    ommocomm_ping_dev = nullptr;
}

#endif

#ifdef OMMOCOMM_1WIRE
void one_hundred_ms_event_check_one_wire_connection()
{
    // If in 1-wire mode, evaluate when to power off
    if(power_status & POWER_REQ_1WIRE)
    {
        if(!comms_1wire.is_connected())
        {
            comms_1wire.uninit();
            modify_power_status(false, POWER_REQ_1WIRE);
            check_power_requirements();
        }
    }
}
#endif

#ifdef OMMO_BQ27Z558
void one_hundred_ms_event_check_battery_state()
{
    bool new_low_battery_state = fuel_gauge_instance.poll_below_charge_threshold();
    #if 0//def DEBUG_NRF  // Means to test that the LED indication will change if the pin changes
    low_battery_toggle_timer_ms += 100;
    if (low_battery_toggle_timer_ms >= 20000)
        low_battery_toggle_timer_ms = 0;
    new_low_battery_state = low_battery_toggle_timer_ms >= 10000;
    #endif
    if (low_battery_state != new_low_battery_state)
    {
        low_battery_state = new_low_battery_state;
        set_leds();
    }
}
#endif

#ifdef OMMOCOMM_ENABLED
bool ping_ommocomm_uarte(ommocomm_uarte * ommocomm)
{
    uint8_t wai;
    return (ommocomm->get_wai(&wai) == NRF_SUCCESS);
}
#endif

void usb_serial_event_handler(usb_serial_event_t event)
{
    switch(event)
    {
        case USB_SERIAL_EVENT_OPENED:
            //Disable direct comm or data modes, go to synch only
            disable_data_mode(true);
            if(!(power_status & POWER_REQ_1WIRE)) //TODO IS THIS NECESSSARY
                OMMO_APP_ERROR_CHECK(comms_esb_enter_synch_only_mode(), STRING("Wireless communication error"), 0);
            break;

        case USB_SERIAL_EVENT_CLOSED:
            //Go back to searching for a BS
            disable_data_mode(true);
            if(!(power_status & POWER_REQ_1WIRE)) //TODO IS THIS NECESSSARY
                OMMO_APP_ERROR_CHECK(comms_esb_enter_direct_comm_mode(), STRING("Wireless communication error"), 0);
            sensors_on_serial_closed();
            break;

        default:
            break;
    }

    //Update LED state
    set_leds();
}

void shutdown()
{
    //Flush current time to flash
    device_info_reader_flash_save_hours_counter_sum();

    //Shutdown everything we can (except ommocomm)
#ifdef OMMO_ADC_MONITOR
    adc_monitor_uninit();
#endif
    nrfx_power_usbevt_disable();
    timestamp_uninit();
    APP_ERROR_CHECK(app_timer_stop(one_hundred_ms_timer_id));
    sensors_stop_state_machine();

    uint8_t gpregret2 = nrf_power_gpregret2_get();
    uint16_t shutdown_count = 1;
    if ((gpregret2 & OMMO_GPREGRET2_MAGIC_NUMBER_MASK) == OMMO_GPREGRET2_MAGIC_NUMBER)
        shutdown_count = (gpregret2 & OMMO_GPREGRET2_SHUTDOWN_COUNT_MASK) + 1;

    if (shutdown_count > OMMO_GPREGRET2_SHUTDOWN_COUNT_MASK)
        shutdown_count = OMMO_GPREGRET2_SHUTDOWN_COUNT_MASK;
    nrf_power_gpregret2_set(OMMO_GPREGRET2_MAGIC_NUMBER | shutdown_count);

    // Delay "attempting to shut down" LED indication until the 6th shutdown attempt.
    if (shutdown_count >= 6)
        change_led_state(APP_STATE_POWER_DOWN);
    //main_led.uninit();

    #ifdef OMMO_DATA_EN_PIN
    nrf_gpio_pin_write(OMMO_DATA_EN_PIN, 0);
    #endif

   #ifdef OMMO_CHRG_TO_VCON_PIN
   //Disconnect battery from VCON
   nrf_gpio_pin_write(OMMO_CHRG_TO_VCON_PIN, 0); //OFF
   #endif

    #ifdef OMMO_SENSOR_TO_VCON_PIN
    nrf_gpio_pin_write(OMMO_SENSOR_TO_VCON_PIN, 0); //OFF
    #endif

    //Turn off power pin
    #ifdef OMMO_POWER_ON_PIN
    #ifdef OMMO_BUTTON_PIN
    nrfx_gpiote_out_task_trigger(OMMO_POWER_ON_PIN);
    #else
    nrf_gpio_pin_write(OMMO_POWER_ON_PIN, 0);
    #endif
    #endif

    //We should be shutting down
    //NRF_POWER->SYSTEMOFF = 1;

    //If the DATA line is disconnected, then BUCK_EN should be low.  Turning off POWER_ON should
    //have shut off the buck regulator.  If it did not, then DATA was not really disconnected,
    //and bounce noise or something may have interfered with communications that would normally
    //keep the device powered on.  Reboot and attempt to reconnect.
    //Note that BUCK_EN has a low pass filter: 100kR * 2.2uF = 220 ms tau
    for (int i = 0; i < 1000; i++)
    {
        advanced_wdt_feed();

        nrf_delay_ms(1);
    }

    NVIC_SystemReset();
}


void reset_shutdown_counter()
{
    nrf_power_gpregret2_set(0);
}


#ifdef OMMO_ADC_MONITOR_VCON
void charge_loop_check_vcon()
{
    static uint8_t vcon_lost_count = 0;

    // Make sure we are still connected to power
    if(adc_monitor_vcon() < OMMO_ADC_MONITOR_VCON_THRESHOLD)
    {
        vcon_lost_count++;
        if(vcon_lost_count > 10) //1000ms
        {
            //Flush current time to flash
            device_info_reader_flash_save_hours_counter_sum();

            //Reboot to check for any other devices
            NVIC_SystemReset();
        }
    }
    else
    {
        vcon_lost_count = 0;
    }
}

#ifdef OMMO_CHARGE_LED
static const uint8_t CHARGE_LED_UPDATE_INTERVAL = 5; // 100ms ticks; 0.5 second
static uint8_t charge_loop_process_count = CHARGE_LED_UPDATE_INTERVAL; // init to threshold so LED updates on first cycle
static comm_device *charge_led_charger_dev = nullptr;
static comm_device *charge_led_fuel_gauge_dev = nullptr;

bool charge_loop_update_leds_pending()
{
    charge_loop_process_count++;
    return charge_loop_process_count >= CHARGE_LED_UPDATE_INTERVAL;
}

bool charge_loop_update_leds_acquire()
{
    return port_master_acquire(OMMO_BQ25188_PORT, IC_BUS_TYPE_I2C, OMMO_BQ25188_BUS_LOCATION, &charge_led_charger_dev) == NRF_SUCCESS;
}

void charge_loop_update_leds_release()
{
    charge_led_fuel_gauge_dev->release();
    charge_led_fuel_gauge_dev = nullptr;
}

void charge_loop_update_leds_process()
{
    charge_loop_process_count = 0;

    // Fuel gauge and charger share the same TWI bus; acquire/release each before acquiring the other
    // Although the fuel gauge updates its register once per second, we poll every 0.5s to make the LED appear faster when switching ports
    bool is_charger_charging;
    bq25188_charger_instance.is_charging(charge_led_charger_dev, &is_charger_charging);
    charge_led_charger_dev->release();
    charge_led_charger_dev = nullptr;

    uint16_t soc;
    OMMO_APP_ERROR_CHECK(port_master_acquire(OMMO_FUEL_GAUGE_PORT, IC_BUS_TYPE_I2C, OMMO_FUEL_GAUGE_BUS_LOCATION, &charge_led_fuel_gauge_dev),
                         STRING("Charge LED: fuel gauge acquire failure"), 0);
    fuel_gauge_instance.read_u16(charge_led_fuel_gauge_dev, BQ27Z558_REG_STATE_RSOC, &soc);
    // charge_led_fuel_gauge_dev released by release()

    // [soc > 98%] or [charger finishes charging] will be considered as fully charged
    if(soc >= 98 || !is_charger_charging)
    {
        soc = 100;
    }

    // fill led bar, each bar represents 25% capacity
    uint16_t soc_level;
    soc_level = soc / 25;

    //Update status leds
    // for led_idx on the strip:
    // < soc_level: solid green
    // == soc_level: flashing green
    // > soc_level: off
    led_animator_operation op;
    led_animator_params led_strip_params;
    for(uint8_t led_idx=0; led_idx<OMMO_CHARGE_LED_NUM_LEDS; led_idx++)
    {
        memset(&led_strip_params, 0x00, sizeof(led_strip_params));

        if(led_idx < soc_level)
        {
            op = LED_ANIMATOR_ON;
            led_strip_params.on.on_color = MSIU_CHARGING_DOCK_GREEN_RGB;
        }
        else if(led_idx == soc_level)
        {
            op = LED_ANIMATOR_FLASH_SIMPLE;
            led_strip_params.flash_simple.on_color     = MSIU_CHARGING_DOCK_GREEN_RGB;
            led_strip_params.flash_simple.delay_time_ms = 0;
            led_strip_params.flash_simple.on_time_ms   = MSIU_CHARGING_DOCK_FLASH_ON_MS;
            led_strip_params.flash_simple.off_time_ms  = MSIU_CHARGING_DOCK_FLASH_OFF_MS;
        }
        else    // led_idx > soc_level
        {
            op = LED_ANIMATOR_OFF;
            //no need to fill in params
        }

        led_strip_instance.set_led_operation(OMMO_CHARGE_LED_NUM_LEDS - 1 - led_idx, op, &led_strip_params);
    }
}
#endif

void charge_loop()
{
    //Set main led to charge color
    in_charge_loop = true;
    set_leds();

    //Init port led instances
    OMMO_APP_ERROR_CHECK(charge_led_serial_instance.acquire_spi(OMMO_CHARGE_LED_SPI_BUS_INDEX), 
                         STRING("Battery charge led comms initialization failure"), 0);
    led_strip_instance.init(&charge_led_serial_instance, &led_strip_timer_id, OMMO_CHARGE_LED_NUM_LEDS, 100);

    //Register task loop to 100ms timer queue
    one_hundred_ms_event_queue.reset();
    APP_ERROR_CHECK(one_hundred_ms_event_queue.register_task(one_hundred_ms_event_increment_hours_count));
    APP_ERROR_CHECK(one_hundred_ms_event_queue.register_task(charge_loop_check_vcon));
    APP_ERROR_CHECK(one_hundred_ms_event_queue.register_task(charge_loop_update_leds_pending,
                                                             charge_loop_update_leds_process,
                                                             charge_loop_update_leds_acquire,
                                                             charge_loop_update_leds_release));

    //Create an event queue and add functions
    main_event_queue.reset();
    APP_ERROR_CHECK(main_event_queue.register_task(advanced_wdt_feed)); //Feed the watchdog
    APP_ERROR_CHECK(main_event_queue.register_task(one_hundred_ms_event_pending, one_hundred_ms_event_process));
    APP_ERROR_CHECK(main_led.add_tasks_to_event_queue(&main_event_queue));
    APP_ERROR_CHECK(led_strip_instance.add_tasks_to_event_queue(&main_event_queue));

    //We will stay in this loop until the charger is removed, at which point we will shutdown
    main_event_queue.execute_forever_with_sleep();
}

void dead_battery_monitor_loop()
{
    // VBAT is below the minimum charging threshold. Monitor VCON and reboot when charger is removed.
    in_charge_loop = true;
    set_leds();

#ifdef OMMO_CHARGE_LED
    OMMO_APP_ERROR_CHECK(charge_led_serial_instance.acquire_spi(OMMO_CHARGE_LED_SPI_BUS_INDEX),
                         STRING("Battery charge led comms initialization failure"), 0);
    led_strip_instance.init(&charge_led_serial_instance, &led_strip_timer_id, OMMO_CHARGE_LED_NUM_LEDS, 100);

    // Flash orange for dead battery LED indication
    led_animator_params led_strip_params =
    {
        .flash_simple =
        {
            .on_color      = MSIU_CHARGING_DOCK_ORANGE_RGB,
            .delay_time_ms = 0,
            .on_time_ms    = MSIU_CHARGING_DOCK_FLASH_ON_MS,
            .off_time_ms   = MSIU_CHARGING_DOCK_FLASH_OFF_MS,
        }
    };
    for(uint8_t led_idx = 0; led_idx < OMMO_CHARGE_LED_NUM_LEDS; led_idx++)
    {
        led_strip_instance.set_led_operation(led_idx, LED_ANIMATOR_FLASH_SIMPLE, &led_strip_params);
    }
#endif

    one_hundred_ms_event_queue.reset();
    APP_ERROR_CHECK(one_hundred_ms_event_queue.register_task(one_hundred_ms_event_increment_hours_count));
    APP_ERROR_CHECK(one_hundred_ms_event_queue.register_task(charge_loop_check_vcon));

    main_event_queue.reset();
    APP_ERROR_CHECK(main_event_queue.register_task(advanced_wdt_feed));
    APP_ERROR_CHECK(main_event_queue.register_task(one_hundred_ms_event_pending, one_hundred_ms_event_process));
    APP_ERROR_CHECK(main_led.add_tasks_to_event_queue(&main_event_queue));
#ifdef OMMO_CHARGE_LED
    APP_ERROR_CHECK(led_strip_instance.add_tasks_to_event_queue(&main_event_queue));
#endif

    main_event_queue.execute_forever_with_sleep();
}

#endif // OMMO_ADC_MONITOR_VCON

void charger_enable()
{
#ifdef OMMO_BQ25188
    comm_device *comm_dev;
    OMMO_APP_ERROR_CHECK(port_master_acquire(OMMO_BQ25188_PORT, IC_BUS_TYPE_I2C, OMMO_BQ25188_BUS_LOCATION, &comm_dev),
                         STRING("Battery charger enable failure"), 0);
    OMMO_APP_ERROR_CHECK(bq25188_charger_instance.enable_charge_current(comm_dev, true),
                         STRING("Battery charger enable failure"), 0);
    comm_dev->release();
#endif
}

void charger_init()
{
#ifdef OMMO_BQ25188
    // Charger init              
    comm_device *comm_dev;
    OMMO_APP_ERROR_CHECK(port_master_acquire(OMMO_BQ25188_PORT, IC_BUS_TYPE_I2C, OMMO_BQ25188_BUS_LOCATION, &comm_dev),
                         STRING("Battery charger comms initialization failure"), 0);

    OMMO_APP_ERROR_CHECK(bq25188_charger_instance.init(comm_dev), STRING("Battery charger comms initialization failure"), 0);

#if defined(OMMO_ADC_MONITOR_VIN) && defined(OMMO_ADC_MONITOR_VIN_DEAD_BATTERY)
    // Disable charger immediately on power up - will be re-enabled only after VBAT check
    OMMO_APP_ERROR_CHECK(bq25188_charger_instance.enable_charge_current(comm_dev, false),
                         STRING("Battery charger disable on startup failure"), 0);
#endif

    comm_dev->release();
#endif  //OMMO_BQ25188
}

void fuel_gauge_init()
{
#ifdef OMMO_BQ27Z558
    ret_code_t err_code = NRF_SUCCESS;

    comm_device *comm_dev;
    OMMO_APP_ERROR_CHECK(port_master_acquire(OMMO_FUEL_GAUGE_PORT, IC_BUS_TYPE_I2C, OMMO_FUEL_GAUGE_BUS_LOCATION, &comm_dev),
                         STRING("Fuel gauge initialization failure"), 0);

    OMMO_APP_ERROR_CHECK(port_master_acquire_pins(PORT_PIN_TO_MASK(OMMO_FUEL_GAUGE_INT)), STRING("Fuel gauge initialization failure"), 0);

    OMMO_APP_ERROR_CHECK(fuel_gauge_instance.init(comm_dev, OMMO_FUEL_GAUGE_INT), STRING("Fuel gauge initialization failure"), 0);

    if(!fuel_gauge_instance.is_sealed(comm_dev))
    {
        change_led_state(APP_STATE_FUEL_GAUGE_DFU);

        // ommo_delay_ms is used inside chip_programm
        // need to add wdt feed so it does not time out
        main_event_queue.reset();
        APP_ERROR_CHECK(main_event_queue.register_task(advanced_wdt_feed)); //Feed the watchdog

        #if 1
        uint8_t retry = 0;
        while(retry < 2)
        {
            err_code = fuel_gauge_instance.chip_programm(comm_dev, GM_BIN, GM_BIN_SIZE);
            if(err_code == NRF_SUCCESS)
            {
                break;
            }

            nrf_delay_ms(500);
            retry++;
        }

        if(retry >= 2)
        {
            OMMO_APP_ERROR_CHECK(NRF_ERROR_INTERNAL, STRING("Fuel gauge golden image flash failure"), 0);
        }
        #else
        // temporary golden image flash solution
        err_code = fuel_gauge_instance.chip_programm(comm_dev, GM_BIN, GM_BIN_SIZE);
        #endif

        // Reset again so the registers load the updated flash data
        OMMO_APP_ERROR_CHECK(fuel_gauge_instance.reset(comm_dev), STRING("Fuel gauge init fails after golden image update"), 0);
        OMMO_APP_ERROR_CHECK(fuel_gauge_instance.reset_btp_thresholds(comm_dev), STRING("Fuel gauge init fails after golden image update"), 0);

        set_leds();
    }

    comm_dev->release();

#endif
}


#ifdef OMMOCOMM_1WIRE

static void msiu_1wire_packet_received(uint8_t *buffer, uint16_t length, const void *context)
{
    // The 1-wire interface is intended to emulate a USB connection.  The device is connected to an
    // adapter with a USB CDC instance that simply relays USB packets over 1-wire to the device.
    uint32_t response_length = process_usb_serial_packet_received(buffer, length, tx_buffer, PACKET_RESPONSE_BUFFER_SIZE);

    if(response_length == 0) //No response, must be an unknown command
        response_length = fill_in_ack_packet(tx_buffer, OMMO_ACK_UNKNOWN_COMMAND);

    comms_1wire.send_command_packet_async(tx_buffer, response_length);
}

static void msiu_1wire_event_handler(comms_uarte_siu_1wire::event_t event)
{
    switch (event)
    {
    case comms_uarte_siu_1wire::EVENT_OPENED:
        usb_serial_event_handler(USB_SERIAL_EVENT_OPENED);
        break;

    case comms_uarte_siu_1wire::EVENT_CLOSED:
        usb_serial_event_handler(USB_SERIAL_EVENT_CLOSED);
        break;
    }
}

#endif  // OMMOCOMM_1WIRE


ret_code_t disrupt_realtime_operation(uint16_t timeout_ms)
{
    if (disrupt_realtime_operation_active_count++ != 0)
        return NRF_SUCCESS;

    if (usb_serial_get_state() == USB_SERIAL_STATE_OPENED)
    {
        (void)comms_esb_enter_idle_mode();
    }
    else if (!(power_status & POWER_REQ_1WIRE))
    {
        ret_code_t err_code = comms_esb_temporarily_disconnect_direct_comm_mode(timeout_ms);
        if (err_code != NRF_SUCCESS)
        {
            disrupt_realtime_operation_active_count--;
            return err_code;
        }
    }

    return NRF_SUCCESS;
}

void resume_realtime_operation()
{
    if (--disrupt_realtime_operation_active_count != 0)
        return;

    if (usb_serial_get_state() == USB_SERIAL_STATE_OPENED)
    {
        (void)comms_esb_enter_synch_only_mode();
    }
    else if (!(power_status & POWER_REQ_1WIRE))
    {
        (void)comms_esb_resume_direct_comm_mode_after_temporary_disconnection();
    }
}

bool realtime_operation_requested()
{
    return disrupt_realtime_operation_active_count == 0;
}

int main(void)
{
#if ((defined(NRF52833_XXAA) || defined(NRF52833_XXAB)) && defined(DISABLE_PROTECTION))
    /* nRF52833 has some protection enabled by default, and once a power cycle the device is locked
     * That code keeps the device unlocked */
    if ((NRF_UICR->APPROTECT & UICR_APPROTECT_PALL_Msk) == (UICR_APPROTECT_PALL_Msk))
    {
        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {}
        NRF_UICR->APPROTECT = (UICR_APPROTECT_PALL_HwDisabled << UICR_APPROTECT_PALL_Pos);
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {}
        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {}
        NVIC_SystemReset();
    }
    NRF_APPROTECT->DISABLE = NRF_UICR->APPROTECT;
#endif

    //Init globals
    data_mode = DATA_MODE_DISABLED;
    power_status = POWER_REQ_NONE;
    tx_count = 0;
    skipped_tx_count = 0;
    packet_id = 0xFF;

    //Check for watchdog reset and go to error handler if the dog was the cause
    advanced_wdt_check_boot_and_log();

    //LEDs (must be BEFORE crash reboot check)
#ifdef OMMO_LED_ENABLE
    nrf_gpio_pin_write(OMMO_LED_ENABLE, 1);
    nrf_gpio_cfg_output(OMMO_LED_ENABLE);
#endif
    main_led.init(OMMO_LED_PWM, OMMO_LED_RED, OMMO_LED_GREEN, OMMO_LED_BLUE, NRFX_PWM_PIN_NOT_USED, OMMO_LED_ACTIVE_LOW);

    //Init app timer (must be BEFORE crash reboot check)
    app_timer_init();

    // Crash reboot check
    crash_reboot_check();

    //Initialize WDT
    advanced_wdt_init(NULL, 1000, NRF_WDT_BEHAVIOUR_PAUSE_SLEEP_HALT);
    advanced_wdt_start();

    power_init();

    //Clocks
    clocks_start();

    //Init gpio
    gpio_init();

#ifdef OMMO_TRACE_ENABLED
    trace_init();
#endif

    //Init power off button hardware
#ifdef OMMO_BUTTON_PIN
    button_init();
#endif

    //Init rtc timer
    rtc_timer_init();

    //Init port master
    port_master_init();

    //Timestamp timer
    timestamp_init_with_internal_sample_event(3);

    ////TEST
    //sensors_init(&sample_set_ready);
    //sensors_scan_bus_generate_data_descriptor(tx_buffer, PACKET_RESPONSE_BUFFER_SIZE);
    //sensors_start_state_machine();
    //while(true) __WFE();

#ifdef OMMO_ADC_MONITOR
    //Init adc monitor
    adc_monitor_init();

#ifdef OMMO_ADC_MONITOR_VIN
    adc_monitor_set_low_vin_callback(low_vin_handler);
#endif
#endif //OMMO_ADC_MONITOR

    //Init general task loop timer, which is also used by charge_loop
    APP_ERROR_CHECK(app_timer_create(&one_hundred_ms_timer_id, APP_TIMER_MODE_REPEATED, one_hundred_ms_timer_handler));
    APP_ERROR_CHECK(app_timer_start(one_hundred_ms_timer_id, APP_TIMER_TICKS(100), NULL));

    //Check current vbus condition and turn on USB power monitor
    nrfx_power_usbevt_config_t p_config;
    p_config.handler = nrfx_power_usb_event_handler;
    nrfx_power_usbevt_init(&p_config);
    nrfx_power_usbevt_enable();
    bool usb_power_detected = (nrfx_power_usbstatus_get() != NRFX_POWER_USB_STATE_DISCONNECTED);
    modify_power_status(usb_power_detected, POWER_REQ_USB);

    //Initialize the charger
    charger_init();

#ifdef OMMO_ADC_MONITOR_VCON
    //Check for charger power on vcon (also waits for valid ADC data)
    bool vcon_has_power = adc_monitor_check_vcon_voltage_present();
#ifdef SIMULATE_BATTERY_POWER
    vcon_has_power = false;
#endif

    //Check VBAT before fuel gauge init - BQ27Z558 I2C fails below ~1.6V causing a crash
    if(!usb_power_detected && vcon_has_power)
    {
#if defined(OMMO_ADC_MONITOR_VIN) && defined(OMMO_ADC_MONITOR_VIN_DEAD_BATTERY)
        bool vbat_ok = (adc_monitor_vin() >= OMMO_ADC_MONITOR_VIN_DEAD_BATTERY);
#else
        bool vbat_ok = true;
#endif
        if(!vbat_ok)
            dead_battery_monitor_loop(); //Does not return
    }
#endif // OMMO_ADC_MONITOR_VCON

    //Initialize the fuel gauge if available
    fuel_gauge_init();

#ifdef OMMO_ADC_MONITOR_VCON
    //Enter the charge loop if VCON power is present (vbat is safe if we reach here)
    if(!usb_power_detected && vcon_has_power)
    {
        charger_enable();
        charge_loop(); //Does not return
    }
#endif

#ifdef OMMOCOMM_1WIRE
    //Detect 1-wire connection.  DATA_EN was enabled by gpio_init.
    comms_1wire.init(&comms_1wire_timer, &comms_1wire_timer_timer_id, &msiu_1wire_packet_received, NULL, OMMOCOMM_UARTE_DATA_PIN, 1);

    //Register tasks to event queue that executes every 100ms queue
    one_hundred_ms_event_queue.reset();
    static uint8_t one_wire_connection_timeout_count = 0;
    APP_ERROR_CHECK(one_hundred_ms_event_queue.register_task([]() static {one_wire_connection_timeout_count++;}));
    
    //Register tasks for the one wire connection check loop event queue
    main_event_queue.reset();
    APP_ERROR_CHECK(main_event_queue.register_task(advanced_wdt_feed)); //Feed the watchdog
    APP_ERROR_CHECK(main_event_queue.register_task(one_hundred_ms_event_pending, one_hundred_ms_event_process));
    APP_ERROR_CHECK(comms_1wire.add_tasks_to_event_queue(&main_event_queue));
    while(one_wire_connection_timeout_count<3 && !comms_1wire.is_connected())
    {
        //Execute event queue tasks
        main_event_queue.execute_once();
    }
    if(comms_1wire.is_connected())
        modify_power_status(true, POWER_REQ_1WIRE);
    else
        comms_1wire.uninit();
#else // !OMMOCOMM_1WIRE
    //Init synch uarte
    comms_uarte_init(OMMO_COMMS_UARTE_SYNCH_PIN);

    #ifdef OMMO_ADC_MONITOR_USBC
    adc_monitor_set_usbc_callback(usbc_debug_mode_change);
    #endif
#endif // OMMOCOMM_1WIRE

    //Init sensor sampler
    sensors_init(&sample_set_ready);

    //Tie in hot plug events
    APP_ERROR_CHECK(sensors_add_event_callback(&sensor_event_handler));

#ifdef OMMO_INCLUDE_TIMESTAMP_OFFSET
    sensors_set_data_descriptor_header_format(DATA_HEADER_FORMAT_TS_TSO);
#endif

    // Update any ommocomm virtual devices that may be attached and update power status
#ifdef OMMOCOMM_ENABLED
#ifndef OMMOCOMM_1WIRE
    // Give coprocessor time to startup
    nrf_delay_ms(OMMOCOMM_REBOOT_TIME);
#endif
    sensors_ommocomm_check_and_update_all_ports();
#endif

    //Make sure we have a reason to be "alive", otherwise shutdown
    if(!(power_status & OMMO_POWER_REQUIREMENTS))
        shutdown();
    reset_shutdown_counter();

    // Delay possible synch lost LED indication to avoid interfering with APP_STATE_POWER_DOWN indication
    timestamp_set_synch_lost_found_callback(timestamp_synch_lost_found);

    //Init ESB
    comms_esb_init(comms_esb_direct_comms_packet_received, comms_esb_event);
    if (!(power_status & POWER_REQ_1WIRE))
        comms_esb_enter_direct_comm_mode(); //Start looking for a BS

    // Init hours counter sum if needed
    (void)device_info_reader_flash_init_hours_counter_sum();

    //Init USB serial
    usbd_main_init();
    usb_serial_init(usb_serial_packet_received, usb_serial_event_handler);
#ifdef OMMO_USB_STAGGER_SLOT_COUNT
    // Stagger USB enumeration: asserting D+ simultaneously across many devices overwhelms the Windows
    // USB stack and causes error code 10. Derive a deterministic slot from FICR device ID.
    uint32_t stagger_ms = (NRF_FICR->DEVICEID[0] % OMMO_USB_STAGGER_SLOT_COUNT) * 30;
    for(uint32_t i = 0; i < stagger_ms; i++) {
        nrf_delay_ms(1);
        advanced_wdt_feed();
    }
#endif
    usbd_main_start();

#ifdef OMMOCOMM_1WIRE
    comms_1wire.set_event_handler(msiu_1wire_event_handler);
#endif

    //Update LED state
    set_leds();

#ifdef EMI_AUTO_SAMPLE_SENSOR
    sensors_scan_bus_generate_data_descriptor(packet_response_buffer, PACKET_RESPONSE_BUFFER_SIZE);
    if(sensors_get_total_attached() > 0)
        sensors_start_state_machine();
#endif

    //Register tasks to event queue that executes every 100ms queue
    one_hundred_ms_event_queue.reset();
    APP_ERROR_CHECK(one_hundred_ms_event_queue.register_task(one_hundred_ms_event_increment_hours_count));
#ifdef OMMO_BUTTON_PIN
    APP_ERROR_CHECK(one_hundred_ms_event_queue.register_task(one_hundred_ms_event_check_button));
#endif
#ifdef OMMOCOMM_ENABLED
    APP_ERROR_CHECK(one_hundred_ms_event_queue.register_task(one_hundred_ms_event_ommocomm_ping_pending,
                                                             one_hundred_ms_event_ommocomm_ping_process,
                                                             one_hundred_ms_event_ommocomm_ping_acquire,
                                                             one_hundred_ms_event_ommocomm_ping_release));
#endif
#ifdef OMMOCOMM_1WIRE
    APP_ERROR_CHECK(one_hundred_ms_event_queue.register_task(one_hundred_ms_event_check_one_wire_connection));
#endif
#ifdef OMMO_BQ27Z558
    APP_ERROR_CHECK(one_hundred_ms_event_queue.register_task(one_hundred_ms_event_check_battery_state));
#endif

    //Register tasks with main loop event queue
    main_event_queue.reset();
    APP_ERROR_CHECK(main_event_queue.register_task(advanced_wdt_feed)); //Feed the watchdog
    APP_ERROR_CHECK(main_event_queue.register_task(one_hundred_ms_event_pending, one_hundred_ms_event_process));
    APP_ERROR_CHECK(usb_serial_add_tasks_to_event_queue(&main_event_queue));
    APP_ERROR_CHECK(comms_esb_add_tasks_to_event_queue(&main_event_queue));
    APP_ERROR_CHECK(main_led.add_tasks_to_event_queue(&main_event_queue));
    APP_ERROR_CHECK(sensors_add_tasks_to_event_queue(&main_event_queue));
#ifdef OMMO_ADC_MONITOR
    APP_ERROR_CHECK(adc_monitor_add_tasks_to_event_queue(&main_event_queue));
#endif
#ifdef OMMOCOMM_1WIRE
    APP_ERROR_CHECK(comms_1wire.add_tasks_to_event_queue(&main_event_queue));
#endif

    //Main work loop, execute all tasks and sleep until something new happens
    main_event_queue.execute_forever_with_sleep();
}
