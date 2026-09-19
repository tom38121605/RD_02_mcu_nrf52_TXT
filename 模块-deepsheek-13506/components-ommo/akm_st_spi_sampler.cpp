/**
 * Copyright (c) 2019, Ommo Technologies
 *
 * All rights reserved.
 *
 *
 */

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "nrf_gpio.h"
#include "nrf_delay.h"

#include "nrfx_spim.h"
#include "nrfx_timer.h"
#include "nrfx_ppi.h"
#include "nrfx_gpiote.h"
#include "nrf_gpiote.h"

#include "app_timer.h"

#include "pb_encode.h"
#include "pb_decode.h"

#include "ommo_fifo_uint32.h"
#include "port_master.hpp"
#include "comm_device.hpp"
#include "comm_device_guard.hpp"
#include "device_info_parser.h"
#include "device_info_reader_m24c16.hpp"
#include "timestamp_timer.hpp"
#include "ommo_config.h"
#include "ommo_fw.pb.h"
#include "device_info_command_processor.hpp"
#include "ommo_app_error.h"
#include "akm_st_spi_sampler.hpp"

#include "device_info_reader_ommocomm.hpp"
#include "event_queue_manager.hpp"
#include "spim_basic.hpp"
#include "spim_twi_basic.hpp"
#include "array_container.hpp"

#ifdef OMMO_POWER_REQUIREMENTS
#include "main.hpp"
#endif

#ifdef OMMO_PORT_LED
#include "serial_led_spi.hpp"
#endif

#include "LSM6DSV_SPI_sampler.hpp"

#ifdef OMMOCOMM_ENABLED
#include "ommocomm_types.h"
#include "ommocomm_automode.h"  //for audo_mode_peripheral structs
#include "ommocomm_uarte.hpp"
#endif

#ifdef OMMO_BQ27Z558
#include "bq27z558_twi_driver.hpp"
#endif

/*
Requires: sample_timer.c


//Standalone device bool definition
//If true:
//    a device will be generated for this port if something is connected.
//If false:
//    all devices on all ports with the same device_type are combined.
//    If there is an eeprom no eeprom the firmware will report the stored sensor_id.
//    If there is not an eeprom then sensor_id will report the port number the sensor is attached to
//        The port_id is not necessarily unique, so on ports with multiple sensors an EEPROM is really required
//    The port_id will report the lowest port of all ports used.

Description:

Samples AK09940 and LSM6DS33 sensors with timing based on a sample_timer event.  The sampled data is
passed back via a callback function.

*** IMPORTANT *** The callback function is called shortly after a sample event happens.  However it sends back
data sampled at the LAST sample event while it is sampling the new data.  It always returns data from one
sample event behind.

*** IMPORTANT *** The SPI interrupt priority can be set via OMMO_PM_SPI_IRQ_PRIORITY, but be careful lowering the priority can
issues.  When the SPI interrupt goes off we need to make it to our interrupt and set up the next packet within
SECOND_SAMPLE_TIMER_DELAY (512) cycles.  If not, data will be incorrect.

Sensor sample timing is as follows:

T=0: Encoder sampled
T=0: SPI MAG start packet begins on all CS channels with a mag and on all SPI channels
T=128: SPI MAG start packet finishes on sensor 1 on all channels
SPI read of sensor 1 MAG on all channels.  This is reading the last sensor sample data while the sensor is sampling new data.
if enabled, SPI read of sensor 2 MAG on all channels.  This is reading the last sensor sample data while the sensor is sampling new data.
if enabled, SPI read of sensor 3 MAG on all channels.  This is reading the last sensor sample data while the sensor is sampling new data.
if enabled, SPI read of sensor 1 IMU on all channels
if enabled, SPI read of sensor 2 IMU on all channels
if enabled, SPI read of sensor 3 IMU on all channels
T=16000 repeat

Known concerns:

If we have multiple spi busses with multiple sensors will we have time to set up the SS pins before the PPI triggers the transfer?
*/
static void sample_timer_event_handler(nrf_timer_event_t event_type, void* p_context);
static void spi_event_handler(comm_device * comm_dev, nrfx_spim_evt_t const *p_event, void *p_context);
static void hot_plug_idle_timer_handler(void *p_context);
static void ommocomm_op_complete_callback(ommocomm_event_t event, void *p_context);
static void sensors_state_change();
static void evaluate_hot_plug_data();
static void swap_sample_buffers_and_flag_sample_callback();
static void sensors_schedule_i2c_set_reset(uint32_t prev_timestamp_bs_units);
static void sensors_set_ppi_start_transfer_event_addr_and_enable(uint32_t event);

static void sensors_start_spi_sensor_data_read(uint8_t spi_channel, uint8_t ic_data_index, comm_device_trx_flags_t flags = COMM_DEVICE_FLAGS_NONE);
static uint32_t sensors_sensor_data_read_blocking(comm_device *comm_device_ptr, uint8_t *output_buffer, uint16_t output_buffer_len, uint8_t *output_bytes_written, uint8_t ic_dict_index);
static void sensors_setup_and_start_imu_sensor_data_read();
static void sensors_read_mag_id(uint8_t spi_channel);

static uint8_t find_ic_data_dictionary_index(ICType ic_type);
static ret_code_t scan_port_for_devices(uint8_t port, bool is_virtual_port, ICBusType *ic_bus_type, uint8_t *ic_ss_index_bus_location, ICType *ic_type, uint8_t *num_ics);
static uint16_t sensors_scan_bus_generate_data_descriptor_return_error(uint8_t packet_id_request_buffer[], DeviceGroupDescriptorProto *dgdp, OmmoAck ack_code);
static void sensor_port_pins_test_set_port_pin_output(uint8_t *all_pin_set, uint8_t *port_pin_set, uint8_t *cs0_pin_set, uint8_t pin_under_test, bool value);
static uint32_t* sensors_add_task_to_ppi_start_transfer(uint32_t task_addr);

static ICType sensors_identify_ti_i2c_temp_sensor(comm_device *comm_device_ptr, uint8_t addr);

static void sensors_continue_idle_hot_plug_check();
static void sensors_state_machine_acquire_resources();
static void sensors_state_machine_release_resources_flag_stopped();

static ret_code_t sensors_clean_start_transfer_tasks();
static ret_code_t sensors_add_start_transfer_task(uint32_t task_addr);

#ifdef OMMOCOMM_ENABLED
static uint8_t   ommocomm_uarte_tx_buf[256];
static uint8_t   ommocomm_uarte_rx_buf[256];

static ommocomm_event_t ommocomm_last_sample_complete_event = OMMOCOMM_EVENT_SUCCESS;

static uint8_t ommocomm_sample_error_count;

// we need 3 iteration(last skipped pkt 3->2, pkt after skip 2->1, 2nd pkt after skip 1->0(action to skip))
static const uint8_t ommocomm_packet_malformed_after_skipped = 3; 
static uint8_t ommocomm_packet_malformed_countdown = 0; // only checked when value != 0
#endif //OMMOCOMM_UARTE_ENABLED
static bool ommocomm_op_complete_flag = true;

//Buffers
static uint8_t       m_spi_rx_buf[OMMO_PM_SPI_BUSSES_COUNT][OMMO_SENSOR_SPI_BUFFER_SIZE];

//PPI globals
ArrayContainerAllocated<volatile uint32_t *, OMMO_PM_MAX_PERIPHERALS_COUNT * 2> start_transfer_tasks_assigned;
ArrayContainerAllocated<volatile uint32_t *, OMMO_PM_MAX_PERIPHERALS_COUNT * 2> start_transfer_tasks_free;
ArrayContainerAllocated<nrf_ppi_channel_t, OMMO_PM_MAX_PERIPHERALS_COUNT> ppi_channel_start_transfer;

static nrf_ppi_channel_group_t sampling_ppi_start_group;
static uint32_t sensor_set_sample_time_offset_event;

//Callback function
static sensors_callback_func_type sample_set_ready_callback;

//IC indexes in ic_data array, indexes here represents ic_data entities, use carefully.
#define IC_DATA_DISABLED_INDEX 1

//Sensor information dictionary
volatile ic_data_dictionary_entry ic_data[] =   //volatile to ensure the struct in ram
{
    {IC_TYPE_NONE, false}, //Nothing present
    {IC_TYPE_NONE, true}, //Something present, but disabled
    {IC_TYPE_LSM6DSM, false, false, 1, {{IMU_DSX_DATA_LENGTH_BASE, IMU_DSX_DATA_OFFSET, {IMU_DSX_READ_REG_MASK(IMU_DSX_OUTX_L_G)}, 1, IMU_DSX_DATA_LENGTH_BASE + IMU_DSX_DATA_OFFSET}}},
    {IC_TYPE_LSM6DSV, false, false, 1, {{IMU_DSX_DATA_LENGTH_BASE, IMU_DSX_DATA_OFFSET, {IMU_DSX_READ_REG_MASK(IMU_DSX_OUTX_L_G)}, 1, IMU_DSX_DATA_LENGTH_BASE + IMU_DSX_DATA_OFFSET}}},
    {IC_TYPE_MEMSIC_5983, false, true, 1, {{MMC5983_DATA_LENGTH, MMC5983_DATA_OFFSET, {MMC5983_READ_REG_MASK(MMC5983_XOUT_REG)}, 1, MMC5983_DATA_READ_LENGTH}}},
    {IC_TYPE_ICM_42605, false, false, 1, {{IMU_42605_DATA_LENGTH, IMU_42605_DATA_OFFSET, {IMU_42605_READ_REG_MASK(IMU_42605_REG_ACCEL_DATA_X1)}, 1, IMU_42605_DATA_READ_LENGTH}}},
    {IC_TYPE_TMP126, false, false, 1, {{TMP126_DATA_LEN, TMP126_DATA_OFFSET, {TMP126_CMD_READ, TMP126_TEMP_RESULT_REG}, 2, TMP126_DATA_READ_LEN}}},
    {IC_TYPE_TMP1075N, false, false, 1, {{TMP1075N_DATA_LENGTH, TMP1075N_DATA_OFFSET, {TMP1075N_ADDR}, 1, TMP1075N_DATA_READ_LENGTH}}},
    {IC_TYPE_TMP110,   false, false, 1, {{TMP110_DATA_LENGTH,   TMP110_DATA_OFFSET,   {TMP110_ADDR},   1, TMP110_DATA_READ_LENGTH}}},
    {IC_TYPE_BQ27Z558, false, false, 3, {  {BQ27Z558_REG_READ_RX_LEN, 0, {BQ27Z558_SLAVE_ADDR, BQ27Z558_REG_STATE_RSOC}, 2, BQ27Z558_REG_READ_RX_LEN, 0},
                                           {BQ27Z558_REG_READ_RX_LEN, 0, {BQ27Z558_SLAVE_ADDR, BQ27Z558_REG_AVG_CUR}, 2, BQ27Z558_REG_READ_RX_LEN, BQ27Z558_REG_READ_RX_LEN},
                                           {BQ27Z558_REG_READ_RX_LEN, 0, {BQ27Z558_SLAVE_ADDR, BQ27Z558_REG_REM_CAPACITY}, 2, BQ27Z558_REG_READ_RX_LEN, BQ27Z558_REG_READ_RX_LEN << 1} }}
};

#define MMC5983_SET_RESET_CMD_LEN 2

static uint8_t mmc5983_set_cmd[MMC5983_SET_RESET_CMD_LEN] = //Must be in ram, for use with trx/DMA
{ 
    MMC5983_CONTROL0_REG & 0x7F,
#ifdef MEMSIC_BIPOLAR_EXCITATION
    MMC5983_CONTROL0_SET,
#else
    MMC5983_CONTROL0_SET_CMD,
#endif
};

static uint8_t mmc5983_reset_cmd[MMC5983_SET_RESET_CMD_LEN] = //Must be in ram, for use with trx/DMA
{ 
    MMC5983_CONTROL0_REG & 0x7F,
    MMC5983_CONTROL0_RESET,
};

//Generated ommocomm state
#ifdef OMMOCOMM_ENABLED
static uint8_t ommocomm_num_present = 0;
static uint8_t ommocomm_port_num[OMMO_PM_UARTE_COUNT];
static uint16_t ommocomm_data_len[OMMO_PM_UARTE_COUNT];
static uint16_t ommocomm_data_output_index[OMMO_PM_UARTE_COUNT];
static bool ommocomm_save_data[OMMO_PM_UARTE_COUNT];
static ArrayContainerAllocated <ommocomm_uarte* , OMMO_PM_UARTE_COUNT> ommocomm_instances;
#endif //OMMOCOMM_ENABLED

//Generated sensor configuration variables
uint8_t total_num_ics;
uint8_t ss_map[OMMO_PM_SPI_BUSSES_COUNT][OMMO_PM_SS_PINS_COUNT]; //Contains indexes into the ic data dictionary

static ArrayContainerAllocated <uint8_t, OMMO_PM_SS_PINS_COUNT> ss_pins_with_mag;
static ArrayContainerAllocated <uint8_t, OMMO_PM_SS_PINS_COUNT> ss_pins_with_imu;

uint8_t num_i2c_ics;
static uint8_t i2c_ic_dictionary_index[OMMO_PM_PORTS_COUNT*OMMO_SENSOR_NUM_I2C_BUS_LOCATIONS_SUPPORTED]; //Contains indexes into the ic data dictionary
static uint8_t i2c_ic_port_num[OMMO_PM_PORTS_COUNT*OMMO_SENSOR_NUM_I2C_BUS_LOCATIONS_SUPPORTED];
static ICBusLocation i2c_ic_bus_location[OMMO_PM_PORTS_COUNT*OMMO_SENSOR_NUM_I2C_BUS_LOCATIONS_SUPPORTED];
static uint16_t i2c_data_output_index[OMMO_PM_PORTS_COUNT*OMMO_SENSOR_NUM_I2C_BUS_LOCATIONS_SUPPORTED];
static uint8_t i2c_ic_read_command_index[OMMO_PM_PORTS_COUNT*OMMO_SENSOR_NUM_I2C_BUS_LOCATIONS_SUPPORTED];

//Generated output addr location, used by state machine
static uint16_t data_output_length;
uint16_t data_output_index[OMMO_PM_SPI_BUSSES_COUNT][OMMO_PM_SS_PINS_COUNT];
static bool execute_sample_ready_callback;
static bool lsm6dsv_detected;

//State variables
static volatile sensors_state_t current_state = STATE_MACHINE_STOPPED;
static uint8_t current_ss_index;
static uint8_t i2c_current_state_index;
static uint8_t hot_plug_current_state_index;
static uint32_t i2c_interval_count;
static bool prev_cmd_reset;

//static volatile uint32_t active_ss_pin;
static volatile sensors_stop_cmd_t stop_state_machine_cmd;
static uint32_t sensor_event_buffer[OMMO_SENSOR_EVENT_FIFO_LEN];
static ommo_fifo_t sensor_event_fifo;    

bool mag_calibration_mode;
static bool onboard_sensors_enabled;
static uint8_t disable_ss_pin_on_port[OMMO_PM_PORTS_COUNT] = {0};
static DataHeaderFormat data_header_format = DATA_HEADER_FORMAT_TS;

//State machine variables
static bool hot_unplug_double_check = false;   //used to check the same ss-spi channel twice
static bool hot_plugin_event_set = false;
static bool hot_unplug_event_set = false;
static sensors_event_callback_t sensors_event_callbacks[SENSORS_MAX_EVENTS_CALLBACKS];
static uint8_t sensors_event_callbacks_num = 0;

//State machine configured bool
static class state_machine_configured_t
{
    public:
        bool is_set()
        {
            return state_machine_configured;
        }

        void clear()
        {
            state_machine_configured = false;
            OMMO_APP_ERROR_CHECK(ommo_fifo_put(&sensor_event_fifo, (uint32_t)SENSORS_STATE_MACHINE_UNCONFIGURED), STRING("Sensor event queue full"), 0);
        }

        void set()
        {
            state_machine_configured = true;
            OMMO_APP_ERROR_CHECK(ommo_fifo_put(&sensor_event_fifo, (uint32_t)SENSORS_STATE_MACHINE_CONFIGURED), STRING("Sensor event queue full"), 0);
        }        

    private:
        bool state_machine_configured = false;
} state_machine_configured;


// Clear LEDs on serial close
#ifdef OMMO_PORT_LED
static bool keep_port_leds_on_close = false;
#endif

#ifndef HOT_PLUG_OFF
APP_TIMER_DEF(hot_plug_idle_timer_id);
#endif

// Output data buffers
static uint8_t output_data_1[COMMS_ESB_MAX_MULTIPACKET_TOTAL_LENGTH / OMMO_TIMESTAMP_SYNCH_PERIOD_MULT];
static uint8_t output_data_2[COMMS_ESB_MAX_MULTIPACKET_TOTAL_LENGTH / OMMO_TIMESTAMP_SYNCH_PERIOD_MULT];
static uint8_t output_data_3[COMMS_ESB_MAX_MULTIPACKET_TOTAL_LENGTH / OMMO_TIMESTAMP_SYNCH_PERIOD_MULT];
//current_output_data     data valid
//next_output_data        filling in mag data
//next_next_output_data   filling in imu data
static uint8_t *current_output_data, *next_output_data, *next_next_output_data;
static uint32_t current_output_data_timestamp, current_output_data_timestamp_offset;

//Misc
static uint32_t timing_check_failed_count;

//sample timer timer irq
static uint32_t last_timestamp, current_timestamp;
static uint32_t last_timestamp_offset, current_timestamp_offset;
static bool last_timestamp_used;

//Device info i2c config
static twi_basic_config_t sensors_twi_basic_config = {};

//SPI/TWI bus instances
static spim_twi_basic * spim_twi_basic_direct_instance = nullptr;
static ArrayContainerAllocated <comm_device *, OMMO_PM_SPI_BUSSES_COUNT> spi_bus_comm_devices;

#ifdef OMMO_BQ27Z558
BQ27Z558 sensors_fuel_gauge_instance;
#endif


static inline void sensors_configure_spi_pin(uint32_t pin)
{
    nrf_gpio_cfg(pin, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_H0H1, NRF_GPIO_PIN_NOSENSE);
}

#ifdef OMMO_TIMESTAMP_SYNCH_IN
static void sample_timer_event_handler(nrf_timer_event_t event_type, void* p_context)
{
    //Save current timestamp for packet that will be sent next
    if(last_timestamp_used)
    {
        //If the mag was not read then don't update the last timestamp because
        //the old data matching that timestamp will be read in the next cycle
        last_timestamp = current_timestamp;
        last_timestamp_offset = current_timestamp_offset;
        last_timestamp_used = false;

        //Save the timestamp/encoder for the next packet (after packet in process)
        timestamp_get_last_sample_event_timestamp_basestation_units(current_timestamp, current_timestamp_offset);
    }
#ifdef OMMOCOMM_ENABLED
    else // sample skipped
    {
        ommocomm_packet_malformed_countdown = ommocomm_packet_malformed_after_skipped;
    }
#endif

    //Re enable encoder sampling
    timestamp_reenable_disabled_captures();
}
#endif

static void spi_twi_event_handler(comm_device * comm_dev, comm_device_trx_result_t result, void * p_context)
{
    UNUSED_PARAMETER(p_context);
    UNUSED_PARAMETER(result);
    UNUSED_PARAMETER(comm_dev);

    sensors_state_change();
}

static void spi_event_handler(comm_device * comm_dev, comm_device_trx_result_t result, void * p_context)
{
    UNUSED_PARAMETER(p_context);
    UNUSED_PARAMETER(result);
    UNUSED_PARAMETER(comm_dev);

    // Check if all spi channels have completed
    for (const comm_device * cdev: spi_bus_comm_devices)
    {
        if (cdev && !cdev->is_trx_completed())
        {
            return;
        }
    }
    sensors_state_change();
}

#ifdef OMMO_IMU_IRQ_PIN
static void imu_irq_pin_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action, void *context)
{
    UNUSED_PARAMETER(action);
    nrfx_gpiote_in_event_disable(pin);
    OMMO_APP_ERROR_CHECK(ommo_fifo_put(&sensor_event_fifo, (uint32_t)SENSORS_EVENT_WAKE_ON_IMU), STRING("Sensor event queue full"), 0);
}
#endif

#ifndef HOT_PLUG_OFF
static void hot_plug_idle_timer_handler(void *p_context)
{
    //Do next SS index
    if(current_state == IDLE_HOT_PLUG_CHECK)
        sensors_continue_idle_hot_plug_check();
}
#endif

#ifdef OMMOCOMM_ENABLED
static void ommocomm_op_complete_callback(ommocomm_event_t event, void *p_context)
{
    //Save the "most error" return event
    if(event != OMMOCOMM_EVENT_SUCCESS)
        ommocomm_last_sample_complete_event = event;

    //Check if all ommocomm channels have completed
    bool all_complete = true;
    for (ommocomm_uarte *& ommocomm_dev : ommocomm_instances)
        all_complete &= ommocomm_dev->is_trx_completed();

    if(all_complete)
    {
        ommocomm_op_complete_flag = true;

        // For no-direct-mag configs where a SPI bus is still acquired,
        // a SPI HOLD runs in parallel with OmmoComm. Don't advance the state until both
        // are done or we'll queue a new HOLD onto a still-busy SPI bus.
        bool all_spi_complete = true;
        for(const comm_device * spi_dev : spi_bus_comm_devices)
        {
            if(spi_dev && !spi_dev->is_trx_completed())
            {
                all_spi_complete = false;
                break;
            }
        }

        if(current_state == OMMOCOMM_DELAY_SETUP_MAG_SET_COMMAND ||
           current_state == OMMOCOMM_DELAY_SETUP_STATE_MACHINE_RESTART ||
           current_state == OMMOCOMM_DELAY_STATE_MACHINE_STOP ||
           (ss_pins_with_mag.empty() && all_spi_complete && current_state == WAITING_FOR_MAG_START_SAMPLE_COMPLETION) ||
           (ss_pins_with_mag.empty() && all_spi_complete && current_state == WAITING_FOR_MAG_SET_COMMAND))
        {
            sensors_state_change();
        }
    }
}
#endif

#ifndef HOT_PLUG_OFF
static void evaluate_hot_plug_data()
{
    bool found_connection    = false;
    bool found_disconnection = false;

    // Look for changes in expected results
    for (uint8_t spi_channel = 0; spi_channel < OMMO_PM_SPI_BUSSES_COUNT; spi_channel++)
    {
        uint8_t mag_wai = MMC5983_PRODUCT_ID_RESULT;

        if (MAG_PRESENT(spi_channel, hot_plug_current_state_index) && m_spi_rx_buf[spi_channel][1] != mag_wai)
        {
            // Mag lost
            found_disconnection = true;
        }
        else if (
              !IS_DISABLED(spi_channel, hot_plug_current_state_index) && !MAG_PRESENT(spi_channel, hot_plug_current_state_index)
              && m_spi_rx_buf[spi_channel][1] == mag_wai)
        {
            // Mag found
            found_connection = true;
        }
    }

    // Did we find a change
    if (found_connection || found_disconnection)
    {
        // If either type of event is detected, reconfiguration of the state machine is required before next enable
        state_machine_configured.clear();

        if (found_disconnection) // Disconnections take precidence over connections
        {
            if (!hot_unplug_event_set)
            {
                if (hot_unplug_double_check)
                {
                    hot_unplug_event_set = true;

                    OMMO_APP_ERROR_CHECK(ommo_fifo_put(&sensor_event_fifo, (uint32_t)SENSORS_EVENT_HOTPLUG_REMOVE), STRING("Sensor event queue full"), 0);
                }
                else
                {
                    hot_unplug_double_check = true;
                }
            }
        }
        else
        {
            if (!hot_plugin_event_set)
            {
                hot_plugin_event_set = true;

                OMMO_APP_ERROR_CHECK(ommo_fifo_put(&sensor_event_fifo, (uint32_t)SENSORS_EVENT_HOTPLUG_INSERT), STRING("Sensor event queue full"), 0);
            }
        }
    }
    else // No changes
    {
        hot_unplug_double_check = false;
    }
}
#endif

/*
 * State machine cycle -- one sample per configuration
 *
 * Abbreviations:
 *   SPI HOLD  -- SPI transfer queued in EasyDMA, held until PPI fires
 *   OC        -- ommocomm
 *   PPI+wait  -- PPI enabled; state waits for SPI completion callback
 *   cb        -- state entered via callback (SPI or I2C completion)
 *   instant   -- entered in same loop iteration, no wait
 *   skip      -- SS list empty, loop exits immediately to next state
 *   skip->I2C -- no SPI devices, jumps directly to CHECK_FOR_I2C_EVENT
 *   wait OC   -- waits for ommocomm_op_complete_callback
 *   swap buf  -- swap_sample_buffers_and_flag_sample_callback()
 *
 * Configs: [1] SPI  [2] SPI+I2C  [3] SPI+OC  [4] SPI+I2C+OC  [5] I2C  [6] OC  [7] I2C+OC
 *
 * -- NON-I2C INTERVAL (common cycle) --------------------------------------------------------------------------------------------
 *
 * State                                        [1]                 [2]                 [3]                    [4]                    [5]                 [6]                 [7]
 * SETUP_MAG_START_SAMPLE                       SPI HOLD,PPI+wait   SPI HOLD,PPI+wait   SPI HOLD+OC,PPI+wait   SPI HOLD+OC,PPI+wait   (nothing)           OC synch,PPI+wait   OC synch,PPI+wait
 * WAITING_FOR_MAG_START_SAMPLE_COMPLETION      cb                  cb                  cb                     cb                     instant             OC cb               OC cb
 * SETUP_NEXT_MAG_DATA_READ x N                 wait each           wait each           wait each              wait each              skip                skip                skip
 * SETUP_NEXT_IMU_DATA_READ x N                 wait each           wait each           wait each              wait each              skip                skip                skip
 * SETUP_NEXT_DATA_HOT_PLUG_CHECK               wait                wait                wait                   wait                   skip                skip                skip
 * CHECK_FOR_I2C_EVENT                          OC_DLY_RESTART      OC_DLY_RESTART      OC_DLY_RESTART         OC_DLY_RESTART         OC_DLY_RESTART      OC_DLY_RESTART      OC_DLY_RESTART
 * OMMOCOMM_DELAY_SETUP_STATE_MACHINE_RESTART   instant,swap buf    instant,swap buf    wait OC,swap buf       wait OC,swap buf       instant,swap buf    instant,swap buf    instant,swap buf
 * SETUP_STATE_MACHINE_RESTART                  restart             restart             restart                restart                restart             restart             restart
 *
 * -- I2C INTERVAL CYCLE (every OMMO_SENSOR_I2C_TRX_INTERVAL samples) -----------------------------------------------------------
 *
 * State                                        [1]                 [2]                 [3]                    [4]                    [5]                 [6]                 [7]
 * SETUP_MAG_START_SAMPLE                       SPI HOLD,PPI+wait   SPI HOLD,PPI+wait   SPI HOLD+OC,PPI+wait   SPI HOLD+OC,PPI+wait   (nothing)           OC synch,PPI+wait   OC synch,PPI+wait
 * WAITING_FOR_MAG_START_SAMPLE_COMPLETION      cb                  cb                  cb                     cb                     instant             OC cb               OC cb
 * SETUP_NEXT_MAG_DATA_READ x N                 wait each           wait each           wait each              wait each              skip                skip                skip
 * SETUP_NEXT_IMU_DATA_READ x N                 wait each           wait each           wait each              wait each              skip                skip                skip
 * SETUP_NEXT_DATA_HOT_PLUG_CHECK               wait                wait                wait                   wait                   skip->I2C           skip->I2C           skip->I2C
 * CHECK_FOR_I2C_EVENT                          I2C read            I2C read            I2C read               I2C read               I2C read            I2C read            I2C read
 * SETUP_I2C_DATA_READ x N                      (no I2C) skip       wait each           (no I2C) skip          wait each              wait each           (no I2C) skip       wait each
 * OMMOCOMM_DELAY_SETUP_MAG_SET_COMMAND         instant             instant             wait OC                wait OC                instant             instant             wait OC
 * SETUP_MAG_SET_COMMAND                        SPI HOLD,PPI+wait   SPI HOLD,PPI+wait   SPI HOLD+OC,PPI+wait   SPI HOLD+OC,PPI+wait   (nothing)           OC set,PPI+wait     OC set,PPI+wait
 * WAITING_FOR_MAG_SET_COMMAND                  cb                  cb                  cb                     cb                     instant             OC cb               OC cb
 * OMMOCOMM_DELAY_SETUP_STATE_MACHINE_RESTART   instant,swap buf    instant,swap buf    wait OC,swap buf       wait OC,swap buf       instant,swap buf    instant,swap buf    instant,swap buf
 * SETUP_STATE_MACHINE_RESTART                  restart             restart             restart                restart                restart             restart             restart
 */
static void sensors_state_change()
{
    bool wait_for_callback = false;
    while (!wait_for_callback)
    {
        switch(current_state)
        {
            case SETUP_MAG_START_SAMPLE:
            {
                // Turn on all CS lines with mag
                port_master_ss_select(ss_pins_with_mag);

                // Start all spi bus's
                bool spi_trx_queued = false;
                for (comm_device *& comm_dev : spi_bus_comm_devices)
                {
                    if (comm_dev)
                    {
                        static uint8_t start_mag_sample_cmd[] = { MMC5983_CONTROL0_REG & 0x7F, MMC5983_CONTROL0_START_CMD };
                        APP_ERROR_CHECK(comm_dev->trx(start_mag_sample_cmd, sizeof(start_mag_sample_cmd), nullptr, 0, spi_event_handler, nullptr, COMM_DEVICE_FLAGS_HOLD));
                        spi_trx_queued = true;
                    }
                }

#ifdef OMMOCOMM_ENABLED
                bool ommocomm_instances_empty = ommocomm_instances.empty();
                //Clear ommocomm data ready flags and setup synch packet
                if (!ommocomm_instances_empty)
                {
                    ommocomm_op_complete_flag = false;

                    for (size_t i = 0; i < ommocomm_instances.size(); i++)
                    {
                        uint8_t * data_output_ptr = ommocomm_save_data[i] ? next_output_data + ommocomm_data_output_index[i]
                                                                          : ommocomm_uarte_rx_buf; // If data is not used, just put it somewhere useless
                        APP_ERROR_CHECK(ommocomm_instances[i]->automode_setup_synch_tx_and_hold(
                              0, data_output_ptr, ommocomm_data_len[i], ommocomm_op_complete_callback, nullptr));
                    }
                }
#else
                bool ommocomm_instances_empty = true;
#endif

                current_state = WAITING_FOR_MAG_START_SAMPLE_COMPLETION;

                if(spi_trx_queued || !ommocomm_instances_empty)
                {
                    // Enable PPI to trigger SPI/OC start on next sample event; callback will advance the state
                    sensors_set_ppi_start_transfer_event_addr_and_enable(timestamp_get_sample_event_address());
                    wait_for_callback = true;
                }
                // I2C-only: no SPI or OC, loop falls through WAITING_FOR_MAG_START_SAMPLE_COMPLETION immediately.
                break;
            }

            case WAITING_FOR_MAG_START_SAMPLE_COMPLETION:
                // Turn off PPI
                nrfx_ppi_group_disable(sampling_ppi_start_group);

                // Deselect all CS lines
                port_master_ss_deselect_all();

                // Switch states
                current_ss_index = -1;
                current_state = SETUP_NEXT_MAG_DATA_READ;
                break;

            case SETUP_NEXT_MAG_DATA_READ:
                current_ss_index++;
                if (current_ss_index < ss_pins_with_mag.size())
                {
                    // Setup next ss pin
                    port_master_switch_ss(ss_pins_with_mag[current_ss_index]);
                    for (size_t channel = 0; channel < spi_bus_comm_devices.size(); channel++)
                    {
                        // Start the SPI mag data read on busses with mags
                        if (spi_bus_comm_devices[channel] && MAG_PRESENT(channel, ss_pins_with_mag[current_ss_index]))
                        {
                            sensors_start_spi_sensor_data_read(channel, ss_map[channel][ss_pins_with_mag[current_ss_index]]);
                        }
                    }

                    // Wait for data read to finish
                    current_state     = WAITING_FOR_MAG_DATA_READ_COMPLETION;
                    wait_for_callback = true;
                }
                else
                {
                    // Switch states
                    current_state    = SETUP_NEXT_IMU_DATA_READ;
                    current_ss_index = -1;
                }
                break;

            case WAITING_FOR_MAG_DATA_READ_COMPLETION:
                // Save read data
                for (size_t channel = 0; channel < spi_bus_comm_devices.size(); channel++)
                {
                    if (spi_bus_comm_devices[channel] && MAG_PRESENT(channel, ss_pins_with_mag[current_ss_index]))
                    {
                        const uint8_t len    = ic_data[ss_map[channel][ss_pins_with_mag[current_ss_index]]].read_command[0].data_length;
                        const uint8_t offset = ic_data[ss_map[channel][ss_pins_with_mag[current_ss_index]]].read_command[0].data_offset;
                        
                        memcpy(next_output_data + data_output_index[channel][ss_pins_with_mag[current_ss_index]], m_spi_rx_buf[channel] + offset, len);

#ifdef MEMSIC_BIPOLAR_EXCITATION
                        // Data is read with a one-sample delay and corresponds to the second-to-last set/reset command
                        bool data_from_reset_current = !prev_cmd_reset;
                        next_output_data[data_output_index[channel][ss_pins_with_mag[current_ss_index]] + len] = data_from_reset_current ? 0 : 1;
#endif
                    }
                }

                current_state = SETUP_NEXT_MAG_DATA_READ;
                break;

            case SETUP_NEXT_IMU_DATA_READ:
                current_ss_index++;
                if (current_ss_index < ss_pins_with_imu.size())
                {
                    // Setup next ss pin
                    port_master_switch_ss(ss_pins_with_imu[current_ss_index]);

                    for (size_t channel = 0; channel < spi_bus_comm_devices.size(); channel++)
                    {
                        // Start the SPI imu data read on busses with imus
                        if (spi_bus_comm_devices[channel] && IMU_PRESENT(channel, ss_pins_with_imu[current_ss_index]))
                        {
                            sensors_start_spi_sensor_data_read(channel, ss_map[channel][ss_pins_with_imu[current_ss_index]]);
                        }
                    }

                    // Wait for data read to finish
                    current_state     = WAITING_FOR_IMU_DATA_READ_COMPLETION;
                    wait_for_callback = true;
                }
                else // Finished reading everything -> go to connection check
                {
                    // Switch states
#ifdef HOT_PLUG_OFF
                    port_master_switch_ss(0xFF);
                    current_state = CHECK_FOR_I2C_EVENT;
#else
                    current_state = SETUP_NEXT_DATA_HOT_PLUG_CHECK;
#endif
                }
                break;

            case WAITING_FOR_IMU_DATA_READ_COMPLETION:

                // Save read data
                for (size_t channel = 0; channel < spi_bus_comm_devices.size(); channel++)
                {
                    if (spi_bus_comm_devices[channel] && IMU_PRESENT(channel, ss_pins_with_imu[current_ss_index]))
                    {
                        const uint8_t len = ic_data[ss_map[channel][ss_pins_with_imu[current_ss_index]]].read_command[0].data_length;
                        const uint8_t offset = ic_data[ss_map[channel][ss_pins_with_imu[current_ss_index]]].read_command[0].data_offset;
                        memcpy(next_next_output_data + data_output_index[channel][ss_pins_with_imu[current_ss_index]],
                               m_spi_rx_buf[channel] + offset,
                               len);
                    }
                }

                current_state = SETUP_NEXT_IMU_DATA_READ;
                break;

#ifndef HOT_PLUG_OFF
            case SETUP_NEXT_DATA_HOT_PLUG_CHECK:
                {
#if OMMO_PM_SPI_BUSSES_COUNT == 0
                    // No SPI buses: nothing to poll, skip to I2C handling
                    current_state = CHECK_FOR_I2C_EVENT;
#else  // OMMO_PM_SPI_BUSSES_COUNT != 0
                    // SPI buses present: poll for hot plug events
                    if (!hot_unplug_double_check)
                    {
                        hot_plug_current_state_index = (hot_plug_current_state_index + 1) % OMMO_PM_SS_PINS_COUNT;
                    }

                    // Set pins
                    port_master_switch_ss(hot_plug_current_state_index);
                    for (size_t channel = 0; channel < spi_bus_comm_devices.size(); channel++)
                    {
                        sensors_read_mag_id(channel);
                    }

                    current_state     = WAITING_FOR_DATA_MODE_HOT_PLUG_CHECK;
                    wait_for_callback = true;
#endif  // OMMO_PM_SPI_BUSSES_COUNT != 0
                }
                break;

            case WAITING_FOR_DATA_MODE_HOT_PLUG_CHECK:
                evaluate_hot_plug_data();
                port_master_switch_ss(0xFF);

                // Switch state
                current_state = CHECK_FOR_I2C_EVENT;
                break;
#endif

            case CHECK_FOR_I2C_EVENT:
                // Switch states
                i2c_interval_count++;
#ifndef MEMSIC_DISABLE_SET_RESET
                if (i2c_interval_count >= OMMO_SENSOR_I2C_TRX_INTERVAL)
                {
                    // Schedule next i2c/set/reset
                    sensors_schedule_i2c_set_reset(last_timestamp);

                    // Reset substate
                    i2c_current_state_index = -1;
                    current_state           = SETUP_I2C_DATA_READ;
                }
                else
#endif
                {
#ifdef MEMSIC_BIPOLAR_EXCITATION
                    // Don't need to do a temp/set, go to set/reset cmd
                    current_state = OMMOCOMM_DELAY_SETUP_MAG_SET_COMMAND;
#else
                    // Don't need to do a temp/set, start state machine over once ommocomm is finished
                    current_state = OMMOCOMM_DELAY_SETUP_STATE_MACHINE_RESTART;
#endif
                    // Loop to check to see if we are already finished
                }
                break;

            case SETUP_I2C_DATA_READ:
                i2c_current_state_index++;
                if (i2c_current_state_index < num_i2c_ics)
                {
                    const uint8_t old_sda_pin = sensors_twi_basic_config.sda_pin;
                    const uint8_t old_mosi_pin = sensors_twi_basic_config.scl_pin;

                    // Switch spim_twi to correct twi bus location
                    APP_ERROR_CHECK(port_master_get_twi_bus_config(i2c_ic_port_num[i2c_current_state_index], i2c_ic_bus_location[i2c_current_state_index], &sensors_twi_basic_config));
                    APP_ERROR_CHECK(spim_twi_basic_direct_instance->acquire_twi(&sensors_twi_basic_config, true));

                    // Fix old MOSI/CS line whose config may have been changed
                    if (sensors_twi_basic_config.sda_pin != old_sda_pin)
                        port_master_configure_high_speed_output(old_sda_pin);
                    if (sensors_twi_basic_config.scl_pin != old_mosi_pin)
                        port_master_configure_high_speed_output(old_mosi_pin);


                    // Start data transfer
                    const ic_data_dictionary_entry * ic_data_ptr = const_cast<ic_data_dictionary_entry *>(&ic_data[i2c_ic_dictionary_index[i2c_current_state_index]]);
                    const uint8_t read_command_index = i2c_ic_read_command_index[i2c_current_state_index];

                    // Switch states
                    current_state = WAITING_FOR_I2C_DATA_READ_COMPLETION;
                    wait_for_callback = true;

                    APP_ERROR_CHECK(spim_twi_basic_direct_instance->trx(
                                        ic_data_ptr->read_command[read_command_index].data_read_tx,
                                        ic_data_ptr->read_command[0].data_read_tx_len, m_spi_rx_buf[0],
                                        ic_data_ptr->read_command[read_command_index].data_read_rx_len, spi_twi_event_handler));

                    // The TRX may be called instantly, so we need to return to avoid double processing
                    return;
                }

                // Switch spim_twi back to spi bus
                spim_twi_basic_direct_instance->acquire_spim(nullptr, true); 

                // Fix old CS/MOSI line whose config may have been changed
                // NOTE: Make sure that the SDA pin is not used as MISO pin for any SPI bus
                port_master_configure_high_speed_output(sensors_twi_basic_config.sda_pin);
                port_master_configure_high_speed_output(sensors_twi_basic_config.scl_pin);

                // Wait for ommocomm sensor data read to finish (if it isn't)
                current_state = OMMOCOMM_DELAY_SETUP_MAG_SET_COMMAND;
                break;

            case WAITING_FOR_I2C_DATA_READ_COMPLETION:
            {
                // Save temp data
                const volatile ic_data_dictionary_entry * ic_data_ptr = &ic_data[i2c_ic_dictionary_index[i2c_current_state_index]];

                const uint8_t read_command_index = i2c_ic_read_command_index[i2c_current_state_index];
                const uint8_t len                = ic_data_ptr->read_command[read_command_index].data_length;
                const uint8_t input_offset       = ic_data_ptr->read_command[read_command_index].data_offset;

                const uint8_t output_index = i2c_data_output_index[i2c_current_state_index] + ic_data_ptr->read_command[read_command_index].data_output_index_offset;
                memcpy(output_data_1 + output_index, m_spi_rx_buf[0] + input_offset, len);
                memcpy(output_data_2 + output_index, m_spi_rx_buf[0] + input_offset, len);
                memcpy(output_data_3 + output_index, m_spi_rx_buf[0] + input_offset, len);

                // Read the next command next time we read i2c
                i2c_ic_read_command_index[i2c_current_state_index] = (i2c_ic_read_command_index[i2c_current_state_index] + 1) % ic_data_ptr->num_read_commands;

                // Sample next sensor
                current_state = SETUP_I2C_DATA_READ;
                break;
            }

            case OMMOCOMM_DELAY_SETUP_MAG_SET_COMMAND:
                if (ommocomm_op_complete_flag)
                {
                    // Move on to set command
                    current_state = SETUP_MAG_SET_COMMAND;
                }
                else
                {
                    // ommcomm will call back when it's finished
                    wait_for_callback = true;
                }
                break;

            case SETUP_MAG_SET_COMMAND:
            {
#ifdef OMMO_DEBUG_SET_RESET_PIN
                nrf_gpio_pin_toggle(OMMO_DEBUG_SET_RESET_PIN);
#endif

                // Turn on all CS lines with mag
                port_master_ss_select(ss_pins_with_mag);

                // Start all spi bus's with set command
                bool spi_trx_queued = false;
                for (comm_device * sampling_device : spi_bus_comm_devices)
                {
                    if (sampling_device)
                    {
                        // Setup next sensor set packet for PPI to trigger later
                        const uint8_t *set_cmd = mmc5983_set_cmd;
#ifdef MEMSIC_BIPOLAR_EXCITATION
                        if (!prev_cmd_reset)
                            set_cmd = mmc5983_reset_cmd;
#endif

                        APP_ERROR_CHECK(sampling_device->trx(set_cmd, MMC5983_SET_RESET_CMD_LEN, nullptr, 0, spi_event_handler, nullptr, COMM_DEVICE_FLAGS_HOLD));
                        spi_trx_queued = true;
                    }
                }

#ifdef OMMOCOMM_ENABLED
                bool ommocomm_instances_empty = ommocomm_instances.empty();
                //Setup set/reset auto sequence packet
                if (!ommocomm_instances_empty)
                {
                    ommocomm_op_complete_flag = false;
                    for (ommocomm_uarte* & ommocomm : ommocomm_instances)
                    {
                        APP_ERROR_CHECK(ommocomm->automode_setup_synch_tx_and_hold(1, nullptr, 0, ommocomm_op_complete_callback, nullptr));
                    }
                }
#else
                bool ommocomm_instances_empty = true;
#endif

#ifdef MEMSIC_BIPOLAR_EXCITATION
                // flip the next reset/set action
                prev_cmd_reset = !prev_cmd_reset;
#endif

                current_state = WAITING_FOR_MAG_SET_COMMAND;

                // Enable PPI to trigger SPI set/reset; SPI callback will advance the state
                sensors_set_ppi_start_transfer_event_addr_and_enable(sensor_set_sample_time_offset_event);

                if(spi_trx_queued || !ommocomm_instances_empty)
                {
                    wait_for_callback = true;
                }
                // SPI callback (or ommocomm callback for OC-only) advances state from WAITING_FOR_MAG_SET_COMMAND.
                break;
            }

            case WAITING_FOR_MAG_SET_COMMAND:
                // Turn off PPI
                nrfx_ppi_group_disable(sampling_ppi_start_group);

                // Fix for MMC5983 changing BW settings
                for (uint8_t i = 0; i < OMMO_PM_SPI_BUSSES_COUNT; ++i)
                {
                    if (port_master_spi_busses[i].supports_sampling)
                        nrf_gpio_pin_clear(port_master_spi_busses[i].mosi_pin);
                }
                nrf_delay_us(1);

                for (uint8_t i = 0; i < OMMO_PM_SPI_BUSSES_COUNT; i++)
                {
                    if (port_master_spi_busses[i].supports_sampling)
                        nrf_gpio_pin_set(port_master_spi_busses[i].mosi_pin);
                }
                nrf_delay_us(1);

                // Disable all ss pins to start new SPI command
                port_master_ss_deselect_all();

                // wait for ommocomm set/reset to finish before we set up the data read
                current_state = OMMOCOMM_DELAY_SETUP_STATE_MACHINE_RESTART;

                // Loop to check to see if we are already finished
                break;

            case OMMOCOMM_DELAY_SETUP_STATE_MACHINE_RESTART:
                if (ommocomm_op_complete_flag)
                {
                    // Direct and ommocomm data are both ready
                    swap_sample_buffers_and_flag_sample_callback();

                    // Timestamp from previous start command has been used and we can
                    // resume updating it from at sample events again (if we skipped samples)
                    last_timestamp_used = true;

                    current_state = SETUP_STATE_MACHINE_RESTART;
                }
                else
                {
                    // ommcomm will call back when it's finished
                    wait_for_callback = true;
                }
                break;

            case SETUP_STATE_MACHINE_RESTART:
                if (stop_state_machine_cmd != SENSORS_STOP_CMD_NONE)
                {
#ifdef OMMO_IMU_IRQ_PIN
                    // Do we support waking up by the IMU?
                    if (stop_state_machine_cmd == SENSORS_STOP_CMD_WAKE_ON_IMU)
                    {
                        current_ss_index = -1;
                        current_state    = SLEEP_WITH_IMU_PREPARE;
                    }
                    else
#endif
                    {
                        // Send stop command
#ifdef OMMOCOMM_ENABLED
                        if (!ommocomm_instances.empty())
                        {
                            ommocomm_op_complete_flag  = false;
                            for (ommocomm_uarte* & ommocomm : ommocomm_instances)
                            {
                                APP_ERROR_CHECK(ommocomm->automode_send_stop_tx(ommocomm_op_complete_callback, nullptr));
                            }

                            current_state = OMMOCOMM_DELAY_STATE_MACHINE_STOP;
                        }
                        else
#endif
                        {
                            stop_state_machine_cmd = SENSORS_STOP_CMD_NONE;
                            wait_for_callback      = true; // Break the loop to wait for callback.

                            // Release resources
                            sensors_state_machine_release_resources_flag_stopped();
                        }
                    }
                }
                else
                {
                    current_state = SETUP_MAG_START_SAMPLE;
                }
                break;

            case SLEEP_WITH_IMU_PREPARE:
#ifdef OMMO_IMU_IRQ_PIN
                // Iterates all the SPI buses, it will change the state machine once we complete all the transfers.
                current_ss_index++;
                if(current_ss_index < ss_pins_with_imu.size())
                {
                    // Setup next ss pin
                    port_master_switch_ss(ss_pins_with_imu[current_ss_index]);

                    for (size_t spi_channel = 0; spi_channel < spi_bus_comm_devices.size(); spi_channel++)
                    {
                        // Start the SPI imu data read on busses with imus
                        uint8_t ic_data_index = ss_map[spi_channel][ss_pins_with_imu[current_ss_index]];
                        if (!IMU_PRESENT(spi_channel, ss_pins_with_imu[current_ss_index])
                            || ic_data[ic_data_index].ic_type != IC_TYPE_ICM_42605)
                        {
                            continue;
                        }

                        // Read INT_STATUS2 and INT_STATUS3 registers to clean the IRQ pin
                        static uint8_t        int_status2[] = { IMU_42605_READ_REG_MASK(IMU_42605_REG_INT_STATUS2) };

                        APP_ERROR_CHECK(spi_bus_comm_devices[spi_channel]->trx(
                                int_status2, sizeof(int_status2), m_spi_rx_buf[spi_channel], 2, spi_event_handler));
                        wait_for_callback = true;
                    }

                    // Break switch case to continue upper loop.
                    break;
                }

                // Enable the interrupt as we handled all the SPI ports
                nrfx_gpiote_in_event_enable(OMMO_IMU_IRQ_PIN, true);
#endif

                // Go back to suspend mode
                current_state = SETUP_STATE_MACHINE_RESTART;
                stop_state_machine_cmd = SENSORS_STOP_CMD_SUSPEND;
                break;

            case OMMOCOMM_DELAY_STATE_MACHINE_STOP:
                if(ommocomm_op_complete_flag)
                {
                    stop_state_machine_cmd = SENSORS_STOP_CMD_NONE;
                    wait_for_callback = true;

                    //Release resources
                    sensors_state_machine_release_resources_flag_stopped();
                }
                else
                {
                    //ommcomm will call back when it's finished
                    wait_for_callback = true;
                }
                break;

    #ifndef HOT_PLUG_OFF
            case IDLE_HOT_PLUG_CHECK:
                //Clear SS line
                port_master_switch_ss(0xFF);

                if(stop_state_machine_cmd == SENSORS_STOP_CMD_NONE)
                {
                    evaluate_hot_plug_data();
                    wait_for_callback = true;
                }
                else
                {
                    stop_state_machine_cmd = SENSORS_STOP_CMD_NONE;
                    wait_for_callback = true;

                    //Release resources
                    sensors_state_machine_release_resources_flag_stopped();
                }
                break;
    #endif

            default:
                APP_ERROR_HANDLER(NRF_ERROR_INTERNAL);
        }
    }
}

//TODO if there are insufficient spi resources we should crash the program
static void sensors_state_machine_acquire_resources()
{
    //Flag state sto that we know resources are acquired
    current_state = PREPARED_FOR_START;

    // Setup SS GPIOs
    for (int32_t i = 0; i < OMMO_PM_SS_PINS_COUNT; i++)
    {
        nrf_gpio_pin_set(port_master_ss_pins[i]);
        port_master_configure_high_speed_output(port_master_ss_pins[i]);
    }

#ifdef OMMOCOMM_ENABLED
    // Acquire ommocomm resources
    for (uint8_t ommocomm_index = 0; ommocomm_index < ommocomm_num_present; ommocomm_index++)
    {
        ommocomm_uarte * ommocomm_dev = nullptr;
        APP_ERROR_CHECK(port_master_acquire_ommocomm_direct(ommocomm_port_num[ommocomm_index], true, &ommocomm_dev));
        APP_ERROR_CHECK_BOOL(ommocomm_instances.push_back(ommocomm_dev));
        APP_ERROR_CHECK(sensors_add_start_transfer_task(ommocomm_dev->get_start_task()));
    }
#endif

    // Acquire SPIM/TWI resources
    for (uint8_t i = 0; i < OMMO_PM_SPI_BUSSES_COUNT; ++i)
    {
        spim_basic * spim;
        if (spim_twi_basic_direct_instance == nullptr && port_master_spi_busses[i].supports_sampling &&
            port_master_acquire_spim_twi_direct(i, &spim_twi_basic_direct_instance) == NRF_SUCCESS)            
        {
            APP_ERROR_CHECK(sensors_add_start_transfer_task(spim_twi_basic_direct_instance->get_start_task()));
            APP_ERROR_CHECK_BOOL(spi_bus_comm_devices.push_back(spim_twi_basic_direct_instance));
        }
        else if (port_master_spi_busses[i].supports_sampling && port_master_acquire_spim_direct(i, &spim) == NRF_SUCCESS)
        {
            APP_ERROR_CHECK(sensors_add_start_transfer_task(spim->get_start_task()));
            APP_ERROR_CHECK_BOOL(spi_bus_comm_devices.push_back(spim));
        }
        else
        {
            // Push nullptr to keep the spi_bus_comm_devices array aligned with the ss_pins_with_mag and ss_pins_with_imu arrays
            APP_ERROR_CHECK_BOOL(spi_bus_comm_devices.push_back(nullptr));
        }
    }

#ifdef OMMO_DEBUG_SAMPLE_PIN
    APP_ERROR_CHECK(sensors_add_start_transfer_task(nrfx_gpiote_out_task_addr_get(OMMO_DEBUG_SAMPLE_PIN)));
#endif

#if defined(OMMO_LSM6DSV_INT2_PIN) || defined(OMMO_LSM6DSV_INT2_NOR_PIN)
    if(lsm6dsv_detected)
        APP_ERROR_CHECK(lsm6dsv_int2_enable(timestamp_get_sample_event_address()));
#endif
}

static void sensors_state_machine_release_resources_flag_stopped()
{
    sensors_clean_start_transfer_tasks();

    //Release SPIM/TWI resources
    for (comm_device * device : spi_bus_comm_devices)
    {
        if (device)
        {
            device->release();
        }
    }
    spi_bus_comm_devices.clear();

    if (spim_twi_basic_direct_instance)
    {
        spim_twi_basic_direct_instance->release();
        spim_twi_basic_direct_instance = nullptr;
    }

#ifdef OMMOCOMM_ENABLED
    // Release ommocomm resources
    for (ommocomm_uarte *& ommocomm_dev : ommocomm_instances)
    {
        ommocomm_dev->release();
        ommocomm_dev = nullptr;
    }
    ommocomm_instances.clear();
#endif

#if defined(OMMO_LSM6DSV_INT2_PIN) || defined(OMMO_LSM6DSV_INT2_NOR_PIN)
    if(lsm6dsv_detected)
        lsm6dsv_int2_disable();
#endif

    //Finally, flag state to stopped indicating resources are free
    current_state = STATE_MACHINE_STOPPED;

#ifndef HOT_PLUG_OFF
    //Reset the hot plug idle timer to ensure it doesn't disrupt the next attempt to start idle hot plug
    APP_ERROR_CHECK(app_timer_stop(hot_plug_idle_timer_id));
#endif
}

bool sensors_are_resources_acquired()
{
    return (current_state != STATE_MACHINE_STOPPED);
}

static void swap_sample_buffers_and_flag_sample_callback()
{
uint8_t *temp_ptr;

    if (!execute_sample_ready_callback)
    {
        // Move buffer pointer
        temp_ptr              = current_output_data;
        current_output_data   = next_output_data;
        next_output_data      = next_next_output_data;
        next_next_output_data = temp_ptr;

        // Save matching timestamp
        current_output_data_timestamp        = last_timestamp;
        current_output_data_timestamp_offset = last_timestamp_offset;

#ifdef OMMOCOMM_ENABLED
        // Set flag to execute data ready callback
        if (ommocomm_last_sample_complete_event != OMMOCOMM_EVENT_SUCCESS)
        {
            //Reset error condition
            ommocomm_last_sample_complete_event = OMMOCOMM_EVENT_SUCCESS;

            //Evaluate for hotplug
            ommocomm_sample_error_count++;
            
            if (!hot_unplug_event_set)
            {
                if(ommocomm_sample_error_count > OMMOCOMM_MAX_MISSED_SENSOR_DATA_SAMPLES)
                {
                    hot_unplug_event_set = true;
                    ommocomm_sample_error_count = 0x00;
                
                    // Assume sensors were removed
                    OMMO_APP_ERROR_CHECK(ommo_fifo_put(&sensor_event_fifo, (uint32_t)SENSORS_EVENT_HOTPLUG_REMOVE), STRING("Sensor event queue full"), 0);
                }
            }
        }
        else
        {
            // If samples are skipped (likely due to I2C and set/reset cmd)
            // the second subsequent Ommocomm packet will be out of sync, and needs to be discarded
            if(ommocomm_packet_malformed_countdown > 0)
            {
                ommocomm_packet_malformed_countdown--;
                if(ommocomm_packet_malformed_countdown == 0)
                {
                    return;
                }
            }
            
            execute_sample_ready_callback = true;
            ommocomm_sample_error_count   = 0;
        }
#else
        execute_sample_ready_callback = true;
#endif
    }
}

bool sensors_is_state_machine_ready_to_run()
{
    bool ics_present = total_num_ics > 0;
#ifdef OMMOCOMM_ENABLED
    ics_present |= (ommocomm_num_present > 0);
#endif

    return state_machine_configured.is_set() && ics_present;
}

uint32_t sensors_start_state_machine()
{
    // Don't start state machine if state machine is not ready to run (state machine will fail)
    if (!sensors_is_state_machine_ready_to_run())
        return NRF_ERROR_NOT_FOUND;

    // Stop idle hot_plug
    sensors_stop_state_machine();

    // Acquire resources
    sensors_state_machine_acquire_resources();

#ifdef OMMOCOMM_ENABLED
    // Put ommocomm instances into auto mode (if they have sensors attached)
    for (ommocomm_uarte *& ommocomm_dev : ommocomm_instances)
    {
        APP_ERROR_CHECK(ommocomm_dev->automode_enable());
    }

    ommocomm_packet_malformed_countdown = 0;   //only checked when value != 0
#endif

#ifdef OMMO_IMU_IRQ_PIN
    nrfx_gpiote_in_event_disable(OMMO_IMU_IRQ_PIN);
#endif

    uint32_t prev_timestamp_bs_units, unused;
    timestamp_get_last_sample_event_timestamp_basestation_units(prev_timestamp_bs_units, unused);
    prev_timestamp_bs_units -= OMMO_TIMESTAMP_SAMPLE_PERIOD;
    sensors_schedule_i2c_set_reset(prev_timestamp_bs_units);

    // Change states
#ifdef MEMSIC_DISABLE_SET_RESET
    current_state = SETUP_MAG_START_SAMPLE;
#else
    current_state = SETUP_MAG_SET_COMMAND;
#endif
    sensors_state_change();

    return NRF_SUCCESS;
}

ret_code_t sensors_stop_state_machine(sensors_stop_cmd_t cmd, bool blocking)
{
    if (cmd != SENSORS_STOP_CMD_NONE)
    {
        if(current_state != STATE_MACHINE_STOPPED)
        {
            stop_state_machine_cmd = cmd;
        }

        if(blocking)
        {
            while(current_state != STATE_MACHINE_STOPPED)
            {
                //This fixes ISR safety issue
                //Will make sure current_state == STATE_MACHINE_STOPPED even interrupts with higher priority jump in
                stop_state_machine_cmd = cmd;

                main_event_queue.execute_once();
            }
            stop_state_machine_cmd = SENSORS_STOP_CMD_NONE;
        }

        return NRF_SUCCESS;
    }

    return NRF_ERROR_INVALID_PARAM;
}

static void sensors_schedule_i2c_set_reset(uint32_t prev_timestamp_bs_units)
{
    i2c_interval_count = (prev_timestamp_bs_units % (OMMO_TIMESTAMP_SAMPLE_PERIOD * OMMO_SENSOR_I2C_TRX_INTERVAL)) / OMMO_TIMESTAMP_SAMPLE_PERIOD;
}

uint8_t sensors_get_total_attached()
{
    return total_num_ics;
}

static void sensors_set_ppi_start_transfer_event_addr_and_enable(uint32_t event)
{
    for (const nrf_ppi_channel_t channel : ppi_channel_start_transfer)
    {
        nrf_ppi_event_endpoint_setup(channel, event);
    }

    nrfx_ppi_group_enable(sampling_ppi_start_group); //Next PPI will be on sample time offset

}

void sensors_on_serial_closed()
{
#ifdef OMMO_PORT_LED
    if ((current_state == STATE_MACHINE_STOPPED || current_state == IDLE_HOT_PLUG_CHECK) && !keep_port_leds_on_close)
    {
        sensors_state_t previous_state = current_state;
        sensors_stop_state_machine();

        uint8_t data[OMMO_PM_PORTS_COUNT * 3] = { 0 };
        serial_led_spi port_led_serial_instance(OMMO_PORT_LED_INVERTED, OMMO_PORT_LED_RGB_BYTE_ORDER);
        if (port_led_serial_instance.acquire_spi(OMMO_PORT_LED_SPI_BUS_INDEX) == NRF_SUCCESS)
        {
            APP_ERROR_CHECK(port_led_serial_instance.write_port_leds(data, sizeof(data)));
        }
        port_led_serial_instance.release();


#ifndef HOT_PLUG_OFF
        if(previous_state == IDLE_HOT_PLUG_CHECK)
        {
            sensors_start_idle_hot_plug_check();
        }
#endif
    }
#endif
}

uint16_t sensors_process_packet_received(uint8_t data[], uint16_t data_length, uint8_t response_buffer[], uint16_t response_buffer_size)
{
    switch (data[0])
    {
        case OMMO_COMMAND_SET_MAG_CAL_MODE:
        {
            if (data_length != 3 || data[1] != 0x55)
            {
                return fill_in_ack_packet(response_buffer, OMMO_ACK_INVALID_DATA);
            }
            mag_calibration_mode = data[2];

            // Changes to mag_calibration_mode invalidate the state machine configuration
            state_machine_configured.clear();

            return fill_in_ack_packet(response_buffer, OMMO_ACK_SUCCESS);
        }

        case OMMO_COMMAND_ONBOARD_SENSOR_ENABLE:
        {
            if (data_length != 2)
            {
                return fill_in_ack_packet(response_buffer, OMMO_ACK_INVALID_DATA);
            }

            // if state_machine_configured changes, state machine needs to be reconfigured
            if(onboard_sensors_enabled != (bool)data[1])
            {
                state_machine_configured.clear();
            }

            onboard_sensors_enabled = data[1];
            return fill_in_ack_packet(response_buffer, OMMO_ACK_SUCCESS);
        }

        case OMMO_COMMAND_GET_PACKET_DESCRIPTOR:
        {
            if (current_state != STATE_MACHINE_STOPPED && current_state != IDLE_HOT_PLUG_CHECK)
            {
                return fill_in_ack_packet(response_buffer, OMMO_ACK_INVALID_MODE);
            }

            return sensors_scan_bus_generate_data_descriptor(response_buffer, response_buffer_size);
        }

        case OMMO_COMMAND_MMC_SELF_TEST:
        {
            if (current_state != STATE_MACHINE_STOPPED && current_state != IDLE_HOT_PLUG_CHECK)
            {
                return fill_in_ack_packet(response_buffer, OMMO_ACK_INVALID_MODE);
            }

            if (data_length != 5 || data[1] >= OMMO_PM_PORTS_COUNT || data[2] >= OMMO_PM_MAX_SS_PINS_PER_BUS)
            {
                return fill_in_ack_packet(response_buffer, OMMO_ACK_INVALID_DATA);
            }

            uint16_t count;
            memcpy(&count, &data[3], 2);
            // TODO:  return sensors_read_self_test_info_physical_port(port, port_ss, count, response_buffer);
            break;
        }

        case OMMO_COMMAND_SENSOR_DISABLE:
        {
            if (data_length != 3 || data[1] >= OMMO_PM_PORTS_COUNT)
            {
                return fill_in_ack_packet(response_buffer, OMMO_ACK_INVALID_DATA);
            }

            // if disable_ss_pin_on_port changes, state machine needs to be reconfigured
            if(disable_ss_pin_on_port[data[1]] != data[2])
            {
                state_machine_configured.clear();
            }

            disable_ss_pin_on_port[data[1]] = data[2];
            return fill_in_ack_packet(response_buffer, OMMO_ACK_SUCCESS);
        }

#ifdef OMMO_PORT_LED
        case OMMO_COMMAND_SET_PORT_LED:
        {
            // Check data length
            if((OMMO_PM_PORTS_COUNT*3) == data_length-2)
            {
                // Do we have keep_port_leds_on_close flag set?
                keep_port_leds_on_close = (data[data_length-1] != 0);
            }
            else if((OMMO_PM_PORTS_COUNT*3) != data_length-1)
            {
                return fill_in_ack_packet(response_buffer, OMMO_ACK_INVALID_DATA);
            }

            sensors_state_t previous_state = current_state;
            if(current_state != STATE_MACHINE_STOPPED && current_state != IDLE_HOT_PLUG_CHECK)
            {
                return fill_in_ack_packet(response_buffer, OMMO_ACK_INVALID_MODE);
            }
            else
            {
                sensors_stop_state_machine();

                serial_led_spi port_led_serial_instance(OMMO_PORT_LED_INVERTED, OMMO_PORT_LED_RGB_BYTE_ORDER);
                if (port_led_serial_instance.acquire_spi(OMMO_PORT_LED_SPI_BUS_INDEX) == NRF_SUCCESS)
                {
                    APP_ERROR_CHECK(port_led_serial_instance.write_port_leds(&data[1], (OMMO_PM_PORTS_COUNT*3)));
                }
                port_led_serial_instance.release();
            }

#ifndef HOT_PLUG_OFF
            if(previous_state == IDLE_HOT_PLUG_CHECK)
            {
                sensors_start_idle_hot_plug_check();
            }
#endif

            return fill_in_ack_packet(response_buffer, OMMO_ACK_SUCCESS);
        }
#endif
        case OMMO_COMMAND_READ_SENSOR_DATA:
        {
            //COMMAND PORT BUS_TYPE BUS_LOCATION/SS_INDEX
            if(data_length < 4)
            {
                return fill_in_ack_packet(response_buffer, OMMO_ACK_INVALID_DATA);
            }
            else if(current_state != STATE_MACHINE_STOPPED && current_state != IDLE_HOT_PLUG_CHECK)
            {
                return fill_in_ack_packet(response_buffer, OMMO_ACK_INVALID_MODE);
            }

            const uint8_t port = data[1];
            const ICBusType bus_type = (ICBusType)data[2];
            const uint8_t bus_location_ss_index = data[3];

            //Find ic dictionary index
            uint8_t ic_dict_index = 0;
            if(bus_type == IC_BUS_TYPE_SPI)
            {
                ic_dict_index = ss_map[port][bus_location_ss_index];
            }
            else if(bus_type == IC_BUS_TYPE_I2C)
            {
                for(uint8_t i=0; i<num_i2c_ics; i++)
                {
                    if(i2c_ic_port_num[i] == port && i2c_ic_bus_location[i] == bus_location_ss_index)
                    {
                        ic_dict_index = i2c_ic_dictionary_index[i];
                        break;
                    }
                }
            }

            uint32_t error = NRF_SUCCESS;
            if(ic_dict_index == 0)
                error = NRF_ERROR_NOT_FOUND;

            //Acquire comm_device and read data
            comm_device_guard comm_dev;
            uint16_t response_length;
            if (error == NRF_SUCCESS)
                error = comm_dev.acquire(port, bus_type, bus_location_ss_index);
            if (error == NRF_SUCCESS)
            {
                //Read data, compose response
                error = sensors_sensor_data_read_blocking(comm_dev, response_buffer + 3, response_buffer_size - 3, (uint8_t*)&response_length, ic_dict_index);
                response_buffer[0] = OMMO_COMMAND_READ_SENSOR_DATA;
                memcpy(response_buffer + 1, &response_length, 2); //2 payload length length
                response_length += 3;
            }

            if(error != NRF_SUCCESS)
            {
                return fill_in_ack_packet(response_buffer, convert_nrf_error_code_to_ommo_ack_code(error));
            }

            //response already composed
            return response_length;
        }

        case OMMO_COMMAND_PROCESS_FLASHSTREAM:
        {
            //COMMAND PORT BUS_TYPE BUS_LOCATION/SS_INDEX CMD_DELAY_L CMD_DELAY_H BYTES_L BYTES_H DATA
            if(data_length < 8)
            {
                return fill_in_ack_packet(response_buffer, OMMO_ACK_INVALID_DATA);
            }
            else if(current_state != STATE_MACHINE_STOPPED && current_state != IDLE_HOT_PLUG_CHECK)
            {
                return fill_in_ack_packet(response_buffer, OMMO_ACK_INVALID_MODE);
            }

            uint8_t port = data[1];
            ICBusType bus_type = (ICBusType)data[2];
            uint8_t bus_location_ss_index = data[3];
            uint16_t command_delay = *(uint16_t*)&data[4];
            uint16_t stream_len = *(uint16_t*)&data[6];

            if(data_length < (8 + stream_len))
                return fill_in_ack_packet(response_buffer, OMMO_ACK_INVALID_DATA);

            if(port >= OMMO_PM_PORTS_COUNT)
                return fill_in_ack_packet(response_buffer, OMMO_ACK_INVALID_DATA);

            sensors_state_t previous_state = current_state;
            sensors_stop_state_machine();

            //Acquire comm device and execute flash stream
            comm_device_guard comm_dev;
            ret_code_t error = comm_dev.acquire(port, bus_type, bus_location_ss_index);
            if(error == NRF_SUCCESS)
                error = comm_dev->execute_flashstream_command_sequence(&data[8], stream_len, command_delay);

#ifndef HOT_PLUG_OFF
            if(previous_state == IDLE_HOT_PLUG_CHECK)
            {
                sensors_start_idle_hot_plug_check();
            }
#endif

            return fill_in_ack_packet(response_buffer, convert_nrf_error_code_to_ommo_ack_code(error));
        }

        default:
            break;
    }

    //See if device info processor can process it
    sensors_state_t previous_state = current_state;
    sensors_stop_state_machine();
    ret_code_t ret = device_info_process_packet_received(data, data_length, response_buffer, response_buffer_size, mag_calibration_mode);
#ifndef HOT_PLUG_OFF
    if(previous_state == IDLE_HOT_PLUG_CHECK)
    {
        sensors_start_idle_hot_plug_check();
    }
#endif
    return ret;
}

//static void sensors_clear_all_mag_ss_lines()
//{
//    active_ss_pin = 0xFF;
//    for(uint8_t ss_line = 0; ss_line < num_ss_pins_with_mag; ss_line++)
//        nrf_gpio_pin_write(port_master_ss_pins[ss_pins_with_mag[ss_line]], 1);
//}

//static void sensors_clear_all_ss_lines()
//{
//    active_ss_pin = 0xFF;
//    for(uint8_t ss_line = 0; ss_line < OMMO_PM_SS_PINS_COUNT; ss_line++)
//        nrf_gpio_pin_write(port_master_ss_pins[ss_line], 1);
//}

static void sensors_start_spi_sensor_data_read(uint8_t spi_channel, uint8_t ic_data_index, comm_device_trx_flags_t flags)
{
    if (spi_bus_comm_devices[spi_channel])
    {
        APP_ERROR_CHECK(spi_bus_comm_devices[spi_channel]->trx(
                                    (uint8_t * )ic_data[ic_data_index].read_command[0].data_read_tx, ic_data[ic_data_index].read_command[0].data_read_tx_len,
                                    m_spi_rx_buf[spi_channel], ic_data[ic_data_index].read_command[0].data_read_rx_len, spi_event_handler, nullptr, flags));
    }
}

static void sensors_read_mag_id(uint8_t spi_channel)
{
    if (spi_bus_comm_devices[spi_channel])
    {
        static uint8_t read_product_id = MMC5983_READ_REG_MASK(MMC5983_PRODUCT_ID_REG);
        APP_ERROR_CHECK(spi_bus_comm_devices[spi_channel]->trx(&read_product_id, sizeof(read_product_id), m_spi_rx_buf[spi_channel], 2, spi_event_handler));
    }
}

static uint32_t sensors_sensor_data_read_blocking(comm_device *comm_device_ptr, uint8_t *output_buffer, uint16_t output_buffer_len, uint8_t *output_bytes_written, uint8_t ic_dict_index)
{
    uint32_t error = NRF_SUCCESS;

    *output_bytes_written = 0;
    for(uint8_t read_command_index = 0; error == NRF_SUCCESS && read_command_index < ic_data[ic_dict_index].num_read_commands; read_command_index++)
    {
        comm_device_ptr->trx((const uint8_t*)ic_data[ic_dict_index].read_command[read_command_index].data_read_tx,
                               ic_data[ic_dict_index].read_command[read_command_index].data_read_tx_len,
                               m_spi_rx_buf[0], ic_data[ic_dict_index].read_command[read_command_index].data_read_rx_len);

        //Save data to output buffer
        uint8_t len = ic_data[ic_dict_index].read_command[read_command_index].data_length;
        uint8_t input_offset = ic_data[ic_dict_index].read_command[read_command_index].data_offset;
        //uint8_t output_offset = ic_data[ic_dict_index].read_command[read_command_index].data_output_index_offset;

        if((len + *output_bytes_written) < output_buffer_len)
        {
            memcpy(output_buffer + *output_bytes_written, m_spi_rx_buf[0]+input_offset, len);
            *output_bytes_written += len;
        }
        else
        {
            error = NRF_ERROR_NO_MEM;
        }
    }

    return error;
}

//TODO move elsewhere
// Confirms a TMP1075N/TMP110 family device is present at addr by checking CFGR default,
// then returns the IC type based on the slave address.
static ICType sensors_identify_ti_i2c_temp_sensor(comm_device *comm_device_ptr, uint8_t addr)
{
    uint8_t tx_data[2];
    uint16_t rvalue;

    tx_data[0] = addr;
    tx_data[1] = TI_TMP_REG_CFGR;
    if(comm_device_ptr->trx(tx_data, 2, NULL, 0) != NRF_SUCCESS)
        return IC_TYPE_NONE;
    if(comm_device_ptr->trx(tx_data, 1, (uint8_t*)&rvalue, 2) != NRF_SUCCESS)
        return IC_TYPE_NONE;
    if(rvalue != TI_TMP_CFGR_POR)
        return IC_TYPE_NONE;

    tx_data[1] = TI_TMP_REG_TEMP;
    if(comm_device_ptr->trx(tx_data, 2, NULL, 0) != NRF_SUCCESS)
        return IC_TYPE_NONE;

    if(addr == TMP1075N_ADDR)
        return IC_TYPE_TMP1075N;
    if(addr == TMP110_ADDR)
        return IC_TYPE_TMP110;

    return IC_TYPE_NONE;
}

void sensors_set_data_descriptor_header_format(DataHeaderFormat dhf)
{
    data_header_format = dhf;
}

static uint8_t find_ic_data_dictionary_index(ICType ic_type)
{
    //Find it in dictionary
    for(size_t i = 0; i < sizeof(ic_data) / sizeof(ic_data_dictionary_entry); i++)
    {
        if(ic_type == ic_data[i].ic_type)
        {
            return i;
        }
    }

    return 0; //IC_TYPE_NONE
}

static ICType scan_comm_device_for_ics(comm_device *comm_device_ptr, bool look_for_spi_devices)
{
    uint8_t rx_buf[2], tx_buf[2];

    if(look_for_spi_devices)
    {
        //MMC5983
        ret_code_t error_code = comm_device_ptr->read_byte(MMC5983_PRODUCT_ID_REG | 0x80, rx_buf);
        if(error_code == NRF_SUCCESS && rx_buf[0] == MMC5983_PRODUCT_ID_RESULT)
        {
            return IC_TYPE_MEMSIC_5983;
        }

        //IMU DSM (LSM6DSM)
        error_code = comm_device_ptr->read_byte(IMU_DSX_CMD_WHO_AM_I | 0x80, rx_buf);
        if(error_code == NRF_SUCCESS && rx_buf[0] == IMU_DSM_WHO_AM_I)
        {
            return IC_TYPE_LSM6DSM;
        }

        //IMU DSV (LSM6DSV)
        error_code = comm_device_ptr->read_byte(IMU_DSX_CMD_WHO_AM_I | 0x80, rx_buf);
        if(error_code == NRF_SUCCESS && rx_buf[0] == IMU_DSV_WHO_AM_I)
        {
            return IC_TYPE_LSM6DSV;
        }

        //IMU 42605
        error_code = comm_device_ptr->read_byte(IMU_42605_REG_WHO_AM_I | 0x80, rx_buf);
        if(error_code == NRF_SUCCESS && rx_buf[0] == IMU_42605_WHO_AM_I)
        {
            return IC_TYPE_ICM_42605;
        }

        //TMP126
        tx_buf[0] = TMP126_CMD_READ;
        tx_buf[1] = TMP126_DEVICE_ID_REG;
        error_code = comm_device_ptr->trx(tx_buf, 2, rx_buf, 2);
        if(error_code == NRF_SUCCESS &&
          (rx_buf[0] & TMP126_DEVICE_ID_VAL_MASK_MSB) == TMP126_DEVICE_ID_VAL_MSB &&
          (rx_buf[1] & TMP126_DEVICE_ID_VAL_MASK_LSB) == TMP126_DEVICE_ID_VAL_LSB)
        {
            return IC_TYPE_TMP126;
        }
    }
    else //look for I2C devices
    {
        ICType tmp_type = sensors_identify_ti_i2c_temp_sensor(comm_device_ptr, TMP1075N_ADDR);
        if(tmp_type == IC_TYPE_NONE)
            tmp_type = sensors_identify_ti_i2c_temp_sensor(comm_device_ptr, TMP110_ADDR);
        if(tmp_type != IC_TYPE_NONE)
            return tmp_type;

#ifdef OMMO_BQ27Z558
        if(sensors_fuel_gauge_instance.check_if_present(comm_device_ptr))
        {
            return IC_TYPE_BQ27Z558;
        }
#endif
    }

    return IC_TYPE_NONE;
}

static ret_code_t scan_port_for_devices(uint8_t port, bool is_virtual_port, ICBusType *ic_bus_type, uint8_t *ic_ss_index_bus_location, ICType *ic_type, uint8_t *num_ics)
{
    //Clear count
    *num_ics = 0;
    uint8_t total_num_ics = 0;

#ifdef OMMOCOMM_ENABLED
    // Scan ommocomm port for SPI devices
    if (is_virtual_port && port_master_ports[port].supports_ommocomm_virtual_port_pin != 0xFF)
    {
        for (uint8_t port_ss_index = 0; port_ss_index < OMMOCOMM_NUM_SS_PER_PORT; port_ss_index++)
        {
            // Acquire comm device
            comm_device_guard comm_dev;
            VERIFY_SUCCESS(comm_dev.acquire(port, IC_BUS_TYPE_OMMOCOMM_SPI, port_ss_index));

            ICType found_ic_type = scan_comm_device_for_ics(comm_dev, true);
            if (found_ic_type != IC_TYPE_NONE)
            {
                if (total_num_ics >= OMMO_PM_MAX_ICS_PER_PORT)
                    return NRF_ERROR_NO_MEM;
                ic_bus_type[total_num_ics] = IC_BUS_TYPE_OMMOCOMM_SPI;
                ic_ss_index_bus_location[total_num_ics] = port_ss_index;
                ic_type[total_num_ics] = found_ic_type;
                ++total_num_ics;
            }
        }

        // Scan ommocomm port for I2C devices
        for (uint8_t i2c_bus_index = 0; i2c_bus_index < OMMOCOMM_NUM_I2C_BUSSES_PER_PORT; i2c_bus_index++)
        {
            // Acquire comm device
            comm_device_guard comm_dev;
            VERIFY_SUCCESS(comm_dev.acquire(port, IC_BUS_TYPE_OMMOCOMM_I2C, i2c_bus_index));

            ICType found_ic_type = scan_comm_device_for_ics(comm_dev, false);
            if (found_ic_type != IC_TYPE_NONE)
            {
                if (total_num_ics >= OMMO_PM_MAX_ICS_PER_PORT)
                    return NRF_ERROR_NO_MEM;
                ic_bus_type[total_num_ics] = IC_BUS_TYPE_OMMOCOMM_I2C;
                ic_ss_index_bus_location[total_num_ics] = i2c_bus_index;
                ic_type[total_num_ics] = found_ic_type;
                ++total_num_ics;
            }
        }
    }
    else
#endif
    {
        //Scan port for SPI devices
        for(spi_slice_index_t port_slice_index=0; port_slice_index < port_master_ports[port].spi_count; port_slice_index++)
        {
            //Scan each ss index
            for(spi_ss_index_t port_ss_index=0; port_ss_index < port_master_ports[port].spi_buses[port_slice_index].ss_index_count; port_ss_index++)
            {
                //Acquire comm device
                comm_device_guard comm_dev;
                uint8_t bus_location = port_master_encode_spi_bus_location(port_slice_index, port_ss_index);
                VERIFY_SUCCESS(comm_dev.acquire(port, IC_BUS_TYPE_SPI, bus_location));

                ICType found_ic_type = scan_comm_device_for_ics(comm_dev, true);
                if(found_ic_type != IC_TYPE_NONE)
                {
                    if (total_num_ics >= OMMO_PM_MAX_ICS_PER_PORT)
                        return NRF_ERROR_NO_MEM;
                    ic_bus_type[total_num_ics] = IC_BUS_TYPE_SPI;
                    ic_ss_index_bus_location[total_num_ics] = bus_location;
                    ic_type[total_num_ics] = found_ic_type;
                    ++total_num_ics;
                }
            }
        }

        //Scan i2c port locations for i2c devices
        for(uint8_t twi_bus_index=0; twi_bus_index < port_master_ports[port].twi_count; twi_bus_index++)
        {
            ic_bus_type[total_num_ics] = IC_BUS_TYPE_I2C;
            ic_ss_index_bus_location[total_num_ics] = port_master_ports[port].twi_buses[twi_bus_index].location;

            //Acquire comm device
            comm_device_guard comm_dev;
            VERIFY_SUCCESS(comm_dev.acquire(port, IC_BUS_TYPE_I2C, port_master_ports[port].twi_buses[twi_bus_index].location));

            ICType found_ic_type = scan_comm_device_for_ics(comm_dev, false);
            if(found_ic_type != IC_TYPE_NONE)
            {
                if (total_num_ics >= OMMO_PM_MAX_ICS_PER_PORT)
                    return NRF_ERROR_NO_MEM;
                ic_bus_type[total_num_ics] = IC_BUS_TYPE_I2C;
                ic_ss_index_bus_location[total_num_ics] = port_master_ports[port].twi_buses[twi_bus_index].location;
                ic_type[total_num_ics] = found_ic_type;
                ++total_num_ics;
            }
        }
    }

    *num_ics = total_num_ics;
    return NRF_SUCCESS;
}

uint16_t sensors_scan_bus_generate_data_descriptor(uint8_t packet_id_request_buffer[], uint16_t buffer_size, data_descriptor_callback_t *data_descriptor_callback, const void *context)
{
    device_info_storage_type port_type[OMMO_PM_PORTS_COUNT] = {DEVICE_INFO_NONE};
    PortStatus port_status_list[OMMO_PM_PORTS_COUNT];
    DeviceGroupDescriptorProto dgdp = {0};
    bool disable_onboard_sensors = !onboard_sensors_enabled;

    //Assemble static stream descriptor
    StreamDescriptorProto sdp =
    {
        .descriptors_count = 1,
        .descriptors = &dgdp,
        .port_status_list_count = OMMO_PM_PORTS_COUNT,
        .port_status_list = port_status_list,
    };

    //Clear existing ss_map and data output indicies
    memset(port_status_list, 0x00, sizeof(port_status_list));
    memset(ss_map, 0x00, sizeof(ss_map));
    memset(data_output_index, 0xFF, sizeof(data_output_index)); //OMMO_PM_SPI_BUSSES_COUNT*OMMO_PM_SS_PINS_COUNT*2);
    total_num_ics = 0;
    num_i2c_ics = 0;

#ifdef OMMOCOMM_ENABLED
    ommocomm_num_present = 0;
    memset(ommocomm_data_len, 0, sizeof(ommocomm_data_len));
    memset(ommocomm_save_data, false, sizeof(ommocomm_save_data));
#endif

    //Stop the state machine
    sensors_stop_state_machine();

    //Reset hot_plug state
    hot_unplug_event_set = false;
    hot_plugin_event_set = false;
    hot_unplug_double_check = false;

    //Reset LSM6DSV int2 state
    lsm6dsv_detected = false;

#if OMMO_PM_SPIM_TWI_BASIC_COUNT || OMMO_PM_TWI_BASIC_COUNT
    //Set I2C speed based on SCL/SDA rise times
    //TODO consider running every time an i2c port on bus_location 0 is acquired
    APP_ERROR_CHECK(port_master_measure_and_calculate_i2c_MOSI_CS0_frequency());
#endif

    //Allocate device list large enough for all ports
    if((dgdp.devices = (DeviceDescriptorProto *)malloc(OMMO_PM_PORTS_COUNT*sizeof(DeviceDescriptorProto))) == NULL)
        return sensors_scan_bus_generate_data_descriptor_return_error(packet_id_request_buffer, &dgdp, OMMO_ACK_INTERNAL_ERROR);
    memset(dgdp.devices, 0x00, OMMO_PM_PORTS_COUNT*sizeof(DeviceDescriptorProto));

    //Search ports for devices
    for(uint8_t port = 0; port < OMMO_PM_PORTS_COUNT; port++)
    {
        bool device_present = false;
        uint8_t current_device = dgdp.devices_count;

        //Skip ports that do not support sampling
        if (!port_master_ports[port].supports_sampling)
            continue; 

        //Make sure this device is cleared, it is possible we could have started filling it out and decided the device was invalid
        pb_release(&DeviceDescriptorProto_msg, &dgdp.devices[current_device]);
        memset(&dgdp.devices[current_device], 0x00, sizeof(DeviceDescriptorProto));

        //Set port number
        dgdp.devices[current_device].port_num = port;

        //Determine port type and read device info
        port_type[port] = device_info_scan_port_for_storage_media(port);
        if(port_type[port] != DEVICE_INFO_NONE) //Is storage media present?
        {
            if(device_info_allocate_and_read_from_port(port, port_type[port], (void**)&dgdp.devices[current_device].device_info, DEVICE_INFO_FIELD_PERM, &dgdp.devices[current_device].device_info_structure) == NRF_SUCCESS)
            {
                device_present = true;

                device_info_allocate_and_read_from_port(port, port_type[port], (void**)&dgdp.devices[current_device].device_info_user, DEVICE_INFO_FIELD_USER);

                if(dgdp.devices[current_device].device_info_user == NULL)
                {
                    if(device_info_allocate_and_create_user(&dgdp.devices[current_device].device_info_user) != NRF_SUCCESS)
                        return sensors_scan_bus_generate_data_descriptor_return_error(packet_id_request_buffer, &dgdp, OMMO_ACK_NO_MEM);
                }
            }
        }

        //Scan port to so that we can flag detected sensors that are invalid
        ICBusType ic_bus_type[OMMO_PM_MAX_ICS_PER_PORT];
        uint8_t ic_ss_index_bus_location[OMMO_PM_MAX_ICS_PER_PORT];
        ICType ic_type[OMMO_PM_MAX_ICS_PER_PORT];
        uint8_t num_ics;

        //Scan port for all devices
        APP_ERROR_CHECK(scan_port_for_devices(port, port_type[port] == DEVICE_INFO_OMMOCOMM_VIRTUAL, ic_bus_type, ic_ss_index_bus_location, ic_type, &num_ics));

        //If we are in mag_cal mode scan generate device_info
        if(mag_calibration_mode)
        {
            uint64_t uuid = 0;
            uint64_t calibration_date = 0;

            //If we have an allocated device info, pull out data and free memory
            if(device_present)
            {
                calibration_date = dgdp.devices[current_device].device_info->calibration_date;
                uuid = dgdp.devices[current_device].device_info->uuid;

                device_info_free_perm_proto(&dgdp.devices[current_device].device_info);
                device_info_free_user_proto(&dgdp.devices[current_device].device_info_user);
                device_present = false;
            }

            //Create and allocate device info
            if(port_type[port] != DEVICE_INFO_NONE) //Is storage media present?
            {
                uint32_t error_code = device_info_allocate_and_create_perm(num_ics, ic_bus_type, ic_ss_index_bus_location, ic_type, &dgdp.devices[current_device].device_info);
                if(error_code != NRF_SUCCESS)
                    return sensors_scan_bus_generate_data_descriptor_return_error(packet_id_request_buffer, &dgdp, OMMO_ACK_INTERNAL_ERROR);
                device_present = true;

                //Set saved/default values
                dgdp.devices[current_device].device_info->uuid = uuid;
                dgdp.devices[current_device].device_info->calibration_date = calibration_date;
                dgdp.devices[current_device].device_info->data_sourcing_strat = DSS_INITIALIZATION;

                //Flag onboard device if it is
                if(port_master_ports[port].flash_media_present) dgdp.devices[current_device].device_info->onboard_siu_parent = APP_USBD_PID;

                error_code = device_info_allocate_and_create_user(&dgdp.devices[current_device].device_info_user);
                if(error_code != NRF_SUCCESS)
                    return sensors_scan_bus_generate_data_descriptor_return_error(packet_id_request_buffer, &dgdp, OMMO_ACK_INTERNAL_ERROR);
            }
        }

        //If we don't have a device but there are sensors/eeprom, flag them for hotplug detected and flag port as invalid
        if(!device_present)
        {
            if(port_type[port] != DEVICE_INFO_NONE || num_ics > 0)
            {
                port_status_list[port] = PORT_STATUS_ERROR;
                for(uint8_t i=0; i<num_ics; i++)
                {
                    if(ic_bus_type[i] == IC_BUS_TYPE_SPI)
                    {
                        spi_slice_index_t slice_index = port_master_decode_spi_slice_index(ic_ss_index_bus_location[i]);
                        spi_ss_index_t ss_index = port_master_decode_spi_ss_index(ic_ss_index_bus_location[i]);
                        ss_map[port_master_ports[port].spi_buses[slice_index].spi_bus_index][ss_index] = IC_DATA_DISABLED_INDEX;
                    }
                }
            }

            continue;
        }

#ifndef EMI_RUN_WITHOUT_SENSOR

        //Figure out firmware spi sample delay on this port
        uint32_t synch_to_spi_start_sample_complete_delay = SENSOR_SPI_START_DELAY; //This constant is exactly the time for a 2Mhz 2 byte SPI TX

#ifdef OMMOCOMM_ENABLED
         if(port_type[port] == DEVICE_INFO_OMMOCOMM_VIRTUAL)
         {
            ommocomm_uarte *ommocomm_uarte;
            if(port_master_acquire_ommocomm_direct(port, true, &ommocomm_uarte) == NRF_SUCCESS)
            {
                 uint32_t ommocomm_synch_complete_to_spi_start_sample_complete_delay;
                 if(ommocomm_uarte->get_synch_to_spi_tx_delay(&ommocomm_synch_complete_to_spi_start_sample_complete_delay) != NRF_SUCCESS)
                    device_present = false;
                 synch_to_spi_start_sample_complete_delay = ommocomm_synch_complete_to_spi_start_sample_complete_delay - OMMO_TIMESTAMP_SAMPLE_PERIOD; //Ommocomm samples are delayed by one cycle
            }
            ommocomm_uarte->release();
         }
#endif

        //Init and verify ic, fill in blank values ic/sensor values
        for(uint8_t ic_num = 0; device_present && ic_num < dgdp.devices[current_device].device_info->ics_count; ic_num++)
        {
            ICBusType bus_type = dgdp.devices[current_device].device_info->ics[ic_num].ic_bus_type;
            uint8_t bus_location_ss_index = dgdp.devices[current_device].device_info->ics[ic_num].ic_ss_index_bus_location;

            comm_device_guard comm_dev;
            if(comm_dev.acquire(port, bus_type, bus_location_ss_index) != NRF_SUCCESS)
                device_present = false;

            if(device_present)
            {
                switch(dgdp.devices[current_device].device_info->ics[ic_num].ic_type)
                {
                    case IC_TYPE_MEMSIC_5983:
                    {
                        //Check product id
                        uint8_t reg_value;
                        if (device_present)
                            device_present = comm_dev->read_byte(MMC5983_PRODUCT_ID_REG | 0x80, &reg_value) == NRF_SUCCESS && reg_value == MMC5983_PRODUCT_ID_RESULT;

                        ////Turn off various compensations
                        bool dis_tc = dgdp.devices[current_device].device_info->ics[ic_num].config_disable_temp_comp;
                        bool dis_sensitivity = dgdp.devices[current_device].device_info->ics[ic_num].config_disable_sensitivity_ortho_matrix;
                        bool dis_yz_slope = dgdp.devices[current_device].device_info->ics[ic_num].config_disable_yz_slope_matrix;
                        if(dis_tc | dis_sensitivity | dis_yz_slope)
                        {
                            uint8_t val = 0;

                            //Unlock device
                            if(device_present) device_present = comm_dev->write_byte(MMC5983_UNLOCK_REG, MMC5983_UNLOCK_VAL);

                            //Do disables
                            if(dis_tc)
                            {
                                if(device_present) device_present = comm_dev->read_byte(0x29, &val);
                                val |= 0x01;
                                if(device_present) device_present = comm_dev->write_byte(0x29, val);
                            }
                            if(dis_sensitivity)
                            {
                                if(device_present) device_present = comm_dev->read_byte(0x2C, &val);
                                val &= 0x07;
                                if(device_present) device_present =comm_dev->write_byte(0x2C, val);
                                if(device_present) device_present =comm_dev->write_byte(0x2D, 0x00);
                                if(device_present) device_present =comm_dev->write_byte(0x2E, 0x00);
                            }
                            if(dis_yz_slope)
                            {
                                if(device_present) device_present = comm_dev->read_byte(0x21, &val);
                                val |= 0x80;
                                if(device_present) device_present = comm_dev->write_byte(0x21, val);
                            }
                        }

                        //Init
                        if(device_present) device_present = comm_dev->write_byte(MMC5983_CONTROL1_REG, MMC5983_CONTROL1_CHIP_RESET) == NRF_SUCCESS;
                        ommo_delay_ms(20);
                        if(device_present) device_present = comm_dev->write_byte(MMC5983_CONTROL0_REG, MMC5983_CONTROL0_INIT) == NRF_SUCCESS;
                        if(device_present) device_present = comm_dev->write_byte(MMC5983_CONTROL1_REG, MMC5983_CONTROL1_INIT) == NRF_SUCCESS;

                        //Verify
                        // uint8_t control0, control1;
                        //if(device_present) device_present = comm_dev->read_byte(MMC5983_CONTROL0_REG, &control0) == NRF_SUCCESS; // && control0 == MMC5983_CONTROL0_INIT;
                        //if(device_present) device_present = comm_dev->read_byte(MMC5983_CONTROL1_REG, &control1) == NRF_SUCCESS; // && control1 == MMC5983_CONTROL1_INIT;

                        if(!device_present)
                            break;

                        //Defaults
                        if(device_info_allocate_and_fill_in_default_ic_data(&dgdp.devices[current_device].device_info->ics[ic_num], &ic_def_mmc5983, synch_to_spi_start_sample_complete_delay, dgdp.devices[current_device].device_info->data_sourcing_strat) != NRF_SUCCESS)
                            device_present = false;

                        break;
                    }

                    case IC_TYPE_LSM6DSM: //Does this channel have an LSM6DSM IMU
                    {
                        //Check product id
                        uint8_t reg_value;
                        if(device_present) device_present = comm_dev->read_byte(IMU_DSX_CMD_WHO_AM_I, &reg_value) == NRF_SUCCESS && reg_value == IMU_DSM_WHO_AM_I;

                        //Init
                        if(device_present) device_present = comm_dev->write_byte(IMU_DSX_CTRL9_XL, IMU_DSM_CTRL9_XL_INIT) == NRF_SUCCESS;
                        if(device_present) device_present = comm_dev->write_byte(IMU_DSX_CTRL1_XL, IMU_DSM_CTRL1_XL_INIT) == NRF_SUCCESS;
                        if(device_present) device_present = comm_dev->write_byte(IMU_DSX_CTRL10_C, IMU_DSM_CTRL10_C_INIT) == NRF_SUCCESS;
                        if(device_present) device_present = comm_dev->write_byte(IMU_DSX_CTRL2_G, IMU_DSM_CTRL2_G_INIT) == NRF_SUCCESS;
                        if(device_present) device_present = comm_dev->write_byte(IMU_DSX_CTRL3_C, IMU_DSM_CTRL3_C_INIT) == NRF_SUCCESS;

                        //Verify
                        if(device_present) device_present = (comm_dev->read_byte(IMU_DSX_CTRL9_XL, &reg_value) == NRF_SUCCESS) && reg_value == IMU_DSM_CTRL9_XL_INIT;
                        if(device_present) device_present = (comm_dev->read_byte(IMU_DSX_CTRL1_XL, &reg_value) == NRF_SUCCESS) && reg_value == IMU_DSM_CTRL1_XL_INIT;
                        if(device_present) device_present = (comm_dev->read_byte(IMU_DSX_CTRL10_C, &reg_value) == NRF_SUCCESS) && reg_value == IMU_DSM_CTRL10_C_INIT;
                        if(device_present) device_present = (comm_dev->read_byte(IMU_DSX_CTRL2_G, &reg_value) == NRF_SUCCESS) && reg_value == IMU_DSM_CTRL2_G_INIT;
                        if(device_present) device_present = (comm_dev->read_byte(IMU_DSX_CTRL3_C, &reg_value) == NRF_SUCCESS) && reg_value == IMU_DSM_CTRL3_C_INIT;

                        if(!device_present)
                            break;

                        //Defaults
                        if(device_info_allocate_and_fill_in_default_ic_data(&dgdp.devices[current_device].device_info->ics[ic_num], &ic_def_lsm6dsm_no_options, 0, dgdp.devices[current_device].device_info->data_sourcing_strat) != NRF_SUCCESS)
                            device_present = false;
                        break;
                    }

                    case IC_TYPE_LSM6DSV: //Does this channel have an LSM6DSV IMU
                    {
#if defined(OMMO_LSM6DSV_INT2_PIN) || defined(OMMO_LSM6DSV_INT2_NOR_PIN)
                        if(device_present) device_present = lsm6dsv_init_sensor(comm_dev, IMU_DSV_CTRL1_INIT, IMU_DSV_CTRL2_G_INIT) == NRF_SUCCESS;

                        if(!device_present)
                            break;

                        if(device_info_allocate_and_fill_in_default_ic_data(&dgdp.devices[current_device].device_info->ics[ic_num], &ic_def_lsm6dsv_no_options, 0, dgdp.devices[current_device].device_info->data_sourcing_strat) != NRF_SUCCESS)
                            device_present = false;
                        if(device_present)
                            lsm6dsv_detected = true;
#else
                        // if we don't have int2 pin defined, we will have a random odr for the imu
                        device_present = false;
#endif
                        break;
                    }

                    case IC_TYPE_ICM_42605: //Does this channel have an ICM-42605 IMU
                    {
                        // Check product id
                        uint8_t reg_value = 0;
                        device_present
                              = comm_dev->read_byte(IMU_42605_READ_REG_MASK(IMU_42605_REG_WHO_AM_I), &reg_value) == NRF_SUCCESS
                                  && reg_value == IMU_42605_WHO_AM_I;
#ifdef OMMO_IMU_IRQ_PIN
                        const uint8_t int_cfg = (port == 0 ? IMU_42605_INT_CONFIG_VALUE : 0);
                        const uint8_t int4_cfg = (port == 0 ? IMU_42605_INT_SOURCE4_VALUE : 0);
#else
                        // On some HW:
                        // 12668 pointer - conflicted IMU INT2 pin OMMOCOMM uart pin
                        // Some microsd sensors conflicted IMU INT2 pin and FSYNC output
                        const uint8_t int_cfg =  0;
                        const uint8_t int4_cfg = 0;
#endif

                        const uint8_t imu_config[][2] = {
                            { IMU_42605_REG_DEVICE_CFG, IMU_42605_DEVICE_CFG_RESET },
                            { IMU_42605_CMD_DELAY, 2 }, // Wait for 2ms
                            { IMU_42605_REG_INT_CONFIG, int_cfg},
                            { IMU_42605_REG_INTF_CONFIG, IMU_42605_INTF_CONFIG_VALUE},
                            { IMU_42605_REG_GYRO_ACCEL_CONFIG0, IMU_42605_GYRO_ACCEL_CONFIG0_VALUE },
                            { IMU_42605_REG_GYRO_CONFIG0, IMU_42605_GYRO_CONFIG0_VALUE },
                            { IMU_42605_REG_GYRO_CONFIG1, IMU_42605_GYRO_CONFIG1_VALUE },
                            { IMU_42605_REG_ACCEL_CONFIG0, IMU_42605_ACCEL_CONFIG0_VALUE },
                            { IMU_42605_REG_ACCEL_CONFIG1, IMU_42605_ACCEL_CONFIG1_VALUE },

                            { IMU_42605_REG_PWR_MGMT0,  IMU_42605_PWR_MGMT0_VALUE},

                            // Configure Wake on Motion to 0.2G threshold for all axis
                            { IMU_42605_REG_BANK_SEL, IMU_42605_BANK_4_SEL },
                            { IMU_42605_REG_4_ACCEL_WOM_X_THR, IMU_42605_ACCEL_WOM_THR(0.2f) },
                            { IMU_42605_REG_4_ACCEL_WOM_Y_THR, IMU_42605_ACCEL_WOM_THR(0.2f) },
                            { IMU_42605_REG_4_ACCEL_WOM_Z_THR, IMU_42605_ACCEL_WOM_THR(0.2f) },
                            { IMU_42605_REG_BANK_SEL, IMU_42605_BANK_0_SEL },
                            { IMU_42605_REG_INT_SOURCE4, int4_cfg},
                            { IMU_42605_REG_SMD_CONFIG, IMU_42605_SMD_CONFIG_VALUE },
                        };

                        // Write the configuration
                        for (size_t i = 0; device_present && i < sizeof(imu_config) / sizeof(imu_config[0]); ++i)
                        {
                            if (imu_config[i][0] == IMU_42605_CMD_DELAY)
                            {
                                ommo_delay_ms(imu_config[i][1]);
                                continue;
                            }

                            // Write the configuration
                            device_present = comm_dev->write_byte(imu_config[i][0], imu_config[i][1]) == NRF_SUCCESS;
                        }

                        if (device_present)
                        {
                            device_present = device_info_allocate_and_fill_in_default_ic_data(&dgdp.devices[current_device].device_info->ics[ic_num],
                                                   &ic_def_icm42605, 0,dgdp.devices[current_device].device_info->data_sourcing_strat)
                                             == NRF_SUCCESS;
                        }
                        break;
                    }

                    case IC_TYPE_TMP126:
                    {
                        //Check product id
                        uint8_t tx_buf[4];
                        uint8_t rx_buf[2];
                        tx_buf[0] = TMP126_CMD_READ;
                        tx_buf[1] = TMP126_DEVICE_ID_REG;
                        if(device_present) device_present = comm_dev->trx(tx_buf, 2, rx_buf, 2) == NRF_SUCCESS &&
                           ((rx_buf[0] & TMP126_DEVICE_ID_VAL_MASK_MSB) == TMP126_DEVICE_ID_VAL_MSB) &&
                           ((rx_buf[1] & TMP126_DEVICE_ID_VAL_MASK_LSB) == TMP126_DEVICE_ID_VAL_LSB);

                        // Reset sensor (0.5ms)
                        tx_buf[0] = TMP126_CMD_WRITE;
                        tx_buf[1] = TMP126_CONFIG_REG;
                        tx_buf[2] = TMP126_CONFIG_RESET_MSB;
                        tx_buf[3] = TMP126_CONFIG_RESET_LSB;
                        if(device_present) device_present = comm_dev->trx(tx_buf, 4, NULL, 0) == NRF_SUCCESS;
                        ommo_delay_ms(10);

                        // Setup
                        tx_buf[2] = TMP126_CONFIG_VAL_MSB;
                        tx_buf[3] = TMP126_CONFIG_VAL_LSB;
                        if(device_present) device_present = comm_dev->trx(tx_buf, 4, NULL, 0) == NRF_SUCCESS;

                        // Verify
                        tx_buf[0] = TMP126_CMD_READ;
                        tx_buf[1] = TMP126_CONFIG_REG;
                        if(device_present) device_present = comm_dev->trx(tx_buf, 2, rx_buf, 2) == NRF_SUCCESS &&
                                                            rx_buf[0] == TMP126_CONFIG_VAL_MSB &&
                                                            rx_buf[1] == TMP126_CONFIG_VAL_LSB;

                        if(!device_present)
                            break;

                        //Defaults
                        if(device_info_allocate_and_fill_in_default_ic_data(&dgdp.devices[current_device].device_info->ics[ic_num], &ic_def_tmp126, 0, dgdp.devices[current_device].device_info->data_sourcing_strat) != NRF_SUCCESS)
                        {
                            device_present = false;
                            break;
                        }
                        break;
                    }

                    case IC_TYPE_TMP1075N:
                    {
                        //Verify I2C temperature sensor
                        if(sensors_identify_ti_i2c_temp_sensor(comm_dev, TMP1075N_ADDR) != IC_TYPE_TMP1075N)
                        {
                            device_present = false;
                            break;
                        }

                        //Defaults
                        if(device_info_allocate_and_fill_in_default_ic_data(&dgdp.devices[current_device].device_info->ics[ic_num], &ic_def_tmp1075n, 0, dgdp.devices[current_device].device_info->data_sourcing_strat) != NRF_SUCCESS)
                        {
                            device_present = false;
                            break;
                        }
                        break;
                    }

                    case IC_TYPE_TMP110:
                    {
                        //Verify I2C temperature sensor
                        if(sensors_identify_ti_i2c_temp_sensor(comm_dev, TMP110_ADDR) != IC_TYPE_TMP110)
                        {
                            device_present = false;
                            break;
                        }

                        //Defaults
                        if(device_info_allocate_and_fill_in_default_ic_data(&dgdp.devices[current_device].device_info->ics[ic_num], &ic_def_tmp110, 0, dgdp.devices[current_device].device_info->data_sourcing_strat) != NRF_SUCCESS)
                        {
                            device_present = false;
                            break;
                        }
                        break;
                    }

                    case IC_TYPE_BQ27Z558:
                    {
                        if(device_info_allocate_and_fill_in_default_ic_data(&dgdp.devices[current_device].device_info->ics[ic_num], &ic_def_bq27z558, 0, dgdp.devices[current_device].device_info->data_sourcing_strat) != NRF_SUCCESS)
                        {
                            device_present = false;
                            break;
                        }
                        break;
                    }

                    case IC_TYPE_BUTTON:
                     if(device_info_allocate_and_fill_in_default_ic_data(&dgdp.devices[current_device].device_info->ics[ic_num], &ic_def_analog_button, 0, dgdp.devices[current_device].device_info->data_sourcing_strat) != NRF_SUCCESS)
                     {
                        device_present = false;
                     }
                     break;

                    default: //Unknown type
                    {
                        device_present = false;
                        break;
                    }
                }
            }
        }
#else
        if(device_info_allocate_and_fill_in_default_ic_data(&dgdp.devices[current_device].device_info->ics[0], &ic_def_mmc5983, SENSOR_SPI_START_DELAY, dgdp.devices[current_device].device_info->data_sourcing_strat) != NRF_SUCCESS)
        {
            device_present = false;
            continue;
        }
#endif

        //Check for failed initialization, if failed flag sensors for hotplug detected and flag port as invalid
        if(!device_present)
        {
            port_status_list[port] = PORT_STATUS_ERROR;
            for(uint8_t i=0; i<num_ics; i++)
            {
                if(ic_bus_type[i] == IC_BUS_TYPE_SPI)
                {
                    spi_slice_index_t slice_index = port_master_decode_spi_slice_index(ic_ss_index_bus_location[i]);
                    spi_ss_index_t ss_index = port_master_decode_spi_ss_index(ic_ss_index_bus_location[i]);
                    ss_map[port_master_ports[port].spi_buses[slice_index].spi_bus_index][ss_index] = IC_DATA_DISABLED_INDEX;
                }
            }
            continue;
        }

        //Found a good sensor
        port_status_list[port] = PORT_STATUS_CONNECTED;

        //Check for ignore onboard sensors flag
        disable_onboard_sensors |= dgdp.devices[current_device].device_info_user->config_disable_onboard_sensors;

        //Move on to next device
        dgdp.devices_count++;
    }

    //Setup sampling system
    for(size_t current_device =0; current_device < dgdp.devices_count; current_device++)
    {
        uint8_t port = dgdp.devices[current_device].port_num;

        //Disable ics on disabled port ss lines
        for(size_t ic_num = 0; ic_num < dgdp.devices[current_device].device_info->ics_count; ic_num++)
        {
            DeviceInfoICProto *current_ic = &dgdp.devices[current_device].device_info->ics[ic_num];

            ICBusType bus_type = current_ic->ic_bus_type;
            uint8_t port_ss_index_bus_location = current_ic->ic_ss_index_bus_location;

            uint16_t spi_ss_bitmask = 0x01<<port_ss_index_bus_location;
            uint16_t disabled_spi_ss_bitmask = disable_ss_pin_on_port[port] | dgdp.devices[current_device].device_info_user->config_disable_ic;

            // Control flags
            bool disable_by_onboard_cmd = port_master_ports[port].flash_media_present && disable_onboard_sensors;   // disable via onboard sensor disable command
            bool disable_by_ic_cmd = (bus_type == IC_BUS_TYPE_SPI || bus_type == IC_BUS_TYPE_OMMOCOMM_SPI) && (spi_ss_bitmask & disabled_spi_ss_bitmask);   // disable via IC disable command

            // 1st check: is it required to disable something
            if (!disable_by_onboard_cmd && !disable_by_ic_cmd)
                continue;

            // Action 1: Disable device
            bool ic_reset_done = false;
            comm_device_guard comm_dev;
            APP_ERROR_CHECK(comm_dev.acquire(port, bus_type, port_ss_index_bus_location));

            if(current_ic->ic_type == IC_TYPE_MEMSIC_5983)
            {
                comm_dev->write_byte(MMC5983_CONTROL1_REG, MMC5983_CONTROL1_CHIP_RESET);
                ic_reset_done = true;
            }
            else if(disable_by_ic_cmd && current_ic->ic_type == IC_TYPE_ICM_42605)
            {
                comm_dev->write_byte(IMU_42605_REG_DEVICE_CFG, IMU_42605_DEVICE_CFG_RESET);
                ic_reset_done = true;
            }
            //Add shutdown commands for other ic types

            // 2nd check: did IC reset any sensor
            if(!ic_reset_done)
                continue;

            // Action 2: Change device info
            current_ic->ic_data_format = IC_DATA_FORMAT_OFF;
            for(size_t i=0; i< current_ic->sensors_count; i++)
            {
                current_ic->sensors[i].sensor_disabled = true;
            }
        }

        //Setup sampling system
        switch(port_type[port])
        {
            case DEVICE_INFO_FLASH:
            case DEVICE_INFO_I2C:
            case DEVICE_INFO_OMMOCOMM_EEPROM:
            {
                //Sensor verified, fill in ss map for state machine
                for(uint8_t ic_num = 0; ic_num < dgdp.devices[current_device].device_info->ics_count; ic_num++)
                {
                    //Ignore disabled ic's
                    bool disabled_device = (dgdp.devices[current_device].device_info->ics[ic_num].ic_data_format == IC_DATA_FORMAT_OFF);

                    //Keep track of total ic count
                    if(!disabled_device)
                        total_num_ics++;

                    ICBusType bus_type = dgdp.devices[current_device].device_info->ics[ic_num].ic_bus_type;
                    uint8_t ic_data_dictionary_index = find_ic_data_dictionary_index(dgdp.devices[current_device].device_info->ics[ic_num].ic_type);

                    if(bus_type == IC_BUS_TYPE_SPI)
                    {
                        uint8_t bus_location = dgdp.devices[current_device].device_info->ics[ic_num].ic_ss_index_bus_location;
                        spi_slice_index_t port_bus_index = port_master_decode_spi_slice_index(bus_location);
                        spi_ss_index_t port_ss_index = port_master_decode_spi_ss_index(bus_location);
                        spi_bus_index_t spi_bus_index = port_master_ports[port].spi_buses[port_bus_index].spi_bus_index;
                        uint8_t global_ss_index = port_master_ports[port].spi_buses[port_bus_index].ss_index_list[port_ss_index];

                        if(disabled_device)
                            ss_map[spi_bus_index][global_ss_index] = IC_DATA_DISABLED_INDEX;
                        else
                            ss_map[spi_bus_index][global_ss_index] = ic_data_dictionary_index;
                    }
                    else if(bus_type == IC_BUS_TYPE_I2C && !disabled_device)
                    {
                        i2c_ic_dictionary_index[num_i2c_ics] = ic_data_dictionary_index;
                        i2c_ic_port_num[num_i2c_ics] = port;
                        i2c_ic_bus_location[num_i2c_ics] = (ICBusLocation)dgdp.devices[current_device].device_info->ics[ic_num].ic_ss_index_bus_location;
                        i2c_ic_read_command_index[num_i2c_ics] = 0;
                        num_i2c_ics++;
                    }
                }
            }
            break;

#ifdef OMMOCOMM_ENABLED
            case DEVICE_INFO_OMMOCOMM_VIRTUAL:
            {
                uint32_t ommocomm_error = NRF_SUCCESS;
                uint8_t cs_mask_mag = 0;

                //Ommocomm device attached and working
                //ommocomm_uarte_present = true;

                ommocomm_uarte *ommocomm_uarte;
                if(port_master_acquire_ommocomm_direct(port, true, &ommocomm_uarte) == NRF_SUCCESS)
                {
                    //Clear auto sequence and then add each read in sequence
                    if(ommocomm_error == NRF_SUCCESS) ommocomm_error = ommocomm_uarte->automode_seq_clear(0);
                    if(ommocomm_error == NRF_SUCCESS) ommocomm_error = ommocomm_uarte->automode_seq_clear(1);

                    //Find all CS lines with mag
                    for(uint8_t ic_num = 0; ic_num < dgdp.devices[current_device].device_info->ics_count; ic_num++)
                    {
                        uint8_t port_ss_index = dgdp.devices[current_device].device_info->ics[ic_num].ic_ss_index_bus_location;
                        if(dgdp.devices[current_device].device_info->ics[ic_num].ic_data_format != IC_DATA_FORMAT_OFF &&
                           dgdp.devices[current_device].device_info->ics[ic_num].ic_type == IC_TYPE_MEMSIC_5983)
                        {
                            cs_mask_mag |= (0x01<<port_ss_index);
                        }
                    }

                    size_t auto_seq_len = 0;
                    bool any_have_nonzero_rx_len = false;

                    //Add ommocomm MMC5983 start sample to auto sequence
                    if(cs_mask_mag != 0)
                    {
                        auto_mode_spi_trx *command = (auto_mode_spi_trx*)ommocomm_uarte_tx_buf;
                        command->command = OMMOCOMM_AUTO_CMD_SPI_TXRX_TRIGGERED;
                        command->cs_mask = cs_mask_mag;
                        command->rx_len = 0;
                        command->tx_len = 2;
                        command->tx_data[0] = MMC5983_CONTROL0_REG;
                        command->tx_data[1] = MMC5983_CONTROL0_START_CMD;

                        if(ommocomm_error == NRF_SUCCESS)
                            ommocomm_error = ommocomm_uarte->automode_seq_add(0, ommocomm_uarte_tx_buf, command->tx_len + 4);
                        auto_seq_len++;
                    }

                    //Setup auto sample sequence, send to co processor and calculate data len
                    for(size_t ic_num = 0; ic_num < dgdp.devices[current_device].device_info->ics_count; ic_num++)
                    {
                        ICBusType bus_type = dgdp.devices[current_device].device_info->ics[ic_num].ic_bus_type;
                        ic_data_dictionary_entry *ic_data_ptr = (ic_data_dictionary_entry*)&ic_data[find_ic_data_dictionary_index(dgdp.devices[current_device].device_info->ics[ic_num].ic_type)];

                        //Ignore disabled ic's
                        if(dgdp.devices[current_device].device_info->ics[ic_num].ic_data_format == IC_DATA_FORMAT_OFF)
                            continue;

                        switch(bus_type)
                        {
                            case IC_BUS_TYPE_OMMOCOMM_SPI:
                            {
                                //Add ommocomm spi read to auto sequence
                                auto_mode_spi_trx *command = (auto_mode_spi_trx*)ommocomm_uarte_tx_buf;
                                command->command = auto_seq_len == 0 ? OMMOCOMM_AUTO_CMD_SPI_TXRX_TRIGGERED : OMMOCOMM_AUTO_CMD_SPI_TXRX;
                                command->cs_mask = (0x01<<dgdp.devices[current_device].device_info->ics[ic_num].ic_ss_index_bus_location);
                                command->rx_len = ic_data_ptr->read_command[0].data_length; //ommocomm removes SPI bytes from tx automatically
                                command->tx_len = ic_data_ptr->read_command[0].data_read_tx_len;
                                memcpy(command->tx_data, ic_data_ptr->read_command[0].data_read_tx, command->tx_len);

                                if(command->rx_len != 0)
                                    any_have_nonzero_rx_len = true;

                                if(ommocomm_error == NRF_SUCCESS)
                                    ommocomm_error = ommocomm_uarte->automode_seq_add(0, ommocomm_uarte_tx_buf, command->tx_len + 4);
                                auto_seq_len++;
                                break;
                            }

                            case IC_BUS_TYPE_OMMOCOMM_I2C:
                            {
                                //Add ommocomm i2c read to auto sequence
                                auto_mode_i2c_trx *command = (auto_mode_i2c_trx*)ommocomm_uarte_tx_buf;
                                command->command = OMMOCOMM_AUTO_CMD_I2C_TXRX;
                                command->addr = ic_data_ptr->read_command[0].data_read_tx[0];
                                command->rx_len = ic_data_ptr->read_command[0].data_length;
                                command->tx_len = ic_data_ptr->read_command[0].data_read_tx_len - 1; //addr was included in tx data
                                memcpy(command->tx_data, ic_data_ptr->read_command[0].data_read_tx + 1, command->tx_len);

                                if(command->rx_len != 0)
                                    any_have_nonzero_rx_len = true;

                                if(ommocomm_error == NRF_SUCCESS)
                                    ommocomm_error = ommocomm_uarte->automode_seq_add(0, ommocomm_uarte_tx_buf, command->tx_len + 4);
                                auto_seq_len++;
                                break;
                            }

                            case IC_BUS_TYPE_CUSTOM:
                            {
                                auto_mode_analog_button *command = (auto_mode_analog_button*)ommocomm_uarte_tx_buf;
                                for(uint8_t sensor_index = 0; sensor_index < dgdp.devices[current_device].device_info->ics[ic_num].sensors_count; sensor_index++)
                                {
                                    command->command = OMMOCOMM_AUTO_CMD_ANALOG_BUTTON_SAMPLE;
                                    command->pin_index = dgdp.devices[current_device].device_info->ics[ic_num].sensors[sensor_index].config_button_pin_index;
                                    command->num_steps = dgdp.devices[current_device].device_info->ics[ic_num].sensors[sensor_index].config_analog_button_num_steps;
                                    command->sample_delay = dgdp.devices[current_device].device_info->ics[ic_num].sensors[sensor_index].config_analog_button_sample_delay;

                                    any_have_nonzero_rx_len = true;

                                    if(ommocomm_error == NRF_SUCCESS)
                                        ommocomm_error = ommocomm_uarte->automode_seq_add(0, ommocomm_uarte_tx_buf, 4);
                                    auto_seq_len++;
                                }
                                break;
                            }

                            default:
                                OMMO_APP_ERROR_CHECK(NRF_ERROR_NOT_SUPPORTED, STRING("IC BUS TYPE NOT DEFINED"), 0);
                                break;
                        }
                    }

                    //Make sure there is some sort of response
                    ommocomm_save_data[ommocomm_num_present] = true;
                    if(!any_have_nonzero_rx_len)
                    {
                        ommocomm_uarte_tx_buf[0] = OMMOCOMM_AUTO_CMD_WAI;
                        ommocomm_save_data[ommocomm_num_present] = false;

                        if(ommocomm_error == NRF_SUCCESS)
                            ommocomm_error = ommocomm_uarte->automode_seq_add(0, ommocomm_uarte_tx_buf, 1);
                    }

                    //Create set/reset auto command
                    auto_mode_spi_trx *command = (auto_mode_spi_trx*)ommocomm_uarte_tx_buf;
                    command->command = OMMOCOMM_AUTO_CMD_SPI_TXRX_MOSI_TWIDDLE;
                    command->cs_mask = cs_mask_mag;
                    command->rx_len = 0;
                    command->tx_len = 2;
                    command->tx_data[0] = MMC5983_CONTROL0_REG & 0x7F;
                    command->tx_data[1] = MMC5983_CONTROL0_SET_CMD;
                    if(ommocomm_error == NRF_SUCCESS)
                        ommocomm_error = ommocomm_uarte->automode_seq_add(1, ommocomm_uarte_tx_buf, command->tx_len + 4);

                    //Increase ommocomm count
                    if(ommocomm_error == NRF_SUCCESS)
                    {
                        ommocomm_num_present++;
                        if(ommocomm_num_present > OMMO_PM_UARTE_COUNT)
                            ommocomm_error = NRF_ERROR_NO_MEM;
                    }
                }
                ommocomm_uarte->release();

                if(ommocomm_error != NRF_SUCCESS)
                {
                    return sensors_scan_bus_generate_data_descriptor_return_error(packet_id_request_buffer, &dgdp, convert_nrf_error_code_to_ommo_ack_code(ommocomm_error));
                }
            }
            break;
#endif

            default:
            {
                APP_ERROR_CHECK(NRF_ERROR_NOT_SUPPORTED);
            }
            break;
        }
    }

#ifdef OMMOCOMM_ENABLED

    //Set ommocomm flags
    ommocomm_sample_error_count = 0;
#endif

    // Compute list of ss pins that have a mag on at least 1 bus
    // Compute list of ss pins that has an imu on at least 1 bus
    ss_pins_with_mag.clear();
    ss_pins_with_imu.clear();
    for (uint8_t ss_pin = 0; ss_pin < OMMO_PM_SS_PINS_COUNT; ss_pin++)
    {
        // Search for a mag present on this ss line
        for (uint8_t spi_channel = 0; spi_channel < OMMO_PM_SPI_BUSSES_COUNT; spi_channel++)
        {
            if (MAG_PRESENT(spi_channel, ss_pin))
            {
                ss_pins_with_mag.push_back(ss_pin);
                break;
            }
        }

        // Search for any sensor on this ss line
        for (uint8_t spi_channel = 0; spi_channel < OMMO_PM_SPI_BUSSES_COUNT; spi_channel++)
        {
            if (IMU_PRESENT(spi_channel, ss_pin))
            {
                ss_pins_with_imu.push_back(ss_pin);
                break;
            }
        }
    }

    //Compute data output indexes
    uint16_t cur_data_output_index = 0;
    uint8_t cur_i2c_ic = 0;
    uint8_t cur_ommocomm_device = 0;
    for(size_t device = 0; device < dgdp.devices_count; device++)
    {
        uint8_t port = dgdp.devices[device].port_num;

        switch(port_type[port])
        {
            case DEVICE_INFO_FLASH:
            case DEVICE_INFO_I2C:
            case DEVICE_INFO_OMMOCOMM_EEPROM:
            {
                for(size_t ic = 0; ic < dgdp.devices[device].device_info->ics_count; ic++)
                {
                    //Skip disabled IC's
                    if(dgdp.devices[device].device_info->ics[ic].ic_data_format == IC_DATA_FORMAT_OFF)
                        continue;

                    ICBusType bus_type = dgdp.devices[device].device_info->ics[ic].ic_bus_type;
                    if(bus_type == IC_BUS_TYPE_SPI)
                    {
                        bus_location_t bus_location = dgdp.devices[device].device_info->ics[ic].ic_ss_index_bus_location;
                        spi_slice_index_t port_bus_index = port_master_decode_spi_slice_index(bus_location);
                        spi_ss_index_t port_ss_index = port_master_decode_spi_ss_index(bus_location);
                        spi_bus_index_t global_bus_index = port_master_ports[port].spi_buses[port_bus_index].spi_bus_index;
                        uint8_t global_ss_index = port_master_ports[port].spi_buses[port_bus_index].ss_index_list[port_ss_index];

                        dgdp.devices[device].device_info->ics[ic].data_packet_offset = cur_data_output_index;
                        data_output_index[global_bus_index][global_ss_index] = cur_data_output_index;
                        cur_data_output_index += ic_data[ss_map[global_bus_index][global_ss_index]].read_command[0].data_length;

                        // For bipolar mmc5983 we have an additional channel(1 byte) for set/reset info, and that is not from SPI read
                        if(dgdp.devices[device].device_info->ics[ic].ic_data_format == IC_DATA_FORMAT_MMC5983_BIPOLAR)
                        {
                            cur_data_output_index += sizeof(prev_cmd_reset);
                        }
                    }
                    else if(bus_type == IC_BUS_TYPE_I2C)
                    {
                        dgdp.devices[device].device_info->ics[ic].data_packet_offset = cur_data_output_index;
                        i2c_data_output_index[cur_i2c_ic] = cur_data_output_index;
                        for(uint8_t read_command_index = 0; read_command_index < ic_data[i2c_ic_dictionary_index[cur_i2c_ic]].num_read_commands; read_command_index++)
                            cur_data_output_index += ic_data[i2c_ic_dictionary_index[cur_i2c_ic]].read_command[read_command_index].data_length;
                        cur_i2c_ic++;
                    }
                    //TODO analog pins
                }
            }
            break;

#ifdef OMMOCOMM_ENABLED
            case DEVICE_INFO_OMMOCOMM_VIRTUAL:
            {
                //Save data location
                if(ommocomm_save_data[cur_ommocomm_device])
                {
                    ommocomm_data_output_index[cur_ommocomm_device] = cur_data_output_index;
                    ommocomm_data_len[cur_ommocomm_device] = 0x00;
                    for(size_t ic = 0; ic < dgdp.devices[device].device_info->ics_count; ic++)
                    {
                        //Skip disabled IC's
                        if(dgdp.devices[device].device_info->ics[ic].ic_data_format == IC_DATA_FORMAT_OFF)
                            continue;

                        ICBusType bus_type = dgdp.devices[device].device_info->ics[ic].ic_bus_type;
                        ic_data_dictionary_entry *ic_data_ptr = (ic_data_dictionary_entry*)&ic_data[find_ic_data_dictionary_index(dgdp.devices[device].device_info->ics[ic].ic_type)];
                        size_t data_len = 0;
                        switch(bus_type)
                        {
                        case IC_BUS_TYPE_OMMOCOMM_SPI:
                        case IC_BUS_TYPE_OMMOCOMM_I2C:
                            data_len = ic_data_ptr->read_command[0].data_length;
                            break;

                        case IC_BUS_TYPE_CUSTOM:
                            data_len = dgdp.devices[device].device_info->ics[ic].sensors_count;
                            break;

                        default:
                            OMMO_APP_ERROR_CHECK(NRF_ERROR_NOT_SUPPORTED, STRING("IC BUS TYPE NOT DEFINED"), 0);
                            //Unreachable
                            return 0x00;
                        }

                        dgdp.devices[device].device_info->ics[ic].data_packet_offset = cur_data_output_index;
                        cur_data_output_index += data_len;
                        ommocomm_data_len[cur_ommocomm_device] += data_len;
                    }
                }
                else
                {
                    ommocomm_data_len[cur_ommocomm_device] = 1;
                }

                //Save port
                ommocomm_port_num[cur_ommocomm_device] = port;

                //Advance to next ommocomm_device
                cur_ommocomm_device++;
            }
            break;
#endif

            default:
            {
                APP_ERROR_CHECK(NRF_ERROR_NOT_SUPPORTED);
            }
            break;
        }
    }
    data_output_length = cur_data_output_index;
    if(data_output_length > COMMS_ESB_MAX_MULTIPACKET_TOTAL_LENGTH/OMMO_TIMESTAMP_SYNCH_PERIOD_MULT)
        return sensors_scan_bus_generate_data_descriptor_return_error(packet_id_request_buffer, &dgdp, OMMO_ACK_NO_MEM);

    //State machine configuration variables been set at this point
    state_machine_configured.set();

    //Fill in misc data
    dgdp.header_format = data_header_format;
    dgdp.timestamp_ticks_per_second = HFCLK_FREQ;
    dgdp.sample_period = OMMO_TIMESTAMP_SAMPLE_PERIOD;
    dgdp.siu_uuid = NRF_FICR->DEVICEID[0];

    if(dgdp.devices_count == 0)
        dgdp.data_packet_length = 0;
    if(data_header_format == DATA_HEADER_FORMAT_TS)
        dgdp.data_packet_length = data_output_length + 1 + 4;    //pkt_id(1) + ts(4)
    else if(data_header_format == DATA_HEADER_FORMAT_TS_TSO)
        dgdp.data_packet_length = data_output_length +  1 + 4 + 4; //pkt_id(1) + ts(4) + tso(4)
    else
        APP_ERROR_CHECK(NRF_ERROR_NOT_SUPPORTED);

    //Generate packet id request info
    uint16_t index = 0;
    copyUint8(packet_id_request_buffer, index, OMMO_COMMAND_GET_PACKET_DESCRIPTOR); //command
    copyUint8(packet_id_request_buffer, index, DEVICE_GROUP_DESCRIPTOR_FORMAT_MULTI_PROTOBUF); //dataDescriptorFormat

    //Provide final descriptor to caller
    if(data_descriptor_callback)
        data_descriptor_callback(&dgdp, context);

    //Encode protobuf and save size
    pb_ostream_t stream = pb_ostream_from_buffer(packet_id_request_buffer+4, buffer_size-4);
    if(!pb_encode(&stream, &StreamDescriptorProto_msg, &sdp))
        return sensors_scan_bus_generate_data_descriptor_return_error(packet_id_request_buffer, &dgdp, OMMO_ACK_INTERNAL_ERROR);
    copyUint16_LE(packet_id_request_buffer, index, stream.bytes_written);
    index += stream.bytes_written;

    //Free allocated memory
    pb_release(&DeviceGroupDescriptorProto_msg, &dgdp);

#ifndef HOT_PLUG_OFF
    //Start idle hot_plug detection
    sensors_start_idle_hot_plug_check();
#endif

#ifdef OMMO_POWER_REQUIREMENTS
    //Update ommocomm present status in case we found a problem with an ommocomm device
#ifdef OMMOCOMM_ENABLED
    modify_power_status(ommocomm_num_present > 0, POWER_REQ_OMMOCOMM_SAMPLER);
#endif
    check_power_requirements();
#endif

    return index;
}

static uint16_t sensors_scan_bus_generate_data_descriptor_return_error(uint8_t packet_id_request_buffer[], DeviceGroupDescriptorProto *dgdp, OmmoAck ack_code)
{
    //Free allocated memory
    pb_release(&DeviceGroupDescriptorProto_msg, dgdp);

#ifdef OMMO_POWER_REQUIREMENTS
    //Update ommocomm present status in case we found a problem with an ommocomm device
#ifdef OMMOCOMM_ENABLED
    modify_power_status(ommocomm_num_present > 0, POWER_REQ_OMMOCOMM_SAMPLER);
#endif
    check_power_requirements();
#endif

    return fill_in_ack_packet(packet_id_request_buffer, ack_code);
}

bool sensors_hot_plugin_event_check()
{
    return hot_plugin_event_set;
}

bool sensors_hot_unplug_event_check()
{
    return hot_unplug_event_set;
}

uint32_t sensors_add_event_callback(sensors_event_callback_t event_callback)
{
    if(sensors_event_callbacks_num < SENSORS_MAX_EVENTS_CALLBACKS)
    {
        sensors_event_callbacks[sensors_event_callbacks_num++] = event_callback;
        return NRF_SUCCESS;
    }
    return NRF_ERROR_NO_MEM;
}

uint8_t sensors_test(uint8_t port, device_info_storage_type *port_type, ICType *ic_type, uint8_t *ic_ss_index_bus_location, ICBusType *ic_bus_type)
{
    if (port_master_has_twi_bus(port, IC_BUS_LOCATION_I2C_MOSI_CS0))
    {
#if OMMO_PM_SPIM_TWI_BASIC_COUNT || OMMO_PM_TWI_BASIC_COUNT
        //Set I2C speed based on SCL/SDA rise times
        //TODO consider running every time an i2c port on bus_location 0 is acquired
        APP_ERROR_CHECK(port_master_measure_and_calculate_i2c_MOSI_CS0_frequency());
#endif
    }

    *port_type = device_info_scan_port_for_storage_media(port);

    uint8_t num_ics = 0;
    APP_ERROR_CHECK(scan_port_for_devices(port, *port_type == DEVICE_INFO_OMMOCOMM_VIRTUAL, ic_bus_type, ic_ss_index_bus_location, ic_type, &num_ics));
    return num_ics;
}

//use #ifdef to get rid of compile error on standard proj
#ifdef SENSOR_PORT_PIN_TEST
static void sensor_gpio_set_output(const uint8_t pins[], uint8_t num_pins, uint8_t high_low)
{
    for(uint8_t i=0; i<num_pins; i++)
    {
        if(pins[i] != 0xFF)
        {
            //reconfig output pin to strong drive
            port_master_configure_high_speed_output(pins[i]);
            nrf_gpio_pin_write(pins[i], high_low);
        }
    }
}

static void sensor_gpio_set_input(const uint8_t pins[], uint8_t num_pins)
{
    for(uint8_t i=0; i<num_pins; i++)
    {
        if(pins[i] != 0xFF)
        {
            nrf_gpio_cfg_input(pins[i], NRF_GPIO_PIN_NOPULL);
        }
    }
}

//This function set's a port pin to the desired level while keeping all buffer
//off, other than the one on the selected port
static void sensor_port_pins_test_set_port_pin_output(uint8_t *all_pin_set, uint8_t *port_pin_set, uint8_t *cs0_pin_set, uint8_t pin_under_test, bool value)
{
    //Set all pins on all ports to oppisite value of test pin
    sensor_gpio_set_output(all_pin_set, OMMO_SENSOR_TOTAL_PIN_COUNT, !value);

    //Make sure our buffer stays enabled so that the caps are in circuit
    //Also make sure no other buffers are enabled
    switch(pin_under_test)
    {
        case OMMO_SENSOR_PORT_PINS_TEST_MOSI_INDEX:
        default: //Keep our buffer on if we are not using MOSI or CS0 by setting MOSI to be opposite our CS0
            nrf_gpio_pin_write(port_pin_set[OMMO_SENSOR_PORT_PINS_TEST_MOSI_INDEX], value); //MOSI
            for(uint8_t i = 0; i < OMMO_PM_PORTS_COUNT; i++) //Set all _other_ port's CS0 lines to match our MOSI so that their buffer is off
                if(port_pin_set[OMMO_SENSOR_PORT_PINS_TEST_CS0_INDEX] != cs0_pin_set[i] && cs0_pin_set[i] != 0xFF)
                    nrf_gpio_pin_write(cs0_pin_set[i], value);
            break;

        case OMMO_SENSOR_PORT_PINS_TEST_CS0_INDEX:
            nrf_gpio_pin_write(port_pin_set[OMMO_SENSOR_PORT_PINS_TEST_CS0_INDEX], value); //CS0
            for(uint8_t i = 0; i < OMMO_PM_SPI_BUSSES_COUNT; i++) //Set all _other_ port's MOSI lines to match our CS0 so that their buffer is off
                if(port_pin_set[OMMO_SENSOR_PORT_PINS_TEST_MOSI_INDEX] != port_master_spi_busses[i].mosi_pin)
                    nrf_gpio_pin_write(port_master_spi_busses[i].mosi_pin, value);
            break;
    }

    //Set test pin (may or may not have already been done)
    nrf_gpio_pin_write(port_pin_set[pin_under_test], value);
}

void sensor_port_pins_test(uint8_t port_num, sensor_port_pin_status* pin_test_result)
{
    uint8_t index, i, j, result;
    bool buffer_pins_failed = false;

    //delay value for each port pin
    uint32_t charge_time_80percent[OMMO_SENSOR_NUM_PINS_PER_PORT];
    uint32_t charge_time_10percent[OMMO_SENSOR_NUM_PINS_PER_PORT];

    int spi_ss_pull_value[] = OMMO_SENSOR_SPI_SS_PIN_PULL_VALUE;
    int spi_mosi_pull_value[] = OMMO_SENSOR_SPI_MOSI_PIN_PULL_VALUE;
    int spi_miso_pull_value[] = OMMO_SENSOR_SPI_MISO_PIN_PULL_VALUE;
    int spi_sck_pull_value[] = OMMO_SENSOR_SPI_SCK_PIN_PULL_VALUE;

    //Clear return values
    memset(pin_test_result, PIN_STATUS_GOOD, OMMO_SENSOR_NUM_PINS_PER_PORT);

    //Find the set of ALL pins going to ANY port
    uint8_t all_pin_set[OMMO_SENSOR_TOTAL_PIN_COUNT];
    for(i=0, index=0; i<OMMO_PM_SPI_BUSSES_COUNT; i++)
    {
        all_pin_set[index++] = port_master_spi_busses[i].mosi_pin;
        all_pin_set[index++] = port_master_spi_busses[i].miso_pin;
        all_pin_set[index++] = port_master_spi_busses[i].sck_pin;
    }
    for(i=0; i<OMMO_PM_SS_PINS_COUNT; i++)
    {
        all_pin_set[index++] = port_master_ss_pins[i];
    }

    // init pin array with 0xFF(not present)
    uint8_t port_pin_set[OMMO_SENSOR_NUM_PINS_PER_PORT];
    memset(port_pin_set, 0xFF, OMMO_SENSOR_NUM_PINS_PER_PORT);

    int port_pin_pull_value[OMMO_SENSOR_NUM_PINS_PER_PORT];
    for(spi_slice_index_t spi_slice_index = 0; spi_slice_index < port_master_ports[port_num].spi_count; spi_slice_index++)
    {
        uint8_t spi_ch_index = port_master_ports[port_num].spi_buses[spi_slice_index].spi_bus_index;

        //Find the set of pins going to the port
        port_pin_set[OMMO_SENSOR_PORT_PINS_TEST_MOSI_INDEX] = port_master_spi_busses[spi_ch_index].mosi_pin;
        port_pin_set[OMMO_SENSOR_PORT_PINS_TEST_CS0_INDEX] = port_master_ss_pins[port_master_ports[port_num].spi_buses[spi_slice_index].ss_index_list[0]];
        port_pin_set[OMMO_SENSOR_PORT_PINS_TEST_MISO_INDEX] =  port_master_spi_busses[spi_ch_index].miso_pin;
        port_pin_set[OMMO_SENSOR_PORT_PINS_TEST_SCK_INDEX] = port_master_spi_busses[spi_ch_index].sck_pin;
        for(i=1; i<port_master_ports[port_num].spi_buses[spi_slice_index].ss_index_count; i++)
        {
            uint8_t ss_index = port_master_ports[port_num].spi_buses[spi_slice_index].ss_index_list[i];
            port_pin_set[spi_slice_index*4 + 3+i] = port_master_ss_pins[ss_index];
        }
        //pull value: + for pullup, - for pulldown
        port_pin_pull_value[OMMO_SENSOR_PORT_PINS_TEST_MOSI_INDEX] = spi_mosi_pull_value[spi_ch_index];
        port_pin_pull_value[OMMO_SENSOR_PORT_PINS_TEST_CS0_INDEX] = spi_ss_pull_value[port_master_ports[port_num].spi_buses[spi_slice_index].ss_index_list[0]];
        port_pin_pull_value[OMMO_SENSOR_PORT_PINS_TEST_MISO_INDEX] =  spi_miso_pull_value[spi_ch_index];
        port_pin_pull_value[OMMO_SENSOR_PORT_PINS_TEST_SCK_INDEX] = spi_sck_pull_value[spi_ch_index];
        for(i=1; i<port_master_ports[port_num].spi_buses[spi_slice_index].ss_index_count; i++)
        {
            uint8_t ss_index = port_master_ports[port_num].spi_buses[spi_slice_index].ss_index_list[i];
            port_pin_pull_value[spi_slice_index*4 + 3+i] = spi_ss_pull_value[ss_index];
        }
    }

    //Find the set of all port level CS0 pins
    uint8_t cs0_pin_set[OMMO_PM_PORTS_COUNT * OMMO_PM_MAX_SPI_BUSSES_PER_PORT] = {0xFF};
    memset(cs0_pin_set, 0xFF, sizeof(cs0_pin_set));
    for(i=0, index=0; i<NRFX_ARRAY_SIZE(cs0_pin_set); i++)
    {
        for(spi_slice_index_t spi_slice_index = 0; spi_slice_index < port_master_ports[port_num].spi_count; spi_slice_index++)
        {
            uint8_t cs0_pin_index = port_master_ports[i].spi_buses[spi_slice_index].ss_index_list[0];

            //Make sure it's not already in the list
            bool already_present = false;
            for(j = 0; j < index; j++)
            {
                if(port_master_ss_pins[cs0_pin_index] == cs0_pin_set[j])
                {
                    already_present = true;
                    break;
                }
            }

            //Add to list if unique
            if(!already_present)
                cs0_pin_set[index++] = port_master_ss_pins[cs0_pin_index];
        }
    }

    //Step 0, calculate time constant and charge delay of each cap
    for(uint8_t pin_under_test = 0; pin_under_test < OMMO_SENSOR_NUM_PINS_PER_PORT; pin_under_test++)
    {
        //constant time realted var for the cap
        const float gpio_pull_value = 13000.0;
        const float time_constant_c_uf = PORT_TEST_BREAKOUT_CAP_VALUE_UF;
        float time_constant_r;

        if(port_pin_pull_value[pin_under_test] > 0)
        {
            // r_series + parallel pullup ex_r and in_r
            time_constant_r = ((float)port_pin_pull_value[pin_under_test] * gpio_pull_value) / ((float)port_pin_pull_value[pin_under_test] + gpio_pull_value) + PORT_TEST_BREAKOUT_SERIES_R_VALUE;
        }
        else
        {
            time_constant_r = gpio_pull_value + PORT_TEST_BREAKOUT_SERIES_R_VALUE;
        }
        uint32_t time_constant_us = (uint32_t)(time_constant_r * time_constant_c_uf);
        //charge equation: v(t) = Vo(1−e−t/τ)
        charge_time_80percent[pin_under_test] = time_constant_us * 1.609;
        //consider vdd=~3V, we want v(t)=0.1Vo (~1V will turn off xor gate)
        //t = 0.105 * τ
        charge_time_10percent[pin_under_test] = (uint32_t)(0.105 * (float)time_constant_us);
    }

    //Go through each pin in port
    //If PUT has a pull up or has no pull
    //  Set PUT internal pullup
    //  Set all other pins low and read value
    //  Save into result as bit 1
    //  Set all other pins high and read value
    //  Save into result as bit 0
    //  01 shorted neighbor
    //  00 shorted ground
    //  10 invalid
    //  11 normal
    //Else if PUT has a (pulldown or has no pull) and is still NORMAL
    //  Set PUT internal pulldown
    //  Set all other pins low and read value
    //  Save into result as bit 1
    //  Set all other pins high and read value
    //  Save into result as bit 0
    //  00 shorted neighbor
    //  01 normal
    //  11 invalid
    //  10 shorted vdd

    //Step 1
    //  The capacitor bank may be in circuit or out of circuit, it does not matter
    for(uint8_t pin_under_test = 0; pin_under_test < OMMO_SENSOR_NUM_PINS_PER_PORT; pin_under_test++)
    {
        //Make sure pin is used on this port
        if(port_pin_set[pin_under_test] == 0xFF)
            continue;

#ifdef SENSOR_PORT_PIN_TEST_13335_48_PORT
        if(pin_under_test == OMMO_SENSOR_PORT_PINS_TEST_MOSI_INDEX ||
           pin_under_test == OMMO_SENSOR_PORT_PINS_TEST_MISO_INDEX ||
           pin_under_test == OMMO_SENSOR_PORT_PINS_TEST_SCK_INDEX)
        {
            continue;
        }
#endif

        result = 0;

        //Check PUT with pullup for short to ground/neighbor
        if(port_pin_pull_value[pin_under_test] >= 0)
        {
            //Test PUT with all other pins low
            sensor_gpio_set_output(all_pin_set, OMMO_SENSOR_TOTAL_PIN_COUNT, 0);
            nrf_gpio_cfg_input(port_pin_set[pin_under_test], NRF_GPIO_PIN_PULLUP);
            //Wait a LONG time.  Give time for pullup to charge/discharge cap (if in circuit)
            //If this is CS0 or MOSI the bufffer will be turning on and off until the pins are in their final state
            nrf_delay_ms(2);
            result |= (nrf_gpio_pin_read(port_pin_set[pin_under_test]) << 1);

            //Test PUT with all other pins high
            sensor_gpio_set_output(all_pin_set, OMMO_SENSOR_TOTAL_PIN_COUNT, 1);
            nrf_gpio_cfg_input(port_pin_set[pin_under_test], NRF_GPIO_PIN_PULLUP);
            //Wait a LONG time.  Give time for pullup to charge/discharge cap (if in circuit)
            //If this is CS0 or MOSI the bufffer will be turning on and off until the pins are in their final state
            nrf_delay_ms(2);
            result |= nrf_gpio_pin_read(port_pin_set[pin_under_test]);

            //Save bad results
            switch(result)
            {
                case 0b01:
                    pin_test_result[pin_under_test] = PIN_STATUS_SHORTED_NEIGHBOR;
                    break;
                case 0b00:
                    pin_test_result[pin_under_test] = PIN_STATUS_SHORTED_GND;
                    break;
                case 0b10:
                    pin_test_result[pin_under_test] = PIN_STATUS_INVALID;
                    break;
            }
        }

        result = 0;
        //Check PUT with pulldown for short to vdd/neighbor
        if(port_pin_pull_value[pin_under_test] <= 0 && pin_test_result[pin_under_test] == PIN_STATUS_GOOD)
        {
            //Test PUT with all other pins low
            sensor_gpio_set_output(all_pin_set, OMMO_SENSOR_TOTAL_PIN_COUNT, 0);
            nrf_gpio_cfg_input(port_pin_set[pin_under_test], NRF_GPIO_PIN_PULLDOWN);
            //Wait a LONG time.  Give time for pullup to charge/discharge cap (if in circuit)
            //If this is CS0 or MOSI the bufffer will be turning on and off until the pins are in their final state
            nrf_delay_ms(2);
            result |= (nrf_gpio_pin_read(port_pin_set[pin_under_test]) << 1);

            //Test PUT with all other pins high
            sensor_gpio_set_output(all_pin_set, OMMO_SENSOR_TOTAL_PIN_COUNT, 1);
            nrf_gpio_cfg_input(port_pin_set[pin_under_test], NRF_GPIO_PIN_PULLDOWN);
            //Wait a LONG time.  Give time for pullup to charge/discharge cap (if in circuit)
            //If this is CS0 or MOSI the bufffer will be turning on and off until the pins are in their final state
            nrf_delay_ms(2);
            result |= nrf_gpio_pin_read(port_pin_set[pin_under_test]);

            //Save bad results
            switch(result)
            {
                case 0b01:
                    pin_test_result[pin_under_test] = PIN_STATUS_SHORTED_NEIGHBOR;
                    break;
                case 0b11:
                    pin_test_result[pin_under_test] = PIN_STATUS_SHORTED_VDD;
                    break;
                case 0b10:
                    pin_test_result[pin_under_test] = PIN_STATUS_INVALID;
                    break;
            }
        }
    }

    //If buffer pins are not working, then there is no need to continue
    if(pin_test_result[OMMO_SENSOR_PORT_PINS_TEST_MOSI_INDEX] != PIN_STATUS_GOOD ||
       pin_test_result[OMMO_SENSOR_PORT_PINS_TEST_CS0_INDEX] != PIN_STATUS_GOOD)
    {
        sensor_gpio_set_input(all_pin_set, OMMO_SENSOR_TOTAL_PIN_COUNT);
        return;
    }

    //Step 2: Check for open pins
    //MOSI and CS0 need to be check first
    for(uint8_t pin_under_test = 0; pin_under_test < OMMO_SENSOR_NUM_PINS_PER_PORT; pin_under_test++)
    {
        //If CS0 or MOSI pin is bad, buffer is in unknown state, no need to continue
        //CS0 and MOSI test first
        if(pin_test_result[OMMO_SENSOR_PORT_PINS_TEST_MOSI_INDEX] != PIN_STATUS_GOOD ||
           pin_test_result[OMMO_SENSOR_PORT_PINS_TEST_CS0_INDEX] != PIN_STATUS_GOOD)
        {
            break;
        }

        //Make sure pin is used on this port
        if(port_pin_set[pin_under_test] == 0xFF || pin_test_result[pin_under_test] != PIN_STATUS_GOOD)
            continue;

#ifdef SENSOR_PORT_PIN_TEST_13335_48_PORT
        if(pin_under_test == OMMO_SENSOR_PORT_PINS_TEST_MOSI_INDEX ||
           pin_under_test == OMMO_SENSOR_PORT_PINS_TEST_MISO_INDEX ||
           pin_under_test == OMMO_SENSOR_PORT_PINS_TEST_SCK_INDEX)
        {
            continue;
        }
#endif

        result = 0;

        //Set test pin low
        sensor_port_pins_test_set_port_pin_output(all_pin_set, port_pin_set, cs0_pin_set, pin_under_test, 0);
        //Wait a LONG time.  Give time for pullup to charge/discharge cap (if in circuit)
        //If this is CS0 or MOSI the bufffer will be turning on and off until the pins are in their final state
        nrf_delay_ms(2); //Let cap discharge
        //Pull up test pin
        nrf_gpio_cfg_input(port_pin_set[pin_under_test], NRF_GPIO_PIN_PULLUP); //Pull only applies if there isn't an external pullup/pulldown
        nrf_delay_us(charge_time_10percent[pin_under_test]);
        //Check test pin value, should still be low
        //1 if pin is either open or shorted to VDD
        //0 is success
        result = nrf_gpio_pin_read(port_pin_set[pin_under_test]) << 1;

        //Set test pin high
        sensor_port_pins_test_set_port_pin_output(all_pin_set, port_pin_set, cs0_pin_set, pin_under_test, 1);
        //Wait a LONG time.  Give time for pullup to charge/discharge cap (if in circuit)
        //If this is CS0 or MOSI the bufffer will be turning on and off until the pins are in their final state
        nrf_delay_ms(2); //Let cap charge
        //Pull down test pin
        nrf_gpio_cfg_input(port_pin_set[pin_under_test], NRF_GPIO_PIN_PULLDOWN); //Pull only applies if there isn't an external pullup/pulldown
        nrf_delay_us(charge_time_10percent[pin_under_test]);
        //Check test pin value, should still be high
        //0 if pin is either open or shorted to GND
        //1 is success
        result |= nrf_gpio_pin_read(port_pin_set[pin_under_test]);

        //Save bad results
        switch(result)
        {
            case 0b00:
                pin_test_result[pin_under_test] = (port_pin_pull_value[pin_under_test] < 0 ? PIN_STATUS_SHORTED_GND : PIN_STATUS_INVALID);
                break;
            case 0b10:
                pin_test_result[pin_under_test] = PIN_STATUS_OPEN;
                break;
            case 0b11:
                pin_test_result[pin_under_test] = (port_pin_pull_value[pin_under_test] > 0 ? PIN_STATUS_SHORTED_VDD : PIN_STATUS_INVALID);
                break;
        }
    }

    //change pins back to input, strong drive draws current
    sensor_gpio_set_input(all_pin_set, OMMO_SENSOR_TOTAL_PIN_COUNT);
}
#endif

//Returns int16 self test values x, y, z followed by int16 reference vales x, y, z
//static uint16_t sensors_read_self_test_info_physical_port(uint8_t port, uint8_t port_ss_index, uint8_t *output)
//uint16_t sensors_read_self_test_info_physical_port(uint8_t port, uint8_t port_ss_index, uint16_t num_samples, uint8_t *output)
//{
//    uint16_t output_index;
//    uint8_t temp_data[MMC5983_DATA_READ_LENGTH];

//    //Stop the state machine
//    sensors_stop_state_machine();

//    //Verify mag is present
//    uint8_t reg_value;
//    if(sensors_read_byte_blocking(port, port_ss_index, MMC5983_PRODUCT_ID_REG, &reg_value) != NRF_SUCCESS || reg_value != MMC5983_PRODUCT_ID_RESULT)
//        return fill_in_ack_packet(output, OMMO_ACK_NOT_FOUND);

//    //Compose packet
//    output_index = 0;
//    copyUint8(output, output_index, OMMO_COMMAND_MMC_SELF_TEST);
//    copyUint16_LE(output, output_index, num_samples);

//    //Init
//    bool success = sensors_write_byte_blocking(port, port_ss_index, MMC5983_CONTROL0_REG, MMC5983_CONTROL0_INIT) == NRF_SUCCESS;
//    if(success) success = sensors_write_byte_blocking(port, port_ss_index, MMC5983_CONTROL1_REG, MMC5983_CONTROL1_INIT) == NRF_SUCCESS;
//    if(success) success = sensors_write_byte_blocking(port, port_ss_index, MMC5983_CONTROL0_REG, MMC5983_CONTROL0_SET_CMD) == NRF_SUCCESS;
//    if(success) success = sensors_write_byte_blocking(port, port_ss_index, MMC5983_CONTROL1_REG, MMC5983_CONTROL1_INIT) == NRF_SUCCESS;

//    if(!success)
//        return fill_in_ack_packet(output, OMMO_ACK_NOT_FOUND);

//    int field_setting = 0;
//    for(uint16_t cycle = 0; cycle < (num_samples + 2); cycle++)
//    {
//        if((cycle&0x01) == 0)
//            sensors_write_byte_blocking(port, port_ss_index, MMC5983_CONTROL3_REG, MMC5983_CONTROL3_ST_ENP);
//        else
//            sensors_write_byte_blocking(port, port_ss_index, MMC5983_CONTROL3_REG, MMC5983_CONTROL3_ST_ENM);

//        //Take a sample
//        sensors_write_byte_blocking(port, port_ss_index, MMC5983_CONTROL0_REG, MMC5983_CONTROL0_START_CMD);

//        //check if the data is ready to be read
//        while(sensors_read_byte_blocking(port, port_ss_index, MMC5983_STATUS_REG, &reg_value) == NRF_SUCCESS && (reg_value & 0x01) != 0x01);

//        //Read data
//        uint8_t reg = 0x80 | MMC5983_XOUT_REG;
//        sensors_spi_trx_blocking(port, port_ss_index, &reg, 1, temp_data, MMC5983_DATA_LENGTH);

//        //Save data
//        if(cycle >= 2)
//        {
//            memcpy(output + output_index, temp_data + MMC5983_DATA_OFFSET, MMC5983_DATA_LENGTH);
//            output_index += MMC5983_DATA_LENGTH;
//        }
//    }

//    return output_index;
//}

void sensors_disable_outputs()
{
    for(int32_t i=0; i<OMMO_PM_SPI_BUSSES_COUNT; i++)
    {
        nrf_gpio_cfg(port_master_spi_busses[i].mosi_pin, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_H0D1, NRF_GPIO_PIN_NOSENSE);
        nrf_gpio_cfg(port_master_spi_busses[i].sck_pin, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_H0D1, NRF_GPIO_PIN_NOSENSE);
    }

    for(int32_t i=0; i<OMMO_PM_SS_PINS_COUNT; i++)
    {
        nrf_gpio_cfg(port_master_ss_pins[i], NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_H0D1, NRF_GPIO_PIN_NOSENSE);  //Partial CS activation can cause AKM to hang
    }
}

void sensors_enable_outputs()
{
    for(int32_t i=0; i<OMMO_PM_SPI_BUSSES_COUNT; i++)
    {
        sensors_configure_spi_pin(port_master_spi_busses[i].mosi_pin);
        sensors_configure_spi_pin(port_master_spi_busses[i].sck_pin);
    }

    for(int32_t i=0; i<OMMO_PM_SS_PINS_COUNT; i++)
    {
        sensors_configure_spi_pin(port_master_ss_pins[i]);
    }
}

ret_code_t sensors_add_start_transfer_task(const uint32_t task_addr)
{
    if (start_transfer_tasks_free.empty())
    {
        nrf_ppi_channel_t ppi_channel;
        if (nrfx_ppi_channel_alloc(&ppi_channel) == NRF_SUCCESS)
        {
            // Include the channel in the PPI group.
            APP_ERROR_CHECK(nrfx_ppi_channel_include_in_group(ppi_channel, sampling_ppi_start_group));
            APP_ERROR_CHECK_BOOL(ppi_channel_start_transfer.push_back(ppi_channel));

            // Assign the PPI channel to the sample event and the task address.
            APP_ERROR_CHECK(nrfx_ppi_channel_assign(ppi_channel, timestamp_get_sample_event_address(), task_addr));
            APP_ERROR_CHECK_BOOL(start_transfer_tasks_assigned.push_back(&NRF_PPI->CH[ppi_channel].TEP));

            // Clear the fork to ensure it is not used.
            APP_ERROR_CHECK(nrfx_ppi_channel_fork_assign(ppi_channel, 0));
            APP_ERROR_CHECK_BOOL(start_transfer_tasks_free.push_back(&NRF_PPI->FORK[ppi_channel].TEP));
            return NRF_SUCCESS;
        }
    }
    else
    {
        // Get a free task address location from the free list.
        volatile uint32_t *task_addr_location = nullptr;
        //APP_ERROR_CHECK_BOOL(start_transfer_tasks_free.pop_back(task_addr_location)); //Modified to please gcc analzyer
        VERIFY_TRUE(start_transfer_tasks_free.pop_back(task_addr_location) && task_addr_location != nullptr, NRF_ERROR_INTERNAL);
        *task_addr_location = task_addr;

        // Add the task address to the assigned list.
        APP_ERROR_CHECK_BOOL(start_transfer_tasks_assigned.push_back(task_addr_location));
        return NRF_SUCCESS;
    }

    return NRF_ERROR_INTERNAL;
}

ret_code_t sensors_clean_start_transfer_tasks()
{
    const ret_code_t result = nrfx_ppi_group_disable(sampling_ppi_start_group);
    if (result == NRF_SUCCESS)
    {
        for (auto &task_addr_location : start_transfer_tasks_assigned)
        {
            // Reset the task address location to 0.
            *task_addr_location = 0;

            APP_ERROR_CHECK_BOOL(start_transfer_tasks_free.push_back(task_addr_location));
        }
        start_transfer_tasks_assigned.clear();
    }

    return result;
}

void sensors_init(sensors_callback_func_type sample_set_ready_callback_function)
{
#ifdef OMMO_IMU_IRQ_PIN
    // Initialize GPIOTE module
    if (!nrfx_gpiote_is_init())
    {
       APP_ERROR_CHECK(nrfx_gpiote_init());
    }

    // Configure the IRQ pin as an input with sense for rising edge
    nrfx_gpiote_in_config_t in_config = NRFX_GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);

    // Initialize and enable the IRQ pin
    APP_ERROR_CHECK(nrfx_gpiote_in_init(OMMO_IMU_IRQ_PIN, &in_config, imu_irq_pin_handler));
#endif

    //Save callback
    sample_set_ready_callback = sample_set_ready_callback_function;

#ifdef OMMO_TIMESTAMP_SYNCH_IN
    //Sample timestamp callback
    timestamp_add_sample_event_callback(sample_timer_event_handler);
#endif

    //Init variables
    current_output_data = output_data_1;
    next_output_data = output_data_2;
    next_next_output_data = output_data_3;
    memset(output_data_1, 0x55, sizeof(output_data_1));
    memset(output_data_2, 0x55, sizeof(output_data_2));
    memset(output_data_3, 0x55, sizeof(output_data_3));

    current_state = STATE_MACHINE_STOPPED;
    timing_check_failed_count = 0;
    total_num_ics = 0;
#ifdef SENSORS_MAG_CALIBRATION_ON
    mag_calibration_mode = true;
#else
    mag_calibration_mode = false;
#endif
    onboard_sensors_enabled = true;

    i2c_current_state_index = 0;
    hot_plug_current_state_index = 0;
    hot_unplug_double_check = false;
    sensors_event_callbacks_num = 0;
    execute_sample_ready_callback = false;

    memset(ss_map, 0, sizeof(ss_map));
    ss_pins_with_mag.clear();
    ss_pins_with_imu.clear();

    last_timestamp = current_timestamp = 0;
    last_timestamp_offset = current_timestamp_offset = 0;

    current_output_data_timestamp = 0;
    current_output_data_timestamp_offset = 0;

    OMMO_APP_ERROR_CHECK(ommo_fifo_init(&sensor_event_fifo, (uint32_t*)sensor_event_buffer, OMMO_SENSOR_EVENT_FIFO_LEN), STRING("Sensor event FIFO initialization failure"), 0);

    //Allocate PPI channel groups
    APP_ERROR_CHECK(nrfx_ppi_group_alloc(&sampling_ppi_start_group));

    //Init event for triggering mag set (needs to be after mag sample is complete)
    sensor_set_sample_time_offset_event = timestamp_create_sample_event_offset_event(OMMO_SENSOR_MAG_SET_TIME);

    //Init TWI instance for reading device info
    sensors_twi_basic_config.frequency = NRF_TWIM_FREQ_100K;
    sensors_twi_basic_config.scl_push_pull = true;

#ifndef HOT_PLUG_OFF
    //Init hot_plug idle timer
    APP_ERROR_CHECK(app_timer_create(&hot_plug_idle_timer_id, APP_TIMER_MODE_SINGLE_SHOT, hot_plug_idle_timer_handler));
#endif

    //Counter for set/reset and making temperature measurements
    i2c_interval_count = 0; //OMMO_SENSOR_I2C_TRX_INTERVAL;
    prev_cmd_reset = false; //for MEMSIC_BIPOLAR_EXCITATION: Apply reset current first

#ifdef OMMO_BQ27427_FUEL_GAUGE
    DeviceInfoProto *device_info;
    if(device_info_allocate_and_read_from_port(OMMO_BQ27427_FUEL_GAUGE_PORT, (void**)&device_info, DEVICE_INFO_FIELD_PERM) == NRF_SUCCESS)
    {
        uint8_t ic = 0xFF;
        for(uint8_t i=0; i<device_info->ics_count; i++)
        {
            if(device_info->ics[i].ic_type == IC_TYPE_BQ27427 &&
               device_info->ics[i].ic_calibration != NULL &&
               device_info->ics[i].ic_calibration->size > 0)
            {
                ic = i;
                break;
            }
        }
        if(ic != 0xFF)
        {
            //Make sure we can read gauge
            uint16_t devicetype = sensors_bq27427_get_devicetype(OMMO_BQ27427_FUEL_GAUGE_PORT, IC_BUS_LOCATION_FIXED_0);
            if(devicetype != BQ27427_DEVICE_TYPE_RESP)
                APP_ERROR_CHECK(NRF_ERROR_NOT_FOUND);

            //See if device is sealed
            if(!sensors_bq27427_is_sealed(OMMO_BQ27427_FUEL_GAUGE_PORT, IC_BUS_LOCATION_FIXED_0))
            {
                push_led_state(APP_STATE_FUEL_GAUGE_DFU);

                APP_ERROR_CHECK(execute_flashstream_command_sequence(OMMO_BQ27427_FUEL_GAUGE_PORT, IC_BUS_TYPE_I2C,
                                                                     IC_BUS_LOCATION_FIXED_0,
                                                                     device_info->ics[ic].ic_calibration->bytes,
                                                                     device_info->ics[ic].ic_calibration->size,
                                                                     70));

                pop_led_state();
            }

            fuel_gauge_avail = true;
        }

        //Release allocated memory
        pb_release(&DeviceInfoProto_msg, device_info);
    }
#endif

#ifdef OMMO_DEBUG_SAMPLE_PIN
    APP_ERROR_CHECK(port_master_acquire_pins(PORT_PIN_TO_MASK(OMMO_DEBUG_SAMPLE_PIN)));
    nrfx_gpiote_out_config_t debug_sample_config = NRFX_GPIOTE_CONFIG_OUT_TASK_TOGGLE(false);
    APP_ERROR_CHECK(nrfx_gpiote_out_init(OMMO_DEBUG_SAMPLE_PIN, &debug_sample_config));
    nrfx_gpiote_out_task_enable(OMMO_DEBUG_SAMPLE_PIN);
#endif

#ifdef OMMO_DEBUG_SET_RESET_PIN
    APP_ERROR_CHECK(port_master_acquire_pins(PORT_PIN_TO_MASK(OMMO_DEBUG_SET_RESET_PIN)));
    nrf_gpio_cfg_output(OMMO_DEBUG_SET_RESET_PIN);
#endif
}

bool sensors_event_queue_sample_ready_pending()
{
    return execute_sample_ready_callback;
}

void sensors_event_queue_sample_ready_process()
{
    // Call callback
    (*sample_set_ready_callback)(current_output_data, data_output_length, current_output_data_timestamp, current_output_data_timestamp_offset);

    // Clear pending flag after capturing state so that state and buffers are not modified by ISRs
    execute_sample_ready_callback = false;
}

bool sensors_event_queue_event_callback_pending()
{
    return ommo_fifo_length(&sensor_event_fifo) > 0;
}

void sensors_event_queue_event_callback_process()
{
    uint32_t sensor_event;
    ommo_fifo_get(&sensor_event_fifo, &sensor_event);

    for (uint8_t i = 0; i < sensors_event_callbacks_num; i++)
        (*sensors_event_callbacks[i])((sensors_event_t)sensor_event);
}

ret_code_t sensors_add_tasks_to_event_queue(event_queue_manager *event_queue)
{
    // main_loop_only: writing a new packet mid-flush re-sets tx_buffer_transport_flags, preventing flush_tx_buffers from exiting
    VERIFY_SUCCESS(event_queue->register_task(sensors_event_queue_sample_ready_pending,
                                              sensors_event_queue_sample_ready_process,
                                              nullptr, nullptr, true));
    VERIFY_SUCCESS(event_queue->register_task(sensors_event_queue_event_callback_pending, sensors_event_queue_event_callback_process));
    return NRF_SUCCESS;
}

#ifndef HOT_PLUG_OFF
void sensors_start_idle_hot_plug_check()
{
    //Acquire resources
    sensors_state_machine_acquire_resources();

    //Setup first read
    sensors_continue_idle_hot_plug_check();
}

static void sensors_continue_idle_hot_plug_check()
{
    // Make sure the SPI ISR doesn't go off before state is set
    CRITICAL_REGION_ENTER();

    // Advance ss state
    if (!hot_unplug_double_check)
        hot_plug_current_state_index = (hot_plug_current_state_index + 1) % OMMO_PM_SS_PINS_COUNT;

    // Set pins
    port_master_switch_ss(hot_plug_current_state_index);
    for (uint8_t spi_channel = 0; spi_channel < OMMO_PM_SPI_BUSSES_COUNT; spi_channel++)
    {
        sensors_read_mag_id(spi_channel);
    }

    // Switch states
    current_state = IDLE_HOT_PLUG_CHECK;

    CRITICAL_REGION_EXIT();

    // Start 100ms timer
    APP_ERROR_CHECK(app_timer_start(hot_plug_idle_timer_id, APP_TIMER_TICKS(100), NULL));
}
#endif

#ifdef OMMOCOMM_ENABLED

static void stm32_dfu_progress(fw_prog_stage_t stage, size_t progress)
{
    switch(stage)
    {
        case PROG_STAGE_PREPARE:
            // Update leds to DFU state
            OMMO_APP_ERROR_CHECK(ommo_fifo_put(&sensor_event_fifo, (uint32_t)SENSORS_EVENT_DFU_STARTED), STRING("Sensor event queue full"), 0);
            sensors_event_queue_event_callback_process();
            break;
    
        case PROG_STAGE_REBOOT:
            // Update leds to DFU state
            OMMO_APP_ERROR_CHECK(ommo_fifo_put(&sensor_event_fifo, (uint32_t)SENSORS_EVENT_DFU_FINISHED), STRING("Sensor event queue full"), 0);
            sensors_event_queue_event_callback_process();
            break;
    }
}

void sensors_ommocomm_check_and_update_all_ports()
{
    // Check all off the ports for Ommocomm device
    for (uint8_t port = 0; port < OMMO_PM_PORTS_COUNT; ++port)
    {
        ommocomm_uarte * ommocomm = nullptr;
        if (port_master_acquire_ommocomm_direct(port, true, &ommocomm) == NRF_SUCCESS)
        {
            bool success = (ommocomm->check_fw_and_update(stm32_dfu_progress, advanced_wdt_feed) == NRF_SUCCESS);

#ifdef OMMO_POWER_REQUIREMENTS
            if(success)
            {
                //Found an ommocomm device
                modify_power_status(true, POWER_REQ_OMMOCOMM_SAMPLER);
            }
#endif
            //Release uarte
            ommocomm->release();
        }
    }
}
#endif
