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

#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrfx_spim.h"
#include "nrfx_timer.h"
#include "nrfx_ppi.h"
#include "nrfx_gpiote.h"
#include "nrfx_timer.h"
#include "nrf_saadc.h"
#include "nrfx_comp.h"

#include "device_info_parser.h"
#include "main.hpp"
#include "ommo_config.h"
#include "port_master.hpp"
#include "comm_device.hpp"
#include "timestamp_timer.hpp"
#include "ommo_config.h"
#include "ADS127L11_mux_sampler.hpp"
#include "ic_constants.h"
#include "ommo_app_error.h"
#include "event_queue_manager.hpp"

typedef enum
{
    ADC_SAMPLING_STATE_IDLE,
    ADC_SAMPLING_STATE_X_CHANNEL_PENDING,
    ADC_SAMPLING_STATE_X_CHANNEL_COMPLETE,
    ADC_SAMPLING_STATE_Y_CHANNEL_PENDING,
    ADC_SAMPLING_STATE_Y_CHANNEL_COMPLETE,
    ADC_SAMPLING_STATE_Z_CHANNEL,
    ADC_SAMPLING_STATE_ERROR,
} adc_sampling_state_t;

#define SPI_BUFFER_SIZE 32

#define ADC_MUX_SWITCH_TIME_US(x)   ((x) * ADS127L11_CHANNEL_PERIOD_US)
#define ADC_START_SAMPLE_TIME_US(x) (ADS127L11_MUX_SETTLING_TIME_US + ADC_MUX_SWITCH_TIME_US(x))

#define ADC_SET_RESET_START_TIME_US  ADC_MUX_SWITCH_TIME_US(3) //Right after Z is finished
#define ADC_SET_RESET_DURATION_US    1

#define ADC_MUX_SWITCH_CC_CHANNEL      NRF_TIMER_CC_CHANNEL0
#define ADC_START_SAMPLE_CC_CHANNEL    NRF_TIMER_CC_CHANNEL1
#define ADC_START_SAMPLE_CC_EVENT      NRF_TIMER_EVENT_COMPARE1
#define ADC_SET_RESET_START_CC_CHANNEL NRF_TIMER_CC_CHANNEL2
#define ADC_SET_RESET_END_CC_CHANNEL   NRF_TIMER_CC_CHANNEL3
#define ADC_SET_RESET_END_CC_SHORT_MASK (nrf_timer_short_mask_t)(NRF_TIMER_SHORT_COMPARE3_STOP_MASK | NRF_TIMER_SHORT_COMPARE3_CLEAR_MASK)

static const nrf_saadc_channel_config_t adc_config_vbias = 
{
    .resistor_p = NRF_SAADC_RESISTOR_DISABLED,
    .resistor_n = NRF_SAADC_RESISTOR_DISABLED,
    .gain       = ADC_SAADC_RBRIDGE_VBIAS_ADC_GAIN_REG,
    .reference  = NRF_SAADC_REFERENCE_INTERNAL,
    .acq_time   = NRF_SAADC_ACQTIME_5US,
    .mode       = ADC_SAADC_RBRIDGE_VBIAS_ADC_MODE,
    .burst      = NRF_SAADC_BURST_ENABLED,
    .pin_p      = (nrf_saadc_input_t)ADC_SAADC_RBRIDGE_VBIAS_ADC_PIN_P,
    .pin_n      = (nrf_saadc_input_t)ADC_SAADC_RBRIDGE_VBIAS_ADC_PIN_N,
};

static const nrf_saadc_channel_config_t adc_config_vsense = 
{
    .resistor_p = NRF_SAADC_RESISTOR_DISABLED,
    .resistor_n = NRF_SAADC_RESISTOR_DISABLED,
    .gain       = ADC_SAADC_RBRIDGE_RSENSE_ADC_GAIN_REG,
    .reference  = NRF_SAADC_REFERENCE_INTERNAL,
    .acq_time   = NRF_SAADC_ACQTIME_5US,
    .mode       = ADC_SAADC_RBRIDGE_RSENSE_ADC_MODE,
    .burst      = NRF_SAADC_BURST_ENABLED,
    .pin_p      = (nrf_saadc_input_t)ADC_SAADC_RBRIDGE_RSENSE_ADC_PIN_P,
    .pin_n      = ADC_SAADC_RBRIDGE_RSENSE_ADC_PIN_N,
};

//ADC buffers
nrf_saadc_value_t adc_vbias_buffer[3];
nrf_saadc_value_t adc_vsense_buffer[3];
float vbias_x, vbias_y, vbias_z, vbias;
float vsense_x, vsense_y, vsense_z, vsense;
float rbridge;

// Hardware instances
static nrf_ppi_channel_group_t adc_sampling_ppi_group;
static nrf_ppi_channel_t       adc_ready_spi_read_ppi;
static nrf_ppi_channel_t       adc_start_ppi;
static nrf_ppi_channel_t       mux_select_ppi;
static nrf_ppi_channel_t       channel_bridge_select_ppi;
#if OMMO_ADC_SET_RESET_ENABLED
static nrf_ppi_channel_t       set_reset_end_ppi;
static nrf_ppi_channel_t       set_reset_start_ppi;
#endif
#ifndef OMMO_MAG_IDLE_BRIDGE_ON
static nrf_ppi_channel_t       channel_bridge_off_ppi;
#endif
static comm_device             *comm_device_ptr;
nrfx_timer_t adc_sampling_timer = OMMO_ADC_SAMPLING_TIMER;
uint64_t adc_sensors_intermittent_pin_gpiote = 0x00;

//Callback function
adc_sensors_callback_func_type adc_sample_set_ready_callback;

#ifdef ADS127L11_INCLUDE_DEBUG_DATA
uint16_t adc_sample_data_size = ADS127L11_DATA_LENGTH + ADS127L11_DEBUG_DATA_LENGTH;
int32_t adc_data_buffer[ADS127L11_NUM_CHANNELS + ADS127L11_NUM_DEBUG_CHANNELS];
#else
uint16_t adc_sample_data_size = ADS127L11_DATA_LENGTH;
int32_t adc_data_buffer[ADS127L11_NUM_CHANNELS];
#endif

//Measured data variables
uint8_t spi_rx_buffer[SPI_BUFFER_SIZE] __ALIGNED(4);
uint8_t spi_tx_buffer[SPI_BUFFER_SIZE] __ALIGNED(4);
bool overvoltage_triggered;

uint32_t curent_sample_timestamp;
uint32_t current_sample_timestamp_offest;

volatile bool stop_state_machine_bit = false;

#ifdef MAG_CALIBRATION_ON
bool mag_calibration_mode = true;
#else
bool mag_calibration_mode = false;
#endif

// State machine
static bool state_machine_configured = false;
static volatile adc_sampling_state_t adc_sampling_state = ADC_SAMPLING_STATE_IDLE;
static volatile bool rbridge_vbias_measure_state = false;
static volatile int32_t rbridge_init_value = 0;
static volatile uint8_t sampler_dry_run = 0;
static uint8_t sampler_dry_run_total = 0;
static uint32_t adc_state_errors = 0;

// Private functions
static void adc_sensors_start_data_read_hold();
static void adc_sensors_start_state_machine();
static void adc_sensors_restart_state_machine();
static bool check_stop_statemachine_bit();
static void adc_sensors_intermittent_gpiote_pin_init(uint32_t pin);
static void adc_sensors_intermittent_gpiote_pin_enable_gpiote(uint32_t pin);
static void adc_sensors_intermittent_gpiote_pin_disable_gpiote(uint32_t pin);
static void adc_sensors_intermittent_gpiote_pin_set(uint32_t pin);
static void adc_sensors_intermittent_gpiote_pin_clear(uint32_t pin);
static void adc_sensors_dfu_ommocomm_eeprom();

/**
 * @brief Read 24-bit signed integer from buffer at given offset in big-endian format.
 *
 * @note It will work on little-endian architecture only.
 *
 * @param buf Pointer to the buffer containing the data.
 * @param offset Offset in the buffer where the data starts.
 *
 * @return The 24-bit signed integer value read from the buffer.
 */
__STATIC_FORCEINLINE int32_t read_spi_buf_i24(const uint8_t *buf, size_t offset)
{
    uint32_t  word = __builtin_bswap32(*(uint32_t *)(buf + offset));
    return ((int32_t)word) >> 8;
}

/**
 * @brief Function to handle the sample timer event from the system timer.
 */
static void sample_timer_event_handler(nrf_timer_event_t event_type, void* p_context)
{
    UNUSED_PARAMETER(p_context);
    UNUSED_PARAMETER(event_type);

    //Save the timestamp/encoder for the next packet (after packet in process)
    timestamp_get_last_sample_event_timestamp_basestation_units(curent_sample_timestamp, current_sample_timestamp_offest);

    //Re enable encoder sampling
    timestamp_reenable_disabled_captures();
}

void sampling_timer_event_handler(nrf_timer_event_t event_type, void* p_context)
{
    UNUSED_PARAMETER(p_context);
#ifdef OMMO_DEBUG_ADC_PRI0_ISR_PIN
    nrf_gpio_pin_set(OMMO_DEBUG_ADC_PRI0_ISR_PIN);
#endif

    //Called just after ADC start sample happens while ADC is sampling
    if(event_type == ADC_START_SAMPLE_CC_EVENT)
    {
        // Actions in every case is preparations for next state, X is prepared at beginning.
        // For the rbridge selection the task is used, fork is used for cleaning only.
        switch(adc_sampling_state)
        {
            case ADC_SAMPLING_STATE_X_CHANNEL_PENDING:
            {
                // Set new sample time (Y channel)
                nrfx_timer_compare(&adc_sampling_timer, ADC_MUX_SWITCH_CC_CHANNEL, US_TO_TICKS_16MHZ(ADC_MUX_SWITCH_TIME_US(1)), false);
                nrfx_timer_compare(&adc_sampling_timer, ADC_START_SAMPLE_CC_CHANNEL, US_TO_TICKS_16MHZ(ADC_START_SAMPLE_TIME_US(1)), true);

#ifndef OMMO_ADC_IOXYZ_MUX_CTRL
                // Configure MUX select 0b00 (0b01 -> 0b00) for Y channel
                nrf_ppi_task_endpoint_setup(mux_select_ppi, nrfx_gpiote_clr_task_addr_get(OMMO_ADC_MUX_A1_PIN));
                nrf_ppi_fork_endpoint_setup(mux_select_ppi, nrfx_gpiote_clr_task_addr_get(OMMO_ADC_MUX_A0_PIN));
#endif

                // Configure bridge select on for Y channel and bridge select off for X channel
                adc_sensors_intermittent_gpiote_pin_disable_gpiote(OMMO_IO_Z_EN);
                adc_sensors_intermittent_gpiote_pin_enable_gpiote(OMMO_IO_Y_EN);
                nrf_ppi_task_endpoint_setup(channel_bridge_select_ppi, nrfx_gpiote_set_task_addr_get(OMMO_IO_Y_EN));
                nrf_ppi_fork_endpoint_setup(channel_bridge_select_ppi, nrfx_gpiote_clr_task_addr_get(OMMO_IO_X_EN));
                adc_sampling_state = ADC_SAMPLING_STATE_X_CHANNEL_COMPLETE;
                break;
            }

            case ADC_SAMPLING_STATE_Y_CHANNEL_PENDING:
            {
                // Set new sample time (Z channel)
                nrfx_timer_compare(&adc_sampling_timer, ADC_MUX_SWITCH_CC_CHANNEL, US_TO_TICKS_16MHZ(ADC_MUX_SWITCH_TIME_US(2)), false);
                nrfx_timer_compare(&adc_sampling_timer, ADC_START_SAMPLE_CC_CHANNEL, US_TO_TICKS_16MHZ(ADC_START_SAMPLE_TIME_US(2)), false);

#ifndef OMMO_ADC_IOXYZ_MUX_CTRL
                // Configure MUX select 0b10 (0b00 -> 0b10) for Z channel
                nrf_ppi_task_endpoint_setup(mux_select_ppi, nrfx_gpiote_set_task_addr_get(OMMO_ADC_MUX_A1_PIN));
                nrf_ppi_fork_endpoint_setup(mux_select_ppi, nrfx_gpiote_clr_task_addr_get(OMMO_ADC_MUX_A0_PIN));
#endif

                // Configure bridge select on for Z channel and bridge select off for Y channel
                adc_sensors_intermittent_gpiote_pin_disable_gpiote(OMMO_IO_X_EN);
                adc_sensors_intermittent_gpiote_pin_enable_gpiote(OMMO_IO_Z_EN);
                nrf_ppi_task_endpoint_setup(channel_bridge_select_ppi, nrfx_gpiote_set_task_addr_get(OMMO_IO_Z_EN));
                nrf_ppi_fork_endpoint_setup(channel_bridge_select_ppi, nrfx_gpiote_clr_task_addr_get(OMMO_IO_Y_EN));
#ifndef OMMO_MAG_IDLE_BRIDGE_ON
                // Note that DRDY -> clear Y fork may not have occurred yet.
                nrf_ppi_task_endpoint_setup(channel_bridge_off_ppi,    nrfx_gpiote_clr_task_addr_get(OMMO_IO_Z_EN));
#endif
                adc_sampling_state = ADC_SAMPLING_STATE_Y_CHANNEL_COMPLETE;
                break;
            }

            case ADC_SAMPLING_STATE_X_CHANNEL_COMPLETE:
            case ADC_SAMPLING_STATE_Y_CHANNEL_COMPLETE:
                // SPI RXEND ISR runs at a lower priority and did not complete in time
                adc_sampling_state = ADC_SAMPLING_STATE_ERROR;
                // Prevent any further interrupts until the state machine resets
                nrfx_timer_compare_int_disable(&adc_sampling_timer, ADC_START_SAMPLE_CC_CHANNEL);
                break;

            case ADC_SAMPLING_STATE_IDLE:
                break;

            default:
                OMMO_APP_ERROR_CHECK(NRF_ERROR_INTERNAL, L(adc_sampling_state));
        }
    }

#ifdef OMMO_DEBUG_ADC_PRI0_ISR_PIN
    nrf_gpio_pin_clear(OMMO_DEBUG_ADC_PRI0_ISR_PIN);
#endif
}

/**
 * @brief SPI user event handler.
 */
void comm_device_event_handler(comm_device * comm_dev, comm_device_trx_result_t result, void * p_context)
{
    UNUSED_PARAMETER(comm_dev);
    UNUSED_PARAMETER(p_context);
#ifdef OMMO_DEBUG_ADC_PRI1_ISR_PIN
    nrf_gpio_pin_set(OMMO_DEBUG_IO);
#endif

    if (result == COMM_DEVICE_TRX_RESULT_SUCCESS)
    {
        bool state_machine_error = false;
        switch(adc_sampling_state)
        {
            adc_sampling_state_t check_state;

            case ADC_SAMPLING_STATE_X_CHANNEL_COMPLETE:
            {
                // Setup for Y data read
                adc_data_buffer[0] = read_spi_buf_i24(spi_rx_buffer, 0);
                check_state = ADC_SAMPLING_STATE_X_CHANNEL_COMPLETE;
                if (__atomic_compare_exchange_n(&adc_sampling_state, &check_state, ADC_SAMPLING_STATE_Y_CHANNEL_PENDING, false, __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST))
                    adc_sensors_start_data_read_hold();
                else
                    state_machine_error = true;
                break;
            }

            case ADC_SAMPLING_STATE_Y_CHANNEL_COMPLETE:
            {
                // Setup for Z data read
                adc_data_buffer[1] = read_spi_buf_i24(spi_rx_buffer, 0);
                check_state = ADC_SAMPLING_STATE_Y_CHANNEL_COMPLETE;
                if (__atomic_compare_exchange_n(&adc_sampling_state, &check_state, ADC_SAMPLING_STATE_Z_CHANNEL, false, __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST))
                    adc_sensors_start_data_read_hold();
                else
                    state_machine_error = true;
                break;
            }

            case ADC_SAMPLING_STATE_Z_CHANNEL:
            {
                adc_data_buffer[2] = read_spi_buf_i24(spi_rx_buffer, 0);

                // Trigger STOP to update the ADC buffer later
                nrf_saadc_task_trigger(NRF_SAADC_TASK_STOP);

                //Scale new data
                if(rbridge_vbias_measure_state)
                {
                    vbias_x = ((float)adc_vbias_buffer[0] * ADC_SAADC_RBRIDGE_VBIAS_ADC_VREF / 8192.0f + ADC_SAADC_RBRIDGE_VBIAS_NOMINAL_V) / ADC_SAADC_RBRIDGE_VBIAS_GAIN_EXT;
                    vbias_y = ((float)adc_vbias_buffer[1] * ADC_SAADC_RBRIDGE_VBIAS_ADC_VREF / 8192.0f + ADC_SAADC_RBRIDGE_VBIAS_NOMINAL_V) / ADC_SAADC_RBRIDGE_VBIAS_GAIN_EXT;
                    vbias_z = ((float)adc_vbias_buffer[2] * ADC_SAADC_RBRIDGE_VBIAS_ADC_VREF / 8192.0f + ADC_SAADC_RBRIDGE_VBIAS_NOMINAL_V) / ADC_SAADC_RBRIDGE_VBIAS_GAIN_EXT;
                }
                else
                {
                    vsense_x = (((float)adc_vsense_buffer[0]) / 16384.0f * ADC_SAADC_RBRIDGE_RSENSE_ADC_VREF + ADC_SAADC_RBRIDGE_RSENSE_NOMINAL_V) / ADC_SAADC_RBRIDGE_RSENSE_GAIN_EXT;
                    vsense_y = (((float)adc_vsense_buffer[1]) / 16384.0f * ADC_SAADC_RBRIDGE_RSENSE_ADC_VREF + ADC_SAADC_RBRIDGE_RSENSE_NOMINAL_V) / ADC_SAADC_RBRIDGE_RSENSE_GAIN_EXT;
                    vsense_z = (((float)adc_vsense_buffer[2]) / 16384.0f * ADC_SAADC_RBRIDGE_RSENSE_ADC_VREF + ADC_SAADC_RBRIDGE_RSENSE_NOMINAL_V) / ADC_SAADC_RBRIDGE_RSENSE_GAIN_EXT;
                }

                //Recalculate rbridge
                //rbridge = (vbias * ADC_SAADC_RBRIDGE_RSENSE / vsense) - ADC_SAADC_RBRIDGE_RSENSE;
                float vsense3x = vsense_x + vsense_y + vsense_z;
                float vbias3x = vbias_x + vbias_y + vbias_z;
                if(vsense3x > 0)
                    rbridge = ((vbias3x) * ADC_SAADC_RBRIDGE_RSENSE / (vsense3x)) - ADC_SAADC_RBRIDGE_RSENSE;
                else
                    rbridge = 0;
                
                //Save data
                adc_data_buffer[3] = (int32_t)(rbridge * 10.0f); //Resistance * 10 for extra resolution

                #ifdef ADS127L11_INCLUDE_DEBUG_DATA
                memcpy(&adc_data_buffer[4], &vbias_x, sizeof(vbias_x));
                memcpy(&adc_data_buffer[5], &vbias_y, sizeof(vbias_y));
                memcpy(&adc_data_buffer[6], &vbias_z, sizeof(vbias_z));
                memcpy(&adc_data_buffer[7], &vsense_x, sizeof(vsense_x));
                memcpy(&adc_data_buffer[8], &vsense_y, sizeof(vsense_y));
                memcpy(&adc_data_buffer[9], &vsense_z, sizeof(vsense_z));
                #endif

                if (sampler_dry_run == 0)
                {
                    (*adc_sample_set_ready_callback)((uint8_t *)adc_data_buffer, adc_sample_data_size, curent_sample_timestamp, current_sample_timestamp_offest);
                    adc_sensors_restart_state_machine();
                }
                else
                {
                    sampler_dry_run--;

                    //Only add data to average every time BOTH vsense and vbias have been measured
                    if((sampler_dry_run%2)==0x00)
                    {
                        rbridge_init_value += adc_data_buffer[3];
                        if (sampler_dry_run == 0)
                        {
                            rbridge_init_value /= sampler_dry_run_total;

                            // Stop the state machine
                            stop_state_machine_bit = true;
                            check_stop_statemachine_bit();
                        }
                        else
                        {
                            adc_sensors_restart_state_machine();
                        }
                    }
                    else
                    {
                        adc_sensors_restart_state_machine();
                    }
                }
                break;
            }

            case ADC_SAMPLING_STATE_ERROR:
            default:
                state_machine_error = true;
                break;
        }

        if (state_machine_error)
        {
            adc_state_errors++;

            stop_state_machine_bit = true;
            check_stop_statemachine_bit();

            adc_sensors_start_state_machine();
        }
    }

#ifdef OMMO_DEBUG_ADC_PRI1_ISR_PIN
    nrf_gpio_pin_clear(OMMO_DEBUG_ADC_PRI1_ISR_PIN);
#endif
}

static void restore_default_sampling_ppi_config()
{
    nrfx_gpiote_clr_task_trigger(OMMO_ADC_START_PIN);

    // Reset the bridge drive
    adc_sensors_intermittent_gpiote_pin_disable_gpiote(OMMO_IO_X_EN);
    adc_sensors_intermittent_gpiote_pin_disable_gpiote(OMMO_IO_Y_EN);
    adc_sensors_intermittent_gpiote_pin_disable_gpiote(OMMO_IO_Z_EN);
    adc_sensors_intermittent_gpiote_pin_enable_gpiote(OMMO_IO_X_EN);
    adc_sensors_intermittent_gpiote_pin_enable_gpiote(OMMO_IO_Z_EN);
#ifdef OMMO_MAG_IDLE_BRIDGE_ON
    adc_sensors_intermittent_gpiote_pin_set(OMMO_IO_Z_EN);
    nrf_ppi_fork_endpoint_setup(channel_bridge_select_ppi, nrfx_gpiote_clr_task_addr_get(OMMO_IO_Z_EN));
#else
    adc_sensors_intermittent_gpiote_pin_clear(OMMO_IO_Z_EN);
    nrf_ppi_fork_endpoint_setup(channel_bridge_select_ppi, 0);
#endif
    adc_sensors_intermittent_gpiote_pin_clear(OMMO_IO_X_EN);
    adc_sensors_intermittent_gpiote_pin_clear(OMMO_IO_Y_EN);

    nrf_ppi_task_endpoint_setup(channel_bridge_select_ppi, nrfx_gpiote_set_task_addr_get(OMMO_IO_X_EN));
#ifndef OMMO_MAG_IDLE_BRIDGE_ON
    nrf_ppi_task_endpoint_setup(channel_bridge_off_ppi, 0);
    nrf_ppi_fork_endpoint_setup(channel_bridge_off_ppi, 0);
#endif

#ifndef OMMO_ADC_IOXYZ_MUX_CTRL
    // 0b11(init)/0b10(z) -> 0b01 for x
    nrf_ppi_task_endpoint_setup(mux_select_ppi, nrfx_gpiote_clr_task_addr_get(OMMO_ADC_MUX_A1_PIN));
    nrf_ppi_fork_endpoint_setup(mux_select_ppi, nrfx_gpiote_set_task_addr_get(OMMO_ADC_MUX_A0_PIN));
#endif
}

static bool check_stop_statemachine_bit()
{
    if (stop_state_machine_bit)
    {
        // Disable the timer IRQ.
        nrfx_timer_compare_int_disable(&adc_sampling_timer, ADC_START_SAMPLE_CC_CHANNEL);
        adc_sampling_state = ADC_SAMPLING_STATE_IDLE;
        stop_state_machine_bit = false;

        APP_ERROR_CHECK(nrfx_ppi_group_disable(adc_sampling_ppi_group));
        //TODO was this necessary
        //nrfx_spim_abort(&spi_peripheral);
        restore_default_sampling_ppi_config();
        nrf_saadc_disable();
        return true;
    }
    return false;
}

nrfx_gpiote_out_config_t adc_sensors_intermittent_pin_gpiote_config =
{
    NRF_GPIOTE_POLARITY_TOGGLE,
    NRF_GPIOTE_INITIAL_VALUE_LOW,
    true, //Action pin
    true //Do not configure gpio system
};

static void adc_sensors_intermittent_gpiote_pin_init(uint32_t pin)
{
    uint64_t pin_mask = (1ul<<pin);

    NRFX_ASSERT(pin < 64);
    NRFX_ASSERT( !((pin_mask & adc_sensors_intermittent_pin_gpiote)) );

    //Set bridge pins, we MUST enable the input buffer so that we can read the current pin state even in output mode
    //We need this so that we can identify the current pin state when switching from gpiote to gpio pin control
    nrf_gpio_cfg(pin, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_CONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE);
}

static void adc_sensors_intermittent_gpiote_pin_enable_gpiote(uint32_t pin)
{
    uint64_t pin_mask = (1ul<<pin);

    NRFX_ASSERT(pin < 64);
    NRFX_ASSERT( !((pin_mask & adc_sensors_intermittent_pin_gpiote)) );

    //Match gpiote pin state matches current current pin state
    bool pin_value = nrf_gpio_pin_read(pin);
    adc_sensors_intermittent_pin_gpiote_config.init_state = (pin_value ? NRF_GPIOTE_INITIAL_VALUE_HIGH : NRF_GPIOTE_INITIAL_VALUE_LOW);
    APP_ERROR_CHECK(nrfx_gpiote_out_init(pin, &adc_sensors_intermittent_pin_gpiote_config));

    nrf_gpio_pin_write(pin, pin_value);
    nrfx_gpiote_out_task_enable(pin);

    adc_sensors_intermittent_pin_gpiote |= pin_mask;
}
static void adc_sensors_intermittent_gpiote_pin_disable_gpiote(uint32_t pin)
{
    uint64_t pin_mask = (1ul<<pin);

    NRFX_ASSERT(pin < 64);

    //Make sure gpio system's output matches current pin state
    nrf_gpio_pin_write(pin, nrf_gpio_pin_read(pin));

    if(pin_mask & adc_sensors_intermittent_pin_gpiote) nrfx_gpiote_out_uninit(pin);
    adc_sensors_intermittent_pin_gpiote &= (~pin_mask);
}
static void adc_sensors_intermittent_gpiote_pin_set(uint32_t pin)
{
    uint64_t pin_mask = (1ul<<pin);

    //Set in gpiote
    if(pin_mask & adc_sensors_intermittent_pin_gpiote) 
        nrfx_gpiote_set_task_trigger(pin);
    
    //Set in gpio
    nrf_gpio_pin_set(pin);
}
static void adc_sensors_intermittent_gpiote_pin_clear(uint32_t pin)
{
    uint64_t pin_mask = (1ul<<pin);

    //Set in gpiote
    if(pin_mask & adc_sensors_intermittent_pin_gpiote) 
        nrfx_gpiote_clr_task_trigger(pin);
    
    //Set in gpio
    nrf_gpio_pin_clear(pin);
}

void overvoltage_event_handler(nrf_comp_event_t event)
{
    overvoltage_triggered = true;

    //Disable state machine
    adc_sensors_disable();
}

static void adc_sensors_start_data_read_hold()
{
    if(!check_stop_statemachine_bit())
        APP_ERROR_CHECK(comm_device_ptr->trx(nullptr, 0, spi_rx_buffer, ADS127L11_SPI_READ_LENGTH, comm_device_event_handler, nullptr, COMM_DEVICE_FLAGS_HOLD));
}

static void adc_sensors_restart_state_machine()
{
    // Configure X channel time compare registers.
    adc_sampling_state = ADC_SAMPLING_STATE_X_CHANNEL_PENDING;
    nrfx_timer_compare(&adc_sampling_timer, ADC_MUX_SWITCH_CC_CHANNEL, US_TO_TICKS_16MHZ(ADC_MUX_SWITCH_TIME_US(0)) + 1, false); //+1 to trigger right at start
    nrfx_timer_compare(&adc_sampling_timer, ADC_START_SAMPLE_CC_CHANNEL, US_TO_TICKS_16MHZ(ADC_START_SAMPLE_TIME_US(0)), true);

    // Restore default PPI anf GPIOTE configuration.
    restore_default_sampling_ppi_config();

    //Setup adc for next sample
    nrf_saadc_task_trigger(NRF_SAADC_TASK_STOP);
    rbridge_vbias_measure_state = !rbridge_vbias_measure_state;
    if(rbridge_vbias_measure_state)
    {
        nrf_saadc_channel_init(0, &adc_config_vbias);
        nrf_saadc_buffer_init(adc_vbias_buffer, 3);
    }
    else
    {
        nrf_saadc_channel_init(0, &adc_config_vsense);
        nrf_saadc_buffer_init(adc_vsense_buffer, 3);
    }
    nrf_saadc_task_trigger(NRF_SAADC_TASK_START);

    // Prepare SPI transfer.
    adc_sensors_start_data_read_hold();
}

static void adc_sensors_start_state_machine()
{
    // Stop and clear the timer.
    APP_ERROR_CHECK(nrfx_ppi_group_disable(adc_sampling_ppi_group));
    nrfx_timer_disable(&adc_sampling_timer);
    nrfx_timer_clear(&adc_sampling_timer);

    //Enable saadc
    nrf_saadc_enable();

    adc_sensors_restart_state_machine();
    APP_ERROR_CHECK(nrfx_ppi_group_enable(adc_sampling_ppi_group));
}

//cmd sent on first frame and reg data received on 2nd frame
static uint8_t adc_sensors_read_reg(uint8_t reg)
{
    const uint8_t frame_size = 3;

    //first frame
    spi_tx_buffer[0] = 0x00;
    spi_tx_buffer[1] = ADS127L11_RREG_CMD | reg;
    spi_tx_buffer[2] = 0x00;
    APP_ERROR_CHECK(comm_device_ptr->trx(spi_tx_buffer, frame_size, nullptr, 0));

    //Give the cs pin same time to stay high, min 20ns
    nrf_delay_us(1);

    //2nd frame
    APP_ERROR_CHECK(comm_device_ptr->trx(nullptr, 0, spi_rx_buffer, frame_size));

    return spi_rx_buffer[0];    //first byte on 2nd frame
}

static void adc_sensors_write_reg(uint8_t reg, uint8_t val)
{
    const uint8_t frame_size = 3;

    spi_tx_buffer[1] = ADS127L11_WREG_CMD | reg;
    spi_tx_buffer[2] = val;
    spi_tx_buffer[3] = 0x00;
 
    APP_ERROR_CHECK(comm_device_ptr->trx(spi_tx_buffer, frame_size, nullptr, 0));
}

static uint32_t adc_sensors_config_adc()
{
    adc_sensors_disable();

    //Reset device
    nrf_gpio_pin_write(OMMO_ADC_RESET_PIN, 0);
    nrf_delay_ms(5);   //need 10004 clk cycles, ~.6ms
    nrf_gpio_pin_write(OMMO_ADC_RESET_PIN, 1);

    adc_sensors_write_reg(ADS127L11_CONFIG1_REG, ADS127L11_CONFIG1_VAL);
    adc_sensors_write_reg(ADS127L11_CONFIG2_REG, ADS127L11_CONFIG2_VAL);
    adc_sensors_write_reg(ADS127L11_CONFIG3_REG, ADS127L11_CONFIG3_VAL);
    uint8_t config1 = adc_sensors_read_reg(ADS127L11_CONFIG1_REG);
    uint8_t config2 = adc_sensors_read_reg(ADS127L11_CONFIG2_REG);
    uint8_t config3 = adc_sensors_read_reg(ADS127L11_CONFIG3_REG);

    if (config1 == ADS127L11_CONFIG1_VAL &&
        config2 == ADS127L11_CONFIG2_VAL &&
        config3 == ADS127L11_CONFIG3_VAL)
    {
        return NRF_SUCCESS;
    }

    return NRF_ERROR_NOT_FOUND;
}

ret_code_t adc_sensors_dry_run_blocking(uint8_t dry_run_times)
{
    //Make sure we are stopped
    VERIFY_TRUE(adc_sampling_state == ADC_SAMPLING_STATE_IDLE, NRF_ERROR_INVALID_STATE);

    rbridge_init_value = 0;
    sampler_dry_run = dry_run_times*2; //We only get a sample every other time
    sampler_dry_run_total = dry_run_times;
    adc_sensors_start_state_machine();

    while(sampler_dry_run != 0) main_event_queue.execute_once();;

    return NRF_SUCCESS;
}

uint16_t adc_process_packet_received(uint8_t data[], uint16_t data_length, uint8_t response_buffer[], uint16_t response_buffer_size)
{
    if (data[0] == OMMO_COMMAND_SET_MAG_CAL_MODE)
    {
        if (data_length != 3 || data[1] != 0x55)
        {
            return fill_in_ack_packet(response_buffer, OMMO_ACK_INVALID_DATA);
        }

        mag_calibration_mode = data[2];
        return fill_in_ack_packet(response_buffer, OMMO_ACK_SUCCESS);
    }

    if (data[0] == OMMO_COMMAND_GET_PACKET_DESCRIPTOR)
    {
        if (adc_sampling_state != ADC_SAMPLING_STATE_IDLE)
            return fill_in_ack_packet(response_buffer, OMMO_ACK_INVALID_MODE);
        else
            return adc_scan_bus_generate_data_descriptor(response_buffer, response_buffer_size);
    }

    // Does nothing. Make a resp to fit python script setup procedure
    if (data[0] == OMMO_COMMAND_ONBOARD_SENSOR_ENABLE)
    {
        return fill_in_ack_packet(
              response_buffer, (data_length == 2) ? OMMO_ACK_SUCCESS : OMMO_ACK_INVALID_DATA);
    }

    // See if device info processor can process it
    return device_info_process_packet_received(data, data_length, response_buffer, response_buffer_size, mag_calibration_mode);
}

ret_code_t allocate_device_info_with_mag_cal(uint8_t port, uint8_t num_ics, ICBusType *ic_bus_type, uint8_t *ic_ss_index_bus_location, const DeviceInfoICProto *default_ic_protos, DeviceDescriptorProto *ddp, PortStatus *port_status)
{
    bool ddp_allocated = false;

    //Determine port type and read device info
    device_info_storage_type port_type = device_info_scan_port_for_storage_media(port);

    //Is storage media present?
    if(port_type == DEVICE_INFO_NONE)
    {
        *port_status = PORT_STATUS_DISCONNECTED;
        return NRF_SUCCESS;
    }

    if(device_info_allocate_and_read_from_port(port, port_type, (void**)&ddp->device_info, DEVICE_INFO_FIELD_PERM) == NRF_SUCCESS)
    {
        ddp_allocated = true;

        device_info_allocate_and_read_from_port(port, port_type, (void**)&ddp->device_info_user, DEVICE_INFO_FIELD_USER);
        if(ddp->device_info_user == NULL)
        {
            if(device_info_allocate_and_create_user(&ddp->device_info_user) != NRF_SUCCESS)
            {
                *port_status = PortStatus_PORT_STATUS_ERROR;
                return NRF_ERROR_NO_MEM;
            }
        }
    }

    if(mag_calibration_mode)
    {
        uint64_t uuid = 0;
        uint64_t calibration_date = 0;

        //If we have an allocated device info, pull out data and free memory
        if(ddp_allocated)
        {
            calibration_date = ddp->device_info->calibration_date;
            uuid = ddp->device_info->uuid;

            device_info_free_perm_proto(&ddp->device_info);
            device_info_free_user_proto(&ddp->device_info_user);
            ddp_allocated = false;
        }

        //Fill in ic_type array
        ICType ic_type[16];
        for(uint8_t ic = 0; ic<num_ics; ic++)
            ic_type[ic] = default_ic_protos->ic_type;

        ret_code_t error_code = device_info_allocate_and_create_perm(num_ics, ic_bus_type, ic_ss_index_bus_location, ic_type, &ddp->device_info);
        if(error_code == NRF_SUCCESS) error_code = device_info_allocate_and_create_user(&ddp->device_info_user);
        if(error_code != NRF_SUCCESS)
        {
            *port_status = PortStatus_PORT_STATUS_ERROR;
            return error_code;
        }

        ddp_allocated = true;

        //Fill in device info value
        ddp->device_info->calibration_date = calibration_date;
        ddp->device_info->uuid = uuid;
        ddp->device_info->data_sourcing_strat = DSS_INITIALIZATION;

        //Flag onboard device if it is
        if(port_type == DEVICE_INFO_FLASH)
            ddp->device_info->onboard_siu_parent = (SiuPids)APP_USBD_PID;
    }

    //Media was present, but we didn't find/make a ddp
    if(!ddp_allocated)
    {
        *port_status = PortStatus_PORT_STATUS_ERROR;
        return NRF_SUCCESS;
    }

    //Fill in default values
    bool valid_ddp = ddp->device_info->ics_count == num_ics;
    for(uint8_t ic = 0; ic < num_ics; ic++)
        valid_ddp &= (device_info_allocate_and_fill_in_default_ic_data(&ddp->device_info->ics[ic], &default_ic_protos[ic], 0, 
                                                                       ddp->device_info->data_sourcing_strat) == NRF_SUCCESS);
    if(!valid_ddp)
    {
        *port_status = PortStatus_PORT_STATUS_ERROR;
        return NRF_SUCCESS;
    }

    //Set port number
    ddp->port_num = port;

    *port_status = PORT_STATUS_CONNECTED;
    return NRF_SUCCESS;
}

uint16_t adc_scan_bus_generate_data_descriptor(uint8_t packet_id_request_buffer[], uint16_t buffer_size)
{
    PortStatus port_status_list[OMMO_PM_PORTS_COUNT] = {PORT_STATUS_DISCONNECTED};
    DeviceGroupDescriptorProto dgdp = DeviceGroupDescriptorProto_init_default;

    //Reset state machine okay to run
    state_machine_configured = false;

    //Assemble static stream descriptor
    StreamDescriptorProto sdp =
    {
        .descriptors_count = 1,
        .descriptors = &dgdp,
        .port_status_list_count = OMMO_PM_PORTS_COUNT,
        .port_status_list = port_status_list,
    };

    //Update ommcoomm device if connected
    adc_sensors_dfu_ommocomm_eeprom();

    //Allocate device list large enough for all ports
    if((dgdp.devices = (DeviceDescriptorProto *)malloc(OMMO_PM_PORTS_COUNT*sizeof(DeviceDescriptorProto))) == NULL)
        return fill_in_ack_packet(packet_id_request_buffer, OMMO_ACK_INTERNAL_ERROR);
    memset(dgdp.devices, 0x00, OMMO_PM_PORTS_COUNT*sizeof(DeviceDescriptorProto));

    //Read and fill in data
    uint8_t port = OMMO_ADC_PORT;
    uint8_t num_ics = 1;
    ICBusType ic_bus_type = OMMO_ADC_BUS_TYPE;
    uint8_t ic_ss_index = OMMO_ADC_SS_INDEX;
#ifdef ADS127L11_INCLUDE_DEBUG_DATA
    const DeviceInfoICProto *default_ic_protos = &ic_def_adc_ads127l11_debug;
#else
    const DeviceInfoICProto *default_ic_protos = &ic_def_generic_adc_ads127l11;
#endif
    ret_code_t error_code = allocate_device_info_with_mag_cal(port, num_ics, &ic_bus_type, &ic_ss_index, default_ic_protos, &dgdp.devices[dgdp.devices_count], &port_status_list[port]);

    //Set firmware defaults
    if(error_code == NRF_SUCCESS && port_status_list[port] == PORT_STATUS_CONNECTED)
    {
        //Update timestamp offsets using firmware delays since channel dependent timestamp offsets are not support in device_info_allocate_and_fill_in_default_ic_data
        uint8_t count = dgdp.devices[dgdp.devices_count].device_info->ics[0].sensors[0].timestamp_offsets_count;
        if(count > 0) dgdp.devices[dgdp.devices_count].device_info->ics[0].sensors[0].timestamp_offsets[0] += ADS127L11_X_CHANNEL_DELAY;
        if(count > 1) dgdp.devices[dgdp.devices_count].device_info->ics[0].sensors[0].timestamp_offsets[1] += ADS127L11_Y_CHANNEL_DELAY;
        if(count > 2) dgdp.devices[dgdp.devices_count].device_info->ics[0].sensors[0].timestamp_offsets[2] += ADS127L11_Z_CHANNEL_DELAY;

        //Run state machine 10x
        APP_ERROR_CHECK(adc_sensors_dry_run_blocking(10));

        // Copy rbridge into initial value for temp sensor
        dgdp.devices[dgdp.devices_count].device_info->ics[0].sensors[1].initial_value = rbridge_init_value;
    }

    if(error_code != NRF_SUCCESS || port_status_list[port] != PORT_STATUS_CONNECTED)
    {
        pb_release(&DeviceDescriptorProto_msg, &dgdp.devices[dgdp.devices_count]);
        memset(&dgdp.devices[dgdp.devices_count], 0x00, sizeof(DeviceDescriptorProto));
    }
    else
    {
        dgdp.devices_count++;
    }

    //Release memory and let user know if we had a failure
    if(error_code != NRF_SUCCESS)
    {
        pb_release(&DeviceGroupDescriptorProto_msg, &dgdp);
        return fill_in_ack_packet(packet_id_request_buffer, convert_nrf_error_code_to_ommo_ack_code(error_code));
    }

    //Set basic dgdp properties
    dgdp.timestamp_ticks_per_second = HFCLK_FREQ;
    dgdp.sample_period = OMMO_TIMESTAMP_SAMPLE_PERIOD;
    dgdp.siu_uuid = NRF_FICR->DEVICEID[0];
    dgdp.header_format = DATA_HEADER_FORMAT_TS;
    dgdp.data_packet_length = adc_sample_data_size + 1 + 4;    //pkt_id(1) + ts(4)

    //Generate packet id request info
    uint16_t index = 0;
    copyUint8(packet_id_request_buffer, index, OMMO_COMMAND_GET_PACKET_DESCRIPTOR); //command
    copyUint8(packet_id_request_buffer, index, DEVICE_GROUP_DESCRIPTOR_FORMAT_MULTI_PROTOBUF); //dataDescriptorFormat

    //Encode protobuf and save size
    pb_ostream_t stream = pb_ostream_from_buffer(packet_id_request_buffer+4, buffer_size-4);
    if(!pb_encode(&stream, &StreamDescriptorProto_msg, &sdp))
    {
        pb_release(&DeviceGroupDescriptorProto_msg, &dgdp);
        return fill_in_ack_packet(packet_id_request_buffer, OMMO_ACK_INTERNAL_ERROR);
    }
    copyUint16_LE(packet_id_request_buffer, index, stream.bytes_written);
    index += stream.bytes_written;

    //Free allocated memory
    pb_release(&DeviceGroupDescriptorProto_msg, &dgdp);

    //Flag system ready to run if ADC connected
    if(port_status_list[OMMO_ADC_PORT] == PORT_STATUS_CONNECTED)
        state_machine_configured = true;

    return index;
}

bool adc_sensors_is_state_machine_configured()
{
    return state_machine_configured;
}

ret_code_t adc_sensors_enable()
{
    //Make sure we are stopped
    VERIFY_TRUE(adc_sampling_state == ADC_SAMPLING_STATE_IDLE, NRF_ERROR_INVALID_STATE);
    VERIFY_TRUE(state_machine_configured, NRF_ERROR_INVALID_STATE);

    
    //Turn on a bridge
#ifdef OMMO_MAG_IDLE_BRIDGE_ON
    adc_sensors_intermittent_gpiote_pin_set(OMMO_IO_Z_EN);

#ifdef OMMO_MAG_BRIDGE_OVERVOLT_COMP
    //Start with all three bridges on, then switch to desired bridge to avoid vbias overshoot
    adc_sensors_intermittent_gpiote_pin_set(OMMO_IO_X_EN);
    adc_sensors_intermittent_gpiote_pin_set(OMMO_IO_Y_EN);
    nrf_delay_us(100);
#endif
#endif
    
    //Turn on power, and let current source settle
    overvoltage_triggered = false;
#ifdef OMMO_MAG_BRIDGE_OVERVOLT_COMP
    nrfx_gpiote_set_task_trigger(OMMO_VBRIDGE_EN);    
    nrf_delay_us(100);

    if(overvoltage_triggered)
        return NRF_ERROR_INTERNAL;    
#else
    nrf_gpio_pin_set(OMMO_VBRIDGE_EN);
#endif
//#ifdef OMMO_BOOST_EN
//    nrf_gpio_pin_set(OMMO_BOOST_EN);
//#endif

    //Start state machine
    adc_sensors_start_state_machine();
    return NRF_SUCCESS;
}

void adc_sensors_disable()
{
    //Turn off bridge power
#ifdef OMMO_MAG_BRIDGE_OVERVOLT_COMP
    nrfx_gpiote_clr_task_trigger(OMMO_VBRIDGE_EN);
    nrf_delay_us(100);
#else
    nrf_gpio_pin_clear(OMMO_VBRIDGE_EN);
#endif
//#ifdef OMMO_BOOST_EN
//    nrf_gpio_pin_clear(OMMO_BOOST_EN);
//#endif
    
    //Turn off bridges
#ifdef OMMO_MAG_IDLE_BRIDGE_ON
    adc_sensors_intermittent_gpiote_pin_clear(OMMO_IO_X_EN);
    adc_sensors_intermittent_gpiote_pin_clear(OMMO_IO_Y_EN);
    adc_sensors_intermittent_gpiote_pin_clear(OMMO_IO_Z_EN);
#endif

    if (adc_sampling_state != ADC_SAMPLING_STATE_IDLE)
    {
        stop_state_machine_bit = true;
        while(adc_sampling_state != ADC_SAMPLING_STATE_IDLE)
            main_event_queue.execute_once();;
    }
}

#if OMMO_PM_UARTE_COUNT

static void stm32_dfu_progress(fw_prog_stage_t stage, size_t progress)
{
    switch(stage)
    {
        case PROG_STAGE_PREPARE:
            if (progress == 0)
            {
                // Update leds to DFU state
                push_led_state(APP_STATE_OMMOCOMM_DFU);
            }
            break;
    
        case PROG_STAGE_REBOOT:
            // Update leds to DFU state
            pop_led_state();
            break;
    }
}

static void adc_sensors_dfu_ommocomm_eeprom()
{
    ommocomm_uarte * ommocomm = nullptr;
    if (port_master_acquire_ommocomm_direct(1, false, &ommocomm) == NRF_SUCCESS)
    {
        ommocomm->check_fw_and_update(stm32_dfu_progress);
        ommocomm->release();
    }
}
#endif

static void adc_ppi_init()
{
    // Init GPIOTE.
    if (!nrfx_gpiote_is_init())
    {
        APP_ERROR_CHECK(nrfx_gpiote_init());
    }

    nrfx_gpiote_in_config_t input_config = NRFX_GPIOTE_CONFIG_IN_SENSE_HITOLO(true);

    APP_ERROR_CHECK(nrfx_gpiote_in_init_errata155(OMMO_ADC_DATA_READY_PIN, &input_config, NULL, NULL, false));  //NOTE If OMMO_DEBUG_ABSOLUTE_ALIGN_TEST the errata fix causes this pin not to work
    // APP_ERROR_CHECK(nrfx_gpiote_in_init(OMMO_ADC_DATA_READY_PIN, &input_config, NULL));
    nrfx_gpiote_in_event_enable(OMMO_ADC_DATA_READY_PIN, false);

    // GPIOTE output pin config.
    nrfx_gpiote_out_config_t output_config = NRFX_GPIOTE_CONFIG_OUT_TASK_TOGGLE(true);

#ifndef OMMO_ADC_IOXYZ_MUX_CTRL
    // Configure MUX pins.
    APP_ERROR_CHECK(nrfx_gpiote_out_init(OMMO_ADC_MUX_A0_PIN, &output_config));
    APP_ERROR_CHECK(nrfx_gpiote_out_init(OMMO_ADC_MUX_A1_PIN, &output_config));
    nrfx_gpiote_out_task_enable(OMMO_ADC_MUX_A0_PIN);
    nrfx_gpiote_out_task_enable(OMMO_ADC_MUX_A1_PIN);
#endif

    output_config.init_state = NRF_GPIOTE_INITIAL_VALUE_LOW;
    APP_ERROR_CHECK(nrfx_gpiote_out_init(OMMO_ADC_START_PIN, &output_config));
    nrfx_gpiote_out_task_enable(OMMO_ADC_START_PIN);

    //Set bridge pin
    adc_sensors_intermittent_gpiote_pin_init(OMMO_IO_X_EN);
    adc_sensors_intermittent_gpiote_pin_init(OMMO_IO_Y_EN);
    adc_sensors_intermittent_gpiote_pin_init(OMMO_IO_Z_EN);

    APP_ERROR_CHECK(nrfx_ppi_channel_alloc(&adc_start_ppi));
    APP_ERROR_CHECK(nrfx_ppi_channel_alloc(&adc_ready_spi_read_ppi));
    APP_ERROR_CHECK(nrfx_ppi_channel_alloc(&mux_select_ppi));
    APP_ERROR_CHECK(nrfx_ppi_channel_alloc(&channel_bridge_select_ppi));
#ifndef OMMO_MAG_IDLE_BRIDGE_ON
    APP_ERROR_CHECK(nrfx_ppi_channel_alloc(&channel_bridge_off_ppi));
#endif

    // Setup select event for the MUX and channel bridge.
    nrf_ppi_event_endpoint_setup(mux_select_ppi,
        nrfx_timer_compare_event_address_get(&adc_sampling_timer, ADC_MUX_SWITCH_CC_CHANNEL));
    nrf_ppi_event_endpoint_setup(channel_bridge_select_ppi,
        nrfx_timer_compare_event_address_get(&adc_sampling_timer, ADC_MUX_SWITCH_CC_CHANNEL));

#ifndef OMMO_MAG_IDLE_BRIDGE_ON
    // Setup to switch channel bridge off after each ADC sample.
    nrf_ppi_event_endpoint_setup(channel_bridge_off_ppi,
        nrfx_gpiote_in_event_addr_get(OMMO_ADC_DATA_READY_PIN));
#endif

#if OMMO_ADC_SET_RESET_ENABLED
    /* ATTENTION !!! SET/RESET pin configuration, modify carefully, it may damage the IC !!! */
    APP_ERROR_CHECK(nrfx_ppi_channel_alloc(&set_reset_start_ppi));
    APP_ERROR_CHECK(nrfx_ppi_channel_alloc(&set_reset_end_ppi));

#if OMMO_ADC_SET_RESET_POLARITY_INVERTED
    const uint32_t set_reset_pin_a = OMMO_ADC_SET_RESET_INB_PIN;
    const uint32_t set_reset_pin_b = OMMO_ADC_SET_RESET_INA_PIN;
#else
    const uint32_t set_reset_pin_a = OMMO_ADC_SET_RESET_INA_PIN;
    const uint32_t set_reset_pin_b = OMMO_ADC_SET_RESET_INB_PIN;
#endif

    // Low active output pins.
    nrf_gpio_pin_clear(set_reset_pin_a);
    nrf_gpio_pin_clear(set_reset_pin_b);
    nrf_gpio_cfg_output(set_reset_pin_a);
    nrf_gpio_cfg_output(set_reset_pin_b);

    // Enable driver
#ifdef OMMO_ADC_SET_RESET_EN_PIN
    nrf_gpio_pin_set(OMMO_ADC_SET_RESET_EN_PIN);
    nrf_gpio_cfg_output(OMMO_ADC_SET_RESET_EN_PIN);
#endif

    // Configure set/reset PPI (Initial value is high).
    APP_ERROR_CHECK(nrfx_gpiote_out_init(set_reset_pin_a, &output_config));
    nrfx_gpiote_out_task_enable(set_reset_pin_a);

    // Set on start.
    APP_ERROR_CHECK(nrfx_ppi_channel_assign(set_reset_start_ppi,
        nrfx_timer_compare_event_address_get(&adc_sampling_timer, ADC_SET_RESET_START_CC_CHANNEL),
        nrfx_gpiote_set_task_addr_get(set_reset_pin_a)));

    // Set on end.
    APP_ERROR_CHECK(nrfx_ppi_channel_assign(set_reset_end_ppi,
        nrfx_timer_compare_event_address_get(&adc_sampling_timer, ADC_SET_RESET_END_CC_CHANNEL),
        nrfx_gpiote_clr_task_addr_get(set_reset_pin_a)));

    // Enable END task first to avoid IC damage if this code will be moved somewhere else.
    APP_ERROR_CHECK(nrfx_ppi_channel_enable(set_reset_end_ppi));
    APP_ERROR_CHECK(nrfx_ppi_channel_enable(set_reset_start_ppi));
#endif //OMMO_ADC_SET_RESET_ENABLED

    // Once the ADC data is ready, start the SPI transfer.
    APP_ERROR_CHECK(nrfx_ppi_channel_assign(adc_ready_spi_read_ppi,
        nrfx_gpiote_in_event_addr_get(OMMO_ADC_DATA_READY_PIN),
        ((spim_basic*)comm_device_ptr)->get_start_task()));

    // Clear the start PPI once the ADC data is ready (toggle mode).
    APP_ERROR_CHECK(nrfx_ppi_channel_fork_assign(adc_ready_spi_read_ppi,
        nrfx_gpiote_out_task_addr_get(OMMO_ADC_START_PIN)));

    // Set PPI for external ADC sample/conversion start.
    APP_ERROR_CHECK(nrfx_ppi_channel_assign(adc_start_ppi,
        nrfx_timer_compare_event_address_get(&adc_sampling_timer, ADC_START_SAMPLE_CC_CHANNEL),
        nrfx_gpiote_out_task_addr_get(OMMO_ADC_START_PIN)));

    // We have to enable the ADC sample task for R-Bridge measurements
    nrf_ppi_fork_endpoint_setup(adc_start_ppi, nrf_saadc_task_address_get(NRF_SAADC_TASK_SAMPLE));
    
    // PPI group for sampling and SPI read.
    APP_ERROR_CHECK(nrfx_ppi_group_alloc(&adc_sampling_ppi_group));
    APP_ERROR_CHECK(nrfx_ppi_channel_include_in_group(adc_start_ppi, adc_sampling_ppi_group));
    APP_ERROR_CHECK(nrfx_ppi_channel_include_in_group(adc_ready_spi_read_ppi, adc_sampling_ppi_group));
    APP_ERROR_CHECK(nrfx_ppi_channel_include_in_group(mux_select_ppi, adc_sampling_ppi_group));
    APP_ERROR_CHECK(nrfx_ppi_channel_include_in_group(channel_bridge_select_ppi, adc_sampling_ppi_group));
#ifndef OMMO_MAG_IDLE_BRIDGE_ON
    APP_ERROR_CHECK(nrfx_ppi_channel_include_in_group(channel_bridge_off_ppi, adc_sampling_ppi_group));
#endif

    //Setup overvoltage control
#ifdef OMMO_MAG_BRIDGE_OVERVOLT_COMP
    //Create gpiote to turn power on/off
    output_config.init_state = NRF_GPIOTE_INITIAL_VALUE_LOW;
    APP_ERROR_CHECK(nrfx_gpiote_out_init(OMMO_VBRIDGE_EN, &output_config));
    nrfx_gpiote_out_task_enable(OMMO_VBRIDGE_EN);

    //Setup comparator for overvoltage
    nrfx_comp_config_t comp_config = NRFX_COMP_DEFAULT_CONFIG(OMMO_VBRIDGE_ANALOG);
    comp_config.reference = NRF_COMP_REF_VDD;
    comp_config.threshold.th_up = NRFX_VOLTAGE_THRESHOLD_TO_INT(OMMO_ADC_OVERVOLTAGE_TRIGGER_POINT, 3.3);
    comp_config.threshold.th_down = 0;   
    comp_config.hyst = NRF_COMP_HYST_NoHyst;
    comp_config.speed_mode = NRF_COMP_SP_MODE_High;
    APP_ERROR_CHECK(nrfx_comp_init(&comp_config, overvoltage_event_handler));

    //Setup PPI to kill power on overvoltage
    nrf_ppi_channel_t overvoltage_ppi;
    APP_ERROR_CHECK(nrfx_ppi_channel_alloc(&overvoltage_ppi));
    APP_ERROR_CHECK(nrfx_ppi_channel_assign(overvoltage_ppi,
        nrfx_comp_event_address_get(NRF_COMP_EVENT_UP),
        nrfx_gpiote_clr_task_addr_get(OMMO_VBRIDGE_EN)));
    APP_ERROR_CHECK(nrfx_ppi_channel_enable(overvoltage_ppi));

    //Enable comparator
#ifndef OMMO_ADC_DISABLE_OVERVOLTAGE
    nrfx_comp_start(NRFX_COMP_EVT_EN_UP_MASK, 0x00);
#endif
#endif

    // Restore default PPI configuration.
    restore_default_sampling_ppi_config();
}

void adc_sensors_init(adc_sensors_callback_func_type sample_set_ready_callback_function)
{
    //Save callback/timestamp timer
    adc_sample_set_ready_callback = sample_set_ready_callback_function;

    //Init gpio pins
#ifdef OMMO_ADC_MUX_EN_PIN
    // Enable the MUX.
    nrf_gpio_pin_set(OMMO_ADC_MUX_EN_PIN);
    nrf_gpio_cfg_output(OMMO_ADC_MUX_EN_PIN);
#endif

#ifdef OMMO_BOOST_EN
    // Turn off boost
    //nrf_gpio_pin_clear(OMMO_BOOST_EN);
    nrf_gpio_pin_set(OMMO_BOOST_EN);
    nrf_gpio_cfg_output(OMMO_BOOST_EN);
#endif
#ifdef OMMO_VBRIDGE_EN
    // Disable bridge
    nrf_gpio_pin_clear(OMMO_VBRIDGE_EN);
    nrf_gpio_cfg_output(OMMO_VBRIDGE_EN);
#endif

    //Acquire SPI connection permanently
    APP_ERROR_CHECK(port_master_acquire(OMMO_ADC_PORT, OMMO_ADC_BUS_TYPE, OMMO_ADC_SS_INDEX, &comm_device_ptr));

    //Configure adc
    // Setup SAADC without using the nrfx library.
    nrf_saadc_disable();
    nrf_saadc_int_set(0);
    NRFX_IRQ_DISABLE(SAADC_IRQn);
    nrf_saadc_continuous_mode_disable();
    nrf_saadc_channel_init(0, &adc_config_vbias);
    nrf_saadc_resolution_set(NRF_SAADC_RESOLUTION_14BIT);
    nrf_saadc_oversample_set(NRF_SAADC_OVERSAMPLE_16X);

    // Configure PPI and GPIOTE.
    adc_ppi_init();

    // Init sampling timer and propagate START task.
    nrfx_timer_config_t timer_cfg = NRFX_TIMER_DEFAULT_CONFIG;
    timer_cfg.interrupt_priority = OMMO_ADC_SAMPLING_TIMER_PRIORITY;
    APP_ERROR_CHECK(nrfx_timer_init(&adc_sampling_timer, &timer_cfg, sampling_timer_event_handler));

#if OMMO_ADC_SET_RESET_ENABLED
    // Configure CC for set/reset.
    nrfx_timer_compare(&adc_sampling_timer, ADC_SET_RESET_START_CC_CHANNEL, US_TO_TICKS_16MHZ(ADC_SET_RESET_START_TIME_US), false);
    nrfx_timer_extended_compare(&adc_sampling_timer, ADC_SET_RESET_END_CC_CHANNEL, US_TO_TICKS_16MHZ(ADC_SET_RESET_START_TIME_US + ADC_SET_RESET_DURATION_US), ADC_SET_RESET_END_CC_SHORT_MASK, false);
#endif

    nrf_ppi_channel_t timer_channel = timestamp_add_task_on_sample_event(nrfx_timer_task_address_get(&adc_sampling_timer, NRF_TIMER_TASK_START), false);
    APP_ERROR_CHECK(nrfx_ppi_channel_include_in_group(timer_channel, adc_sampling_ppi_group));
    timestamp_add_sample_event_callback(sample_timer_event_handler);

    nrf_delay_ms(200); //Let ADC boot up
    OMMO_APP_ERROR_CHECK(adc_sensors_config_adc(), STRING("Failed to config ADC"), 0);
}
