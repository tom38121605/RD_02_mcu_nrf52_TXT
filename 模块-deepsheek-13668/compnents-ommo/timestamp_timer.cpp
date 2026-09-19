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
#include <cstring>

#include "ommo_config.h"
#include "nrfx_timer.h"
#include "nrfx_ppi.h"
#include "nrfx_gpiote.h"
#include "nrf_gpio.h"

#include "utils.hpp"
#include "trace.h"
#include "timestamp_timer.hpp"

#include "ommo_app_error.h"

/*

Description:

Known concerns:

*/

//Static const variables
//NRF_TIMER_CC_CHANNEL's are sequentially numbered 0-x
//static const nrf_timer_cc_channel_t capture_channels[] = {NRF_TIMER_CC_CHANNEL0, NRF_TIMER_CC_CHANNEL1, NRF_TIMER_CC_CHANNEL2, NRF_TIMER_CC_CHANNEL3, NRF_TIMER_CC_CHANNEL4, NRF_TIMER_CC_CHANNEL5};

#define TIMESTAMP_TIMER_NUM_CAPTURE_CHANNELS 6

static const nrf_timer_task_t capture_tasks[] = {NRF_TIMER_TASK_CAPTURE0, NRF_TIMER_TASK_CAPTURE1, NRF_TIMER_TASK_CAPTURE2, NRF_TIMER_TASK_CAPTURE3, NRF_TIMER_TASK_CAPTURE4, NRF_TIMER_TASK_CAPTURE5};
static const nrf_timer_event_t compare_events[] = {NRF_TIMER_EVENT_COMPARE0, NRF_TIMER_EVENT_COMPARE1, NRF_TIMER_EVENT_COMPARE2, NRF_TIMER_EVENT_COMPARE3, NRF_TIMER_EVENT_COMPARE4, NRF_TIMER_EVENT_COMPARE5};
static const nrf_timer_short_mask_t short_compare_clear[] = {NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, NRF_TIMER_SHORT_COMPARE1_CLEAR_MASK, NRF_TIMER_SHORT_COMPARE2_CLEAR_MASK, NRF_TIMER_SHORT_COMPARE3_CLEAR_MASK, NRF_TIMER_SHORT_COMPARE4_CLEAR_MASK, NRF_TIMER_SHORT_COMPARE5_CLEAR_MASK};

//Hardware instances
static nrfx_timer_t timer_timestamp = OMMO_TIMESTAMP_TIMER;
static nrf_ppi_channel_t ppi_channel_disable_timestamp_capture;
static nrf_ppi_channel_group_t ppi_group_disable_timestamp_capture;

//Globals
static uint8_t capture_channels_used_mask = 0;
static bool initialized = false;

#define OMMO_TIMESTAMP_MAX_SAMPLE_TIME_OFFSET_CHANNELS 2
static uint8_t next_free_sample_time_offset_index = 0;
static nrf_timer_cc_channel_t sample_time_offset_cc_channel[OMMO_TIMESTAMP_MAX_SAMPLE_TIME_OFFSET_CHANNELS];
static uint32_t sample_time_offset_time[OMMO_TIMESTAMP_MAX_SAMPLE_TIME_OFFSET_CHANNELS];

//Debug
#ifdef OMMO_DEBUG_ABSOLUTE_ALIGN_TEST
static nrf_timer_cc_channel_t absolute_align_channel = 0;
#endif

//Sample event triggered tasks ppi
static bool sample_trigger_ppi_allocated = false;
static nrf_ppi_channel_t sample_trigger_ppi;

//Timestamp synch
static uint32_t last_synch_event_value_bs_units = 0;
static uint32_t synch_event_trigger_value = 0;
static nrf_timer_cc_channel_t timestamp_synch_event_compare_channel = (nrf_timer_cc_channel_t)0xFF;
static uint32_t timestamp_synch_offset = 0;

//Timestamp capture
static nrf_timer_cc_channel_t current_timestamp_capture_channel = (nrf_timer_cc_channel_t)0xFF;

//Synch event triggered tasks ppi
static bool timestamp_synch_trigger_ppi_allocated = false;
static nrf_ppi_channel_t timestamp_synch_trigger_ppi;

//Synch event callbacks
static timestamp_synch_execute_func_type timestamp_synch_execute_functions[TIMESTAMP_SYNCH_MAX_EXECUTE_FUNCTIONS];
static uint8_t timestamp_synch_execute_functions_num = 0;

#ifdef OMMO_TIMESTAMP_SYNCH_IN
  static uint32_t timestamp_synch_consequtive_misses = 0;
  static volatile uint8_t timestamp_synch_received_msk = 0;
  static uint32_t timestamp_synch_esb_ignore_count = 0;

  static bool timestamp_synch_lost = true;
  static uint8_t timestamp_lost_compare_channel;
  static timestamp_synch_lost_found_func_type timestamp_synch_lost_found_callback = NULL;

  static bool timestamp_synch_esb_enabled = true;
#endif

//Internal sample event globals
static nrfx_timer_event_handler_t sample_event_callbacks[TIMESTAMP_MAX_SAMPLE_EVENT_CALLBACKS];
static uint8_t sample_event_callbacks_num = 0;

static nrf_timer_cc_channel_t sample_event_timestamp_capture_channel;
static uint32_t sample_event_trigger_value, last_sample_event_trigger_value;
static nrf_timer_cc_channel_t sample_event_compare_channel = (nrf_timer_cc_channel_t)0xFF;

static void timestamp_on_synch_event(void)
{
#ifdef OMMO_TIMESTAMP_SYNCH_IN
    //Keep track of timestamp events since last synch
    if(timestamp_synch_received_msk&0x02)
    {
        timestamp_synch_consequtive_misses = 0;

        //Clear our timestamp received bit
        __atomic_and_fetch(&timestamp_synch_received_msk, ~0x02, __ATOMIC_SEQ_CST);
    }
    else
    {
        timestamp_synch_consequtive_misses++;
    }
#endif
}

static void timestamp_timer_event_handler(nrf_timer_event_t event_type, void* p_context)
{
    //Save last sample timestamp
    last_sample_event_trigger_value = sample_event_trigger_value;

#ifdef OMMO_TIMESTAMP_SYNCH_OUT
    //Roll forward to next sample event
    sample_event_trigger_value += OMMO_TIMESTAMP_SAMPLE_PERIOD;

    NRFX_CRITICAL_SECTION_ENTER();
    
    //Roll forward a whole synch event if we missed this isr (only happens during USB connection)
    while(calculate_wrapped_delta(nrfx_timer_capture(&timer_timestamp, sample_event_compare_channel) + 100, sample_event_trigger_value, UINT32_MAX) > 0)
        sample_event_trigger_value += TIMESTAMP_SYNCH_PERIOD;

    //Set up next sample event
    nrfx_timer_compare(&timer_timestamp, sample_event_compare_channel, sample_event_trigger_value, true);

    //Check if this was a synch event
    if(nrf_timer_event_check(timer_timestamp.p_reg, compare_events[timestamp_synch_event_compare_channel]))
    {
        timestamp_on_synch_event();

        //UARTE is actively transmitting the trigger value via DMA right now and sample_event_trigger_value has already been rolled forward by 1 cycle
        synch_event_trigger_value = sample_event_trigger_value + (OMMO_TIMESTAMP_SYNCH_PERIOD_MULT-1)*OMMO_TIMESTAMP_SAMPLE_PERIOD;

        //Setup next synch event
        nrfx_timer_compare(&timer_timestamp, timestamp_synch_event_compare_channel, synch_event_trigger_value, false);
        nrf_timer_event_clear(timer_timestamp.p_reg, compare_events[timestamp_synch_event_compare_channel]);

        //Send timestamp uart on next sample event (using the timestamp trigger value in the future)
        for(uint8_t i = 0; i < timestamp_synch_execute_functions_num; i++)
            timestamp_synch_execute_functions[i](synch_event_trigger_value);
    }

    NRFX_CRITICAL_SECTION_EXIT();

#else //OMMO_TIMESTAMP_SYNCH_IN or NO_TIMESTAMP_SYNCH

    #ifdef OMMO_TIMESTAMP_SAMPLE_PERIOD
    if(sample_event_compare_channel != (nrf_timer_cc_channel_t)0xFF)
    {
        NRFX_CRITICAL_SECTION_ENTER();

        //Setup sample time offset events based off of previous sample time
        for(uint8_t i=0; i<next_free_sample_time_offset_index; i++)
            nrfx_timer_compare(&timer_timestamp, sample_time_offset_cc_channel[i], sample_event_trigger_value + sample_time_offset_time[i], false);

        //Setup main sample event
        do
        {
            sample_event_trigger_value += OMMO_TIMESTAMP_SAMPLE_PERIOD;
        }//Roll forward if we missed this isr (only happens during USB connection)
        while(calculate_wrapped_delta(nrfx_timer_capture(&timer_timestamp, sample_event_compare_channel) + 100, sample_event_trigger_value, UINT32_MAX) > 0);

        //Set up next sample event
        nrfx_timer_compare(&timer_timestamp, sample_event_compare_channel, sample_event_trigger_value, true);

        //Check if this was a synch event
        if(nrf_timer_event_check(timer_timestamp.p_reg, compare_events[timestamp_synch_event_compare_channel]))
        {
            timestamp_on_synch_event();

            //UARTE is actively transmitting the trigger value via DMA right now and sample_event_trigger_value has already been rolled forward by 1 cycle
            synch_event_trigger_value = sample_event_trigger_value + (OMMO_TIMESTAMP_SYNCH_PERIOD_MULT-1)*OMMO_TIMESTAMP_SAMPLE_PERIOD;

            //Setup next synch event
            nrf_timer_event_clear(timer_timestamp.p_reg, compare_events[timestamp_synch_event_compare_channel]);
            nrfx_timer_compare(&timer_timestamp, timestamp_synch_event_compare_channel, synch_event_trigger_value, false);
            
            //Callback that a synch event just happened and the next will occur at the following BS timestamp trigger value in the future
            for(uint8_t i = 0; i < timestamp_synch_execute_functions_num; i++)
                timestamp_synch_execute_functions[i](synch_event_trigger_value + timestamp_synch_offset); //Convert value to BS units
        }

        NRFX_CRITICAL_SECTION_EXIT();
    }
    #endif

#endif

    //Call callback if supplied
    for(uint8_t i = 0; i < sample_event_callbacks_num; i++)
        sample_event_callbacks[i](event_type, p_context);
}

#ifdef OMMO_TIMESTAMP_SYNCH_IN
void timestamp_lost_timeout(void *p_context)
{
    //Timestamp synch lost, fire event
    if(timestamp_synch_lost_found_callback != NULL)
    {
        timestamp_synch_lost = true;
        timestamp_synch_lost_found_callback(!timestamp_synch_lost, timestamp_synch_esb_ignore_count > 0);
    
        #ifdef OMMO_DEBUG_ALIGNMENT_TEST
        nrfx_gpiote_out_task_disable(OMMO_DEBUG_ALIGNMENT_TEST);
        #endif
    }
}
#endif

void timestamp_init_with_no_sample_event()
{
    //Init timer
    nrfx_timer_config_t timer_cfg = NRFX_TIMER_DEFAULT_CONFIG;
    APP_ERROR_CHECK(nrfx_timer_init(&timer_timestamp, &timer_cfg, timestamp_timer_event_handler));
    nrfx_timer_enable(&timer_timestamp);

    //Allocate group for capture disables
    APP_ERROR_CHECK(nrfx_ppi_group_alloc(&ppi_group_disable_timestamp_capture));
 
    //No internal sampling
    sample_event_compare_channel = (nrf_timer_cc_channel_t)0xFF;
    sample_event_timestamp_capture_channel = (nrf_timer_cc_channel_t)0xFF;

    //Init timestamp debugging options
    timestamp_debug_init(0);

    initialized = true;
}

#ifdef OMMO_TIMESTAMP_SAMPLE_PERIOD
void timestamp_init_with_internal_sample_event(uint8_t isr_priority)
{
    //Init timer
    nrfx_timer_config_t timer_cfg = NRFX_TIMER_DEFAULT_CONFIG;
    timer_cfg.interrupt_priority = isr_priority;
    APP_ERROR_CHECK(nrfx_timer_init(&timer_timestamp, &timer_cfg, timestamp_timer_event_handler));
    nrfx_timer_enable(&timer_timestamp);

    //Setup sample event
    sample_event_compare_channel = timestamp_get_capture_compare_channel();

    //Grab next event compare time
    nrfx_timer_capture(&timer_timestamp, sample_event_compare_channel);
    sample_event_trigger_value = nrfx_timer_capture_get(&timer_timestamp, sample_event_compare_channel) + OMMO_TIMESTAMP_SAMPLE_PERIOD*4;

    //Setup first ISR/event
    nrfx_timer_compare(&timer_timestamp, sample_event_compare_channel, sample_event_trigger_value, true);

    //Setup synch event
    timestamp_synch_event_compare_channel = timestamp_get_capture_compare_channel();

    //Setup first ISR/event at the same time as the next sample event
    nrfx_timer_compare(&timer_timestamp, timestamp_synch_event_compare_channel, sample_event_trigger_value, false);
    nrf_timer_event_clear(timer_timestamp.p_reg, compare_events[timestamp_synch_event_compare_channel]);

    //Allocate group for capture disables
    APP_ERROR_CHECK(nrfx_ppi_group_alloc(&ppi_group_disable_timestamp_capture));
    APP_ERROR_CHECK(nrfx_ppi_group_enable(ppi_group_disable_timestamp_capture));

    //Setup PPI for disabling timestamp capture on sample timer evt
    APP_ERROR_CHECK(nrfx_ppi_channel_alloc(&ppi_channel_disable_timestamp_capture));
    APP_ERROR_CHECK(nrfx_ppi_channel_assign(ppi_channel_disable_timestamp_capture, timestamp_get_sample_event_address(), nrfx_ppi_task_addr_group_disable_get(ppi_group_disable_timestamp_capture)));
    APP_ERROR_CHECK(nrfx_ppi_channel_enable(ppi_channel_disable_timestamp_capture));

    //Init timestamp debugging options
    timestamp_debug_init(isr_priority);

    initialized = true;
}

void timestamp_uninit()
{
    nrfx_timer_disable(&timer_timestamp);
    nrfx_timer_uninit(&timer_timestamp);
}

#endif

void timestamp_init_with_external_sample_event(uint32_t sample_event)
{
    //Init timer
    nrfx_timer_config_t timer_cfg = NRFX_TIMER_DEFAULT_CONFIG;
    APP_ERROR_CHECK(nrfx_timer_init(&timer_timestamp, &timer_cfg, timestamp_timer_event_handler));
    nrfx_timer_enable(&timer_timestamp);

    //Allocate group for capture disables
    APP_ERROR_CHECK(nrfx_ppi_group_alloc(&ppi_group_disable_timestamp_capture));

    //Setup PPI for disabling timestamp capture on sample timer evt
    APP_ERROR_CHECK(nrfx_ppi_channel_alloc(&ppi_channel_disable_timestamp_capture));
    APP_ERROR_CHECK(nrfx_ppi_channel_assign(ppi_channel_disable_timestamp_capture, sample_event, nrfx_ppi_task_addr_group_disable_get(ppi_group_disable_timestamp_capture)));
    APP_ERROR_CHECK(nrfx_ppi_channel_enable(ppi_channel_disable_timestamp_capture));
    
    //Add capture for sample timestamp
    sample_event_timestamp_capture_channel = timestamp_add_capture_event(sample_event);

    APP_ERROR_CHECK(nrfx_ppi_group_enable(ppi_group_disable_timestamp_capture));

    //Init timestamp debugging options
    timestamp_debug_init(0);

    initialized = true;
}

static void timestamp_debug_init(uint8_t isr_priority)
{
    //Debug code for measuring the timestamp alignment between the basestation and main board
    #if defined OMMO_DEBUG_ALIGNMENT_TEST || defined OMMO_DEBUG_TXRX_TEST || defined OMMO_DEBUG_ABSOLUTE_ALIGN_TEST
    if (!nrfx_gpiote_is_init())
    {
        APP_ERROR_CHECK(nrfx_gpiote_init());
    }
    nrfx_gpiote_out_config_t test_output_config;
    test_output_config.action = NRF_GPIOTE_POLARITY_TOGGLE;
    test_output_config.init_state = NRF_GPIOTE_INITIAL_VALUE_LOW;
    test_output_config.task_pin = true;
    #endif

    #ifdef OMMO_DEBUG_ALIGNMENT_TEST
    APP_ERROR_CHECK(nrfx_gpiote_out_init(OMMO_DEBUG_ALIGNMENT_TEST, &test_output_config));
    nrfx_gpiote_out_task_enable(OMMO_DEBUG_ALIGNMENT_TEST);

    //Setup PPI for toggling the output pin    
    APP_ERROR_CHECK(nrfx_ppi_channel_fork_assign(ppi_channel_disable_timestamp_capture, nrfx_gpiote_out_task_addr_get(OMMO_DEBUG_ALIGNMENT_TEST))); //happens at sample time
    #endif

    #ifdef OMMO_DEBUG_TXRX_TEST
    APP_ERROR_CHECK(nrfx_gpiote_out_init(OMMO_DEBUG_TXRX_TEST, &test_output_config));
    nrfx_gpiote_out_task_enable(OMMO_DEBUG_TXRX_TEST);
    nrf_ppi_channel_t test_ppi;
    APP_ERROR_CHECK(nrfx_ppi_channel_alloc(&test_ppi));
    APP_ERROR_CHECK(nrfx_ppi_channel_assign(test_ppi,  nrf_radio_event_address_get(NRF_RADIO_EVENT_END), nrfx_gpiote_out_task_addr_get(OMMO_DEBUG_TXRX_TEST)));
    APP_ERROR_CHECK(nrfx_ppi_channel_enable(test_ppi));

    APP_ERROR_CHECK(nrfx_ppi_channel_alloc(&test_ppi));
    APP_ERROR_CHECK(nrfx_ppi_channel_assign(test_ppi,  nrf_radio_event_address_get(NRF_RADIO_EVENT_READY), nrfx_gpiote_out_task_addr_get(OMMO_DEBUG_TXRX_TEST)));
    APP_ERROR_CHECK(nrfx_ppi_channel_enable(test_ppi));
    #endif

    #ifdef OMMO_DEBUG_ABSOLUTE_ALIGN_TEST
    absolute_align_channel = timestamp_get_capture_compare_channel();
    nrfx_timer_compare(&timer_timestamp, absolute_align_channel, 10*16000000ul, false);

    APP_ERROR_CHECK(nrfx_gpiote_out_init(OMMO_DEBUG_ABSOLUTE_ALIGN_TEST, &test_output_config));
    nrfx_gpiote_out_task_enable(OMMO_DEBUG_ABSOLUTE_ALIGN_TEST);
    nrf_ppi_channel_t test_ppi;
    APP_ERROR_CHECK(nrfx_ppi_channel_alloc(&test_ppi));
    APP_ERROR_CHECK(nrfx_ppi_channel_assign(test_ppi,  nrfx_timer_event_address_get(&timer_timestamp, compare_events[absolute_align_channel]), nrfx_gpiote_out_task_addr_get(OMMO_DEBUG_ABSOLUTE_ALIGN_TEST)));
    APP_ERROR_CHECK(nrfx_ppi_channel_enable(test_ppi));
    #endif
}

uint32_t timestamp_add_sample_event_callback(nrfx_timer_event_handler_t func)
{
    if(sample_event_callbacks_num < TIMESTAMP_MAX_SAMPLE_EVENT_CALLBACKS)
    {
        sample_event_callbacks[sample_event_callbacks_num] = func;
        sample_event_callbacks_num++;

        return NRF_SUCCESS;
    }

    return NRF_ERROR_NO_MEM;
}

extern "C" uint32_t timestamp_get_current_timestamp()
{
    if(initialized)
    {
        if(current_timestamp_capture_channel == (nrf_timer_cc_channel_t)0xFF)
        {
            NRFX_CRITICAL_SECTION_ENTER();
            if(current_timestamp_capture_channel == (nrf_timer_cc_channel_t)0xFF)
                current_timestamp_capture_channel = timestamp_get_capture_compare_channel();
            NRFX_CRITICAL_SECTION_EXIT();
        }

        uint32_t rvalue = nrfx_timer_capture(&timer_timestamp, current_timestamp_capture_channel);
        return rvalue;
    }
    else
    {
        return 0;
    }
}

uint32_t timestamp_get_sample_event_address()
{
    ASSERT(sample_event_compare_channel != 0xFF);

    return nrfx_timer_event_address_get(&timer_timestamp, compare_events[sample_event_compare_channel]);
}

nrf_ppi_channel_t timestamp_add_task_on_sample_event(uint32_t task_address,  bool enable)
{
    if(!sample_trigger_ppi_allocated)
    {
        //Allocate new PPI and use primary tep
        APP_ERROR_CHECK(nrfx_ppi_channel_alloc(&sample_trigger_ppi));
        APP_ERROR_CHECK(nrfx_ppi_channel_assign(sample_trigger_ppi, timestamp_get_sample_event_address(), task_address));
        if (enable)
        {
            APP_ERROR_CHECK(nrfx_ppi_channel_enable(sample_trigger_ppi));
        }
    }
    else
    {
        //Use existing ppi form tep
        APP_ERROR_CHECK(nrfx_ppi_channel_fork_assign(sample_trigger_ppi, task_address));
        sample_trigger_ppi_allocated = false;
    }

    return sample_trigger_ppi;
}

uint32_t timestamp_get_last_sample_event_timestamp()
{
    if(sample_event_compare_channel != 0xFF) //Internal sampling
    {
        return last_sample_event_trigger_value + 1; //+1 simulates the PPI capture latency
    }
    else if(sample_event_timestamp_capture_channel != 0xFF) //External sampling
    {
        return timestamp_read_capture(sample_event_timestamp_capture_channel);
    }
    else //No sampling
    {
        APP_ERROR_HANDLER(NRF_ERROR_INVALID_STATE);
        return 0; //unreachable
    }
}

uint32_t timestamp_get_last_synch_event_timestamp()
{
    return synch_event_trigger_value;
}

nrf_timer_cc_channel_t timestamp_get_capture_compare_channel()
{
    //Find next available channel
    for(uint8_t i=0; i<TIMESTAMP_TIMER_NUM_CAPTURE_CHANNELS; i++)
    {
        if( ((0x01<<i) & capture_channels_used_mask) == 0x00)
        {
            capture_channels_used_mask |= (0x01<<i);
            return (nrf_timer_cc_channel_t)(NRF_TIMER_CC_CHANNEL0 + i);
        }
    }

    APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);

    //unreachable
    return (nrf_timer_cc_channel_t)-1;
}

void timestamp_release_capture_compare_channel(nrf_timer_cc_channel_t channel)
{
    uint8_t index = channel - NRF_TIMER_CC_CHANNEL0;
    capture_channels_used_mask &= ~(0x01<<index);
}

nrf_timer_cc_channel_t timestamp_add_capture_event(uint32_t trigger_event_addr, nrf_ppi_channel_t *ppi_channel_timestamp_capture_ptr)
{   
nrf_ppi_channel_t ppi_channel_timestamp_capture;
nrf_timer_cc_channel_t capture_channel;

    capture_channel = timestamp_get_capture_compare_channel();

    //Setup PPI for capturing timestamp at sample read event to CC0
    APP_ERROR_CHECK(nrfx_ppi_channel_alloc(&ppi_channel_timestamp_capture));
    APP_ERROR_CHECK(nrfx_ppi_channel_assign(ppi_channel_timestamp_capture, trigger_event_addr, nrfx_timer_task_address_get(&timer_timestamp, capture_tasks[capture_channel])));
    APP_ERROR_CHECK(nrfx_ppi_channel_enable(ppi_channel_timestamp_capture));

    if (ppi_channel_timestamp_capture_ptr)
        *ppi_channel_timestamp_capture_ptr = ppi_channel_timestamp_capture;

    return capture_channel;
}

nrf_timer_cc_channel_t timestamp_add_capture_event(uint32_t trigger_event_addr)
{
    return timestamp_add_capture_event(trigger_event_addr, NULL);
}

void timestamp_add_capture_event(nrf_ppi_channel_t ppi_channel_timestamp_capture, uint32_t trigger_event_addr, nrf_timer_cc_channel_t capture_channel)
{
    APP_ERROR_CHECK(nrfx_ppi_channel_assign(ppi_channel_timestamp_capture, trigger_event_addr, nrfx_timer_task_address_get(&timer_timestamp, capture_tasks[capture_channel])));
}

void timestamp_add_capture_event_fork(nrf_ppi_channel_t ppi_channel_timestamp_capture, nrf_timer_cc_channel_t capture_channel)
{
    APP_ERROR_CHECK(nrfx_ppi_channel_fork_assign(ppi_channel_timestamp_capture, nrfx_timer_task_address_get(&timer_timestamp, capture_tasks[capture_channel])));
}

nrf_timer_cc_channel_t timestamp_add_capture_event_with_disable(uint32_t trigger_event_addr)
{
nrf_timer_cc_channel_t capture_channel;

    capture_channel = timestamp_get_capture_compare_channel();

    //Actually add capture event
    timestamp_add_capture_event_with_disable(trigger_event_addr, capture_channel);

    return capture_channel;
}

void timestamp_add_capture_event_with_disable(uint32_t trigger_event_addr, nrf_timer_cc_channel_t capture_channel)
{
nrf_ppi_channel_t ppi_channel_timestamp_capture;

    //Setup PPI for capturing timestamp at encoder edge event
    APP_ERROR_CHECK(nrfx_ppi_channel_alloc(&ppi_channel_timestamp_capture));
    APP_ERROR_CHECK(nrfx_ppi_channel_assign(ppi_channel_timestamp_capture, trigger_event_addr, nrfx_timer_task_address_get(&timer_timestamp, capture_tasks[capture_channel])));
    APP_ERROR_CHECK(nrfx_ppi_channel_enable(ppi_channel_timestamp_capture));

    //Include in disable group
    APP_ERROR_CHECK(nrfx_ppi_channel_include_in_group(ppi_channel_timestamp_capture, ppi_group_disable_timestamp_capture));
}

nrf_timer_cc_channel_t timestamp_add_capture_event_with_disable(nrf_ppi_channel_t ppi_channel_timestamp_capture, uint32_t trigger_event_addr)
{
nrf_timer_cc_channel_t capture_channel;

    capture_channel = timestamp_get_capture_compare_channel();

    //Actually add capture event to the existing ppi channel
    APP_ERROR_CHECK(nrfx_ppi_channel_assign(ppi_channel_timestamp_capture, trigger_event_addr, nrfx_timer_task_address_get(&timer_timestamp, capture_tasks[capture_channel])));
    APP_ERROR_CHECK(nrfx_ppi_channel_enable(ppi_channel_timestamp_capture));

    //Include in disable group
    APP_ERROR_CHECK(nrfx_ppi_channel_include_in_group(ppi_channel_timestamp_capture, ppi_group_disable_timestamp_capture));

    return capture_channel;
}

void timestamp_add_capture_event_with_disable(nrf_ppi_channel_t ppi_channel_timestamp_capture, uint32_t trigger_event_addr, nrf_timer_cc_channel_t capture_channel)
{
    //Actually add capture event to the existing ppi channel
    APP_ERROR_CHECK(nrfx_ppi_channel_assign(ppi_channel_timestamp_capture, trigger_event_addr, nrfx_timer_task_address_get(&timer_timestamp, capture_tasks[capture_channel])));
    APP_ERROR_CHECK(nrfx_ppi_channel_enable(ppi_channel_timestamp_capture));

    //Include in disable group
    APP_ERROR_CHECK(nrfx_ppi_channel_include_in_group(ppi_channel_timestamp_capture, ppi_group_disable_timestamp_capture));
}

void timestamp_remove_ppi_from_group_disable(nrf_ppi_channel_t ppi_channel_timestamp_capture)
{
    APP_ERROR_CHECK(nrfx_ppi_channel_remove_from_group(ppi_channel_timestamp_capture, ppi_group_disable_timestamp_capture));
}

NRF_TIMER_Type* timestamp_get_timer_p_reg()
{
    return timer_timestamp.p_reg;
}

uint32_t timestamp_read_capture(nrf_timer_cc_channel_t channel)
{
    return nrfx_timer_capture_get(&timer_timestamp, channel);
}

void timestamp_reenable_disabled_captures()
{
    nrf_ppi_group_enable(ppi_group_disable_timestamp_capture);
}

uint32_t timestamp_get_compare_event_addr(nrf_timer_cc_channel_t compare_channel)
{
    return nrfx_timer_compare_event_address_get(&timer_timestamp, compare_channel);
}

void timestamp_set_compare_event_time(nrf_timer_cc_channel_t compare_channel, uint32_t value)
{
    nrfx_timer_compare(&timer_timestamp, compare_channel, value, false);
}

uint32_t timestamp_get_timer_clear_task()
{
    return nrfx_timer_task_address_get(&timer_timestamp, NRF_TIMER_TASK_CLEAR);
}

uint32_t timestamp_create_sample_event_offset_event(uint32_t offset)
{
    NRFX_ASSERT(next_free_sample_time_offset_index < OMMO_TIMESTAMP_MAX_SAMPLE_TIME_OFFSET_CHANNELS);

    uint8_t sample_time_offset_index = next_free_sample_time_offset_index;
    next_free_sample_time_offset_index++;

    sample_time_offset_cc_channel[sample_time_offset_index] = timestamp_get_capture_compare_channel();
    sample_time_offset_time[sample_time_offset_index] = offset;

    return nrfx_timer_event_address_get(&timer_timestamp, compare_events[sample_time_offset_cc_channel[sample_time_offset_index]]);
}

#ifdef OMMO_TIMESTAMP_SYNCH_IN

void timestamp_set_esb_synch_enabled(bool enabled)
{
    timestamp_synch_esb_enabled = enabled;
}

void timestamp_reset_synch_lost_timer()
{
    if(timestamp_synch_lost_found_callback != NULL)
    {
        //Reset timer to 1/2s out
        rtc_timer_reset_timeout(timestamp_lost_compare_channel);

        if(timestamp_synch_lost)
        {
            timestamp_synch_lost = false;
            timestamp_synch_lost_found_callback(!timestamp_synch_lost, timestamp_synch_esb_ignore_count > 0);
                
            #ifdef OMMO_DEBUG_ALIGNMENT_TEST
            nrfx_gpiote_out_task_enable(OMMO_DEBUG_ALIGNMENT_TEST);
            #endif
        }
    }
}

uint32_t timestamp_synch_get_num_misses()
{
    return timestamp_synch_consequtive_misses;
}

void timestamp_synch_recieved(uint32_t basestation_ts, uint32_t local_event_ts, timestamp_source_t source)
{
    uint32_t tx_delay = 0;

    //Flag that we have a synch pulse
    timestamp_synch_received_msk = 0xFF;

    switch(source)
    {
    
        case TIMESTAMP_SYNCH_SOURCE_ESB_NO_TS:
#ifndef OMMO_TIMESTAMP_DO_NOT_MATCH_BS_SAMPLE_TIME
          //basestation ts should be the last sych event trigger value in BS units
          basestation_ts = synch_event_trigger_value - OMMO_TIMESTAMP_SYNCH_PERIOD + timestamp_synch_offset + 1; //+1 is from shift below
          [[fallthrough]];
#else
          return;
#endif

        case TIMESTAMP_SYNCH_SOURCE_ESB:
            //Ignore ESB synch's if not enabled
            if(!timestamp_synch_esb_enabled)
                return;

            //Reset timestamp synch lost timer
            timestamp_reset_synch_lost_timer();

            //Ignore esb synch pulses if we are getting uart pulses 
            if(timestamp_synch_esb_ignore_count > 0)
            {
                timestamp_synch_esb_ignore_count--;
                if(timestamp_synch_lost_found_callback != NULL && timestamp_synch_esb_ignore_count == 0)
                    timestamp_synch_lost_found_callback(!timestamp_synch_lost, false);

                return;
            }

            tx_delay = TIMESTAMP_SYNCH_ESB_TX_DELAY;
            break;

        case TIMESTAMP_SYNCH_SOURCE_1WIRE:
            tx_delay = TIMESTAMP_SYNCH_1WIRE_TX_DELAY;
            goto synch_source_uart;

        case TIMESTAMP_SYNCH_SOURCE_UART:
            tx_delay = TIMESTAMP_SYNCH_UART_TX_DELAY;
synch_source_uart:
            {
                //Reset timestamp synch lost timer
                timestamp_reset_synch_lost_timer();

                //Ignore esb synch pulses if we are getting uart pulses
                bool was_wireless = !timestamp_is_synch_wired();
                timestamp_synch_esb_ignore_count = 16;
                if(timestamp_synch_lost_found_callback != NULL && was_wireless)
                    timestamp_synch_lost_found_callback(!timestamp_synch_lost, true);
            }
            break;

        default:
            APP_ERROR_CHECK(NRF_ERROR_INTERNAL);
            break;
    }

    //-1 accounts for the delay between the actual timer event and the PPI capture of the timer value
    basestation_ts -= 1;

    //Compute basestation/local offset
    uint32_t last_timestamp_synch_offset = timestamp_synch_offset;
    timestamp_synch_offset = (basestation_ts + tx_delay) - local_event_ts;
    if(last_timestamp_synch_offset - timestamp_synch_offset > 1000)
    {
        __NOP();
    }

    //Update synch and sample time to match basestation
    #ifndef OMMO_TIMESTAMP_DO_NOT_MATCH_BS_SAMPLE_TIME
    sample_event_trigger_value = basestation_ts + OMMO_TIMESTAMP_SAMPLE_PERIOD - timestamp_synch_offset;
    nrfx_timer_compare(&timer_timestamp, sample_event_compare_channel, sample_event_trigger_value, true);

#if OMMO_TIMESTAMP_SYNCH_PERIOD_MULT > 1
    //Sometimes synch events come in at a higher rate than the expected synch period
    //Ignore synch time changes until 1 OMMO_TIMESTAMP_SAMPLE_PERIOD before the expected synch period
    if((basestation_ts - last_synch_event_value_bs_units) > (OMMO_TIMESTAMP_SAMPLE_PERIOD*(OMMO_TIMESTAMP_SYNCH_PERIOD_MULT-1)))
#endif
    {
        last_synch_event_value_bs_units = basestation_ts;
        synch_event_trigger_value = basestation_ts + OMMO_TIMESTAMP_SYNCH_PERIOD - timestamp_synch_offset;
        nrfx_timer_compare(&timer_timestamp, timestamp_synch_event_compare_channel, synch_event_trigger_value, false);
    }
    #endif //OMMO_TIMESTAMP_DO_NOT_MATCH_BS_SAMPLE_TIME

    #ifdef OMMO_DEBUG_ABSOLUTE_ALIGN_TEST
    nrfx_timer_compare(&timer_timestamp, absolute_align_channel, 5*16000000ul - timestamp_synch_offset, false);
    #endif
}

uint32_t timestamp_get_last_synch_offset()
{
    return timestamp_synch_offset;
}

void timestamp_set_synch_lost_found_callback(timestamp_synch_lost_found_func_type func)
{
    //Save callback
    timestamp_synch_lost_found_callback = func;

    //Update cc channel to 1/2s out
    APP_ERROR_CHECK(rtc_timer_get_timeout_channel(&timestamp_lost_compare_channel, timestamp_lost_timeout, NULL));
    rtc_timer_set_timeout(timestamp_lost_compare_channel, MS_TO_RTC_TIMEOUT_GEN(500));
}

uint32_t timestamp_get_current_timestamp_basestation_units()
{
      return timestamp_get_current_timestamp() + timestamp_synch_offset;
}

void timestamp_get_last_sample_event_timestamp_basestation_units(uint32_t &timestamp, uint32_t &timestamp_offset)
{
      timestamp = timestamp_get_last_sample_event_timestamp() + timestamp_synch_offset;

      if(!(timestamp_synch_received_msk&0x01))
          timestamp_offset = 0x00;
      else if(timestamp_synch_offset == 0x00)
          timestamp_offset = 0x01;
      else
          timestamp_offset = timestamp_synch_offset;
      
      //Clear our timestamp received bit
      __atomic_and_fetch(&timestamp_synch_received_msk, ~0x01, __ATOMIC_SEQ_CST);
}

bool timestamp_is_synch_lost()
{
    ASSERT(timestamp_synch_lost_found_callback != NULL);

#ifndef RUN_WITHOUT_SYNC
    return timestamp_synch_lost;
#else
    return false;
#endif
}

bool timestamp_is_synch_wired()
{
    return timestamp_synch_esb_ignore_count > 0;
}

uint32_t timestamp_read_capture_basestation_units(nrf_timer_cc_channel_t channel)
{
    return timestamp_read_capture(channel) + timestamp_synch_offset;
}

uint32_t timestamp_get_basestation_offset()
{
    return timestamp_synch_offset;
}

#endif

uint32_t timestamp_get_timestamp_synch_event_address()
{
    ASSERT(timestamp_synch_event_compare_channel != 0xFF);

    return nrfx_timer_event_address_get(&timer_timestamp, compare_events[timestamp_synch_event_compare_channel]);
}

uint32_t timestamp_get_next_synch_event_timestamp()
{
    return synch_event_trigger_value;
}

uint32_t timestamp_add_task_on_timestamp_synch_event(uint32_t task_address)
{
    uint32_t error = NRF_SUCCESS;

    if(!timestamp_synch_trigger_ppi_allocated)
    {
        //Allocate new PPI and use primary tep
        error = nrfx_ppi_channel_alloc(&timestamp_synch_trigger_ppi);
        if(error == NRF_SUCCESS) error = nrfx_ppi_channel_assign(timestamp_synch_trigger_ppi, timestamp_get_timestamp_synch_event_address(), task_address);
        if(error == NRF_SUCCESS) error = nrfx_ppi_channel_enable(timestamp_synch_trigger_ppi);
    }
    else
    {
        //Use existing ppi form tep
        error = nrfx_ppi_channel_fork_assign(timestamp_synch_trigger_ppi, task_address);
        timestamp_synch_trigger_ppi_allocated = false;
    }

    return error;
}

uint32_t timestamp_add_timestamp_synch_event_callback(timestamp_synch_execute_func_type func)
{
    if(timestamp_synch_execute_functions_num < TIMESTAMP_SYNCH_MAX_EXECUTE_FUNCTIONS)
    {
        timestamp_synch_execute_functions[timestamp_synch_execute_functions_num] = func;
        timestamp_synch_execute_functions_num++;

        return NRF_SUCCESS;
    }

    return NRF_ERROR_NO_MEM;
}
