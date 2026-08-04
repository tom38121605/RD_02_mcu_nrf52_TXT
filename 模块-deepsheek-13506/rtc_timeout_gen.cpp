#include "rtc_timeout_gen.hpp"
        
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "ommo_config.h"
#include "sdk_config.h"
#include "trace.h"

#ifdef OMMO_RTC_TIMEOUT_GEN_RTC
static nrfx_rtc_t timeout_rtc = OMMO_RTC_TIMEOUT_GEN_RTC;
#else
static nrfx_rtc_t timeout_rtc = {0};
#endif
static uint8_t capture_channels_used_mask = 0;
static rtc_timeout_func_type callback_funcs[8] = {0};
static void * callback_context[8] = {0};
static volatile nrfx_atomic_flag_t timeout_running[8] = {0};
static uint32_t last_timeout_used[8] = {0};

void rtc_timer_handler(nrfx_rtc_int_type_t int_type, void * p_context)
{
    //Only call callback from either timer OR 
    if(nrfx_atomic_flag_clear_fetch(&timeout_running[int_type]))
    {
        nrfx_rtc_cc_disable(&timeout_rtc, int_type);

        rtc_timeout_func_type callback = callback_funcs[int_type];
        if(callback)
        {
            OMMO_TRACE_ISR_ACTION_CALLBACK(TRACE_ISR_RTC_TIMEOUT, callback);
            timeout_running[int_type] = false;
            callback(callback_context[int_type]);
        }
    }
}

uint32_t rtc_timer_disable_timeout(uint8_t channel)
{
    if(channel >= timeout_rtc.cc_channel_count)
        return NRF_ERROR_INVALID_ADDR;

    if(nrfx_atomic_flag_clear_fetch(&timeout_running[channel]))
        nrfx_rtc_cc_disable(&timeout_rtc, channel); //ignore TIMEOUT err if ISR is pending

    return NRF_SUCCESS;    
}

uint32_t rtc_timer_get_timeout_channel(uint8_t *channel, rtc_timeout_func_type callback, void *p_context)
{
    //check if we have the rtc instance assigned
    if(timeout_rtc.p_reg == NULL)
        return NRF_ERROR_NOT_FOUND;

    //Find next available channel
    for(uint8_t i=0; i<timeout_rtc.cc_channel_count; i++)
    {
        if( ((0x01<<i) & capture_channels_used_mask) == 0x00)
        {
            capture_channels_used_mask |= (0x01<<i);
            *channel = i;
            callback_funcs[i] = callback;
            callback_context[i] = p_context;
            return NRF_SUCCESS;
        }
    }

    return NRF_ERROR_NO_MEM;
}

void rtc_timer_release_timeout_channel(uint8_t channel)
{
    capture_channels_used_mask &= ~(0x01<<channel);
}


uint32_t rtc_timer_set_timeout(uint8_t channel, uint32_t timeout)
{
    ret_code_t ret_code;

    if (channel >= timeout_rtc.cc_channel_count)
        return NRF_ERROR_INVALID_ADDR;

    if (timeout_running[channel])
        return NRF_ERROR_INVALID_STATE;

    last_timeout_used[channel] = timeout;
    timeout_running[channel]   = true;

    CRITICAL_REGION_ENTER();
    ret_code = nrfx_rtc_cc_set(&timeout_rtc, channel, nrfx_rtc_counter_get(&timeout_rtc) + timeout, true);
    CRITICAL_REGION_EXIT();

    return ret_code;
}

uint32_t rtc_timer_reset_timeout(uint8_t channel)
{
    ret_code_t ret_code;

    if(channel >= timeout_rtc.cc_channel_count)
        return NRF_ERROR_INVALID_ADDR;

    timeout_running[channel] = true;

    CRITICAL_REGION_ENTER();
    
    ret_code =  nrfx_rtc_cc_set(&timeout_rtc, channel, nrfx_rtc_counter_get(&timeout_rtc) + last_timeout_used[channel], true);
    CRITICAL_REGION_EXIT();

    return ret_code;
}

void rtc_timer_init()
{
    //Initialize timeout timer
    nrfx_rtc_config_t timer_cfg;
    timer_cfg.prescaler          = RTC_FREQ_TO_PRESCALER(NRFX_RTC_DEFAULT_CONFIG_FREQUENCY);
    timer_cfg.interrupt_priority = NRFX_RTC_DEFAULT_CONFIG_IRQ_PRIORITY;
    timer_cfg.reliable           = NRFX_RTC_DEFAULT_CONFIG_RELIABLE;
    timer_cfg.tick_latency       = NRFX_RTC_US_TO_TICKS(NRFX_RTC_MAXIMUM_LATENCY_US, NRFX_RTC_DEFAULT_CONFIG_FREQUENCY);

    APP_ERROR_CHECK(nrfx_rtc_init(&timeout_rtc, &timer_cfg, rtc_timer_handler, NULL));
    nrfx_rtc_enable(&timeout_rtc);
}