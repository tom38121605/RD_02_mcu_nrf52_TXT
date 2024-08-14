
#include "nrf_drv_timer.h"
#include "port_timer_blink.h"

extern nrf_drv_timer_t TIMER_BLINK;

volatile bool timer_blink_timeout = false;

bool timer_blink_initiated = 0;
/**
 * @brief Handler for timer events.
 */
void timer_blink_event_handler(nrf_timer_event_t event_type, void* p_context)
{
    if (event_type == NRF_TIMER_EVENT_COMPARE0)
    {
        timer_blink_timeout = true;
        port_timer_blink_disable();
    }
}


/**
 * @brief Function for main application entry.
 */
void port_timer_blink_init(void)
{
    uint32_t err_code = NRF_SUCCESS;

    //Configure TIMER_BLINK 
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    err_code = nrf_drv_timer_init(&TIMER_BLINK, &timer_cfg, timer_blink_event_handler);
    APP_ERROR_CHECK(err_code);

    timer_blink_initiated = 1;
}

bool port_timer_is_initiated(void)
{ 
    return timer_blink_initiated;
}

bool port_timer_has_event(void)
{ 
    return timer_blink_timeout;
}

void port_timer_clear_event(void)
{ 
    timer_blink_timeout = false;
}

void port_timer_blink_disable(void)
{
    nrf_drv_timer_disable(&TIMER_BLINK);
}

void port_timer_blink_enable(void)
{
    nrf_drv_timer_enable(&TIMER_BLINK);
}

void port_timer_blink_cfg(uint32_t delay)
{
    //uint32_t time_ms = delay; //Time(in miliseconds) between consecutive compare events.
    uint32_t time_ticks;

    if(!port_timer_is_initiated())
    {
        port_timer_blink_init();
    }

    port_timer_blink_disable();

    time_ticks = nrf_drv_timer_ms_to_ticks(&TIMER_BLINK, delay);

    nrf_drv_timer_extended_compare(
         &TIMER_BLINK, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);

    port_timer_blink_enable();
}