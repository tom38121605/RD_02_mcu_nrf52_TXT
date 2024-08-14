/*! ----------------------------------------------------------------------------
 * @file    port_timer.c
 * @brief   Abstraction for timer functions
 *
 * @attention
 *
 * Copyright 2019 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */

#include "port_timer.h"
#include <nrfx.h>

uint32_t SysTickCount ;

/* SysTick interrupt Handler. */
void SysTick_Handler(void)  
{
    SysTickCount++;
}

/*! ----------------------------------------------------------------------------
 * @fn port_timer_start
 * @brief Start a timer and save the current ime
 * @param[out] p_timestamp pointer of current system timestamp
 */
void port_timer_start(volatile uint32_t * p_timestamp)
{
    *p_timestamp = SysTickCount;
}

/*! ----------------------------------------------------------------------------
 * @fn port_timer_check
 * @brief Checks if the timer is time milliseconds past the timestamp
 * @return true if time has passed, false otherwise
 */
bool port_timer_check(uint32_t timestamp, uint32_t time)
{
    uint32_t current_time = SysTickCount;
    uint32_t passed_time = (current_time >= timestamp) ?
        current_time - timestamp:
        0xffffffffUL - timestamp + current_time;

    return (passed_time >= time);
} 
 