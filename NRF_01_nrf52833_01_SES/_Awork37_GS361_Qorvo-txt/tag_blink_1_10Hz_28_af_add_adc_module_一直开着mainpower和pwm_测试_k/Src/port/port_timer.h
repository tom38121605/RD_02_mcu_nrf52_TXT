/*! ----------------------------------------------------------------------------
 * @file    port_timer.h
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

#ifndef _PORT_TIMER_H_
#define _PORT_TIMER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/*! ----------------------------------------------------------------------------
 * @fn port_timer_start
 * @brief Start a timer and save the current ime
 * @param[out] p_timestamp pointer of current system timestamp
 */
void port_timer_start(volatile uint32_t * p_timestamp);

/*! ----------------------------------------------------------------------------
 * @fn port_timer_check
 * @brief Checks if the timer is time milliseconds past the timestamp
 * @return true if time has passed, false otherwise
 */
bool port_timer_check(uint32_t timestamp, uint32_t time);

#ifdef __cplusplus
}
#endif

#endif /* _PORT_TIMER_H_ */
