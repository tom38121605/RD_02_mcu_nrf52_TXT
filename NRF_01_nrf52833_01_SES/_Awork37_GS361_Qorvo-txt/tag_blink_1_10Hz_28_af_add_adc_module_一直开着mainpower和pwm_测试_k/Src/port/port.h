/*! ----------------------------------------------------------------------------
 * @file  port.h
 * @brief Various platform specific functions
 *
 * @attention
 * Copyright 2015-2019 (c) DecaWave Ltd, Dublin, Ireland.
 * All rights reserved.
 *
 * @author DecaWave
 */

#ifndef _PORT_H_
#define _PORT_H_

#include "nrf_delay.h"
#include <boards.h>

#ifdef __cplusplus
extern "C" {
#endif

/*! ----------------------------------------------------------------------------
 * Two Way Ranging internal delay
 * Poll_Rx to Resp_Tx internal delay in Us
 * This delay is MCU dependent. The user shall modify the value so the TWR runs
 * as expected and to ensure communication between initiator and responder
 * Note that the delay below could potentially be optimised if fast TWR is 
 * required
 * DWM1004 - STM32L041G6 : 500 us 
 * DWS1000 - Nucleo F429ZI (STM32F429ZIT6) : 500 us
 * DWM1001 - NRF5282 : 900 us
 */
#define MCU_DEPENDENT_TWR_DELAY 900


/*! ----------------------------------------------------------------------------
 * @fn port_wait()
 * @brief Wait for a given amount of time
 *
 * @param time_ms time to wait in milliseconds
 */
void port_wait(unsigned int time_ms);

/*! ----------------------------------------------------------------------------
 * @fn    port_reset_dw1000
 * @brief Reset the DW1000 using the the RSTn pin
 */
void port_reset_dw1000();

/*! ----------------------------------------------------------------------------
 * @fn    port_wakeup_dw1000
 * @brief Wake up the DW1000 from SLEEP or DEEPSLEEP
 */
void port_wakeup_dw3000(void);

#ifdef __cplusplus
}
#endif

#endif /* _PORT_H_ */
