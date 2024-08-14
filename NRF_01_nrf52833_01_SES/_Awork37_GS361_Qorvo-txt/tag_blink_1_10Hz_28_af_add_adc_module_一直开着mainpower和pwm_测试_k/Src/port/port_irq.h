/*! ----------------------------------------------------------------------------
 * @file    port_irq.h
 * @brief   Platform specific interrupt control functions
 *
 * @attention
 *
 * Copyright 2019 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */

#ifndef _PORT_IRQ_H_
#define _PORT_IRQ_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include "nrf_drv_gpiote.h"

void dw1000_irq_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);

/*! ----------------------------------------------------------------------------
 * @fn port_irq_enable_dw_irq()
 * @brief Low level abstract function to enable interrupts from the DW_IRQ pin
 */
void port_irq_enable_dw_irq(void);

/*! ----------------------------------------------------------------------------
 * @fn port_irq_disable_dw_irq()
 * @brief Low level abstract function to disable interrupts from the DW_IRQ pin
 */
void port_irq_disable_dw_irq(void);

/*! ----------------------------------------------------------------------------
 * @fn deca_irq_dw_irq_enabled()
 * @brief Low level abstract function check if interrupts from the DW_IRQ pin
 * are enabled
 * @return true if DW_IRQ interrupts enabled, false if disabled
 */
bool port_irq_dw_irq_enabled(void);

#ifdef __cplusplus
}
#endif

#endif /* _PORT_IRQ_H_ */
