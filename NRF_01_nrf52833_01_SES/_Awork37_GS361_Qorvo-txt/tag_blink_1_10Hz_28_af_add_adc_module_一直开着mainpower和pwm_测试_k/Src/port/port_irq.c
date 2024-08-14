/*! ----------------------------------------------------------------------------
 * @file    port_irq.c
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
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "port_stdio.h"
#include "port_irq.h"
#include "port.h"
#include "cmd_task.h"
#include "deca_device_api.h"
#include "production_uwb.h"

static char str[256];

static bool irq_enabled = 0 ;

/*! ----------------------------------------------------------------------------
 * @fn dw1000_irq_handler
 * @brief dw1000 Irq Handler
 * 
 * This handler is called when DW1000 Irq is raised.
 * It calls the DW1000 interrupt routine service function
 *
 * @param[in] GPIO pin configured to received DW1000 interrupt
 * @param[in] Polarity of GPIO Pin
 */

volatile uint8_t isr_flag = 0;
void dw1000_irq_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    isr_flag = 1;
    //dwt_isr();
}

/*! ----------------------------------------------------------------------------
 * @fn port_irq_enable_dw_irq()
 * @brief Low level abstract function to enable interrupts from the DW_IRQ pin
 */
void port_irq_enable_dw_irq(void)
{
//    ret_code_t err_code;
    
//    nrf_drv_gpiote_in_config_t in_config = NRFX_GPIOTE_RAW_CONFIG_IN_SENSE_LOTOHI(true);
//    err_code = nrf_drv_gpiote_in_init(DW1000_IRQ, &in_config, dw1000_irq_handler);
//    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(DW1000_IRQ, true);

    irq_enabled = 1 ;
}

/*! ----------------------------------------------------------------------------
 * @fn port_irq_disable_dw_irq()
 * @brief Low level abstract function to disable interrupts from the DW_IRQ pin
 */
void port_irq_disable_dw_irq(void)
{
    nrf_drv_gpiote_in_event_disable(DW1000_IRQ);
    
//    nrf_drv_gpiote_in_uninit(DW1000_IRQ);

    irq_enabled = 0 ;
}

/*! ----------------------------------------------------------------------------
 * @fn deca_irq_dw_irq_enabled()
 * @brief Low level abstract function check if interrupts from the DW_IRQ pin
 * are enabled
 * @return true if DW_IRQ interrupts enabled, false if disabled
 */
bool port_irq_dw_irq_enabled(void)
{
    return (bool) irq_enabled;
}
