/*! ----------------------------------------------------------------------------
 * @file    deca_mutex.c
 * @brief   Platform specific irq functions used by deca_driver.
 *
 * Platform specific irq functions used by deca_driver, defined in
 * deca_driver/deca_device_api.h
 *
 * @attention
 *
 * Copyright 2020 (c) DecaWave Ltd, Dublin, Ireland.
 * All rights reserved.
 *
 * @author DecaWave
 */

#include "port_irq.h"
#include "deca_device_api.h"

inline decaIrqStatus_t decamutexon(void)
{
    decaIrqStatus_t s = (decaIrqStatus_t) port_irq_dw_irq_enabled();
    if (s)
    {
        port_irq_disable_dw_irq();
    }
    return s;
}

inline void decamutexoff(decaIrqStatus_t s)
{
    if (s)
    {
        port_irq_enable_dw_irq();
    }
}
