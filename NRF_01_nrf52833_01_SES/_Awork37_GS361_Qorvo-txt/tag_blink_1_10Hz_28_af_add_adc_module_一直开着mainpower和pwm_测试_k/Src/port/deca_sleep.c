/*! ----------------------------------------------------------------------------
 * @file    deca_sleep.c
 * @brief   Platform specific sleep functions used by deca_driver.
 *
 * Platform specific sleep functions used by deca_driver, defined in
 * deca_driver/deca_device_api.h
 *
 * @attention
 *
 * Copyright 2020 (c) DecaWave Ltd, Dublin, Ireland.
 * All rights reserved.
 *
 * @author DecaWave
 */

#include "port.h"
#include "deca_device_api.h"

inline void deca_sleep(unsigned int time_ms)
{
    port_wait(time_ms);
}
