/*! ----------------------------------------------------------------------------
 * @file  port.c
 * @brief Various platform specific functions
 *
 * @attention
 * Copyright 2015-2019 (c) DecaWave Ltd, Dublin, Ireland.
 * All rights reserved.
 *
 * @author DecaWave
 */

#include "port.h"

/*! ----------------------------------------------------------------------------
 * @fn port_wait()
 * @brief Wait for a given amount of time
 *
 * @param time_ms time to wait in milliseconds
 */
void port_wait(unsigned int time_ms)
{
    nrf_delay_ms(time_ms);
}

/*! ----------------------------------------------------------------------------
 * @fn    port_reset_dw1000
 * @brief Reset the DW1000 using the the RSTn pin
 */
void port_reset_dw1000()
{  
    nrf_gpio_cfg_output(DWIC_RST);
    nrf_gpio_pin_clear(DWIC_RST);

    port_wait(100);

    nrf_gpio_cfg_input(DWIC_RST, NRF_GPIO_PIN_NOPULL);

    /* wait for RSTn to go up XXXXXXXXXXXXXXXXXXXX  */
//    while(nrf_gpio_pin_read(DWIC_RST) != 1)
//    {
//        port_wait(1);
//    }

    port_wait(5); //(5)
}

/*! ----------------------------------------------------------------------------
 * @fn    port_wakeup_dw1000
 * @brief Wake up the DW1000 from SLEEP or DEEPSLEEP
 */
void port_wakeup_dw3000(void)
{
    nrf_gpio_pin_clear(SPI_CS_PIN);

    port_wait(1);
    nrf_gpio_pin_set(SPI_CS_PIN);

    while(nrf_gpio_pin_read(DWIC_RST) != 1)
    {
        port_wait(1);
    }

}