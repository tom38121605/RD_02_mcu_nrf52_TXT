/**
 * @file      deca_phy.c
 *
 * @brief     Decawave PHY layer
 *
 * @author    Decawave
 *
 * @attention Copyright 2017-2019 (c) Decawave Ltd, Dublin, Ireland.
 *            All rights reserved.
 *
 */

//#include <stdint.h>
//#include <string.h>
#include "deca_phy.h"
#include "port_spi.h"
#include "deca_device_api.h"
#include "deca_regs.h"
//#include "app.h"

#include "assert.h"

//The table below specifies the default TX spectrum configuration parameters.
//const dwt_txconfig_t txSpectrumConfig[2 /* channel */] = {
//        /* Values for the PG_DELAY and TX_POWER registers reflect the bandwidth and power of the spectrum at the current
//         * temperature. These values can be calibrated prior to taking reference measurements. */
//        { 0x36, /* PG delay. */ 0x80808080 /* TX power. */},
//        { 0x36, /* PG delay. */ 0x80808080 /* TX power. */}
//};

int phy_init(void)
{
    // DW_CODE_FLOW : CONFIG : Initialize Chip
    // Reset the DW3000 (might be needed if not getting here from POWER ON)
    //dwt_softreset();
    //dwt_calibratesleepcnt();

        port_spi_set_fastrate();

    uint8_t timeout = 4; // 4ms timeout period
    //while (!dwt_checkidlerc() && timeout--) /* Need to make sure DW IC is in IDLE_RC before proceeding */
    while (!dwt_checkidlerc()) /* Need to make sure DW IC is in IDLE_RC before proceeding */
    { 
        deca_sleep(1);
    };

    int result = dwt_initialise(DWT_DW_INIT | DWT_READ_OTP_PID | DWT_READ_OTP_LID);

    // if using auto CRC check (DWT_INT_RFCG and DWT_INT_RFCE) are used instead of DWT_INT_RDFR flag
    // other errors which need to be checked (as they disable receiver) are (*) below
    //
    //dwt_setinterrupt(DWT_INT_RXOVRR | DWT_INT_TFRS | DWT_INT_RFCG | (DWT_INT_ARFE | DWT_INT_RFSL | DWT_INT_SFDT | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFTO /*| DWT_INT_RXPTO*/), 1);

    dwt_setinterrupt(SYS_STATUS_TXFRS_BIT_MASK | SYS_STATUS_RXFCG_BIT_MASK  \
                                  | (SYS_STATUS_ARFE_BIT_MASK | SYS_STATUS_RXFSL_BIT_MASK   \
                                  | SYS_STATUS_RXSTO_BIT_MASK | SYS_STATUS_RXPHE_BIT_MASK   \
                                  | SYS_STATUS_RXFCE_BIT_MASK | SYS_STATUS_RXFTO_BIT_MASK), DWT_ENABLE_INT_ONLY);
    //this is platform dependent - only program if DW EVK/EVB
    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);    //configure the GPIOs which control the LEDs on HW

    //check all is OK
    if (DWT_SUCCESS != result)
    {
        return (-1);   // device initialise has failed
    }

    // Initialize Callbacks and app Events pointers
    dwt_setcallbacks(NULL, NULL, NULL, NULL, NULL, NULL);   //Initialize app callbacks to none


    return (0);
}

