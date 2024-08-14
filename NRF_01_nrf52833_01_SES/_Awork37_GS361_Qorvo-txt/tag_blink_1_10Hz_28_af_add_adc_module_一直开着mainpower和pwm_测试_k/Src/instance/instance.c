/*
 * @file       instance.c
 *
 * @brief      Application level message exchange for ranging demo
 *
 * @author     Decawave Software
 *
 * @attention  Copyright 2018 (c) DecaWave Ltd, Dublin, Ireland.
 *             All rights reserved.
 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "deca_device_api.h"
#include "deca_regs.h"
#include "port_config.h"
#include "production_uwb.h"
#include "port_stdio.h"
#include "port_spi.h"
#include "port_irq.h"
#include "port.h"
#include "cmd_task.h"
#include "instance.h"
//#include "main.h"
#include "deca_phy.h"

app_cfg_t app;

/*******************************************************************************
 * Experimental values in ms of time to restart blink
 **/
/* Observed that timer interrupt is not serviced for the default value of
LOWPOWER_RESTART_TIME. So LOWPOWER_RESTART_TIME is configured as 9 */
#define LOWPOWER_RESTART_TIME           15 //9
#define NOSLEEP_RESTART_TIME            12 //1
#define NUM_INST                        1

#define DWT_SIG_RX_TIMEOUT              4

static char str[256];

enum inst_states
{
   TA_INIT,
   TA_SLEEP_DONE,
   TA_TXBLINK_WAIT_SEND
};

/*The table below specifies the default TX spectrum configuration parameters...
  this has been tuned for DW EVK hardware units
  the table is set for smart power - see below in the instance_config function
  how this is used when not using smart power
*/

const uint16 rfDelays[2] = {
    (uint16) ((DWT_PRF_16M_RFDLY/ 2.0f) * (float)1e-9 / DWT_TIME_UNITS),//PRF 16
    (uint16) ((DWT_PRF_64M_RFDLY/ 2.0f) * (float)1e-9 / DWT_TIME_UNITS)
};
// -----------------------------------------------------------------------------

instance_data_t instance_data ;

// -----------------------------------------------------------------------------
// Functions
// -----------------------------------------------------------------------------

/*
 * @fn   instancesettagaddress
 * @param  *inst
 *
 * */
void instancesettagaddress(instance_data_t *inst)
{
    uint8 eui64[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    param_block_t * config = port_config_get();

    if(config->tagIDset == 0)
    {
        //we want exact Address representation in Little- and Big- endian
        uint32 id= ulong2littleEndian(dwt_getpartid());
        memcpy(eui64, &id, sizeof(uint32));

        //we want exact Address representation in Little- and Big- endian
        id= ulong2littleEndian(dwt_getlotid());
        memcpy(&eui64[4], &id, sizeof(uint32));

        //set source address into the message structure
        memcpy(inst->msg.tagID, eui64, ADDR_BYTE_SIZE);
        memcpy(config->tagID, eui64, ADDR_BYTE_SIZE);
    }
    else
    {
        //set source address into the message structure
        memcpy(inst->msg.tagID, config->tagID, ADDR_BYTE_SIZE);
    }
}

// -----------------------------------------------------------------------------
#if NUM_INST != 1
#error These functions assume one instance only
#else
 
extern uint8_t rx_enabled;
extern uint8_t isr_flag;
void dwt_isr_task(void)
{
    static uint8_t cnt = 0;

    if(isr_flag)
    {
//        sprintf(str,"debug info: isr task %d\r\n", cnt++);
//        port_dbg_print((uint8_t*)str, strlen(str));
        dwt_isr();
        isr_flag = 0;
        if(rx_enabled)
        {
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
            //cmd_uwb_rx_enable();
        }
    }
}
/* @fn  function to initialise instance structures
 * @brief  Returns 0 on success and -1 on error
 * */
int instance_init()
{
//    int result ;
    //param_block_t * config = port_config_get();


//    sprintf(str,"instance_init starting...\r\n");
//    port_dbg_print((uint8_t*)str, strlen(str));

    instance_data.testAppState = TA_INIT ;

    // Reset the IC (might be needed if not getting here from POWER ON)
    port_reset_dw1000();
    phy_init();

//    // debug: check if xtaltrim value is correct. 
//    uint8 xtaltrim = dwt_getxtaltrim();     
//    sprintf(str,"xtaltrim = 0x%02x\r\n", xtaltrim);
//    port_dbg_print((uint8_t*)str, strlen(str));

    // Reading Unique ID
    instancesettagaddress(&instance_data);

    //configure the on wake parameters (upload the IC config settings)
    dwt_configuresleep(DWT_CONFIG , DWT_PRESRVE_SLP| DWT_WAKE_CS | DWT_SLP_EN);

    // this will set 0 if zero and 1 otherwise
    //dwt_setsmarttxpower( (config->smartPowerEn != 0) );

    dwt_entersleepaftertx(0); // sleep disabled

//    if (DWT_SUCCESS != result)
//    {
//        return (DWT_ERROR) ;        // device initialize has failed
//    }

    // required for production test
    // need to re-enable receiver after first receive when using URX
    dwt_setcallbacks(instance_txcallback, instance_rx, instance_rx, instance_rx, NULL, NULL);
    
    cmd_task_add(dwt_isr_task);

    instance_data.frame_sn = 0;
    instance_data.timeron = 0;
    instance_data.event[0] = 0;
    instance_data.event[1] = 0;
    instance_data.eventCnt = 0;

    return 0 ;
}


/**
 * @fn  instance_config
 * @brief function to allow application configuration be passed into instance
 *        and affect underlying device operation
 *
 * */
void instance_config(param_block_t * config)
{
    dwt_txconfig_t  configTx ;
    trx_config_t * cur_trx_config;
    uint16_t rxa_delay;
    uint16_t txa_delay;

    cur_trx_config = current_ch_cfg();
    if (!cur_trx_config)
            return;

    dwt_configure(&config->dwt_config) ;

    configTx.power =cur_trx_config->ref_tx_pwr;
    configTx.PGdly =cur_trx_config->ref_pgdly;
    rxa_delay = cur_trx_config->rxa_ant_delay;
    txa_delay = cur_trx_config->txa_ant_delay;

    /* smart power is always used*/
    //if(config->smartPowerEn == 0)
    {
        /* when smart TX power is not used then the low nibble should be copied
           into the whole 32 bits*/
        uint32 pow = configTx.power & 0xff;
        configTx.power = (pow << 24) + (pow << 16) + (pow << 8) + pow;
    }
    dwt_configuretxrf(&configTx);
    dwt_setrxantennadelay(rxa_delay);
    dwt_settxantennadelay(txa_delay);
}

/**
 * @fn  instance_txcallback
 * @brief  Tx callback (if TX irq present)
 *
 * */
void instance_txcallback(const dwt_cb_data_t *txd)
{
    //empty function
}

/**
 * @fn  instance_rx
 * @brief Currently using only one RX call back as
 * it is only used to re-enable receiver, regardless
 * of RX event
 *
 * */
void instance_rx(const dwt_cb_data_t *rxd)
{ 
//    sprintf(str,"in instance_rx\r\n");
//    port_dbg_print((uint8_t*)str, strlen(str));

//    dwt_forcetrxoff();
//    dwt_setrxtimeout(0);
//    dwt_rxenable(0);

}

#endif

/*****************************************************************************
 *  @fn          ulong2littleEndian
 *  @brief  convert input value uint32 to little-endian order
 *                  assumes only little-endian and big-endian presents.
 *                  (not support middle-endian)
 */
uint32 ulong2littleEndian(uint32 prm)
{
    union {
      uint32  ui;
      uint8   uc[sizeof(uint32)];
    } v;

    v.ui = 1;        //check endian
    if(v.uc[0] == 1) {
        v.ui=prm;        //little
    } else {
        v.ui=prm;        //
        SWAP(v.uc[0], v.uc[3]);
        SWAP(v.uc[1], v.uc[2]);
    }
    return v.ui;
}


/*! ----------------------------------------------------------------------------
 * @fn inittestapplication
 *
 */
//static
int inittestapplication(void)
{
    int32_t result = 0;
    //int devID; // Decawave Device ID
    uint32_t devID = 0xffffffff; // Decawave Device ID

    port_irq_disable_dw_irq();

    // Set SPI clock to ~2MHz
    port_spi_set_slowrate();
    // Read Decawave chip ID

    devID = dwt_readdevid();

#ifdef DWM3000_C0_DEVID
    if ((DWT_C0_DEV_ID != devID) && (DWT_C0_PDOA_DEV_ID != devID) \
     && (DWT_B0_DEV_ID != devID) && (DWT_B0_PDOA_DEV_ID != devID))
#else
    if (0xffffffff == devID)
#endif //DWM3000_C0_DEVID
    {
            //wake up device from low power mode
            //NOTE - in the ARM  code just drop chip select for 200us
            // SPI not working or Unsupported Device ID
            port_wakeup_dw3000();
            devID = dwt_readdevid();
#ifdef DWM3000_C0_DEVID
            if ((DWT_C0_DEV_ID != devID) && (DWT_C0_PDOA_DEV_ID != devID) \
             && (DWT_B0_DEV_ID != devID) && (DWT_B0_PDOA_DEV_ID != devID))
#else
            if (0xffffffff == devID)
#endif //DWM3000_C0_DEVID
            {
                return -1;
            }
    }

    sprintf(str,"devID = 0x%08lx\r\n", devID);
    port_dbg_print((uint8_t*)str, strlen(str));

    // configure: if DW1000 is calibrated then OTP config is used, but
    // disable sleep
    result = instance_init();


//    sprintf(str,"instance_init returns: %d\r\n", result);
//    port_dbg_print((uint8_t*)str, strlen(str));

    if (0 > result) {
        sprintf(str,"instance_init() = %d\r\n", result);
        port_dbg_print((uint8_t*)str, strlen(str));
        return result;
    }

    // Set SPI to 16MHz clock
    port_spi_set_fastrate();
    // Read Decawave chip ID
//    devID = dwt_readdevid();

#ifdef DWM3000_C0_DEVID
    if ((DWT_C0_DEV_ID != devID) && (DWT_C0_PDOA_DEV_ID != devID) \
     && (DWT_B0_DEV_ID != devID) && (DWT_B0_PDOA_DEV_ID != devID))
#else
    if (0xffffffff == devID)
#endif //DWM3000_C0_DEVID
    {
        // SPI not working or Unsupported Device ID
        return -1;
    }

    if (port_config_load() != 0)
    {
        return -1;
    }
    app.pConfig = port_config_get();
    app.pConfig->dw_dev.dev_id = devID;

    instance_config(app.pConfig) ;  // Set operating channel etc

    port_irq_enable_dw_irq();

    return result;
}


