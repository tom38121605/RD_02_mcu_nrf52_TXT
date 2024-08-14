/**
 * @file      production_cmd.c
 *
 * @brief     Commands required for module calibration, production and manufacturing
 *
 * @author    DecaWave Applications
 *
 * @attention Copyright 2017-2019 (c) DecaWave Ltd, Dublin, Ireland.
 *                    All rights reserved.
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>

#include "cmd.h"
#include "cmd_task.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "instance.h"
#include "port.h"
#include "port_irq.h"
#include "port_spi.h"
#include "port_stdio.h"
#include "port_timer.h"
#include "port_config.h"
#include "production_uwb.h"
#include "translate.h"
//#include "main.h"
#include "default_config.h"

#define SFAILED "failed"
#define SEINVAL "invalid value"
#define DWT_TXDTS 0x00FFFFFFFE00

#define MAX_STR_SIZE        256

#define dwt_rxreset(x)
#define SYS_STATUS_RXFCG                        SYS_STATUS_RXFCG_BIT_MASK
#define SYS_STATUS_TXFRS                        SYS_STATUS_TXFRS_BIT_MASK
#define RX_FINFO_RXFLEN_MASK            RX_FINFO_STD_RXFLEN_MASK
#define RX_FINFO_RXFL_MASK_1023         0x000003FFUL/* Receive Frame Length Extension (0 to 1023) */

extern app_cfg_t app;

/*OTP Memory Addresses definition*/
#define TX_CH5_ADDRESS          (0x11)
#define TX_CH9_ADDRESS          (0x13)
#define PG_CNT_ADDRESS          (0x1B)  // Channel 5 -> Bytes [1:0] , Channel 2 -> Bytes [3:2]
#define PG_DLY_TRIM_ADDRESS (0x1E)  //Trim -> Byte [0], PGDLY Channel 5 -> Bytes [2] , Channel 2 -> Bytes [3]
#define ANT_DLY_ADDRESS         (0x1C)  // Channel 5 -> Bytes [1:0] , Channel 2 -> Bytes [3:2]
#define REF_ADDRESS             (0x1D)  // Temp -> Byte [0] , VBAT -> Byte [1]

#define OTP_VALID(x)    ((x) != 0)
#define OTP_ADDR_MAX    (0x7f)

/*Define and static variable for TWR application*/
#define RNG_DELAY_MS 80
#define ALL_MSG_COMMON_LEN 10
#define ALL_MSG_SN_IDX 2

#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 15
#define RESP_MSG_TS_LEN 5
#define RX_BUF_LEN 24

#define UUS_TO_DWT_TIME 65536

#define TWR_TOTAL_TIMEOUT_MS (3000)
/* Delay between frames, in UWB microseconds. */
#define POLL_RX_TO_RESP_TX_DLY_UUS  DEFAULT_POLL_RX_TO_RESP_TX_DLY_UUS
/* Delay between frames, in UWB microseconds. */
#define POLL_TX_TO_RESP_RX_DLY_UUS  DEFAULT_POLL_TX_TO_RESP_RX_DLY_UUS
/* Receive response timeout. */
#define RESP_RX_TIMEOUT_UUS         DEFAULT_RESP_RX_TIMEOUT_UUS


#define HERTZ_TO_PPM_MULTIPLIER_CHAN_2_INT     (-1*100000/39936*10000)
#define DWT_TIME_UNITS_INT          (1/(4992*100000)/128)

/* Multiplication factors to convert carrier integrator value to a frequency offset in Hertz */
#define FREQ_OFFSET_MULTIPLIER          (998.4e6/2.0/1024.0/131072.0) // 3.719329833984375

/* Multiplication factors to convert frequency offset in Hertz to PPM crystal offset */
/* NB: also changes sign so a positive value means the local RX clock is running slower than the remote TX device. */
#define HERTZ_TO_PPM_MULTIPLIER_CHAN_2     (-1.0e6/3993.6e6)  // -0.00025040064102564102564102564102564
#define HERTZ_TO_PPM_MULTIPLIER_CHAN_5     (-1.0e6/6489.6e6)  // -0.00015409270216962524654832347140039

#define INT_COEFFICIENT_CHANNEL_9 (16)
#define INT_COEFFICIENT_CHANNEL_5 (13)
#define INT_COEFFICIENT_CHANNEL_2 (8)

#ifndef M_PI
#define M_PI        3.14159265358979323846f
#endif

/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT 299702547

/*Coefficient for distance calculation in INT*/
#define DISTANCE_INT_COEFFICIENT (9606/2048)

/*Mask used to determine what field has already been written by user in config */
#define MASK_PART_ID_WR     0x00000001
#define MASK_DEV_ID_WR      0x00000002
#define MASK_LOT_ID_WR      0x00000004
#define MASK_REF_TEMP_WR    0x00000008
#define MASK_REF_BAT_WR     0x00000010
#define MASK_XTAL_TRIM_WR   0x00000020
#define MASK_CH9_TXP_WR     0x00000040
#define MASK_CH9_PGC_WR     0x00000080
#define MASK_CH9_PGD_WR     0x00000100
#define MASK_CH9_ARX_WR     0x00000200
#define MASK_CH9_ATX_WR     0x00000400
#define MASK_CH5_TXP_WR     0x00000800
#define MASK_CH5_PGC_WR     0x00001000
#define MASK_CH5_PGD_WR     0x00002000
#define MASK_CH5_ARX_WR     0x00004000
#define MASK_CH5_ATX_WR     0x00008000

/* Mask to select all DW1000 Interrupts */
#ifdef DW1000
#define DW1000_ALL_INTERRUPTS \
    DWT_INT_TFRS | \
    DWT_INT_LDED | \
    DWT_INT_RFCG | \
    DWT_INT_RPHE | \
    DWT_INT_RFCE | \
    DWT_INT_RFSL | \
    DWT_INT_RFTO | \
    DWT_INT_RXOVRR | \
    DWT_INT_RXPTO | \
    DWT_INT_GPIO | \
    DWT_INT_SFDT | \
    DWT_INT_ARFE
#endif

static uint8 poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0};
static uint8 resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
/*End of defines and static variable  for TWR*/

/*SPRINTF char buffer*/
static char str[MAX_STR_SIZE];

/*Size of DW1000 register dump buffer*/
#define REG_DUMP_BUFSIZE    3584

void resp_msg_get_ts(uint8 *ts_field, int32 *ts);
void resp_msg_set_ts(uint8_t *ts_field, const uint64_t ts);
uint64_t get_rx_timestamp_u64(void);
uint64_t get_tx_timestamp_u64(void);


/*****************************************************************************
 *  @fn     current_ch_cfg
 *  @brief  Return the current channel configuration
 *          Support only channel 2 and 5 for dwm1004
 *
 *
 */
trx_config_t * current_ch_cfg(void) {
    trx_config_t * cur_trx_config ;
    uint8_t ch = app.pConfig->dwt_config.chan;

    // Checking current channel
    switch (ch)
    {
    case 5:
        cur_trx_config = &app.pConfig->dw_dev.trx_ref_config[chan_to_deca(ch)];
        break;

    case 9:
        cur_trx_config = &app.pConfig->dw_dev.trx_ref_config[chan_to_deca(ch)];
        break;

    default:
        return NULL;
    }
    return cur_trx_config;
}


/*****************************************************************************
 *  @fn     uwbmac_tx_power_set
 *  @brief  Configure DW1000 with the input pgdelay and Txpower and save to BSS/application
 *  @param  uint8_t pg_delay
 *          uint32_t tx_power
 *
 */
int uwbmac_tx_power_set(uint8_t pg_delay, uint32_t tx_power)
{
    trx_config_t * cur_trx_config ;
    dwt_txconfig_t tx_cfg;
    int ch = app.pConfig->dwt_config.chan;
    int mask;

    // Checking current channel
    cur_trx_config = current_ch_cfg();
    if (!cur_trx_config)
        return INVALID ;

    tx_cfg.power = cur_trx_config->ref_tx_pwr = tx_power;
    tx_cfg.PGdly = cur_trx_config->ref_pgdly = pg_delay;

    dwt_configuretxrf(&tx_cfg);

    switch (ch)
    {
    case 5:
        mask = MASK_CH5_TXP_WR ;
        break;

    case 9:
        mask = MASK_CH9_TXP_WR ;
        break;

    default:
        return INVALID;
    }

    app.pConfig->dw_dev.field_status |= mask;

    port_config_save();

    return 0;
}


/*****************************************************************************
 *  @fn     dw_get_ant_cfg
 *  @brief  Get antenna delay configuration from current configuration and write it to
 *          input parameters.
 *  @param  uint16_t *tx_delay
 *          uint16_t *rx_delay
 *
 */
int dw_get_ant_cfg(uint16_t *txa_delay, uint16_t *rxa_delay, uint16_t *rxb_delay)
{
    trx_config_t * cur_trx_config ;

    // Checking current channel
    cur_trx_config = current_ch_cfg();
    if (!cur_trx_config)
        return INVALID;

    *txa_delay = cur_trx_config->txa_ant_delay;
    *rxa_delay = cur_trx_config->rxa_ant_delay;
    *rxb_delay = cur_trx_config->rxb_ant_delay;

    return 0;
}


/*****************************************************************************
 *  @fn     dw_set_ant_cfg
 *  @brief  Configure DW1000 with input antenna delay and save to BSS/application
 *  @param  uint16_t tx_delay
 *          uint16_t rx_delay
 *
 */
int dw_set_ant_cfg( uint16_t tx_delay, uint16_t rx_delay)
{
    trx_config_t * cur_trx_config ;
    int ch = app.pConfig->dwt_config.chan;
    int mask;

    // Checking current channel
    cur_trx_config = current_ch_cfg();
    if (!cur_trx_config)
        return INVALID;

    cur_trx_config->rxa_ant_delay = rx_delay;
    cur_trx_config->txa_ant_delay = tx_delay;

    dwt_setrxantennadelay(rx_delay);
    dwt_settxantennadelay(tx_delay);

    switch (ch)
    {
    case 5:
        mask = MASK_CH5_ARX_WR | MASK_CH5_ATX_WR ;
        break;
    case 9:
        mask = MASK_CH9_ARX_WR | MASK_CH9_ATX_WR ;
        break;

    default:
        return INVALID;
    }

    app.pConfig->dw_dev.field_status |= mask;

    port_config_save();
    return 0;


}


/*****************************************************************************
 *  @fn     uwb_off
 *  @brief  Stop RX/TX and disable DW IRQ
 *
 */
void uwb_off()
{
    decaIrqStatus_t s = decamutexon();
    dwt_forcetrxoff();

    dwt_setinterrupt(SYS_ENABLE_LO_MASK, DWT_ENABLE_INT_ONLY);

    decamutexoff(s);
}


/*****************************************************************************
 *  @fn     cmd_uwb_reset
 *  @brief  DW1000 soft reset
 *    @return VALID when reset is performed
 *
 */
check_return_t cmd_uwb_reset()
{
    port_reset_dw1000();
//    if (inittestapplication() < 0)
//    {
//        cmd_uwb_urd();
//        sprintf(str,"reset failed\r\n");
//        port_stdio_transmit((uint8_t*)str, strlen(str));
//        return INVALID;
//    }

    sprintf(str,"reset done\r\n");
    port_stdio_transmit((uint8_t*)str, strlen(str));

    // Reconfiguring device
    instance_config(app.pConfig) ;

    port_wait(1);
    return VALID;
}


///*****************************************************************************
// *  @fn     cmd_uwb_cw_mode
// *  @brief  Enable continuous wave transmission
// *    @return VALID when continuous wave mode enabled
// *
// */
//check_return_t cmd_uwb_cw_mode(void)
//{
//    uint8_t ch = app.pConfig->dwt_config.chan;
//
//    uwb_off();
//    //dw_configcwmode must be called at slow spi speed
//    port_spi_set_slowrate();
//
//    dwt_configure(&app.pConfig->dwt_config);
//    dwt_configuretxrf(&app.pConfig->dw_dev.tx_config[chan_to_deca(ch)]);
//
//    dwt_configcwmode(ch);
//
//    sprintf(str,"Cont wave ON - CH%d\r\n", ch);
//    port_stdio_transmit((uint8_t*)str, strlen(str));
//    port_spi_set_fastrate();
//    return VALID;
//}

/*****************************************************************************
 *  @fn     cmd_uwb_cw_mode
 *  @brief  Enable continuous wave transmission
 *    @return VALID when continuous wave mode enabled
 *
 */
check_return_t cmd_uwb_cw_mode(void)
{
    uint8_t ch;// = app.pConfig->dwt_config.chan;

    if (cmd_get_argc() != 2)
    {
        sprintf(str,"Usage: %s <channel>\r\n", cmd_get_arg(0));
        port_stdio_transmit((uint8_t*)str, strlen(str));
        return INVALID;
    }

    ch = cmd_get_arg_num(1);

    uwb_off();
    //dw_configcwmode must be called at slow spi speed
    port_spi_set_slowrate();

    dwt_configure(&app.pConfig->dwt_config);
    switch(ch)
    {
    case 5:
        dwt_configuretxrf(&app.pConfig->dw_dev.tx_config[0]);
        break;

    case 9:
        dwt_configuretxrf(&app.pConfig->dw_dev.tx_config[1]);
        break;

    default:
        port_spi_set_fastrate();
        sprintf(str,"Invalid channel number: %d, please choose 5 or 9\r\n", ch);
        port_stdio_transmit((uint8_t*)str, strlen(str));
        return INVALID;
    }

    dwt_configcwmode(ch);

    sprintf(str,"Cont wave ON - CH%d\r\n", ch);
    port_stdio_transmit((uint8_t*)str, strlen(str));
    port_spi_set_fastrate();
    return VALID;
}


/*****************************************************************************
 *  @fn     cmd_uwb_set_xtaltrim
 *  @brief  Set crystal trim
 *  @args   trim: uint8_t, 0 =< trim =< 31
 *    @return VALID if trim is correct and DW1000 trim set
 *
 */
check_return_t cmd_uwb_set_xtaltrim(void)
{
    uint8_t trim;

    if (cmd_get_argc() != 2)
    {
        sprintf(str,"Usage: %s <trim>\r\n", cmd_get_arg(0));
        port_stdio_transmit((uint8_t*)str, strlen(str));
        return INVALID;
    }

    trim = cmd_get_arg_num(1);

    if (trim > 31)
    {
        sprintf(str,"EINVAL: %d (must be <0; 31>)\r\n", trim);
        port_stdio_transmit((uint8_t*)str, strlen(str));
        return INVALID;
    }

    dwt_setxtaltrim(trim);
    app.pConfig->dw_dev.xtaltrim = trim ;
    app.pConfig->dw_dev.field_status |= MASK_XTAL_TRIM_WR;
    return VALID;
}


/*****************************************************************************
 *  @fn     cmd_uwb_cf_mode
 *  @brief  Enable continuous frame transmission
 *  @args      per_us:   uint32_t, number of frame per us
 *          len:      uint32_t, length of frame, 3 =< len =< 127
 *    @return VALID if continuous frame mode correctly enabled
 *
 */
check_return_t cmd_uwb_cf_mode()
{
        unsigned int time_s = 0;
    uint8_t msg[1024];
    uint32_t rep_rate = 4;
    uint32_t per_us = 0;
    uint32_t len;
    uint8_t ch = app.pConfig->dwt_config.chan;

    uint8_t argc = cmd_get_argc();

    if (argc < 3)
    {
        sprintf(str,"Usage: %s <frm_len=3..127> <per_us> <optional: period_s>\r\n", cmd_get_arg(0));
        port_stdio_transmit((uint8_t*)str, strlen(str));
        return INVALID;
    }

    len = cmd_get_arg_num(1);
    /*Do not support long frame mode*/
    if (len > 127)
    {
        sprintf(str,"Usage: %s <frm_len=3..127> <per_us> <optional: period_s>\r\n", cmd_get_arg(0));
        port_stdio_transmit((uint8_t*)str, strlen(str));
        return INVALID;
    }

    per_us = cmd_get_arg_num(2);
    if (per_us > 0)
    {
        rep_rate = 124800 * per_us / 1000;
    }

    if(argc == 4)
    {
        time_s = cmd_get_arg_num(3);
    }

    memset(msg, 0xa5, len);

    uwb_off();

    port_spi_set_fastrate();
    //port_spi_set_slowrate();

    dwt_configure(&app.pConfig->dwt_config);
    dwt_configuretxrf(&app.pConfig->dw_dev.tx_config[chan_to_deca(ch)]);

    dwt_configcontinuousframemode(rep_rate, ch);

    dwt_writetxdata(len, (uint8_t *)msg, 0);
    dwt_writetxfctrl(len, 0, 0);

    dwt_starttx(DWT_START_TX_IMMEDIATE);

    sprintf(str,"Cont Frame ON\r\n");
    port_stdio_transmit((uint8_t*)str, strlen(str));

    if(time_s>0)
    {
        sprintf(str,"Staying for %d seconds..\r\n", time_s);
        port_stdio_transmit((uint8_t*)str, strlen(str));

            deca_sleep(time_s*1000);
            dwt_softreset();

        sprintf(str,"Finished\r\n");
        port_stdio_transmit((uint8_t*)str, strlen(str));
    }

    return VALID;
}


/*****************************************************************************
 *  @fn     cmd_uwb_get_tx_power
 *  @brief  Print current RX configuration:
 *          TX power
 *          PG delay
// *          Ref_temp
// *          Ref_vbat
 *          PG_count
 *          Xtal_trim
 *    @return VALID if RX configuration correctly returned
 *
 */
check_return_t cmd_uwb_get_tx_power(void)
{
    uint8_t pgdly;
    uint32_t power;
//    uint8_t ref_temp;
//    uint8_t ref_vbat;
    uint16_t ref_pgcount;
    uint8_t ref_xtaltrim;
    trx_config_t * cur_trx_config ;

    // Checking current channel
    cur_trx_config = current_ch_cfg();
    if (!cur_trx_config)
        return INVALID ;

    power = cur_trx_config->ref_tx_pwr;
    pgdly = cur_trx_config->ref_pgdly;
    ref_pgcount = cur_trx_config->ref_pgcnt;

//    ref_temp = app.pConfig->dw_dev.ref_temp;
//    ref_vbat = app.pConfig->dw_dev.ref_vbat;
    ref_xtaltrim = app.pConfig->dw_dev.xtaltrim;

    // Xtal trim has not been written by user yet, reading default value from dw3000
    if (ref_xtaltrim == 0)
    {
        ref_xtaltrim = dwt_getxtaltrim();
    }

    cmd_get_argc();

    sprintf(str,"%s: pgd=x%02X txp=x%08X pgc=%d xtt=%d\r\n", cmd_get_arg(0), pgdly, (unsigned int)power,
                        ref_pgcount,ref_xtaltrim);
    port_stdio_transmit((uint8_t*)str, strlen(str));
    return VALID;
}


/*****************************************************************************
 *  @fn     cmd_uwb_set_tx_power
 *  @brief  Set TX power to DW1000 and save to BSS/application
 *  @arg    TX power, uint32_t
 *          PG delay, uint8_t
 *    @return VALID TX configuration correctly set
 *
 */
check_return_t cmd_uwb_set_tx_power(void)
{
    uint8_t pgdly;
    uint32_t power;
    check_return_t ret = INVALID;
    int rv;
    int ch = app.pConfig->dwt_config.chan;
    dwt_txconfig_t * p_tx_config = &app.pConfig->dw_dev.tx_config[chan_to_deca(ch)];

    if (cmd_get_argc() != 3)
    {
        sprintf(str,"Usage: %s <pg_delay> <tx_power>\r\n", cmd_get_arg(0));
        port_stdio_transmit((uint8_t*)str, strlen(str));
        return INVALID;
    }

    pgdly = cmd_get_arg_num(1);
    power = cmd_get_arg_num(2);

    p_tx_config->PGdly = pgdly;
    p_tx_config->power = power;

    rv = uwbmac_tx_power_set(pgdly, power);
    if (rv == 0)
    {
        sprintf(str,"%s: pg_delay=x%02X tx_power=x%08X\r\n",\
                cmd_get_arg(0), pgdly, (unsigned int)power);
        port_stdio_transmit((uint8_t*)str, strlen(str));
        ret = VALID;
    }
    else
    {
        sprintf(str,"%s: %s\r\n", cmd_get_arg(0), SFAILED);
        port_stdio_transmit((uint8_t*)str, strlen(str));
        ret = INVALID;
    }

    
    port_config_save();

    return ret;
}


/*****************************************************************************
 *  @fn     cmd_uwb_uts
 *  @brief  Configure TX power,
 *              //ref_temp, ref_vbat,
 *              pg_delay, pgcount in DW1000
 *          and save to BSS/Application
 *  @arg    PG delay, uint8_t
 *          TX power, uint32_t
 *          temp, uint8_t
 *          vbat, uint8_t
 *          PG count, uint16_t
 *    @return VALID if  RF configuration correctly set
 *
 */
check_return_t cmd_uwb_uts(void)
{
    uint32_t tx_pwr;
    uint8_t pgdly;
    uint16_t pgcnt;
    //int8_t temp;
    //int8_t vbat;
    trx_config_t * cur_trx_config ;
    //dw_dev_t * dw_dev_pt ;

    int ch = app.pConfig->dwt_config.chan;
    int mask;

    if (cmd_get_argc() != 6)
    {
        sprintf(str,"Usage: %s <ref_pgdly> <ref_pwr> <ref_temp> <ref_vbat> <ref_pgcnt> \r\n", cmd_get_arg(0));
        port_stdio_transmit((uint8_t*)str, strlen(str));
        return INVALID;
    }

    pgdly = cmd_get_arg_num(1);
    tx_pwr = cmd_get_arg_num(2);
    //temp = cmd_get_arg_num(3);
    //vbat = cmd_get_arg_num(4);
    pgcnt = cmd_get_arg_num(5);

    // Checking current channel
    cur_trx_config = current_ch_cfg();
    if (!cur_trx_config)
        return INVALID ;

    //dw_dev_pt=&app.pConfig->dw_dev;

    cur_trx_config->ref_tx_pwr = tx_pwr;
    cur_trx_config->ref_pgdly = pgdly;
    uwbmac_tx_power_set(pgdly,tx_pwr);

    cur_trx_config->ref_pgcnt = pgcnt;
//    dw_dev_pt->ref_temp = temp;
//    dw_dev_pt->ref_vbat = vbat;

    mask = MASK_REF_BAT_WR | MASK_REF_TEMP_WR;

    switch (ch)
    {
    case 5:
        mask |= MASK_CH5_PGC_WR;
        break;

    case 9:
        mask |= MASK_CH9_PGC_WR;
        break;

    default:
        return INVALID;
    }

    app.pConfig->dw_dev.field_status |= mask;

    port_config_save();

    return VALID;
}


/*****************************************************************************
 *  @fn     cmd_uwb_eventcounter_enable
 *  @brief  Enable event counter
 *    @return VALID after enabling eventcounter
 *
 */
check_return_t cmd_uwb_eventcounter_enable(void)
{
    dwt_configeventcounters(1);
    return VALID;
}


/*****************************************************************************
 *  @fn     cmd_uwb_eventcounter_disable
 *  @brief  Disable event counter
 *    @return VALID after disabling eventcounter
 *
 */
check_return_t cmd_uwb_eventcounter_disable(void)
{
    dwt_configeventcounters(0);
    return VALID;
}


/*****************************************************************************
 *  @fn     cmd_uwb_eventcounter_print
 *  @brief  Print event counter
 *    @return VALID after printing out eventcounter
 *
 */
check_return_t cmd_uwb_eventcounter_print(void)
{
    dwt_deviceentcnts_t ev;
    dwt_readeventcounters(&ev);

    sprintf(str,"PHE %u \r\n", ev.PHE);
    sprintf(&str[strlen(str)],"RSL %u \r\n", ev.RSL);
    sprintf(&str[strlen(str)],"CRCG %u \r\n", ev.CRCG);
    sprintf(&str[strlen(str)],"CRCB %u \r\n", ev.CRCB);
    sprintf(&str[strlen(str)],"ARFE %u \r\n", ev.ARFE);
    sprintf(&str[strlen(str)],"OVER %u \r\n", ev.OVER);
    sprintf(&str[strlen(str)],"SFDTO %u \r\n", ev.SFDTO);
    sprintf(&str[strlen(str)],"PTO %u \r\n", ev.PTO);
    sprintf(&str[strlen(str)],"RTO %u \r\n", ev.RTO);
    sprintf(&str[strlen(str)],"TXF %u \r\n", ev.TXF);
    sprintf(&str[strlen(str)],"HPW %u \r\n", ev.HPW);
#ifdef DW1000
    sprintf(&str[strlen(str)],"TXW %u \r\n", ev.TXW);
#endif
    port_stdio_transmit((uint8_t*)str, strlen(str));

    return VALID;
}


volatile uint8_t rx_enabled = 0;

check_return_t cmd_uwb_rx_enable(void)
{
    uint8_t ch = app.pConfig->dwt_config.chan;

    //uwb_off();

    port_irq_disable_dw_irq();

    /* Reset DW IC */
    //port_reset_dw1000(); /* Target specific drive of RSTn line into DW IC low for a period. */

    //deca_sleep(2); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)

//    int timeout = 4; // 4ms timeout period
//    while (!dwt_checkidlerc() && timeout--) /* Need to make sure DW IC is in IDLE_RC before proceeding */
//    { 
//        deca_sleep(1);
//    };
//
//    if(timeout <= 0)
//    {
//        sprintf(str,"dwt_checkidlerc timed out \r\n");
//        port_dbg_print((uint8_t*)str, strlen(str));
//    }

    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)  /**< set callbacks to NULL inside dwt_initialise*/
    {
        sprintf(str,"INIT FAILED \r\n");
        port_dbg_print((uint8_t*)str, strlen(str));
        return (INVALID);
    }

    //RX interrupt : required to re-enable RX mode in interrupt call back
#define RX_INT_MASK (SYS_STATUS_TXFRS_BIT_MASK | SYS_STATUS_RXFCG_BIT_MASK  \
                  | (SYS_STATUS_ARFE_BIT_MASK | SYS_STATUS_RXFSL_BIT_MASK   \
                  | SYS_STATUS_RXSTO_BIT_MASK | SYS_STATUS_RXPHE_BIT_MASK   \
                  | SYS_STATUS_RXFCE_BIT_MASK | SYS_STATUS_RXFTO_BIT_MASK))
    dwt_setinterrupt(RX_INT_MASK, DWT_ENABLE_INT_ONLY);

    
    dwt_setcallbacks(instance_txcallback, instance_rx, instance_rx, instance_rx, NULL, NULL);

    /* set antenna delays */
    dwt_settxantennadelay(app.pConfig->dw_dev.trx_ref_config[chan_to_deca(ch)].txa_ant_delay);
    dwt_setrxantennadelay(app.pConfig->dw_dev.trx_ref_config[chan_to_deca(ch)].rxa_ant_delay);

    dwt_configure(&app.pConfig->dwt_config);

    dwt_configuretxrf(&app.pConfig->dw_dev.tx_config[chan_to_deca(ch)]);

    dwt_configureframefilter(DWT_FF_DISABLE, 0);

    dwt_setrxtimeout(0);

    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    port_irq_enable_dw_irq();

    return VALID;
}

//check_return_t cmd_uwb_rx_enable(void)
//{
//    uint8_t ch = app.pConfig->dwt_config.chan;
//
//    uwb_off();
//
//    port_irq_disable_dw_irq();
//
//    /* Reset DW IC */
//    port_reset_dw1000(); /* Target specific drive of RSTn line into DW IC low for a period. */
//
//    deca_sleep(2); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)
//
//    uint8_t timeout = 4; // 4ms timeout period
//    while (!dwt_checkidlerc() && timeout--) /* Need to make sure DW IC is in IDLE_RC before proceeding */
//    { 
//        deca_sleep(1);
//    };
//
//    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)  /**< set callbacks to NULL inside dwt_initialise*/
//    {
//        sprintf(str,"INIT FAILED \r\n");
//        port_dbg_print((uint8_t*)str, strlen(str));
//        return (INVALID);
//    }
//
//    //RX interrupt : required to re-enable RX mode in interrupt call back
//#define RX_INT_MASK (SYS_STATUS_TXFRS_BIT_MASK | SYS_STATUS_RXFCG_BIT_MASK  \
//                  | (SYS_STATUS_ARFE_BIT_MASK | SYS_STATUS_RXFSL_BIT_MASK   \
//                  | SYS_STATUS_RXSTO_BIT_MASK | SYS_STATUS_RXPHE_BIT_MASK   \
//                  | SYS_STATUS_RXFCE_BIT_MASK | SYS_STATUS_RXFTO_BIT_MASK))
//    dwt_setinterrupt(RX_INT_MASK, DWT_ENABLE_INT_ONLY);
//
//    /* set antenna delays */
//    dwt_settxantennadelay(app.pConfig->dw_dev.trx_ref_config[chan_to_deca(ch)].txa_ant_delay);
//    dwt_setrxantennadelay(app.pConfig->dw_dev.trx_ref_config[chan_to_deca(ch)].rxa_ant_delay);
//
//    dwt_configureframefilter(DWT_FF_DISABLE, 0);
//
//    dwt_configure(&app.pConfig->dwt_config);
//    dwt_configuretxrf(&app.pConfig->dw_dev.tx_config[chan_to_deca(ch)]);
//
//    dwt_setrxtimeout(0);
//
//    dwt_rxenable(DWT_START_RX_IMMEDIATE);
//
//    port_irq_enable_dw_irq();
//
//    return VALID;
//}

check_return_t cmd_uwb_rx_disable(void)
{
    port_irq_disable_dw_irq();
    dwt_forcetrxoff();
    port_irq_enable_dw_irq();
    return VALID;
}

/*****************************************************************************
 *  @fn     cmd_uwb_rx
 *  @brief  Enable DW1000 receiver
 *    @return VALID after turning on receiver
 *
 */
check_return_t cmd_uwb_rx(void)
{
    if(!rx_enabled)
    {
        rx_enabled = 1;
        sprintf(str,"RX on\r\n");
        port_stdio_transmit((uint8_t*)str, strlen(str));
        return cmd_uwb_rx_enable();
    }
    else
    {
        rx_enabled = 0;
        sprintf(str,"RX off\r\n");
        port_stdio_transmit((uint8_t*)str, strlen(str));
        return cmd_uwb_rx_disable();
    }
}


/*****************************************************************************
 *  @fn     cmd_uwb_tx
 *  @brief  Transmit a given number of frames
 *  @args   cnt,    uint16_t, number of frame to send, > 0
 *          len,    uint16_t, length of frame to send, > 3, < 127
 *          per_us, uint16_t, delay between frames, > 0
 *    @return VALID after sending frames
 *
 */
check_return_t cmd_uwb_tx(void)
{
    uint8_t *msg;
    volatile uint16_t per_ms;
    uint16_t cnt;
    uint16_t len;
    uint16_t i;
    uint8_t ch = app.pConfig->dwt_config.chan;

    check_return_t ret = INVALID;

    sprintf(str,"ch: %d\n", ch);
    port_stdio_transmit((uint8_t*)str, strlen(str));

    if (cmd_get_argc() != 4)
    {
        sprintf(str,"Usage: %s <frm_cnt> <frm_len> <per_ms>\r\n", cmd_get_arg(0));
        port_stdio_transmit((uint8_t*)str, strlen(str));
        return INVALID;
    }

    cnt = cmd_get_arg_num(1);
    len = cmd_get_arg_num(2);
    per_ms = cmd_get_arg_num(3);

    if ((cnt > 0) && (len > 2) && (len <= 1023) && (per_ms > 0))
    {
        msg = malloc(len);
        if (msg)
        {
            memset(msg, 0xa5, len);

            uwb_off();

            dwt_configure(&app.pConfig->dwt_config);
            //app.pConfig->dw_dev.tx_config[chan_to_deca(ch)].power = 0xffffffff;
            dwt_configuretxrf(&app.pConfig->dw_dev.tx_config[chan_to_deca(ch)]);

            dwt_writetxdata(len, (uint8_t *)msg, 0);
            dwt_writetxfctrl(len, 0, 0);

            for (i=0; i<cnt; i++)
            {
                if (dwt_starttx(DWT_START_TX_IMMEDIATE) != DWT_SUCCESS)
                {
                    sprintf(str,"dwt_starttx_err, %d\r\n", i);
                    port_stdio_transmit((uint8_t*)str, strlen(str));
                    ret = INVALID;
                    break;
                }

                ret = VALID;
                while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS)) {}

                port_wait(per_ms);
            }
            sprintf(str,"%s:(sent=%d)\r\n", cmd_get_arg(0), i);
            port_stdio_transmit((uint8_t*)str, strlen(str));
            free(msg);
        }
        else
        {
            sprintf(str,"Cannot allocate memory\r\n");
            port_stdio_transmit((uint8_t*)str, strlen(str));
            return INVALID;
        }
    }
    else
    {
        sprintf(str,"%s: %s\r\n", cmd_get_arg(0), SEINVAL);
        port_stdio_transmit((uint8_t*)str, strlen(str));
    }
    return ret;
}


/*****************************************************************************
 *  @fn     cmd_uwb_off
 *  @brief  Turn off DW1000
 *    @return VALID after DW1000 turned off
 *
 */
check_return_t cmd_uwb_off(void)
{
    uwb_off();
    return VALID;
}


/*****************************************************************************
 *  @fn     cmd_uwb_set_ant_cfg
 *  @brief  Configure DW1000 with antenna delays and save in BSS/application
 *  @args   tx_delay, uint16_t, tx antenna delay
 *          rx_delay, uint16_t, rx antenna delay
 *    @return VALID after setting antenna delays
 *
 */
check_return_t cmd_uwb_set_ant_cfg(void)
{
    uint16_t tx_delay, rx_delay;
    int rv ;

    if (cmd_get_argc() != 3)
    {
        sprintf(str,"Usage: %s <tx_delay> <rx_delay>\r\n", cmd_get_arg(0));
        port_stdio_transmit((uint8_t*)str, strlen(str));
        return INVALID;
    }

    tx_delay = cmd_get_arg_num(1);
    rx_delay = cmd_get_arg_num(2);
    rv = dw_set_ant_cfg(tx_delay, rx_delay);

    if(rv<0)
    {
        return INVALID;
    }
    return VALID;
}


/*****************************************************************************
 *  @fn     cmd_uwb_get_ant_cfg
 *  @brief  Print current antenna delays configuration
 *    @return VALID after printing antenna delays
 *
 */
check_return_t cmd_uwb_get_ant_cfg(void)
{
    uint16_t txa_delay, rxa_delay, rxb_delay;
    int rv ;

    rv = dw_get_ant_cfg(&txa_delay, &rxa_delay, &rxb_delay);

    if (rv <0)
    {
        sprintf(str,"Value not set for this channel\r\n");
        port_stdio_transmit((uint8_t*)str, strlen(str));
        return INVALID;
    }
    sprintf(str,"uwb0: txa_delay=%d rxa_delay=%d rxb_delay=%d\r\n", txa_delay, rxa_delay, rxb_delay);
    port_stdio_transmit((uint8_t*)str, strlen(str));
    return VALID;
}

/*****************************************************************************
 *  @fn     cmd_read_temp_vbat
 *  @brief  Read temp/vbat from DW1000 and print out
 *    @return VALID after reading/printing temp/vbat
 *
 */
check_return_t cmd_read_temp_vbat(void)
{
    uint16_t temp_vbat;

    uwb_off();
    port_spi_set_slowrate();

    temp_vbat = dwt_readtempvbat();

    port_spi_set_fastrate();

    if((temp_vbat >> 8) == 0xff || (temp_vbat >> 8) == 0x00 || (temp_vbat) == 0xff  || (temp_vbat) == 0x00)
    {
        sprintf(str,"Incorrect Reading : temp=0x%x vbat=0x%x \r\n", (uint8_t) (temp_vbat >> 8), (uint8_t) temp_vbat);
        port_stdio_transmit((uint8_t*)str, strlen(str));
        return INVALID;
    }

    sprintf(str,"temp=0x%x vbat=0x%x \r\n", (uint8_t) (temp_vbat >> 8), (uint8_t) temp_vbat);
    port_stdio_transmit((uint8_t*)str, strlen(str));
    return VALID;
}



/*****************************************************************************
 *  @fn     cmd_twr_initiator
 *  @brief  Two Way Ranging - Initiator
 *          UTWRR must be called before UTWRI with the same arguments
 *  @args   nb_twr, uint16_t, number of TWR to perform, > 0
 *          dly_ms, uint16_t, delay between TWR in ms, > 0
 *    @return VALID at the end of two way raging scheme
 *
 */
check_return_t cmd_twr_initiator(void)
{
    uint32_t status_reg;
    uint8_t rx_buffer[RX_BUF_LEN];
    uint16_t nb_twr ;
    uint16_t dly_ms ;
    uint32_t twr_timer;
    uint32_t twr_to_timer;
    uint32_t i ;
    uint32_t frame_cnt = 0 ;
    uint32_t frame_len;
    int32_t poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
    int32_t rtd_init, rtd_resp;
    int32_t carrier_integrator_value_old ;
    uint8_t int_coeff_channel ;
    uint8_t ch = app.pConfig->dwt_config.chan;
    uint32_t dev_id = app.pConfig->dw_dev.dev_id;
    uint8 frame_seq_nb = 0;
    int16 pdoa_raw = 0;
    float pdoa = 0;
    float pdoa_sum = 0;
    float clockOffsetRatio ;
    double tof;
    double distance_mm;
    double distance_sum_mm = 0;

    /* Verifying arguments */
    if (cmd_get_argc() != 3)
    {
        sprintf(str,"Usage: %s <#TWR> <dly_ms>\r\n", cmd_get_arg(0));
        port_stdio_transmit((uint8_t*)str, strlen(str));
        return INVALID;
    }

    nb_twr = cmd_get_arg_num(1);
    dly_ms = cmd_get_arg_num(2);

    if (dly_ms < 5)
    {
        sprintf(str,"Error: min <dly_ms> is 5ms\r\n");
        port_stdio_transmit((uint8_t*)str, strlen(str));
        return INVALID;
    }

    switch (app.pConfig->dwt_config.chan)
    {
    case 5:
        int_coeff_channel = INT_COEFFICIENT_CHANNEL_5;
        break;

    case 9:
        int_coeff_channel = INT_COEFFICIENT_CHANNEL_9;
        break;

    default:
        return INVALID;
    }


    uwb_off();

    /*Disable DW1000 receive TO*/
//    dwt_setrxtimeout(0);

    /*Disable DW1000 interrupt as not used for TWR in this production firmware*/
    port_irq_disable_dw_irq();

    /*SPI fast rate*/
//    port_spi_set_fastrate();
    dwt_forcetrxoff();    //Stop the Receiver and Write Control and Data

    dwt_configure(&app.pConfig->dwt_config);
    app.pConfig->dw_dev.tx_config[chan_to_deca(ch)].power = 0xffffffff;
    dwt_configuretxrf(&app.pConfig->dw_dev.tx_config[chan_to_deca(ch)]);

    dwt_setrxantennadelay(app.pConfig->dw_dev.trx_ref_config[chan_to_deca(ch)].rxa_ant_delay);
    dwt_settxantennadelay(app.pConfig->dw_dev.trx_ref_config[chan_to_deca(ch)].txa_ant_delay);

    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

    dwt_configciadiag(DW_CIA_DIAG_LOG_ALL);         /* DW3000 */

    /*Timer used to quit if no response message receive after 3 seconds*/
    port_timer_start(&twr_to_timer);

    /*Main for loop over the number of TWR sequence*/
    for(i=0;i<nb_twr;i++)
    {
        if(port_timer_check(twr_to_timer,TWR_TOTAL_TIMEOUT_MS)==true)
        {
            sprintf(str,"Error: timeout expired: %d ms\r\n", TWR_TOTAL_TIMEOUT_MS);
            port_dbg_print((uint8_t*)str, strlen(str));
            return INVALID;
        }

        /*Timer used to time each ranging*/
        port_timer_start(&twr_timer);

        /*While time not elapsed => TWR routine */
        while(port_timer_check(twr_timer,dly_ms)==false)
        {
            dwt_forcetrxoff();    //Stop the Receiver and Write Control and Data

            /*TWR initiator routine*/
            poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
            /*Write frame data to DW1000 and prepare transmission.*/
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);

            dwt_writetxdata(sizeof(poll_msg), poll_msg, 0); /* Zero offset in TX buffer. */
            dwt_writetxfctrl(sizeof(poll_msg), 0, 1); /* Zero offset in TX buffer, ranging. */

            /* Start transmission, indicating that a response is expected so that
             * reception is enabled automatically after the frame is sent and
             * the delay set by dwt_setrxaftertxdelay() has elapsed. */
            dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

            /* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout.*/
            /* Waiting for reception of response message*/

            while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & SYS_STATUS_TXFRS)) {} 

            while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)) \
                        && port_timer_check(twr_timer,dly_ms)==false) {}

            /* Increment frame sequence number after transmission of the poll message (modulo 256). */
            frame_seq_nb++;
            
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_TX);


            if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)
            {
                /* Clear good RX frame event in the DW IC status register. */
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

                /*Reseting to timer as good frame received*/
                port_timer_start(&twr_to_timer);

                /* A frame has been received, read it into the local buffer. */
                frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
                if (frame_len <= RX_BUF_LEN)
                {
                    dwt_readrxdata(rx_buffer, frame_len, 0);
                }

                if(dev_id == DWT_C0_PDOA_DEV_ID)
                {
                    pdoa_raw = dwt_readpdoa();
                    pdoa = ((float)pdoa_raw / (1 << 11)) * 180 / M_PI;
                }

                /* Check that the frame is the expected response from the companion "SS TWR responder" example.
                * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
                rx_buffer[ALL_MSG_SN_IDX] = 0;
                if (frame_len == sizeof(resp_msg))
                {
                    /*Counter of received frames*/
                    frame_cnt++;

                    /* Retrieve poll transmission and response reception timestamps.*/
                    poll_tx_ts = dwt_readtxtimestamplo32();
                    resp_rx_ts = dwt_readrxtimestamplo32();

                    /* Read carrier integrator value and calculate clock offset ratio. See NOTE 11 below. */
                    clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32)(1<<26);
                    /* Read carrier integrator value and calculate clock offset ratio. */
                    carrier_integrator_value_old = dwt_readcarrierintegrator();

                    /* Get timestamps embedded in response message. */
                    resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], (int32 *)&poll_rx_ts);
                    resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], (int32 *)&resp_tx_ts);

                    /* Compute time of flight and distance, using clock offset ratio to correct for differing local and remote clock rates */
                    rtd_init = resp_rx_ts - poll_tx_ts;
                    rtd_resp = resp_tx_ts - poll_rx_ts;

                    tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
                    distance_mm = tof * SPEED_OF_LIGHT * 1000;

                    // print distance with old formula 
                    int32_t tof_dwtu_old = (rtd_init - rtd_resp) / 2 - \
                                (int32_t)((((int64_t) rtd_resp * carrier_integrator_value_old) / int_coeff_channel) >> 28);
                    // print distance with old formula 
                    int64_t dist_old = (tof_dwtu_old * 9606) >> 11;


                    distance_sum_mm += distance_mm;
                    pdoa_sum += pdoa;
//                    // print distance with old formula
//                    sprintf(str,"D%ld(mm):%ld, PDoA:%d, old: %ld \r\n",i, (int32_t) distance_mm, (int)pdoa, (int32_t)dist_old);
                    sprintf(str,"D%ld(mm):%ld, PDoA:%d\r\n",i, (int32_t) distance_mm, (int)pdoa);
                    port_stdio_transmit((uint8_t*)str, strlen(str));

//                    dwt_rxdiag_t diag;
//                    dwt_readdiagnostics(&diag);
//                    sprintf(str,"\t\tdiag info: ipPwr:x%08lx, ipAccuCnt:x%04x\r\n", diag.ipatovPower, diag.ipatovAccumCount);
//                    port_stdio_transmit((uint8_t*)str, strlen(str));

                    while(port_timer_check(twr_timer,dly_ms)==false)
                    {}
                }
            }
            else
            {
                sprintf(str,"E %08lx\r\n", status_reg);
                port_stdio_transmit((uint8_t*)str, strlen(str));

                /* Clear RX error/timeout events in the DW IC status register. */
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

                /* Turn Off DW1000 RX/TX*/
                dwt_forcetrxoff();

                /* Reset RX to properly reinitialise LDE operation. */
                dwt_rxreset();

                /*Waiting till the next TWR sequence*/
                while(port_timer_check(twr_timer,dly_ms)==false)
                {}
            }
        }
    }
    sprintf(str,"TX: %u RX: %lu \r\n", nb_twr,frame_cnt);
    port_stdio_transmit((uint8_t*)str, strlen(str));
    sprintf(str,"Dist Avg (mm) : %ld \r\n", (int32_t)distance_sum_mm/frame_cnt);
    port_stdio_transmit((uint8_t*)str, strlen(str));
    sprintf(str,"PDoA Avg (deg): %d \r\n", (int32_t)pdoa_sum/frame_cnt);
    port_stdio_transmit((uint8_t*)str, strlen(str));

    port_irq_enable_dw_irq();
    return VALID;
}


/*****************************************************************************
 *  @fn     cmd_twr_responder
 *  @brief  Two Way Ranging - Responder
 *          UTWRI must be called after UTWRR with the same arguments
 *  @args   nb_twr, uint16_t, number of TWR to perform, > 0
 *          dly_ms, uint16_t, delay between TWR in ms, > 0
 *    @return VALID at the end of two way raging scheme
 *
 */
static uint32_t poll_rx_start;
static uint32_t poll_rx_sys_record;
static uint32_t poll_rx_parse;
static uint32_t resp_tx_calculated;
static uint32_t resp_tx_system_record;
static uint32_t resp_tx_parse;
static uint64_t poll_rx_ts;
static uint64_t resp_tx_ts;
    //int64_t resp_tx_ts;
check_return_t cmd_twr_responder(void)
{
    uint32_t status_reg;
    uint8_t rx_buffer[RX_BUF_LEN];
    uint32_t frame_len;
    uint16_t txa_ant_delay;
    uint32 resp_tx_time;
    volatile uint32_t timer;
    int ret;
    uint16_t nb_twr;
    uint16_t dly_ms;
    uint16_t frame_cnt = 0;
    //uint32_t delay_timer;
    uint8_t ch = app.pConfig->dwt_config.chan;
    uint32_t dev_id = app.pConfig->dw_dev.dev_id;
    int16 pdoa_raw = 0;
    float pdoa = 0;
    float pdoa_sum = 0;

    /*Validating arguments*/
    if (cmd_get_argc() != 3)
    {
        sprintf(str,"Usage: %s <#TWR> <dly_ms>\r\n", cmd_get_arg(0));
        port_stdio_transmit((uint8_t*)str, strlen(str));
        return INVALID;
    }

    nb_twr = cmd_get_arg_num(1);
    dly_ms = cmd_get_arg_num(2);

    if (dly_ms < 5)
    {
        sprintf(str,"Error: min <dly_ms> is 5ms\r\n");
        port_stdio_transmit((uint8_t*)str, strlen(str));
        return INVALID;
    }

    /*Verifying current RX configuration (CH)*/
    trx_config_t * cur_trx_config ;
    cur_trx_config = current_ch_cfg();
    if (!cur_trx_config)
        return INVALID ;

    /*Using antenna delay corresponding to current channel*/
    txa_ant_delay = cur_trx_config->txa_ant_delay;

    uwb_off();

    /*Disable DW1000 IRQ*/
    port_irq_disable_dw_irq();

    /*SPI fast rate*/
    port_spi_set_fastrate();

    dwt_configure(&app.pConfig->dwt_config);
    app.pConfig->dw_dev.tx_config[chan_to_deca(ch)].power = 0xffffffff;
    dwt_configuretxrf(&app.pConfig->dw_dev.tx_config[chan_to_deca(ch)]);
//    sprintf(str,"set tx power x%08lx \r\n", app.pConfig->dw_dev.tx_config[chan_to_deca(ch)].power);
//    port_dbg_print((uint8_t*)str, strlen(str));
//    sprintf(str,"set tx pgdly x%02x \r\n", app.pConfig->dw_dev.tx_config[chan_to_deca(ch)].PGdly);
//    port_dbg_print((uint8_t*)str, strlen(str));

    dwt_setrxantennadelay(cur_trx_config->rxa_ant_delay);
//      sprintf(str,"set rx delay %d \r\n", cur_trx_config.rxa_ant_delay);
//      port_dbg_print((uint8_t*)str, strlen(str));

    dwt_setrxantennadelay(cur_trx_config->txa_ant_delay);
//      sprintf(str,"set tx delay %d \r\n", cur_trx_config.rxa_ant_delay);
//      port_dbg_print((uint8_t*)str, strlen(str));

    // First time anchor listens we don't do a delayed RX
    dwt_setrxaftertxdelay(0);
    /*Disable DW1000 receive TO*/
    dwt_setrxtimeout(0);

    /*Defining initial timer as TWRR will be called before TWRI*/
    //delay_timer = 3000;

    /*Margin for delay ms*/
    dly_ms--;

    /*Starting user defined timer*/
    port_timer_start(&timer);

    while(port_timer_check(timer,TWR_TOTAL_TIMEOUT_MS)==false)
    {
        //poll_rx_start=dwt_readsystimestamphi32();
        /* Activate reception immediately. */
        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        /* Multiplying the delay to make sure the responder is not in TO before receiving the poll message*/
        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR)))
        {
            if(port_timer_check(timer,TWR_TOTAL_TIMEOUT_MS) == true) // Time Out If not Frame received for Timer1_delay - 3sec
            {
                sprintf(str,"TWR responder timeout expired: %d ms\r\n", TWR_TOTAL_TIMEOUT_MS);
                port_dbg_print((uint8_t*)str, strlen(str));
                break ;
            }
        }
        if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)
        {   /*Counter for received frames*/
            frame_cnt++;

            /* Clear good RX frame event in the DW1000 status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

            /* A frame has been received, read it into the local buffer. */
            frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
            if (frame_len > RX_BUF_LEN)
            {
                return INVALID;
            }

            /* Read rx data*/
            dwt_readrxdata(rx_buffer, frame_len, 0);

            if(dev_id == DWT_C0_PDOA_DEV_ID)
            {
                pdoa_raw = dwt_readpdoa();
                pdoa = ((float)pdoa_raw / (1 << 11)) * 180 / M_PI;
                pdoa_sum += pdoa;
            }

            /* Check that the frame is a poll sent by "TD". */
            if (frame_len == sizeof(poll_msg)) // add check of function pointer 0xE0 (check function code for poll message in DW1000 user manual)
            {
                /* Retrieve poll reception timestamp. */
                poll_rx_ts =  get_rx_timestamp_u64();
                //poll_rx_sys_record = poll_rx_ts>>8;
                //poll_rx_parse = dwt_readsystimestamphi32();
                
                /* Turn Off DW1000 RX/TX*/
                dwt_forcetrxoff();

                /* Compute final message transmission time. */
                resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;

                dwt_setdelayedtrxtime(resp_tx_time);

                /* Response TX timestamp is the transmission time we programmed plus the antenna delay. */
                resp_tx_ts = (((int64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + txa_ant_delay;
                resp_tx_calculated = resp_tx_ts>>8;

                /* Write all timestamps in the final message.*/
                resp_msg_set_ts(&resp_msg[RESP_MSG_POLL_RX_TS_IDX], poll_rx_ts);
                resp_msg_set_ts(&resp_msg[RESP_MSG_RESP_TX_TS_IDX], resp_tx_ts);

                /* Write and send the response message.*/
                dwt_writetxdata(sizeof(resp_msg), resp_msg, 0); /* Zero offset in TX buffer. See Note 5 below.*/
                dwt_writetxfctrl(sizeof(resp_msg), 0, 1); /* Zero offset in TX buffer, ranging. */
                

                ret = dwt_starttx(DWT_START_TX_DELAYED);

                /* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. */
                if (ret == DWT_SUCCESS)
                {
                    /* Poll DW1000 until TX frame sent event set.*/
                    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
                    {};

                    /* Clear TXFRS event. */
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

                    resp_tx_ts =  get_tx_timestamp_u64();
                    //resp_tx_system_record = resp_tx_ts>>8;
                    //resp_tx_parse = dwt_readsystimestamphi32();

                    /* Turn Off DW1000 RX/TX*/
                    dwt_forcetrxoff();

                    /*Waiting additional delay to ensure receiver is not turned on too early*/
                    port_wait(dly_ms - 3);

                    /*Reseting timer 1 for next frame*/
                    port_timer_start(&timer);

                    sprintf(str,"Succ #%d, PDoA : %d\r\n", frame_cnt, (int)pdoa);
                    port_stdio_transmit((uint8_t*)str, strlen(str));
                }
                else
                {
                /* Turn Off DW1000 RX/TX*/
                    dwt_forcetrxoff();

                    /* Reset RX to properly reinitialise LDE operation. */
                    dwt_rxreset();
                    sprintf(str,"E1\r\n");
                    port_stdio_transmit((uint8_t*)str, strlen(str));
                }
            }
            else
            {
                /* Turn Off DW1000 RX/TX */
                dwt_forcetrxoff();

                /* Received unexpected frame */
                dwt_rxreset();
                sprintf(str,"E2\r\n");
                port_stdio_transmit((uint8_t*)str, strlen(str));
            }
        }
        else
        {
            /* Clear RX error events in the DW1000 status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

            /* Turn Off DW1000 RX/TX */
            dwt_forcetrxoff();

            /* Reset RX to properly reinitialise LDE operation. */
            dwt_rxreset();
            sprintf(str,"E3: 0x%08lx\r\n", status_reg & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR));
            port_stdio_transmit((uint8_t*)str, strlen(str));
        }
        if(frame_cnt == nb_twr)
        {
            sprintf(str,"All %d TWRs done.\r\n", nb_twr);
            port_dbg_print((uint8_t*)str, strlen(str));
            sprintf(str,"PDoA Avg (deg): %d \r\n", (int32_t)pdoa_sum/frame_cnt);
            port_stdio_transmit((uint8_t*)str, strlen(str));
            break;
        }
    }
    port_irq_enable_dw_irq();
    return VALID;
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn get_rx_timestamp_u64()
*
* @brief Get the RX time-stamp in a 64-bit variable.
*        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
*
* @param  none
*
* @return  unsigned 64-bit value of the read time-stamp.
*/

uint64_t get_rx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64_t ts = 0;
    int i;
    dwt_readrxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}


uint64_t get_tx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64_t ts = 0;
    int i;
    dwt_readtxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn final_msg_set_ts()
*
* @brief Fill a given timestamp field in the response message with the given value. In the timestamp fields of the
*        response message, the least significant byte is at the lower address.
*
* @param  ts_field  pointer on the first byte of the timestamp field to fill
*         ts  timestamp value
*
* @return none
*/
void resp_msg_set_ts(uint8_t *ts_field, const uint64_t ts)
{
    int i;
    for (i = 0; i < RESP_MSG_TS_LEN-1; i++)
    {
        ts_field[i] = (ts >> (i * 8)) & 0xFF;
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn resp_msg_get_ts()
*
* @brief Read a given timestamp value from the response message. In the timestamp fields of the response message, the
*        least significant byte is at the lower address.
*
* @param  ts_field  pointer on the first byte of the timestamp field to get
*         ts  timestamp value
*
* @return none
*/
void resp_msg_get_ts(uint8 *ts_field, int32 *ts)
{
    int i;
    *ts = 0;
    for (i = 0; i < RESP_MSG_TS_LEN-1; i++)
    {
        *ts += ts_field[i] << (i * 8);
    }
}

/*******************************************************************************
 * @fn cmd_dump_otp
 * @brief Read and prinout DW1000 current OTP memory
 * @return Valid if test successful, INVALID if not
 */
check_return_t cmd_dump_otp(void)
{
    uint16 addr = 0;
    uint32 buf = 0;

    port_spi_set_slowrate();

    for (addr=0; addr<31; addr++)
    {
        dwt_otpread(addr,&buf,1);
        sprintf(str,"0x%08x=0x%08x\r\n",(unsigned int)addr, (unsigned int) buf );
        port_stdio_transmit((uint8_t*)str, strlen(str));
    }

    port_spi_set_fastrate();
    return VALID;
}

/**
 * @brief   otp write
 *
 * */
check_return_t cmd_otp_write(void)
{
    //const char* ret_ptr = NULL;
    uint32          value = 0;
    uint16          address;
    uint32          buf = 0;

    if (cmd_get_argc() != 3)
    {
        sprintf(str, "Usage: %s <value> <address>\r\n", cmd_get_arg(0));
        port_dbg_print((uint8_t*)str, strlen(str));
        return INVALID;
    }

    value = cmd_get_arg_num(1);
    address = cmd_get_arg_num(2);

    port_spi_set_slowrate();
        port_reset_dw1000();

        // ============ debug info below ============
        sprintf(str,  "\tTo write: Value=0x%08lx, Address 0x%04x\r\n", value, (unsigned int)address);
        port_dbg_print((uint8_t*)str, strlen(str));

        if(address > OTP_ADDR_MAX)
        {
        sprintf(str, "OTP address out of range 0x00 - 0x%02x\r\n", OTP_ADDR_MAX);
        port_dbg_print((uint8_t*)str, strlen(str));
        return INVALID;
        }

        dwt_otpread(address, &buf, 1);
        //============ debug info below ============
        sprintf(str,  "\tExisting: Value=0x%08x\r\n", (unsigned int)buf );
        port_dbg_print((uint8_t*)str, strlen(str));

        if(!OTP_VALID(buf))// checking the whole 32 bits.
        {
                // ============ debug info below ============
                sprintf(str, "\tActual write: 0x%08x\r\n", (unsigned int)value);
                port_dbg_print((uint8_t*)str, strlen(str));

                if(dwt_otpwriteandverify(value, address) == DWT_SUCCESS)
                {
                    return VALID;
                }
        }
        else
        {
                sprintf(str, "OTP not valid... \r\n");
                port_dbg_print((uint8_t*)str, strlen(str));
        return INVALID;
        }

    return VALID;
}

/*******************************************************************************
 * @fn cmd_uwb_uotp
 * @brief Write DW1000 OTP memory with input arguments:
 * @ args TxPower_ch9,  uint32_t
          TxPower_ch5,  uint32_t
          PgCount_ch9,  uint16_t
          PgCount_ch5,  uint16_t
          PgDelay_ch9,  uint8_t
          PgDelay_ch5,  uint8_t
          AntDelay_ch9, uint32_t
          AntDelay_ch5, uint32_t
          Ref_temp,     uint8_t
          Ref_vbat,     uint8_t
          Xtral_trim,   uint8_t
 * @return Valid if test successful, INVALID if not
 */
check_return_t cmd_uwb_uotp(void)
{
    uint32_t tx_ch9;
    uint32_t tx_ch5;
    uint16_t pgcnt_ch9;
    uint16_t pgcnt_ch5;
    uint8_t pgdly_ch9;
    uint8_t pgdly_ch5;
    uint32_t ant_dly_ch9;
    uint32_t ant_dly_ch5;
    uint32_t mask;
    uint8_t temp;
    uint8_t vbat;
    uint8_t trim;
    uint32 buf = 0;
    int rv;

    if (cmd_get_argc() != 12)
    {
        sprintf(str, "Usage: %s <tx_ch5> <tx_ch9> <pgcnt_ch5> <pgcnt_ch9> <pgdly_ch5> <pgdly_ch9> "
            "<ant_dly_ch5> <ant_dly_ch9> <temp> <vbat> <xtaltrim>\r\n", cmd_get_arg(0));
        port_stdio_transmit((uint8_t*)str, strlen(str));
        return INVALID;
    }

    tx_ch5 = cmd_get_arg_num(1);
    tx_ch9 = cmd_get_arg_num(2);
    pgcnt_ch5 =  cmd_get_arg_num(3);
    pgcnt_ch9 =  cmd_get_arg_num(4);
    pgdly_ch5 =  cmd_get_arg_num(5);
    pgdly_ch9 =  cmd_get_arg_num(6);
    ant_dly_ch5 =  cmd_get_arg_num(7);
    ant_dly_ch9 =  cmd_get_arg_num(8);
    temp =  cmd_get_arg_num(9);
    vbat =  cmd_get_arg_num(10);
    trim =  cmd_get_arg_num(11);

    port_spi_set_slowrate();

    dwt_otpread(ANT_DLY_ADDRESS,&buf,1);
    if (!OTP_VALID(buf))
    {
        rv = dwt_otpwriteandverify((ant_dly_ch5 << 16) | ant_dly_ch9,ANT_DLY_ADDRESS);
        if (rv < 0)
            return INVALID;
    }

    dwt_otpread(PG_CNT_ADDRESS,&buf,1);
    if (!OTP_VALID(buf))
    {
        rv = dwt_otpwriteandverify((pgcnt_ch5 << 16) | pgcnt_ch9,PG_CNT_ADDRESS);
        if (rv < 0)
            return INVALID;
    }

    dwt_otpread(TX_CH5_ADDRESS,&buf,1);
    if (!OTP_VALID(buf))
    {
        rv = dwt_otpwriteandverify(tx_ch5,TX_CH5_ADDRESS);
        if (rv < 0)
            return INVALID;
    }

    dwt_otpread(TX_CH9_ADDRESS,&buf,1);
    if (!OTP_VALID(buf))
    {
        rv = dwt_otpwriteandverify(tx_ch9,TX_CH9_ADDRESS);
        if (rv < 0)
            return INVALID;
    }

    dwt_otpread(PG_DLY_TRIM_ADDRESS,&buf,1);
    if (!OTP_VALID(buf & ~(0x0000ff00)))
    {    mask = buf & 0x0000ff00;
        rv = dwt_otpwriteandverify(((pgdly_ch5 << 24) | (pgdly_ch9 << 16) | mask |  trim), PG_DLY_TRIM_ADDRESS);
        if (rv < 0)
            return INVALID;
    }

    dwt_otpread(REF_ADDRESS,&buf,1);
    if (!OTP_VALID(buf))
    {
        mask = buf & 0xffff0000;
        rv = dwt_otpwriteandverify(((vbat << 8) | temp | mask), REF_ADDRESS);
        if (rv < 0)
            return INVALID;
    }

    port_spi_set_fastrate();

    return VALID;
}

/*******************************************************************************
 * @fn cmd_uwb_reg_get
 * @brief get register value from given address
 * @return Valid if successful, INVALID if not
 */
check_return_t cmd_uwb_reg_get(void)
{
    uint8_t reg;
    uint16_t off;
    uint32_t val;
    uint32_t addr_offset;

    if (cmd_get_argc() != 3) {
        sprintf(str,"Usage: %s <reg> <off> \r\n", cmd_get_arg(0));
        port_stdio_transmit((uint8_t*)str, strlen(str));
        return INVALID;
    }

    reg = cmd_get_arg_num(1);
    off = cmd_get_arg_num(2);
    addr_offset = (reg<<16) + off;

    val = dwt_read32bitoffsetreg(addr_offset, 0);

    sprintf(str,"uwb read: addr_offset=x%06x val=x%08x\r\n", addr_offset, (unsigned int)val);
    port_stdio_transmit((uint8_t*)str, strlen(str));

    return VALID;
}

/*******************************************************************************
 * @fn cmd_uwb_reg_set
 * @brief write register values into given address
 * @return Valid if successful, INVALID if not
 */
check_return_t cmd_uwb_reg_set(void)
{
    uint8_t reg;
    uint16_t off;
    uint32_t addr_offset;
    uint32_t val;

    if (cmd_get_argc() != 4) {
        sprintf(str,"Usage: %s <reg> <off> <uint32 val>\r\n", cmd_get_arg(0));
        port_stdio_transmit((uint8_t*)str, strlen(str));
        return INVALID;
    }

    reg = cmd_get_arg_num(1);
    off = cmd_get_arg_num(2);
    val = cmd_get_arg_num(3);

    addr_offset = (reg << 16) + off;

    dwt_write32bitoffsetreg(addr_offset, 0, val);

    sprintf(str,"uwb write: addr_offset=x%06x val=x%08x\r\n", addr_offset, (unsigned int)val);
    port_stdio_transmit((uint8_t*)str, strlen(str));

    return VALID;
}

/* Described in header file */
void dw_get_temp(float *temp, float *vbat)
{
    uint16_t tempvbat;

    //dwt_readtempvbat: returns  (temp_raw<<8)|(vbat_raw)
    tempvbat = dwt_readtempvbat();
    if (tempvbat) {
        *vbat = dwt_convertrawvoltage(tempvbat & 0xff);
        *temp = dwt_convertrawtemperature((tempvbat >> 8) & 0xff);
    } else {
        *vbat = 0;
        *temp = 0;
    }
}

/*****************************************************************************
 *  @fn     cmd_uwb_hw_test
 *  @brief  vbat and temp read
 *    @return VALID 
 *
 */
check_return_t cmd_uwb_hw_test(void)
{
    uint32_t devID;
    float vbat;
    float temp;
    int i;
    int left, right;

    /* Test device ID */
    devID = dwt_readdevid() ;
#ifdef DWM3000_C0_DEVID
    if (!((DWT_C0_PDOA_DEV_ID == devID) || (DWT_C0_DEV_ID == devID)))
#else
    if (devID != 0xffffffff)
#endif //DWM3000_C0_DEVID
    {
        sprintf(str,"id: 0x%08X - %s\r\n", devID, "passed");
        //sprintf(str,"DW3000 DevID cannot be read - Faulty SPI \r\n");
    }
    else
    {
        sprintf(str,"id: 0x%08X - %s\r\n", devID,"failed");
    }
    port_stdio_transmit((uint8_t*)str, strlen(str));

    dw_get_temp(&temp, &vbat);

    left = vbat;
    right = (vbat-(float)left)*100;
    sprintf(str,"vbat: %d.%d V\r\n", left, right);
    port_stdio_transmit((uint8_t*)str, strlen(str));
    left = temp;
    right = (temp-(float)left)*100;
    sprintf(str,"temp: %d.%d C\r\n", left, right);
    port_stdio_transmit((uint8_t*)str, strlen(str));

    return VALID;
}

/*****************************************************************************
 *  @fn     uwb_cfg_xxxxxx
 *    @return -1 if writings fails
 *    @return 0 if writings succeeds
 */
int uwb_cfg_ch(uint8_t ch)
{
    dwt_config_t *ch_cfg = &app.pConfig->dwt_config;

    switch (ch) {
    case 5:
    case 9:
            ch_cfg->chan = ch;
            break;
    default:
        sprintf(str,"%s: ch must be in {5, 9}\r\n", SEINVAL);
        port_stdio_transmit((uint8_t*)str, strlen(str));
        return DWT_ERROR;
    }

    return DWT_SUCCESS;
}

int uwb_cfg_code(uint8_t code)
{
    dwt_config_t *ch_cfg = &app.pConfig->dwt_config;

    switch (code) {
    case 9:
    case 10:
    case 11:
    case 12: 
        ch_cfg->rxCode = code;
        break;
    default:
        sprintf(str,"%s: code must be in {9, 10, 11, 12}\r\n", SEINVAL);
        port_stdio_transmit((uint8_t*)str, strlen(str));
        return DWT_ERROR;
    }
    return DWT_SUCCESS;
}

int uwb_cfg_plen(uint16_t plen)
{
    dwt_config_t *ch_cfg = &app.pConfig->dwt_config;

    switch (plen) {
    case 64: ch_cfg->txPreambLength = DWT_PLEN_64; break;
    case 128: ch_cfg->txPreambLength = DWT_PLEN_128; break;
    case 256: ch_cfg->txPreambLength = DWT_PLEN_256; break;
    case 512: ch_cfg->txPreambLength = DWT_PLEN_512; break;
    case 1024: ch_cfg->txPreambLength = DWT_PLEN_1024; break;
    case 1536: ch_cfg->txPreambLength = DWT_PLEN_1536; break;
    case 2048: ch_cfg->txPreambLength = DWT_PLEN_2048; break;
    case 4096: ch_cfg->txPreambLength = DWT_PLEN_4096; break;
    default:
        sprintf(str,"%s: plen must be in {64 ~ 4096}\r\n", SEINVAL);
        port_stdio_transmit((uint8_t*)str, strlen(str));
        return DWT_ERROR;
    }
    return DWT_SUCCESS;
}

int uwb_cfg_pac(uint8_t rxpac)
{
    dwt_config_t *ch_cfg = &app.pConfig->dwt_config;

    switch (rxpac) {
    case 8: ch_cfg->rxPAC = DWT_PAC8; break;
    case 16: ch_cfg->rxPAC = DWT_PAC16; break;
    case 32: ch_cfg->rxPAC = DWT_PAC32; break;
    default:
        sprintf(str,"%s: rxpac must be in {8, 16, 32}\r\n", SEINVAL);
        port_stdio_transmit((uint8_t*)str, strlen(str));
        return DWT_ERROR;
    }
    return DWT_SUCCESS;
}

int uwb_cfg_datarate(uint16_t datarate)
{
    dwt_config_t *ch_cfg = &app.pConfig->dwt_config;

    switch (datarate) {
    case 850: ch_cfg->dataRate = DWT_BR_850K; break;
    case 6810: ch_cfg->dataRate = DWT_BR_6M8; break;
    default:
        sprintf(str,"%s: rxpac must be in {850, 6810}\r\n", SEINVAL);
        port_stdio_transmit((uint8_t*)str, strlen(str));
        return DWT_ERROR;
    }
    return DWT_SUCCESS;
}

int uwb_cfg_nssfd(uint8_t nssfd)
{
    dwt_config_t *ch_cfg = &app.pConfig->dwt_config;

    switch (nssfd) {
    case 0: ch_cfg->sfdType = 0; break;
    case 1: ch_cfg->sfdType = 1; break;
    default:
        sprintf(str,"%s: nssfd must be in {0, 1}\r\n", SEINVAL);
        port_stdio_transmit((uint8_t*)str, strlen(str));
        return DWT_ERROR;
    }
    return DWT_SUCCESS;
}

int uwb_cfg_phrext(uint8_t phrext)
{
    dwt_config_t *ch_cfg = &app.pConfig->dwt_config;

    switch (phrext) {
    case 0: ch_cfg->phrMode = DWT_PHRMODE_STD; break;
    case 1: ch_cfg->phrMode = DWT_PHRMODE_EXT; break;
    default:
        sprintf(str,"%s: phrext must be in {0, 1}\r\n", SEINVAL);
        port_stdio_transmit((uint8_t*)str, strlen(str));
        return DWT_ERROR;
    }
    return DWT_SUCCESS;
}

int uwb_cfg_stsmode(uint8_t stsm)
{
    dwt_config_t *ch_cfg = &app.pConfig->dwt_config;

    if(stsm & ~DWT_STS_CONFIG_MASK)
    {
        sprintf(str,"%s: stsm must be in 0x%02x\r\n", SEINVAL, DWT_STS_CONFIG_MASK);
        port_stdio_transmit((uint8_t*)str, strlen(str));
        return DWT_ERROR;
    }
    else
    {
        ch_cfg->stsMode = stsm;
    }
    return DWT_SUCCESS;
}

int uwb_cfg_stslen(uint16_t stslen)
{
    dwt_config_t *ch_cfg = &app.pConfig->dwt_config;

    switch (stslen) {
    case 32: ch_cfg->stsLength = DWT_STS_LEN_32; break;
    case 64: ch_cfg->stsLength = DWT_STS_LEN_64; break;
    case 128: ch_cfg->stsLength = DWT_STS_LEN_128; break;
    case 256: ch_cfg->stsLength = DWT_STS_LEN_256; break;
    case 512: ch_cfg->stsLength = DWT_STS_LEN_512; break;
    case 1024: ch_cfg->stsLength = DWT_STS_LEN_1024; break;
    case 2048: ch_cfg->stsLength = DWT_STS_LEN_2048; break;
    default:
        sprintf(str,"%s: stslen must be in {32 ~ 2048}\r\n", SEINVAL);
        port_stdio_transmit((uint8_t*)str, strlen(str));
        return DWT_ERROR;
    }
    return DWT_SUCCESS;
}

int uwb_cfg_pdoamode(uint16_t pdoa)
{
    dwt_config_t *ch_cfg = &app.pConfig->dwt_config;

    switch (pdoa) {
    case DWT_PDOA_M0:
    case DWT_PDOA_M1:
    case DWT_PDOA_M3:
        ch_cfg->pdoaMode = pdoa;
        break;
    default:
        sprintf(str,"%s: pdoa mode must be in {0, 1, 3}\r\n", SEINVAL);
        port_stdio_transmit((uint8_t*)str, strlen(str));
        return DWT_ERROR;
    }
    return DWT_SUCCESS;
}
 
/*****************************************************************************
 *  @fn     cmd_uwb_param_set
 *  @brief  set parameters
 *    @return VALID if writings succeeds
 *
 */
check_return_t cmd_uwb_param_set(void)
{
    dwt_config_t *ch_cfg = &app.pConfig->dwt_config;
 
    uint8_t ch;
    uint8_t ch_old = ch_cfg->chan;
    //uint8_t prf = 64;
    uint8_t code = 9;
    uint8_t rxpac = 8;
    uint16_t plen = 128;
    uint16_t datarate = 6810;
    uint8_t nssfd = 1;
    uint8_t phrext = 0;

    uint8_t argc = cmd_get_argc();

    if (argc == 2) {
        ch =    cmd_get_arg_num(1);
    } else if (argc == 8) {
        ch =    cmd_get_arg_num(1);
        code =   cmd_get_arg_num(2);
        plen =  cmd_get_arg_num(3);
        rxpac = cmd_get_arg_num(4);
        datarate = cmd_get_arg_num(5);
        nssfd =  cmd_get_arg_num(6);
        phrext= cmd_get_arg_num(7);
    } else {
        //usage
        sprintf(str,"Usage: %s <ch> <code> <plen> <rxpac> <baud_kbps> <nssfd> <phrext>\r\n", cmd_get_arg(0));
        port_stdio_transmit((uint8_t*)str, strlen(str));
        sprintf(str,"Example: %s 5 9 128 8 6810 1 0\r\n", cmd_get_arg(0));
        port_stdio_transmit((uint8_t*)str, strlen(str));
        return INVALID;
    }

    uwb_off();
    
    if(uwb_cfg_ch(ch) == DWT_ERROR)  return INVALID;
    if(uwb_cfg_code(code) == DWT_ERROR)  return INVALID;
    if(uwb_cfg_plen(plen) == DWT_ERROR)  return INVALID;
    if(uwb_cfg_pac(rxpac) == DWT_ERROR)  return INVALID;
    if(uwb_cfg_datarate(datarate) == DWT_ERROR)  return INVALID;
    if(uwb_cfg_nssfd(nssfd) == DWT_ERROR)  return INVALID;
    if(uwb_cfg_phrext(phrext) == DWT_ERROR)  return INVALID;

    stdto_update();

    dwt_configure(ch_cfg);
    port_config_save();

#if defined(CFG_DW_DEBUG) && (CFG_DW_DEBUG == 1)
        dw_dump_cfg(uwb0);
#endif /* CFG_DW_DEBUG */

    if(ch_old != ch)
    {
        if(cmd_uwb_reset() == INVALID)
        {
            return INVALID;
        }
        sprintf(str,"channel changed: %d -> %d\r\n", ch_old, ch);
        port_stdio_transmit((uint8_t*)str, strlen(str));
    }

    return VALID;
}

/*****************************************************************************
 *  @fn     cmd_uwb_pdoa_param_set
 *  @brief  set parameters including pdoa
 *    @return VALID if writings succeeds
 *
 */
check_return_t cmd_uwb_pdoa_param_set(void)
{
    dwt_config_t *ch_cfg = &app.pConfig->dwt_config;
 
    uint8_t ch_old = ch_cfg->chan;
    uint8_t ch;
    uint8_t code = 9;
    uint8_t rxpac = 8;
    uint16_t plen = 128;
    uint16_t datarate = 6810;
    uint8_t nssfd = 1;
    uint8_t phrext = 0;
    uint16_t stslen = 64;
    uint8_t stsmode = 9;
    uint8_t pdoamode = 3;

    uint8_t argc = cmd_get_argc();

    if (argc == 2) {
        ch =    cmd_get_arg_num(1);
    } else if (argc == 11) {
        ch = cmd_get_arg_num(1);
        code = cmd_get_arg_num(2);
        plen = cmd_get_arg_num(3);
        rxpac = cmd_get_arg_num(4);
        datarate = cmd_get_arg_num(5);
        nssfd = cmd_get_arg_num(6);
        phrext = cmd_get_arg_num(7);
        stslen = cmd_get_arg_num(8);
        stsmode = cmd_get_arg_num(9);
        pdoamode = cmd_get_arg_num(10);
    } else {
        //usage
        sprintf(str,"Usage: %s <ch> <code> <plen> <rxpac> <baud_kbps> <nssfd> <phrext> <stslen> <stsmode> <pdoa>\r\n", cmd_get_arg(0));
        port_stdio_transmit((uint8_t*)str, strlen(str));
        sprintf(str,"Example: %s 5 9 128 8 6810 1 0 64 9 3\r\n", cmd_get_arg(0));
        port_stdio_transmit((uint8_t*)str, strlen(str));
        return INVALID;
    }

    uwb_off();

    if(uwb_cfg_ch(ch) == DWT_ERROR)  return INVALID;
    if(uwb_cfg_code(code) == DWT_ERROR)  return INVALID;
    if(uwb_cfg_plen(plen) == DWT_ERROR)  return INVALID;
    if(uwb_cfg_pac(rxpac) == DWT_ERROR)  return INVALID;
    if(uwb_cfg_datarate(datarate) == DWT_ERROR)  return INVALID;
    if(uwb_cfg_nssfd(nssfd) == DWT_ERROR)  return INVALID;
    if(uwb_cfg_phrext(phrext) == DWT_ERROR)  return INVALID;
    if(uwb_cfg_stslen(stslen) == DWT_ERROR)  return INVALID;
    if(uwb_cfg_stsmode(stsmode) == DWT_ERROR)  return INVALID;
    if(uwb_cfg_pdoamode(pdoamode) == DWT_ERROR)  return INVALID;

    stdto_update();

    dwt_configure(ch_cfg);
    port_config_save();

#if defined(CFG_DW_DEBUG) && (CFG_DW_DEBUG == 1)
        dw_dump_cfg(uwb0);
#endif /* CFG_DW_DEBUG */

    if(ch_old != ch)
    {
        if(cmd_uwb_reset() == INVALID)
        {
            return INVALID;
        }
        sprintf(str,"channel changed: %d -> %d\r\n", ch_old, ch);
        port_stdio_transmit((uint8_t*)str, strlen(str));
    }

    return VALID;
}

/*****************************************************************************
 *  @fn     cmd_uwb_upg
 *  @brief  Calculate PGCOUNT based on currently set channel.
 *    @return VALID if PGCOUNT was calculated and print PGCOUNT
 *
 */
check_return_t cmd_uwb_upg(void)
{
    uint8_t ch;
    uint8_t pgdly;
    uint16_t pgcnt;

    ch = app.pConfig->dwt_config.chan;

    switch(ch)
    {
        case 5:
            pgdly = DEFAULT_PGDLY_CH5; //DEFAULT_CONFIG.dw_dev.txcfg[0].PGdly;
            break ;

        case 9:
            pgdly = DEFAULT_PGDLY_CH9; //DEFAULT_CONFIG.dw_dev.txcfg[1].PGdly;
            break;

        default:
            return INVALID;
    }

    port_spi_set_slowrate();

    pgcnt = dwt_calcpgcount(pgdly, ch);

    port_spi_set_fastrate();

    sprintf(str,"pgcnt: %d \r\n",pgcnt);
    port_stdio_transmit((uint8_t*)str, strlen(str));
    return VALID;
}

/*****************************************************************************
 *  @fn     cmd_uwb_udw
 *  @brief  read dw device ID to validate SPI connection within module
 *    @return VALID if ID is correct
 *
 */
check_return_t cmd_uwb_udw(void)
{
    uint32_t devID;
    devID = dwt_readdevid();
#ifdef DWM3000_C0_DEVID
    if (!((DWT_C0_PDOA_DEV_ID == devID) || (DWT_C0_DEV_ID == devID)))
    {
        //wake up device from low power mode
        //NOTE - in the ARM  code just drop chip select for 200us
        port_wakeup_dw3000();
        // SPI not working or Unsupported Device ID
        devID = dwt_readdevid() ;

        if (!((DWT_C0_PDOA_DEV_ID == devID) || (DWT_C0_DEV_ID == devID)))
        {
            sprintf(str,"DW3000 DevID cannot be read - Faulty SPI \r\n");
            port_stdio_transmit((uint8_t*)str, strlen(str));
            return INVALID;
        }
    }
#endif //DWM3000_C0_DEVID

    sprintf(str,"DW DevID : %lx \r\n",devID);
    port_stdio_transmit((uint8_t*)str, strlen(str));
    return VALID;

}

/*****************************************************************************
 *  @fn     print_32bitreg
 *  @brief  Read DW1000 32 bit registers and print it out
 *    @return VALID if ID is correct
 *
 */
void print_32bitreg(uint8_t reg_id, int i) {
    unsigned int reg;

    reg = dwt_read32bitoffsetreg(reg_id,i) ;
    sprintf(str,"reg[%02X:%02X]=%08X\r\n",reg_id,i,reg) ;
    port_stdio_transmit((uint8_t*)str, strlen(str));
}


/*****************************************************************************
 *  @fn     cmd_uwb_urd
 *  @brief  Dump DW1000 registers status
 *    @return VALID if ID is correct
 *
 */
check_return_t cmd_uwb_urd(void)
{
    uint8 buff[5];
    int i;

    //first print all single registers
    for(i=0; i<0x3F; i++)
    {
        dwt_readfromdevice(i, 0, 5, buff) ;
        sprintf(str,"reg[%02X]=%02X%02X%02X%02X%02X\r\n",i,buff[4], buff[3], buff[2], buff[1], buff[0] ) ;
        port_stdio_transmit((uint8_t*)str, strlen(str));
    }

    //reg 0x20
    for(i=0; i<=32; i+=4)
    {
        print_32bitreg(0x20,i);
    }

    //reg 0x21
    for(i=0; i<=44; i+=4)
    {
        print_32bitreg(0x21,i);
    }

    //reg 0x23
    for(i=0; i<=0x20; i+=4)
    {
        print_32bitreg(0x23,i);
    }

    //reg 0x24
    for(i=0; i<=12; i+=4)
    {
        print_32bitreg(0x24,i);
    }

    //reg 0x27
    for(i=0; i<=44; i+=4)
    {
        print_32bitreg(0x27,i);
    }

    //reg 0x28
    for(i=0; i<=64; i+=4)
    {
        print_32bitreg(0x28,i);
    }

    //reg 0x2A
    for(i=0; i<20; i+=4)
    {
        print_32bitreg(0x2A,i);
    }

    //reg 0x2B
    for(i=0; i<24; i+=4)
    {
        print_32bitreg(0x2B,i);
    }

    //reg 0x2f
    for(i=0; i<40; i+=4)
    {
        print_32bitreg(0x2f,i);
    }

    //reg 0x31
    for(i=0; i<84; i+=4)
    {
        print_32bitreg(0x31,i);
    }

//    //reg 0x36 = PMSC_ID
//    for(i=0; i<=48; i+=4)
//    {
//        print_32bitreg(PMSC_ID,i);
//    }
    port_spi_set_fastrate();

    return VALID;
}

/*****************************************************************************
 *  @fn     cmd_uwb_sleep
 *  @brief  Set DW1000, and MCU and peripherials in Sleep mode for current measurement during production
 *          Objective is to verify that unit under test consumption is confirm to expectation
 *    @return VALID if ID is correct
 *
 */
check_return_t cmd_uwb_sleep(void)
{
    uint16_t dly_ms;
    if (cmd_get_argc() != 2)
    {
        sprintf(str,"Usage: %s <dly_ms>\r\n", cmd_get_arg(0));
        port_stdio_transmit((uint8_t*)str, strlen(str));
        return INVALID;
    }

    dly_ms = cmd_get_arg_num(1);

    /* DEEPSLEEP mode for DW1000 */
    //dwt_configuresleep(0x0140, 0x5); //configure DeepSleep and wake parameters, wakeup on chip select
    dwt_configuresleep(DWT_CONFIG, DWT_PRESRVE_SLP | DWT_WAKE_CS |  DWT_SLP_EN);//DWT_SLEEP |
    dwt_entersleep(DWT_DW_IDLE); //go to DeepSleep

    /* STM MCU in sleep mode for 10000 ms */
    port_wait(dly_ms);

    /* WakeUp DW3000 */
    port_wakeup_dw3000();
    return VALID;

}

check_return_t cmd_uwb_idle_pll(void)
{
    return VALID;
}

check_return_t cmd_uwb_idle_rc(void){
    return VALID;
}

/*****************************************************************************
 *  @fn     cmd_uwb_sync_test
 *  @brief  Configure SYNC pin as GPIO7 and set it to 1/0 based on input parameter
 *  @return VALID if ID is correct
 *
 */
check_return_t cmd_uwb_sync_test(void)
{
//    unsigned int reg;
//    uint8_t sync_val = 0;
//
//    if (cmd_get_argc() != 2)
//    {
//        sprintf(str,"Usage: %s <sync_val>\r\n", cmd_get_arg(0));
//        port_stdio_transmit((uint8_t*)str, strlen(str));
//        return INVALID;
//    }
//
//    sync_val = cmd_get_arg_num(1);
//
//    /* Configuring SYNC as GPIO7 */
//    dwt_enablegpioclocks();
//
//    /*Reading default value of GPIO register*/
//    reg = dwt_read32bitoffsetreg(GPIO_CTRL_ID, GPIO_MODE_OFFSET) ;
//    dwt_write32bitoffsetreg(GPIO_CTRL_ID, GPIO_MODE_OFFSET, reg | (1UL << 20));
//
//    dwt_setgpiodirection(GDM7,0);
//
//    if (sync_val)
//    {
//        dwt_setgpiovalue(GDM7,GDP7);
//        port_wait(1);
//    }
//    else
//    {
//        dwt_setgpiovalue(GDM7,0);
//        port_wait(1);
//
//        /*Setting SYNC to default state (non-gpio)*/
//        dwt_write32bitoffsetreg(GPIO_CTRL_ID, GPIO_MODE_OFFSET, reg & ~(3UL << 20));
//    }
    return VALID ;
}

void stdto_update(void)
{
    uint16_t plen;
    uint8_t pac;
    uint8_t sfd;
    uint16_t timeout;
    dwt_config_t * p_cfg = &app.pConfig->dwt_config;

    plen = deca_to_plen(p_cfg->txPreambLength);
    pac = deca_to_pac(p_cfg->rxPAC);
    sfd = deca_to_sfdlen(p_cfg->sfdType);

    p_cfg->sfdTO = plen - pac + sfd + 1;
}


