/*
 * @file       cmd_console.h
 *
 * @brief      Port headers file to Nordic nRF52382 Tag Board for DW1001
 *
 * @author     Decawave
 *
 * @attention  Copyright 2018 (c) DecaWave Ltd, Dublin, Ireland.
 *             All rights reserved.
 */

#ifndef _CONFIG_H_
#define _CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "deca_device_api.h"

/*Number of channels to be characterized during calibration/production*/
/*For DWM1004: CH2, CH5 - 2 channels*/
#define NB_CHANNEL          2
#define FCONFIG_SIZE 0x100

/* TRX configuration parameters structure
 * Those parameters depends on channel*/
struct trx_config_param_struct {
    uint32_t ref_tx_pwr;
    uint16_t ref_pgcnt;
    uint8_t  ref_pgdly;
    float pdoa_offset;
    uint16_t rxa_ant_delay;
    uint16_t txa_ant_delay;
    uint16_t rxb_ant_delay;
};
typedef struct trx_config_param_struct trx_config_t;

/* DW1000 device structure -- store calibration data related to device unit */
struct dw_dev_struct {
    uint32_t part_id;
    uint32_t dev_id;
    uint32_t lot_id;
//    int8_t ref_temp;
//    int8_t ref_vbat;
    uint8_t xtaltrim;
    trx_config_t trx_ref_config[NB_CHANNEL];
    dwt_txconfig_t tx_config[NB_CHANNEL];
    uint32_t field_status;
};
typedef struct dw_dev_struct dw_dev_t;

/*
 * Pointers inside this structure not allowed -- except for dw_dev
 * Creating a structure dw_dev per channel will preserve all required
 * information in nvm before writing to OTP.
 * The structure dw_dev needs to be accessed through pointers in order to write
 * production variable to ensure dw_dev address is aligned.
 */
typedef struct param_block{
    dwt_config_t    dwt_config;
    //uint16_t        smartPowerEn;
    uint8_t         tagID[8];
    uint8_t         tagIDset;
    const char      version[64];
    dw_dev_t        dw_dev;
} param_block_t;

#ifdef __cplusplus
}
#endif

#endif /* _CONFIG_H_ */
