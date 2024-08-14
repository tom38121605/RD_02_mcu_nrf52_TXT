/*
 * @file       default_config.h
 *
 * @brief      Default configuration
 *
 * @author     Decawave
 *
 * @attention  Copyright 2018 (c) DecaWave Ltd, Dublin, Ireland.
 *             All rights reserved.
 */

#ifndef _DEFAULT_CONFIG_H_
#define _DEFAULT_CONFIG_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "deca_version.h"
#include "deca_regs.h"
#include "deca_device_api.h"

/* UWB config */
#define DEFAULT_CHANNEL             5
#define DEFAULT_TXPREAMBLENGTH      DWT_PLEN_128  //DWT_PLEN_1024
#define DEFAULT_RXPAC               DWT_PAC8
#define DEFAULT_PCODE               9
#define DEFAULT_SFD                 1  //! SFD type 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type
#define DEFAULT_DATARATE            DWT_BR_6M8
#define DEFAULT_PHRMODE             DWT_PHRMODE_STD
#define DEFAULT_PHRRATE             DWT_PHRRATE_STD
#define DEFAULT_SFDTO               (128 + 1 + 8 - 8)
//#define DEFAULT_STS_MODE            DWT_STS_MODE_OFF  // (DWT_STS_MODE_1 | DWT_STS_MODE_SDC) //DWT_STS_MODE_OFF    //!< STS mode (no STS, before PHR, after data or STS with no data)
#define DEFAULT_STS_LENGTH          DWT_STS_LEN_256   //!< STS length
#define DEFAULT_PDOA_MODE           DWT_PDOA_M3       //!< pdoa mode (e.g. Mode 3)

#define DEFAULT_ANTD                (513.484f * 1e-9 / DWT_TIME_UNITS) /*Total antenna delay*/

//#define  DEFAULT_CONFIG_PDOA
#if defined(DEFAULT_CONFIG_PDOA)
  #define DEFAULT_STS_MODE                    (DWT_STS_MODE_1 | DWT_STS_MODE_SDC)    
  #define DEFAULT_POLL_RX_TO_RESP_TX_DLY_UUS  1350
  #define DEFAULT_POLL_TX_TO_RESP_RX_DLY_UUS  900
  #define DEFAULT_RESP_RX_TIMEOUT_UUS         600
#else  // non-pdoa
  #define DEFAULT_STS_MODE                    DWT_STS_MODE_OFF   
  #define DEFAULT_POLL_RX_TO_RESP_TX_DLY_UUS  1050//450
  #define DEFAULT_POLL_TX_TO_RESP_RX_DLY_UUS  850//240
  #define DEFAULT_RESP_RX_TIMEOUT_UUS         300//210
#endif


#define DEFAULT_PGDLY_CH5 (0x34)
#define DEFAULT_TXPWR_CH5 (0xfdfdfdfd)
#define DEFAULT_PGDLY_CH9 (0x34)
#define DEFAULT_TXPWR_CH9 (0xfefefefe)


#define DEFAULT_CONFIG \
{\
        .dwt_config.chan            = DEFAULT_CHANNEL, \
        .dwt_config.txPreambLength  = DEFAULT_TXPREAMBLENGTH, \
        .dwt_config.rxPAC           = DEFAULT_RXPAC, \
        .dwt_config.txCode          = DEFAULT_PCODE, \
        .dwt_config.rxCode          = DEFAULT_PCODE, \
        .dwt_config.sfdType         = DEFAULT_SFD, \
        .dwt_config.dataRate        = DEFAULT_DATARATE, \
        .dwt_config.phrMode         = DEFAULT_PHRMODE, \
        .dwt_config.phrRate         = DEFAULT_PHRRATE, \
        .dwt_config.sfdTO           = DEFAULT_SFDTO, \
        .dwt_config.stsMode         = DEFAULT_STS_MODE, \
        .dwt_config.stsLength       = DEFAULT_STS_LENGTH, \
        .dwt_config.pdoaMode        = DEFAULT_PDOA_MODE, \
        .dw_dev = {\
                .xtaltrim = 0, \
                .trx_ref_config = {\
                        {\
                                .ref_tx_pwr = DEFAULT_TXPWR_CH5, .ref_pgcnt = 0, .ref_pgdly = DEFAULT_PGDLY_CH5, \
                                .rxa_ant_delay = (DEFAULT_ANTD/2), .txa_ant_delay = (DEFAULT_ANTD/2), .rxb_ant_delay = (DEFAULT_ANTD/2), \
                        },\
                        {\
                                .ref_tx_pwr = DEFAULT_TXPWR_CH9, .ref_pgcnt = 0, .ref_pgdly = DEFAULT_PGDLY_CH9, \
                                .rxa_ant_delay = (DEFAULT_ANTD/2), .txa_ant_delay = (DEFAULT_ANTD/2), .rxb_ant_delay = (DEFAULT_ANTD/2), \
                        }\
                },\
                .tx_config = { \
                        {.PGdly = DEFAULT_PGDLY_CH5, .power = DEFAULT_TXPWR_CH5, .PGcount = 0,}, \
                        {.PGdly = DEFAULT_PGDLY_CH9, .power = DEFAULT_TXPWR_CH9, .PGcount = 0,} \
                }\
        }\
}
#endif

