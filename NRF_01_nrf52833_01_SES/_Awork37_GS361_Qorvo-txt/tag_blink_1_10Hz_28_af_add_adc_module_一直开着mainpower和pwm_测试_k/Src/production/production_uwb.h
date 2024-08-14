/**
 * @file      production_cmd.h
 *
 * @brief     Commands required for module calibration, production and manufacturing
 *
 * @author    DecaWave Applications
 *
 * @attention Copyright 2017-2018 (c) DecaWave Ltd, Dublin, Ireland.
 *                    All rights reserved.
 */

#ifndef PRODUCTION_UWB_H_
#define PRODUCTION_UWB_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "util.h"
#include "config.h"

/*UCW */
/* Enable continuous wave mode */
check_return_t cmd_uwb_cw_mode(void);
  
/*UXT */
/* Perform crystal trim */
check_return_t cmd_uwb_set_xtaltrim(void);
  
/*UCF*/
/* Enable continuous frame mode */
check_return_t cmd_uwb_cf_mode();
  
/*UTPG*/
/* Get TX power */
check_return_t cmd_uwb_get_tx_power(void);
  
/*UTP*/
/* Set TX power */
check_return_t cmd_uwb_set_tx_power(void);
  
/*UTS*/
/* Set BW and TX power calibration */
check_return_t  cmd_uwb_uts(void);
  
/*UEE*/
/* Enable event counter */
check_return_t cmd_uwb_eventcounter_enable(void);
  
/*UED*/
/* Disable event counter */
check_return_t cmd_uwb_eventcounter_disable(void);
  
/*UEP*/
/* Print event counters */
check_return_t cmd_uwb_eventcounter_print(void);

/*URX*/
/* Enable receiver */
check_return_t cmd_uwb_rx(void);

check_return_t cmd_uwb_rx_enable(void);

check_return_t cmd_uwb_rx_disable(void);
  
/*UTX*/
/* Transmit a given number of test frames */
check_return_t cmd_uwb_tx(void);

/*UOF*/
/* Disable the transceiver */
check_return_t cmd_uwb_off(void);
  
/*URST*/
check_return_t cmd_uwb_reset(void);

/*UTVB*/
check_return_t  cmd_read_temp_vbat(void);
  
/*USA*/
/* Set antenna delay */
check_return_t cmd_uwb_set_ant_cfg(void);
  
/*UGA*/
/* Get antenna delay */
check_return_t cmd_uwb_get_ant_cfg(void);

/*UTVB*/
/*Read temperature and VBAT*/
check_return_t cmd_read_temp_vbat(void);

/*UTWRI*/
/* Two Way ranging - Initiator */
check_return_t cmd_twr_initiator(void);

/*UTWRR*/
/* Two Way Ranging - Responder*/
check_return_t cmd_twr_responder(void);
  
/*UOD*/
/* Printout DW_IC OTP memory */
check_return_t cmd_dump_otp(void);

/*UOW*/
/* Printout DW_IC OTP memory */
check_return_t cmd_otp_write(void);

/*UOTP*/
/* Write DW_IC OTP memory */
check_return_t cmd_uwb_uotp(void);

/*UDW*/
/* Read DW_IC Device ID */
check_return_t cmd_uwb_udw(void);

/*URD*/
/* Dump DW_IC registers status */
check_return_t cmd_uwb_urd(void);

/*Sleep*/
check_return_t cmd_uwb_sleep(void);

check_return_t cmd_uwb_idle_pll(void);
check_return_t cmd_uwb_idle_rc(void);


/*UWB_OFF*/
void uwb_off(void);

/*UPG*/
/* Calculate PGcount for current channel*/
check_return_t cmd_uwb_upg(void);

/*USY*/
/*Configure SYNC pin as GPIO7 and set it to 1/0 based on input parameter*/
check_return_t cmd_uwb_sync_test(void);

/*URS*/
/* Write DW_IC Register */
check_return_t cmd_uwb_reg_set(void);

/*URG*/
/* Read DW_IC Register */
check_return_t cmd_uwb_reg_get(void);

/*UHT*/
/* DW_IC hardware test: vbat and temp read*/
check_return_t cmd_uwb_hw_test(void);

/*UPS*/
/* DW_IC parameter set*/
check_return_t cmd_uwb_param_set(void);

/*UPS*/
/* DW_IC pdoa parameter set*/
check_return_t cmd_uwb_pdoa_param_set(void);

/*Internal commands*/
int uwbmac_tx_power_get(uint8_t *pg_delay, uint32_t *tx_power);

int uwbmac_tx_power_set(uint8_t pg_delay, uint32_t tx_power);

int dw_get_ant_cfg(uint16_t *txa_delay, uint16_t *rxa_delay, uint16_t *rxb_delay);

int dw_set_ant_cfg( uint16_t tx_delay, uint16_t rx_delay);

uint64_t get_rx_timestamp_u64(void);

void resp_msg_set_ts(uint8_t *ts_field, const uint64_t ts);

trx_config_t * current_ch_cfg(void);

void stdto_update(void);

#ifdef __cplusplus
}
#endif

#endif /* PRODUCTION_UWB_H_ */
