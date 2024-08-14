/**
 * @file      deca_phy.h
 *
 * @brief     Decawave PHY layer
 *
 * @author    Decawave
 *
 * @attention Copyright 2017-2019 (c) Decawave Ltd, Dublin, Ireland.
 *            All rights reserved.
 *
 */

#ifndef DECA_PHY_H_
#define DECA_PHY_H_    1

#ifdef __cplusplus
 extern "C" {
#endif

//#include "common.h"
//#include "rtls_interface.h"
//#include "uwb_frames.h"

 int phy_init(void);
//
//#define ADDR_BYTE_SIZE                8
//#define ADDR_BYTE_SIZE_SHORT          2
//
///* RF data packets */
//#pragma pack(push , 1)
//
//typedef enum {
//    APP_TX_ERROR = -1,
//    APP_TX_NONE = 0,
//    APP_TWR_TX_BLINK,
//    APP_TWR_TX_RANGE_INIT,
//    APP_TWR_TX_POLL,
//    APP_TWR_TX_RESP,
//    APP_TWR_TX_FINAL,
//    APP_TWR_TX_TOF,
//    APP_RTLS_TX_CLOCK_SYNC,
//    APP_RTLS_BH_COMMAND,
//    APP_RTLS_BH_DATA
//}txSent_e;
//
//struct phy_tx_data_s
//{
//    uint16_t psduLen;
//    union {
//        twr_msg_t twrMsg;
//        std_msg_t stdMsg;
//        bcast_msg_t bcastMsg;
//        blink_msg_t blinkMsg;
//        ack_msg_t ackMsg;
//        ccp_msg_t ccpMsg;
//    }__attribute__((__packed__)) msg;
//    uint32_t delayedTxTime;
//    uint8_t receiveAfterTx;
//};
//
//typedef struct phy_tx_data_s phy_tx_data_t;
//
//struct phy_rx_data_s
//{
//    uint16_t rxDataLen;
//    union {
//        twr_msg_t twrMsg;
//        std_msg_t stdMsg;
//        bcast_msg_t bcastMsg;
//        blink_msg_t blinkMsg;
//        blinkIMU_msg_t blinkIMUMsg;
//        ack_msg_t ackMsg;
//        ccp_msg_t ccpMsg;
//    }__attribute__((__packed__)) msg;
//
//    uint8_t ackRequested;
//
//    uint8_t timeStamp[TS_40B_SIZE];
//
//    uint16_t firstPath; //First path (raw 10.6)
//
//    //NOTE: diagnostics data format rev 5 (DWT_DIAGNOSTIC_LOG_REV)
//    //00        this could be a header (format version number)
//    //01        register 0xF - length 5 bytes
//    //06        register 0x10 - length 4 bytes
//    //10        register 0x12 - length 8 bytes
//    //18        register 0x13 - length 4 bytes
//    //22        register 0x14 - length 5 bytes
//    //27        register 0x15 - length 14 bytes (5 TS, 2 FP, 2 Diag, 5 TSraw)
//    //41        register 0x25 @first path - 16 bytes around first path + 1 dummy
//    //58        register 0x2E (0x1000) peakPathAmplitude - 2 bytes
//    //60        register 0x27 (0x28) 4 bytes
//    //64        register 0x2E (0x1002) peakPathIndex - 2 bytes
//    //66 total
//    union {
//            uint8_t     diagnostics[DWT_SIZEOFDIAGNOSTICDATA_5 * 2];
//            diag_v5_t   diag[2];
//    };
//};
//typedef struct phy_rx_data_s phy_rx_data_t;
//
//struct phy_tx_st_s
//{
//     txSent_e  txs;
//     uint8_t   timeStamp[TS_40B_SIZE];
//};
//
//typedef struct phy_tx_st_s phy_tx_st_t;
//
//struct events_tx_s
//{
//    phy_tx_st_t  tx[EVENT_TX_BUFSIZE];   // PHY-APP TxData circular buffer
//    uint16_t     head;
//    uint16_t     tail;
//};
//
//typedef struct events_tx_s events_tx_t;
//
//struct events_rx_s
//{
//    phy_rx_data_t rx[EVENT_RX_BUFSIZE]; // PHY-APP RxData circular buffer
//    uint16_t     head;
//    uint16_t     tail;
//    uint32_t    Lock;
//};
//
//typedef struct events_rx_s events_rx_t;
//
//struct phy_rx_cfg_s
//{
//    uint16_t frameFilter;
//    uint16_t frameFilterMode;
//
//    int16_t  rxTimeOut;
//    uint32_t RxAfterTxDelay;
//    uint32_t delayedRxTime;
//
//    uint8_t dblBufferMode;
//    uint8_t autoAckMode;
//};
//
//typedef struct phy_rx_cfg_s phy_rx_cfg_t;
//
//struct phy_cfg_s
//{
//    dwt_config_t    dwCfg;
//    dwt_txconfig_t  dwtxCfg;
//
//    phy_rx_cfg_t    rxCfg;
//
//    phy_tx_data_t txData;
//
//    int16_t txantennaDelay;
//    int16_t rxantennaDelay;
//
//    uint8_t readDiagnostics;
//    uint8_t readAccumulator;
//};
//
//typedef struct phy_cfg_s phy_cfg_t;
//
//#pragma pack(pop)
//
////// PHY API
////
//int  phy_init(void);
//int  phy_rxtx_configure(dwt_config_t * pdwCfg, phy_rx_cfg_t * prxCfg, uint16_t rxant_del , uint16_t txant_del, uint8_t lnaON);
//int  phy_rx_start(phy_rx_cfg_t * pCfg);
//int  phy_tx_start(phy_tx_data_t * ptxData);
////
//void phy_tx_callback(const dwt_cb_data_t *txd);
//void phy_rx_ok_callback(const dwt_cb_data_t *rxd);
//void phy_rx_rxto_callback(const dwt_cb_data_t *rxd);
//void phy_rx_error_callback(const dwt_cb_data_t *rxd);
//void phy_rx_ok_callback_with_diagnostic_logging(const dwt_cb_data_t *rxd);
////
//void phy_rtls_tx_cb(const dwt_cb_data_t *txd);
//void phy_rx_std_cb(const dwt_cb_data_t *rxd);
//void phy_rx_std_to_cb(const dwt_cb_data_t *rxd);
//void phy_rx_std_err_cb(const dwt_cb_data_t *rxd);
//void phy_rx_good_with_diagnostic_cb(const dwt_cb_data_t *rxd);
//void phy_rx_to_cb(const dwt_cb_data_t *txd);
//void phy_rx_err_cb(const dwt_cb_data_t *txd);
////
////void phy_rx_callback_double_buff_auto_reenable(const dwt_callback_data_t *rxd);
////
//void phy_id_configure(uint8_t *eui64,uint16_t panId);
//void phy_setup_diagnostics(uint8_t enable);
//void phy_configure_txpower(uint8_t chan, uint32_t power);
//void phy_setup_std_header(phy_tx_data_t *txMsg, const uint8_t * pCtrl, uint8_t seqNum );
//void phy_setup_ccp_header(phy_tx_data_t *txMsg, const uint8_t * pCtrl, uint8_t seqNum );
////
////void configureREKHW(int32_t chan, int32_t is628);
//#ifdef __cplusplus
//}
//#endif
////
#endif //DECA_PHY_H_
