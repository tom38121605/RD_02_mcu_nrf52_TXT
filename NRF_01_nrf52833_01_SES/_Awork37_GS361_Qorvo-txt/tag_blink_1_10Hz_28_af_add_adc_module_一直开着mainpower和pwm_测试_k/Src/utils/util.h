/*! ---------------------------------------------------------------------------
 * @file    util.h
 * @brief    DecaWave Application Layer utility functions & Macros
 *
 * @attention
 *
 * Copyright 2017 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author
 */

#ifndef __UTIL__H__
#define __UTIL__H__ 1

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>

#ifndef SWAP
#define SWAP(a,b) {a^=b;b^=a;a^=b;}
#endif /* SWAP */

#ifndef STR
#define STR_IMPL_(x) #x
#define STR(x) STR_IMPL_(x)
#endif /* STR */

#define AR2U32(x)               (((uint32_t)x[3])<<24 |\
                                ((uint32_t)x[2])<<16  |\
                                ((uint32_t)x[1])<<8   |\
                                ((uint32_t)x[0]))

#define AR2U16(x)               ((x[1]<<8) | x[0])


#define TS2U64_MEMCPY(x,y) do{\
                            x = (uint64_t)(((uint64_t)y[4]<<32)|\
                                ((uint64_t)y[3]<<24)|\
                                ((uint64_t)y[2]<<16)|\
                                ((uint64_t)y[1]<<8) |\
                                y[0]); \
                         }while(0)

#define TS2TS_MEMCPY(x,y)  do {\
                            for(int i=0;i<TS_40B_SIZE;i++) {x[i] = y[i];}\
                          }while(0)

#define TS2TS_UWB_MEMCPY(x,y)  do {\
                            for(int i=0;i<TS_UWB_SIZE;i++) {x[i] = y[i];}\
                          }while(0)

#define U642TS_MEMCPY(x,y) do {\
                            for(int i=0;i<TS_40B_SIZE;i++) {x[i] = (uint8_t)((y>>(i*8)&0xFF));}\
                          }while(0)

#define U642TS_UWB_MEMCPY(x,y) do {\
                            for(int i=0;i<TS_UWB_SIZE;i++) {x[i] = (uint8_t)((y>>(i*8)&0xFF));}\
                          }while(0)

#define U32TOAR_MEMCPY(x,y) do {\
                            for(int i=0;i<4;i++) {x[i] = (uint8_t)((y>>(i*8)&0xFF));}\
                          }while(0)

typedef enum {
    VALID = 1,
    INVALID = -1
} check_return_t ;

uint64_t util_us_to_dev_time (double us);
double   util_dev_time_to_sec(uint64_t dt);
uint64_t util_sec_to_dev_time (double sec);
double util_us_to_sy(double us);

int16_t  calc_sfd_to(void * pCfg);
uint16_t calc_rx_to_sy(void *p, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __UTIL__H__ */
