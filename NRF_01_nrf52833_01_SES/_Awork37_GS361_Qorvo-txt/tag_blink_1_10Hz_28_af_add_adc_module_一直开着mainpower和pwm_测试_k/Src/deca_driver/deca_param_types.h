/*! ----------------------------------------------------------------------------
 *  @file    deca_param_types.h
 *  @brief   Decawave general type definitions for configuration structures
 *
 * @attention
 *
 * Copyright 2013-2020 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 */
#ifndef _DECA_PARAM_TYPES_H_
#define _DECA_PARAM_TYPES_H_

#ifdef __cplusplus
extern "C" {
#endif
#include "deca_types.h"

#define NUM_BR 2
#define NUM_PRF 3           //supported 16, 64, and SCP
#define NUM_PACS 4			//supported sizes are 8, 16, 32 and 4
#define SFD_TYPE 4          //supported number of SFDs - standard 8 = 0, non-standard 8 = 1,  non-standard 16 = 2, 4z
#define NUM_CH   2          //supported channels are 5, 9
#define NUM_CH_ALL 10       //all channels are '0', '1', '2', '3', '4', 5, '6', '7', '8', '9'

#define NUM_CP_PRF_SUPPORTED 4

extern const uint8 dwnsSFDlen[SFD_TYPE];
extern const uint8 chan_idx[NUM_CH_ALL];
extern const float txpwr_compensation[NUM_CH];

#define MIXER_GAIN_STEP (0.5)
#define DA_ATTN_STEP    (2.5)

#ifdef __cplusplus
}
#endif

#endif


