/*! ----------------------------------------------------------------------------
 *  @file    deca_params_init.c
 *  @brief   DW3000 configuration parameters
 *
 * @attention
 *
 * Copyright 2013-2020 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 *
 * -------------------------------------------------------------------------------------------------------------------
**/
#include <stdio.h>
#include <stdlib.h>

#include "deca_regs.h"
#include "deca_device_api.h"
#include "deca_param_types.h"


//-----------------------------------------
// map the channel number to the index in the configuration arrays below
// 0th element is chan 5, 1st is chan 9,
const uint8 chan_idx[NUM_CH_ALL] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 1 };

//-----------------------------------------


const uint8 dwnsSFDlen[SFD_TYPE] =
{
    8, 8, 16, 8
};


const float txpwr_compensation[NUM_CH] = {
    0.065f,
    0.0f
};



