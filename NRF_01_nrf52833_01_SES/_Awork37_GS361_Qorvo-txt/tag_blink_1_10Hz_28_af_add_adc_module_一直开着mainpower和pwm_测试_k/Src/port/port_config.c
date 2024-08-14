/*! ----------------------------------------------------------------------------
 * @file    port_config.c
 * @brief   Platform specific configuration access
 *
 * Allows storing and retrieving configuration from RAM and non-volatile memory.
 * The non-volatile memory is typically a region in the embedded flash or
 * eeprom, external storage like external flash, eeprom, or an SD card could
 * also be used
 *
 * @attention
 *
 * Copyright 2019 (c) DecaWave Ltd, Dublin, Ireland.
 * All rights reserved.
 *
 * @author DecaWave
 */

#include <string.h>

#include "default_config.h"
#include "port_config.h"
#include "config.h"

/* Application default constant parameters block in the NFLASH. Never changes.
 * Used for Restore command ONLY. */
static const param_block_t default_config = DEFAULT_CONFIG;

/* Runtime configuration */
static param_block_t config = DEFAULT_CONFIG;

/*NVM config */
const param_block_t nvm_config __attribute__((section(".fConfig"))) \
                                   __attribute__((aligned(FCONFIG_SIZE))) = DEFAULT_CONFIG;

/*! ----------------------------------------------------------------------------
 * @fn port_config_load()
 * @brief Abstract function to load configuration from non-volatile memory
 * 
 * The configuration should be stored in an object in RAM managed by this module
 * 
 * @return -1 if on failure, 0 on success
 */
int port_config_load(void)
{
    memcpy(&config, &nvm_config, sizeof(param_block_t));
    return 0;
}

/*! ----------------------------------------------------------------------------
 * @fn port_config_get
 * @brief Get the configuration available in RAM
 * @return A pointer to the configuration
 */
param_block_t* port_config_get(void)
{
    return &config;
}

/*! ----------------------------------------------------------------------------
 * @fn port_config_save
 * @brief Save the configuration in non-volatile memory
 * @return -1 on failure, 0 on success
 */
int port_config_save(void)
{
    __disable_irq();
    nrf_nvmc_page_erase((uint32_t)&nvm_config);

    nrf_nvmc_write_bytes((uint32_t)&nvm_config, (const uint8_t*) &config, FCONFIG_SIZE);
    __enable_irq();
    
    return 0;
}

/*! ----------------------------------------------------------------------------
 * @fn config_restore_default
 * @brief  Restore default configuration and save to nvm
 * @return -1 on failure, 0 on success
 */
int port_config_restore_default(void)
{
    memcpy(&config, &default_config, sizeof(param_block_t));
    return port_config_save();
}
