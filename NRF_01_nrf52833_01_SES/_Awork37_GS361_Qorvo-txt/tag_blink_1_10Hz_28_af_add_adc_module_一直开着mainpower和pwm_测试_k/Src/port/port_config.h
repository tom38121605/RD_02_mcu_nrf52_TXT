/*! ----------------------------------------------------------------------------
 * @file    port_config.h
 * @brief   Platform specific configuration access
 *
 * Allows storing and retrieving configuration from RAM andnon-volatile memory.
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

#ifndef _PORT_CONFIG_H_
#define _PORT_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "config.h"
#include "nrf_nvmc.h"

/*! ----------------------------------------------------------------------------
 * @fn port_config_load
 * @brief Load the configuration from non-volatile memory
 * @return -1 if on failure, 0 on success
 */
int port_config_load(void);

/*! ----------------------------------------------------------------------------
 * @fn port_config_get
 * @brief Get the configuration available in RAM
 * @return A pointer to the configuration
 */
param_block_t* port_config_get(void);

/*! ----------------------------------------------------------------------------
 * @fn config_save
 * @brief Save the configuration in non-volatile memory
 * @return -1 on failure, 0 on success
 */
int port_config_save(void);

/*! ----------------------------------------------------------------------------
 * @fn config_restore_default
 * @brief  Restore default configuration and save to nvm
 * @return -1 on failure, 0 on success
 */
int port_config_restore_default(void);

#ifdef __cplusplus
}
#endif

#endif /* _PORT_CONFIG_H_ */
