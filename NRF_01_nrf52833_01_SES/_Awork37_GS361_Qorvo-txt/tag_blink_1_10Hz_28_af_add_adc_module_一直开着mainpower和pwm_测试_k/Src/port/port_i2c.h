
#include "nrf_drv_twi.h"

#ifndef _PORT_I2C_H_
#define _PORT_I2C_H_

/* Number of possible TWI addresses. */
#define TWI_ADDRESSES      127

// void port_i2c_tx (nrf_drv_twi_t const * p_twi,
                  // uint8_t               address,
                  // uint8_t const *       p_data,
                  // uint8_t               length);
#define port_i2c_rx             nrf_drv_twi_rx

void port_i2c_tx (nrf_drv_twi_t const * p_twi,
                  uint8_t               address,
                  uint8_t const *       p_data,
                  uint8_t               length);
                  
void port_i2c_init (const nrf_drv_twi_t* p_twi, const nrf_drv_twi_config_t* p_twi_config);

void port_i2c_scan(const nrf_drv_twi_t* p_twi);
                  
ret_code_t port_i2c_reg_tx (nrf_drv_twi_t const * p_twi,
                  uint8_t               address,
                  uint8_t               reg,
                  uint8_t const *       p_data,
                  uint8_t               length);
                          
ret_code_t port_i2c_reg_rx (nrf_drv_twi_t const * p_twi,
                  uint8_t               address,
                  uint8_t               reg,
                  uint8_t const *       p_data,
                  uint8_t               length);
                  
#endif // _PORT_I2C_H_

