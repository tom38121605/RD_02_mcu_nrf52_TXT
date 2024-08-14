
#include "nrf_drv_twi.h"
#include "port_i2c.h"
#include "nrf_delay.h"
#include <string.h>

#define I2C_BUF_MAX     16
#define I2C_SUB_IDX     0

static uint8_t buf[I2C_BUF_MAX+1];

/**
 * @brief TWI initialization.
 */
void port_i2c_init (const nrf_drv_twi_t* p_twi, const nrf_drv_twi_config_t* p_twi_config)
{
    ret_code_t err_code;

    err_code = nrf_drv_twi_init(p_twi, p_twi_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(p_twi);
}
                          
void port_i2c_tx (nrf_drv_twi_t const * p_twi,
                  uint8_t               address,
                  uint8_t const *       p_data,
                  uint8_t               length)
{
    ret_code_t err_code;
    err_code = nrf_drv_twi_tx(p_twi, address, p_data, length, false);
    APP_ERROR_CHECK(err_code);
}

ret_code_t port_i2c_reg_tx (nrf_drv_twi_t const * p_twi,
                  uint8_t               address,
                  uint8_t               reg,
                  uint8_t const *       p_data,
                  uint8_t               length)
{
  ret_code_t err_code;
  buf[I2C_SUB_IDX] = reg;
  
  if( length > I2C_BUF_MAX)
  {
    return -1;
  }

  if(length >1)
  {
    buf[I2C_SUB_IDX] |= 0x80;
  }
  memcpy(buf+1, p_data, length);

  err_code = nrf_drv_twi_tx(p_twi, address, buf, length+1, true);
  APP_ERROR_CHECK(err_code);
  return err_code;
}
                          
ret_code_t port_i2c_reg_rx (nrf_drv_twi_t const * p_twi,
                  uint8_t               address,
                  uint8_t               reg,
                  uint8_t const *       p_data,
                  uint8_t               length)
{
  ret_code_t err_code;
  buf[I2C_SUB_IDX] = reg;

  if( length > I2C_BUF_MAX)
  {
    return -1;
  }

  if(length >1)
  {
    buf[I2C_SUB_IDX] |= 0x80;
  }
  err_code = nrf_drv_twi_tx(p_twi, address, buf, 1, false);
  APP_ERROR_CHECK(err_code);

  err_code = nrf_drv_twi_rx(p_twi, address, (uint8_t*)p_data, length);
  APP_ERROR_CHECK(err_code);

  return err_code;
}

void port_i2c_scan(const nrf_drv_twi_t* p_twi)
{
    ret_code_t err_code;
    uint8_t address;
    uint8_t reg_offset;
    uint8_t sample_data;
    bool detected_device = false;
    
    for (address = 1; address <= TWI_ADDRESSES; address++)
    {
        err_code = nrf_drv_twi_rx(p_twi, address, &sample_data, sizeof(sample_data));
        if (err_code == NRF_SUCCESS)
        {
            detected_device = true;
            printf("TWI device detected at address 0x%x\r\n", address);
            for (reg_offset = 0; reg_offset <= 0x7f; reg_offset++)
            {
                err_code = port_i2c_reg_rx(p_twi, address, reg_offset, &sample_data, 1);
                if (err_code == NRF_SUCCESS)
                {
                    printf("\treg 0x%x: 0x%02x \r\n", reg_offset, sample_data);
                    nrf_delay_ms(1);
                }                
            }
        }
    }

    if (!detected_device)
    {
        printf("No device was found.\r\n");
    }
}





