
#include "lis2dh12_reg.h"
#include "port_i2c.h"







/* lis2dh12 instance. */
static stmdev_ctx_t dev_ctx;

extern const nrf_drv_twi_t m_twi;

static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */

static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp,
                              uint16_t len)
{
  port_i2c_reg_tx((const nrf_drv_twi_t*)handle, LIS2DH12_I2C_ADD_H>>1, reg, bufp, len);
  return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
  port_i2c_reg_rx((const nrf_drv_twi_t*)handle, LIS2DH12_I2C_ADD_H>>1, reg, bufp, len);
  return 0;
}

int imu_init(void)
{
  static uint8_t whoamI;  
  uint8_t reg_val;  

  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = (void*)&m_twi; 
  /*
   *  Check device ID
   */
  lis2dh12_device_id_get(&dev_ctx, &whoamI);
  if (whoamI != LIS2DH12_ID)
  { 
    printf("whoamI = 0x%02x, lis2dh12 not found.\r\n", whoamI);
    return -1;
  }


  lis2dh12_block_data_update_set(&dev_ctx, PROPERTY_DISABLE);//Disable Block Data Update

  lis2dh12_data_rate_set(&dev_ctx, LIS2DH12_ODR_10Hz); //Set Output Data Rate to 10Hz??

  lis2dh12_full_scale_set(&dev_ctx, LIS2DH12_2g);//Set full scale to 2g

  lis2dh12_high_pass_int_conf_set(&dev_ctx, LIS2DH12_ON_INT1_GEN);//i1

  lis2dh12_ctrl_reg3_t int1 = {0};
  int1.i1_ia1 = 1;
  lis2dh12_pin_int1_config_set(&dev_ctx, &int1); // int1 enable

  lis2dh12_int1_cfg_t int1cfg = {0};
  int1cfg.xhie = 1;
  int1cfg.yhie = 1;
  int1cfg.zhie = 1;
  lis2dh12_int1_gen_conf_set(&dev_ctx, &int1cfg);//OR combination of interrupt events

  lis2dh12_int1_gen_duration_set(&dev_ctx, 0);// duration
  
  lis2dh12_int1_gen_threshold_set(&dev_ctx, 0x8); //   threshold = 20


//
//  /*
//   * Enable temperature sensor
//   */  
//  lis2dh12_temperature_meas_set(&dev_ctx, LIS2DH12_TEMP_DISABLE);
//
//  /*
//   * Set device in continuous mode with 12 bit resol.
//   */  
//  lis2dh12_operating_mode_set(&dev_ctx, LIS2DH12_HR_12bit);
 
  return 0;
}










