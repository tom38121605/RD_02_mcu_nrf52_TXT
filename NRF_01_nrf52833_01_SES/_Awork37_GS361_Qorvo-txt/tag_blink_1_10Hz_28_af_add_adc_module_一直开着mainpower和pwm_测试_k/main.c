/*
 * @file       main.c
 * @author     Qorvo
 *
 * @attention  Copyright 2020 (c) Qorvo, Inc
 *             All rights reserved.
 *
 */

 
#include "production_main.h"
#include "port_stdio.h"
#include "port_spi.h"
#include "port_irq.h"
#include "port_i2c.h"
#include "lis2dh12_app.h"
#include "port_timer_blink.h"
#include "blink_app.h"
//#include "lis2dh12_reg.h"



int16_t adcbuf[4];

#define PWM_TOP  400    //20K  
#define DUTY  5 //50
#define PWMFIRST (PWM_TOP*DUTY/100) 
uint16_t ipwm_values[4];


void peripherals_init(void);
static char str1[256];

/* SPI instance ID. */
#define SPI_INSTANCE 0
const nrfx_spim_t spi = NRFX_SPIM_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */

/* TWI instance ID. */
#define TWI_INSTANCE_ID     1
const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

const nrf_drv_timer_t TIMER_BLINK = NRF_DRV_TIMER_INSTANCE(0);

/**
  * @brief  The application entry point.
  * @retval int
  */
void pwm_init()    
{
      NRF_PWM0->PSEL.OUT[0] = (PWM0_PIN << 0) |  (0 << 31UL);
      NRF_PWM0->ENABLE = (1 << 0);
      NRF_PWM0->MODE = (1 << 0);

      NRF_PWM0->PRESCALER = (0 << 0);  // 1 DIV, 16MHz
      //NRF_PWM0->PRESCALER = (4 << 0);  // 16 DIV, 1MHz

      //NRF_PWM0->COUNTERTOP = (16000 << 0); //1 ms, 1KHz
      NRF_PWM0->COUNTERTOP = (PWM_TOP << 0); //50us, 20KHz  xx


      NRF_PWM0->LOOP = (0 << 0);   //UP
      //NRF_PWM0->LOOP = (1 << 0);   //upanddown

      //NRF_PWM0->DECODER = (0 << 0) |  (0 << 8);
      NRF_PWM0->DECODER = (2 << 0) |  (0 << 8);


      NRF_PWM0->SEQ[0].PTR = ((uint32_t)&ipwm_values[0] << 0);
      NRF_PWM0->SEQ[0].CNT = ((sizeof(ipwm_values) / sizeof(uint16_t)) << 0);

      NRF_PWM0->SEQ[0].REFRESH = 1;
      NRF_PWM0->SEQ[0].ENDDELAY = 1;

      NRF_PWM0->SEQ[1].PTR = ((uint32_t)&ipwm_values[0] << PWM_SEQ_PTR_PTR_Pos);
      NRF_PWM0->SEQ[1].CNT = ((sizeof(ipwm_values) / sizeof(uint16_t)) <<  PWM_SEQ_CNT_CNT_Pos);
      NRF_PWM0->SEQ[1].REFRESH = 0;
      NRF_PWM0->SEQ[1].ENDDELAY = 0;

      NRF_PWM0->TASKS_SEQSTART[0] = 1;

      ipwm_values[0]=PWM_TOP-PWMFIRST; 
      //--ipwm_values[1]=PWM_TOP-PWMFIRST; 

}

int main(void)
{
  /* Initialization of all peripherals. */
  peripherals_init();

  pwm_init();   


  //----------------------------adc init--------------------------------

   NRF_SAADC->RESOLUTION =2 ;  
   NRF_SAADC->OVERSAMPLE = 0;  

   NRF_SAADC->INTENCLR= 0x7FFFFFFF;       
   NRF_SAADC->EVENTS_END = 0;             

   //nrf_drv_common_irq_enable(SAADC_IRQn, _PRIO_APP_MID);  
   NRF_SAADC->INTENSET |= 1<<1;      
   NRF_SAADC->ENABLE |= 1<<0; 

   //CH[0] -- AIN7-- ADC1

   NRF_SAADC->CH[0].CONFIG &= ~(1<<0|1<<1);          
   NRF_SAADC->CH[0].CONFIG &= ~(1<<4|1<<5);           
   NRF_SAADC->CH[0].CONFIG &= ~(1<<8|1<<9|1<<10) ;    
   NRF_SAADC->CH[0].CONFIG &= ~(1<<12);               
   
   NRF_SAADC->CH[0].CONFIG &= ~(1<<16|1<<17|1<<18);  
   NRF_SAADC->CH[0].CONFIG |=  (1<<17);              
 
   NRF_SAADC->CH[0].CONFIG &=  ~(1<<20);           

   NRF_SAADC->CH[0].PSELN = 0;          
   NRF_SAADC->CH[0].PSELP = 8;    


   //CH[1] -- AIN6 -- ADC2

   NRF_SAADC->CH[1].CONFIG &= ~(1<<0|1<<1);           
   NRF_SAADC->CH[1].CONFIG &= ~(1<<4|1<<5);           
   NRF_SAADC->CH[1].CONFIG &= ~(1<<8|1<<9|1<<10) ;                                
   NRF_SAADC->CH[1].CONFIG &= ~(1<<12);             
   
   NRF_SAADC->CH[1].CONFIG &= ~(1<<16|1<<17|1<<18);   
   NRF_SAADC->CH[1].CONFIG |=  (1<<17);              
 
   NRF_SAADC->CH[1].CONFIG &=  ~(1<<20);            

   NRF_SAADC->CH[1].PSELN = 0;         
   NRF_SAADC->CH[1].PSELP = 7;            


   //adc buf
 
   NRF_SAADC->RESULT.PTR = (uint32_t)&adcbuf[0];     
   NRF_SAADC->RESULT.MAXCNT =2;              


   //iadc_compare=  adc_us * 16UL ;     
   //NRF_SAADC->SAMPLERATE = iadc_compare;   
   //NRF_SAADC->SAMPLERATE |=  (1<<12);

  //----------------------------adc init-------end----------------------


  port_wait(50);

  //port_i2c_scan(&m_twi);

  imu_init();

  blink_app();

  //production_main();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

         
  while (1)
  {

  }

}

void uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}


/* @fn  peripherals_init
 *
 * @param[in] void
 * */
void peripherals_init(void)
{
    ret_code_t err_code ;

    // SysTick Initialization to 1ms
    SysTick_Config(SystemCoreClock / 1000);

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    nrf_gpio_cfg_input(DWIC_RST, NRF_GPIO_PIN_NOPULL);
    //nrf_gpio_cfg_input(DW1000_IRQ, NRF_GPIO_PIN_NOPULL);
    nrf_gpio_cfg_output(SPI_CS_PIN);
    nrf_gpio_pin_set(SPI_CS_PIN);

    nrf_gpio_cfg_output(BOOST_EN_PIN);
    nrf_gpio_pin_clear(BOOST_EN_PIN);
    //--nrf_gpio_pin_set(BOOST_EN_PIN);     //--TEST
    //nrf_delay_ms(100);   //--TEST

    nrf_gpio_cfg_output(DW_PWRDN_PIN);    //Main power 
    nrf_gpio_pin_clear(DW_PWRDN_PIN);    
    
    //nrf_drv_gpiote_in_config_t in_config = GPIOTE_RAW_CONFIG_IN_SENSE_LOTOHI(true);
    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
    in_config.pull = NRF_GPIO_PIN_NOPULL;
    err_code = nrf_drv_gpiote_in_init(DW1000_IRQ, &in_config, dw1000_irq_handler);
    APP_ERROR_CHECK(err_code);
    nrf_drv_gpiote_in_event_enable(DW1000_IRQ, true);

//IMU_IRQ    
    //nrf_drv_gpiote_in_config_t in_config = GPIOTE_RAW_CONFIG_IN_SENSE_LOTOHI(true);
//    in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
//    in_config.pull = NRF_GPIO_PIN_NOPULL;
    err_code = nrf_drv_gpiote_in_init(IMU_IRQ, &in_config, imu_irq_handler);
    APP_ERROR_CHECK(err_code);
    nrf_drv_gpiote_in_event_enable(IMU_IRQ, true);


    bsp_board_init(0);

    const app_uart_comm_params_t comm_params =
      {
          UART_0_RX_PIN,
          UART_0_TX_PIN,
          DW3000_RTS_PIN_NUM,
          DW3000_CTS_PIN_NUM,
          APP_UART_FLOW_CONTROL_DISABLED,
          false,
          NRF_UART_BAUDRATE_115200
      };

    APP_UART_FIFO_INIT(&comm_params,
                     UART_RX_BUF_SIZE,
                     UART_TX_BUF_SIZE,
                     uart_event_handler,
                     APP_IRQ_PRIORITY_LOWEST,
                     err_code);
    APP_ERROR_CHECK(err_code);


    const nrf_drv_twi_config_t twi_config = {
       .scl                = QT_I2C_SCL,
       .sda                = QT_I2C_SDA,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };
    port_i2c_init(&m_twi, &twi_config);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
}

#ifdef USE_FULL_ASSERT 
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */




