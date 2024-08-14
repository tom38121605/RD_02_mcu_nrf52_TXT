


#include "blink_app.h"
#include "port_timer_blink.h"
#include "deca_device_api.h"
#include "instance.h"
#include "port.h"
#include "default_config.h"
#include "port_spi.h"
#include "port_timer.h"
#include "deca_regs.h"
#include "translate.h"
#include <stdio.h>

extern int16_t adcbuf[4];

#define INIT_PERIOD           (500)  
#define BLINK_PERIOD_FAST     (200)   // 200 ms
#define BLINK_PERIOD_SLOW     (1000)
#define BLINK_FAST_PERIOD     (3000)  // 3 s
#define BLINK_CHARGE_TIME     (60)    //

#define BLINK_INTERVAL_FAST   (BLINK_PERIOD_FAST - BLINK_CHARGE_TIME)
#define BLINK_INTERVAL_SLOW   (BLINK_PERIOD_SLOW - BLINK_CHARGE_TIME)
#define BLINK_FAST_NUM        (BLINK_FAST_PERIOD / BLINK_PERIOD_FAST)  // 3s / 0.2S

extern app_cfg_t app;

static uint8_t tx_msg[] = {0xC5, 0, 'D', 'E', 'C', 'A', 'W', 'A', 'V', 'E'};
volatile uint8_t imu_is_moving = 0;

typedef enum /*lint -save -e30 -esym(628,__INTADDR__) */
{
    NRF_SAADC_EVENT_STARTED       = offsetof(NRF_SAADC_Type, EVENTS_STARTED),       ///< The ADC has started.
    NRF_SAADC_EVENT_END           = offsetof(NRF_SAADC_Type, EVENTS_END),           ///< The ADC has filled up the result buffer.
    NRF_SAADC_EVENT_CALIBRATEDONE = offsetof(NRF_SAADC_Type, EVENTS_CALIBRATEDONE), ///< Calibration is complete.
    NRF_SAADC_EVENT_STOPPED       = offsetof(NRF_SAADC_Type, EVENTS_STOPPED),       ///< The ADC has stopped.
    NRF_SAADC_EVENT_CH0_LIMITH    = offsetof(NRF_SAADC_Type, EVENTS_CH[0].LIMITH),  ///< Last result is equal or above CH[0].LIMIT.HIGH.
    NRF_SAADC_EVENT_CH0_LIMITL    = offsetof(NRF_SAADC_Type, EVENTS_CH[0].LIMITL),  ///< Last result is equal or below CH[0].LIMIT.LOW.
    NRF_SAADC_EVENT_CH1_LIMITH    = offsetof(NRF_SAADC_Type, EVENTS_CH[1].LIMITH),  ///< Last result is equal or above CH[1].LIMIT.HIGH.
    NRF_SAADC_EVENT_CH1_LIMITL    = offsetof(NRF_SAADC_Type, EVENTS_CH[1].LIMITL),  ///< Last result is equal or below CH[1].LIMIT.LOW.
    NRF_SAADC_EVENT_CH2_LIMITH    = offsetof(NRF_SAADC_Type, EVENTS_CH[2].LIMITH),  ///< Last result is equal or above CH[2].LIMIT.HIGH.
    NRF_SAADC_EVENT_CH2_LIMITL    = offsetof(NRF_SAADC_Type, EVENTS_CH[2].LIMITL),  ///< Last result is equal or below CH[2].LIMIT.LOW.
    NRF_SAADC_EVENT_CH3_LIMITH    = offsetof(NRF_SAADC_Type, EVENTS_CH[3].LIMITH),  ///< Last result is equal or above CH[3].LIMIT.HIGH.
    NRF_SAADC_EVENT_CH3_LIMITL    = offsetof(NRF_SAADC_Type, EVENTS_CH[3].LIMITL),  ///< Last result is equal or below CH[3].LIMIT.LOW.
    NRF_SAADC_EVENT_CH4_LIMITH    = offsetof(NRF_SAADC_Type, EVENTS_CH[4].LIMITH),  ///< Last result is equal or above CH[4].LIMIT.HIGH.
    NRF_SAADC_EVENT_CH4_LIMITL    = offsetof(NRF_SAADC_Type, EVENTS_CH[4].LIMITL),  ///< Last result is equal or below CH[4].LIMIT.LOW.
    NRF_SAADC_EVENT_CH5_LIMITH    = offsetof(NRF_SAADC_Type, EVENTS_CH[5].LIMITH),  ///< Last result is equal or above CH[5].LIMIT.HIGH.
    NRF_SAADC_EVENT_CH5_LIMITL    = offsetof(NRF_SAADC_Type, EVENTS_CH[5].LIMITL),  ///< Last result is equal or below CH[5].LIMIT.LOW.
    NRF_SAADC_EVENT_CH6_LIMITH    = offsetof(NRF_SAADC_Type, EVENTS_CH[6].LIMITH),  ///< Last result is equal or above CH[6].LIMIT.HIGH.
    NRF_SAADC_EVENT_CH6_LIMITL    = offsetof(NRF_SAADC_Type, EVENTS_CH[6].LIMITL),  ///< Last result is equal or below CH[6].LIMIT.LOW.
    NRF_SAADC_EVENT_CH7_LIMITH    = offsetof(NRF_SAADC_Type, EVENTS_CH[7].LIMITH),  ///< Last result is equal or above CH[7].LIMIT.HIGH.
    NRF_SAADC_EVENT_CH7_LIMITL    = offsetof(NRF_SAADC_Type, EVENTS_CH[7].LIMITL)   ///< Last result is equal or below CH[7].LIMIT.LOW.
} nrf_saadc_event_t;
__STATIC_INLINE bool nrf_saadc_event_check(nrf_saadc_event_t saadc_event)
{
    return (bool)*(volatile uint32_t *)((uint8_t *)NRF_SAADC + (uint32_t)saadc_event);
}

__STATIC_INLINE void nrf_saadc_event_clear(nrf_saadc_event_t saadc_event)
{
    *((volatile uint32_t *)((uint8_t *)NRF_SAADC + (uint32_t)saadc_event)) = 0x0UL;
}


void imu_irq_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    static uint8_t cnt8=0;
    printf("imu irq %d\r\n", cnt8++);
    imu_is_moving = BLINK_FAST_NUM;
}


void prepare_to_sleep(void)
{
    printf("blink --> sleep \r\n");
}

void pre_init(void)
{
    nrf_gpio_pin_set(BOOST_EN_PIN);
    printf("charge for init \r\n");
}
void prepare_to_charge(void)
{
    nrf_gpio_pin_set(BOOST_EN_PIN);
    printf("prepare to charge \r\n");
}


void blink_radio_init(void)
{
    param_block_t blink_cfg = DEFAULT_CONFIG;
    uint8_t ch = blink_cfg.dwt_config.chan;

    nrf_gpio_pin_set(BOOST_EN_PIN);

    port_spi_set_fastrate();

    port_reset_dw1000(); /* Target specific drive of RSTn line into DW IC low for a period. */

    port_wait(2); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)

    while (!dwt_checkidlerc()) /* Need to make sure DW IC is in IDLE_RC before proceeding */
    { };

    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
    {
        printf("INIT FAILED\r\n");
        while (1)
        { };
    }

    dwt_configure(&blink_cfg.dwt_config);
    //app.pConfig->dw_dev.tx_config[chan_to_deca(ch)].power = 0xffffffff;

    dwt_configuretxrf(&blink_cfg.dw_dev.tx_config[chan_to_deca(ch)]);

    dwt_configuresleep(DWT_CONFIG, DWT_PRESRVE_SLP | DWT_WAKE_CS | DWT_SLP_EN);

    dwt_entersleep(DWT_DW_IDLE);
    printf("init done\r\n");
}

int blink(void)
{
    uint16_t len = 10;
    uint16_t i;
    uint32_t status;
//    param_block_t* p_cfg = port_config_get();
//    uint8_t ch = p_cfg->chan;

    printf("blinking \r\n");
    
    port_wakeup_dw3000();

    tx_msg[1]++;

    dwt_writetxdata(len, (uint8_t *)tx_msg, 0);
    dwt_writetxfctrl(len, 0, 0);

    dwt_starttx(DWT_START_TX_IMMEDIATE);

    uint32_t start_ts;
    uint32_t exp_ms = 10;
    port_timer_start(&start_ts);

    do
    {
        status = dwt_read32bitreg(SYS_STATUS_ID);

        if(port_timer_check(start_ts, exp_ms))
        {
            //printf("Tx failed ... \r\n");
            return -1;
            //break;
        }
        if(status & SYS_STATUS_VWARN_BIT_MASK)
        {
            //printf("Brownout detected ......... \r\n");
            return -2;
            //break;
        }
    }while (!(status & SYS_STATUS_TXFRS_BIT_MASK));

    dwt_entersleep(DWT_DW_IDLE);
    //--nrf_gpio_pin_clear(BOOST_EN_PIN);

    return 0;
}

void blink_app(void)
{ 
    static fsm_e stat;

    stat = FSM_PRE_INIT;
    printf("pre init\r\n");    

    port_timer_blink_init();  
      
    pre_init();

    port_timer_blink_cfg(INIT_PERIOD);
    
    while (1)
    {
        //__WFI();

        static uint32_t imaincount =0;

        if(imaincount++>=1000000)
        {
            imaincount=0;

            NRF_SAADC->EVENTS_STARTED =0;                
            NRF_SAADC->TASKS_START=1;                   
            NRF_SAADC->TASKS_SAMPLE=1;           
      
            while (!nrf_saadc_event_check(NRF_SAADC_EVENT_END)) ;          
            nrf_saadc_event_clear(NRF_SAADC_EVENT_END);

            //uartdecode(mybuf1[0],"ch0: ");  
            //printf("run... \r\n");
            printf("adc1 %d\r\n", adcbuf[0]);
            printf("adc2 %d\r\n", adcbuf[1]);

         }

        
        if(port_timer_has_event())
        {
            port_timer_clear_event();

            //printf("imain %d\r\n", imaincount);

            switch(stat)
            {
                case FSM_PRE_INIT:
                    stat = FSM_INIT;
                    blink_radio_init();
                    port_timer_blink_cfg(100);
                    break;

                case FSM_INIT:
                    stat = FSM_CHARGE;
                    prepare_to_charge();
                    port_timer_blink_cfg(100);
                    break;

                case FSM_SLEEP:
                    stat = FSM_CHARGE;

                    nrf_gpio_pin_clear(DW_PWRDN_PIN); //open main power

                    NRF_PWM0->ENABLE=1;      //open pwm
                    NRF_PWM0->TASKS_SEQSTART[0] = 1;


                    //prepare_to_charge();
                    nrf_gpio_pin_set(BOOST_EN_PIN);

                    //port_timer_blink_cfg(BLINK_CHARGE_TIME);
                    port_timer_blink_cfg(10);
                    break;

                case FSM_CHARGE:

                    stat = FSM_SLEEP;

                    //NRF_PWM0->ENABLE=0;

                    //nrf_gpio_pin_set(BOOST_EN_PIN);

                    ////if(blink()!=0)
                    ////{
                    ////    //stat = FSM_PRE_INIT;
                    ////    //pre_init();
                    ////    //port_timer_blink_cfg(INIT_PERIOD);
                    ////    //break;
                    ////   blink();
                    ////}

                    //--NRF_PWM0->ENABLE=0;  //close pwm

                    //port_wait(100);
                    nrf_gpio_pin_clear(BOOST_EN_PIN);    

                    //--nrf_gpio_pin_set(DW_PWRDN_PIN); //close main power

                    prepare_to_sleep();
                    //if(imu_is_moving > 0)
                    //{
                    //    imu_is_moving--;
                    //    port_timer_blink_cfg(BLINK_INTERVAL_FAST);
                    //}
                    //else
                    //{
                    //    port_timer_blink_cfg(BLINK_INTERVAL_SLOW);
                    //}

                    port_timer_blink_cfg(1000); //--test                    

                    break;

                default:
                    stat = FSM_SLEEP;
                    //back to sleep
                    break;
            }
        }
    }
}
