
#ifndef _BLINK_APP_H_
#define _BLINK_APP_H_

#include "nrf_drv_gpiote.h"

typedef enum
{
    FSM_PRE_INIT,
    FSM_INIT,
    FSM_SLEEP,
    FSM_CHARGE,
    FSM_BLINK
} fsm_e;

void imu_irq_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);

void prepare_to_sleep(void);

void prepare_to_charge(void);

int blink(void);

void blink_app(void);

#endif      //_BLINK_APP_H_
