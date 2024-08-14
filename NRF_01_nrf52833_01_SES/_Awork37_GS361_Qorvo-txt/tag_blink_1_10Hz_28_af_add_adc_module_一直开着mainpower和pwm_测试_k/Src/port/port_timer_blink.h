#ifndef _PORT_TIMER_BLINK_H_
#define _PORT_TIMER_BLINK_H_

#include "nrf_drv_timer.h"


void port_timer_blink_init(void);


bool port_timer_is_initiated(void);

bool port_timer_has_event(void);

void port_timer_clear_event(void);

void port_timer_blink_disable(void);

void port_timer_blink_enable(void);

void port_timer_blink_cfg(uint32_t delay);
                  
#endif // _PORT_TIMER_BLINK_H_

