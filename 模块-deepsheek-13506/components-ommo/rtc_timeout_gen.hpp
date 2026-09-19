#pragma  once

#include "nrfx_rtc.h"

#include "sdk_config.h"

typedef void (*rtc_timeout_func_type)(void *p_context);

#define RTC_TIMEOUT_GEN_FREQUENCY NRFX_RTC_DEFAULT_CONFIG_FREQUENCY
#define MS_TO_RTC_TIMEOUT_GEN(x)  ((uint32_t)(((uint64_t)(x) * RTC_TIMEOUT_GEN_FREQUENCY) / 1000UL))
#define US_TO_RTC_TIMEOUT_GEN(x)  ((uint32_t)(((uint64_t)(x) * RTC_TIMEOUT_GEN_FREQUENCY) / 1000000UL))

void static_rtc_timer_handler(nrfx_rtc_int_type_t int_type, void * p_context);

void rtc_timer_init();
void rtc_timer_handler(nrfx_rtc_int_type_t int_type);
uint32_t rtc_timer_get_timeout_channel(uint8_t *channel, rtc_timeout_func_type callback, void *p_context);
void rtc_timer_release_timeout_channel(uint8_t channel);
uint32_t rtc_timer_set_timeout(uint8_t channel, uint32_t timeout);
uint32_t rtc_timer_reset_timeout(uint8_t channel);
uint32_t rtc_timer_disable_timeout(uint8_t channel);
        
