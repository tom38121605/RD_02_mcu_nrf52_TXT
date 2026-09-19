#if !defined(TIMESTAMP_TIMER_H)
#define TIMESTAMP_TIMER_H

  #include <stdint.h>
  #include "nrf_timer.h"

#ifdef __cplusplus
extern "C" {
#endif

  uint32_t timestamp_get_current_timestamp();
  nrf_timer_cc_channel_t timestamp_add_capture_event(uint32_t trigger_event_addr);
  uint32_t timestamp_read_capture(nrf_timer_cc_channel_t channel);

#ifdef __cplusplus
}
#endif

#endif