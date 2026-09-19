#if !defined(TIMESTAMP_TIMER_HPP)
#define TIMESTAMP_TIMER_HPP

#include "nrfx_ppi.h"
#include "nrfx_timer.h"

#include "ommo_config.h"
#include "ommo_macros.h"
#include "rtc_timeout_gen.hpp"
#include "ommo_macros.h"

extern "C" {
#include "timestamp_timer.h"
}

typedef enum
{
    TIMESTAMP_SYNCH_SOURCE_NONE,
    TIMESTAMP_SYNCH_SOURCE_UART,
    TIMESTAMP_SYNCH_SOURCE_1WIRE,
    TIMESTAMP_SYNCH_SOURCE_ESB,
    TIMESTAMP_SYNCH_SOURCE_ESB_NO_TS
} timestamp_source_t;

#define TIMESTAMP_SYNCH_PERIOD             (OMMO_TIMESTAMP_SAMPLE_PERIOD * OMMO_TIMESTAMP_SYNCH_PERIOD_MULT)

#define TIMESTAMP_SYNCH_PERIOD_MS          TICKS_16MHZ_TO_MS(TIMESTAMP_SYNCH_PERIOD)

#define TIMESTAMP_SYNCH_ESB_TX_DELAY       (COMMS_ESB_SD_COMMAND_PACKET_LENGTH_TICKS + 100) //TX time, Don't know what the 100 cycle delay is from?

#define TIMESTAMP_SYNCH_UART_TX_DELAY      (6*10*16 + 1 + (16-4)) //TX time (6 bytes at 1Mbps) + PPI rx time capture delay + uart clock vs basestation timer delay

#define TIMESTAMP_SYNCH_1WIRE_TX_DELAY     (5*10*16 + 1 + (16-4)) //TX time (5 bytes at 1Mbps) + PPI rx time capture delay + uart clock vs basestation timer delay

#define TIMESTAMP_SYNCH_MAX_EXECUTE_FUNCTIONS 4

#define TIMESTAMP_MAX_SAMPLE_EVENT_CALLBACKS 4

    typedef void (*timestamp_synch_lost_found_func_type)(bool synched, bool wired);
    typedef void (*timestamp_synch_execute_func_type)(uint32_t sample_event_value);

    //Public functions
    void timestamp_init_with_no_sample_event();
    void timestamp_init_with_internal_sample_event(uint8_t isr_priority = NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY);
    void timestamp_init_with_external_sample_event(uint32_t sample_event);
    static void timestamp_synch_init(uint8_t isr_priority);
    void timestamp_uninit();

    static void timestamp_debug_init(uint8_t isr_priority);

    //Sample event functions
    uint32_t timestamp_get_sample_event_address();
    nrf_ppi_channel_t timestamp_add_task_on_sample_event(uint32_t task_address, bool enable = true);
    uint32_t timestamp_get_last_sample_event_timestamp();
    uint32_t timestamp_get_last_synch_event_timestamp();

    //Capture/compare functions
    nrf_timer_cc_channel_t timestamp_get_capture_compare_channel();
    void timestamp_release_capture_compare_channel(nrf_timer_cc_channel_t channel);

    //Capture functions
    nrf_timer_cc_channel_t timestamp_add_capture_event_with_disable(uint32_t trigger_event_addr);
    void timestamp_add_capture_event_with_disable(uint32_t trigger_event_addr, nrf_timer_cc_channel_t capture_channel);
    nrf_timer_cc_channel_t timestamp_add_capture_event_with_disable(nrf_ppi_channel_t ppi_channel_timestamp_capture, uint32_t trigger_event_addr);
    void timestamp_add_capture_event_with_disable(nrf_ppi_channel_t ppi_channel_timestamp_capture, uint32_t trigger_event_addr, nrf_timer_cc_channel_t capture_channel);
    void timestamp_remove_ppi_from_group_disable(nrf_ppi_channel_t ppi_channel_timestamp_capture);
    void timestamp_reenable_disabled_captures();
    nrf_timer_cc_channel_t timestamp_add_capture_event(uint32_t trigger_event_addr, nrf_ppi_channel_t *ppi_channel_timestamp_capture_ptr);
    void timestamp_add_capture_event(nrf_ppi_channel_t ppi_channel_timestamp_capture, uint32_t trigger_event_addr, nrf_timer_cc_channel_t capture_channel);
    void timestamp_add_capture_event_fork(nrf_ppi_channel_t ppi_channel_timestamp_capture, nrf_timer_cc_channel_t capture_channel);
    NRF_TIMER_Type* timestamp_get_timer_p_reg();

    //Compare functions
    uint32_t timestamp_get_compare_event_addr(nrf_timer_cc_channel_t compare_channel);
    void timestamp_set_compare_event_time(nrf_timer_cc_channel_t compare_channel, uint32_t value);
    uint32_t timestamp_create_sample_event_offset_event(uint32_t offset);
    uint32_t timestamp_get_timestamp_synch_event_address();
    uint32_t timestamp_get_next_synch_event_timestamp();
    uint32_t timestamp_add_task_on_timestamp_synch_event(uint32_t task_address);

    //Misc
    uint32_t timestamp_get_timer_clear_task();
    uint32_t timestamp_add_sample_event_callback(nrfx_timer_event_handler_t func);
    uint32_t timestamp_add_timestamp_synch_event_callback(timestamp_synch_execute_func_type func);

    //Timestamp synch in
#ifdef OMMO_TIMESTAMP_SYNCH_IN
    void timestamp_set_esb_synch_enabled(bool enabled);
    static void timestamp_reset_synch_lost_timer();
    uint32_t timestamp_synch_get_num_misses();
    void timestamp_synch_recieved(uint32_t basestation_ts, uint32_t local_event_ts, timestamp_source_t source);
    uint32_t timestamp_get_last_synch_offset();
    void timestamp_set_synch_lost_found_callback(timestamp_synch_lost_found_func_type func);
    uint32_t timestamp_get_current_timestamp_basestation_units();
    void timestamp_get_last_sample_event_timestamp_basestation_units(uint32_t &timestamp, uint32_t &timestamp_offset);
    bool timestamp_is_synch_lost();
    bool timestamp_is_synch_wired();
    uint32_t timestamp_read_capture_basestation_units(nrf_timer_cc_channel_t channel);
    uint32_t timestamp_get_basestation_offset();
#endif

#endif
