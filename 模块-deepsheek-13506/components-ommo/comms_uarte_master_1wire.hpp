/**
 * Copyright (c) 2025, Ommo Technologies
 *
 * All rights reserved.
 *
 * Background/charts/documentation: https://docs.google.com/document/d/1Iu4lMgjhMhUoje3RGIvziKPsUy9EXHeXdjMf4TNDk6k/edit?usp=sharing
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "app_timer.h"
#include "nrfx_timer.h"
#include "nrfx_uarte.h"
#include "sdk_errors.h"

#include "comms_uarte_1wire_protocol.h"
#include "debug_buffer.hpp"
#include "ommocomm_uarte.hpp"
#include "utils.hpp"
#include "ommo_macros.h"


#define TEST_PATTERN_MODE 0


class comms_uarte_master_1wire
{
public:

    typedef enum
    {
        EVENT_DISCONNECTED,  //!< MSIU is not connected
        EVENT_CONNECTED,     //!< MSIU is connected
    } event_t;

    typedef void (*rx_complete_callback_t)();
    typedef void (*tx_complete_callback_t)();
    typedef void (*event_handler_t)(event_t event);

    void init(nrfx_timer_t *timer,
              app_timer_id_t const *app_timer_id,
              rx_complete_callback_t rx_complete_callback,
              tx_complete_callback_t tx_complete_callback,
              event_handler_t ev_handler,
              uint8_t rtx_pin,
              uint8_t isr_priority);
    void execute_timestamp_synch(uint32_t sample_event_value);
    ret_code_t receive_packet_async(comms_1wire_slave_tx_data_packet_t *packet);
    ret_code_t send_packet_async(const comms_1wire_master_tx_data_packet_t *packet);
    bool is_tx_pending();
    void cancel_tx_blocking();
    void cancel_rx_blocking();
    void set_app_connection_status(bool);

    void event_queue_process();
    CREATE_STATIC_WRAPPER(comms_uarte_master_1wire, event_queue_process, void);

private:

    typedef enum
    {
        // Scheduled transmission of a timestamp packet, in sync with the timestamp timer
        STATE_TX_TIMESTAMP,

        // Scheduled transmission of a data packet
        STATE_TX_DATA,

        // Scheduled transmission of a ping packet
        STATE_TX_TEST_DATA,

        // First state after calling init, transmitting 1 byte to align UARTE and timestamp timer
        // clocks
        STATE_SYNC_CLOCKS,

        // Connection has been conclusively lost.  Monitoring DATA logic level to determine when
        // either the MSIU 10K or 1K pull-ups are present.
        STATE_DISCONNECTED,

        // DATA logic level is high, waiting 100ms before attempting to drive the line to avoid
        // disrupting slave power during boot-up
        STATE_CONNECTION_DELAY,

        // Scheduled transmission of a handshake packet.  The packet does not contain an actual
        // synch timestamp, but is timed with the timestamp timer to allow the slave to predict
        // the arrival of real timestamp packets.
        STATE_TX_HANDSHAKE_REQUEST,

        // Installed by the software watchdog in the main loop when the UARTE ISR state machine is
        // not making progress
        STATE_ERROR,

        // Transmitted a handshake packet, now waiting for a valid response from the slave
        STATE_RX_HANDSHAKE_ACK,

        // Installed by a timer ISR to interrupt the current receive operation
        STATE_RX_TIMEOUT,

        // Performed an RXABORT, and waiting for RXEND before initiating a new handshake
        STATE_RX_ABORT,

        // Application requested to receive data from the slave
        STATE_RX_DATA,

        // Receiving data from the slave after the application cancelled reception,
        // or in TEST_PACKET_MODE.  Either way, the received data is not passed on to the
        // application.
        STATE_RX_DROPPED,

        STATE_FIRST_DISCONNECTED = STATE_SYNC_CLOCKS,
        STATE_LAST_DISCONNECTED = STATE_RX_ABORT,

        STATE_FIRST_RX = STATE_RX_HANDSHAKE_ACK,
    } state_t;

    volatile state_t state;
    bool last_connection_status;
    uint8_t connection_delay_ms;
    bool app_connection_status;
    event_handler_t ev_handler;

    uarte_basic *uarte_resource;
    const nrfx_uarte_t *uarte;
    uint8_t rtx_pin;
    nrfx_timer_t *timer;
    app_timer_id_t const *app_timer_id;

    comms_1wire_slave_tx_data_packet_t rx_data;
    comms_1wire_slave_tx_data_packet_t *rx_packet;
    rx_complete_callback_t rx_complete_callback;

    const comms_1wire_master_tx_data_packet_t *tx_packet;
    tx_complete_callback_t tx_complete_callback;

    // Timer COMPARE0 -> USARTX STARTTX
    nrf_ppi_channel_t tx_data_ppi;

    comms_1wire_timestamp_packet_t tx_timestamp;

    volatile uint32_t next_timestamp_synch_value;
    volatile bool next_timestamp_synch_value_ready;

    // State machine monitoring
    uint32_t disconnected_count;
    uint32_t handshake_count;
    uint32_t connection_count;
    uint32_t synch_count;
    uint32_t handshake_count_at_disconnection;

    // Software watchdog state
    uint32_t last_disconnected_count;
    uint32_t last_handshake_count;
    uint32_t last_connection_count;
    uint32_t last_synch_count;
    uint8_t sw_watchdog_elapsed_ms;

    // Tracing

#ifdef DEBUG_NRF

    typedef enum
    {
        TRACE_TX,
        TRACE_RX,
        TRACE_RX_DONE,
        TRACE_TX_DONE,
        TRACE_ERROR,
        TRACE_TIMEOUT,
        TRACE_SW_WATCHDOG,
        TRACE_ERROR_RECOVERY,
        TRACE_DISCONNECTED,
    } trace_event_type_t;

    typedef struct
    {
        trace_event_type_t type;
        state_t state;
    } trace_event_t;

    trace_event_t trace_events[20];
    volatile size_t trace_index;

    trace_event_t *trace(trace_event_type_t type);
#endif

    debug_buffer<100, uint8_t> debug_msiu_tx_packet;
    debug_buffer<100, uint8_t> debug_msiu_tx_req;

    // Testing
#ifdef DEBUG_NRF
    uint32_t rx_error_count;
    uint32_t tx_error_count;
    uint32_t timeout_count;
    uint32_t rx_zero_length_count;
    uint32_t sw_watchdog_count;
    uint32_t connection_duration;
    uint32_t connection_max_duration;
#endif
#if TEST_PATTERN_MODE
    bool test_pattern_tx_vs_rx;
    uint32_t test_packet_count;
    uint32_t incorrect_test_length_count;
    uint32_t incorrect_test_data_count;
#endif

    typedef enum
    {
        TX_TRIGGER_NOW,
        TX_TRIGGER_SYNCH,
        TX_TRIGGER_TIMER,
    } tx_trigger_t;

    void tx_at_trigger(const void *buffer, size_t length, tx_trigger_t trigger);

    void rx(void *buffer, size_t length);

    void initiate_handshake();
    void verify_test_pattern(const comms_1wire_slave_tx_data_packet_t *packet);
    void on_disconnection();
    void check_ready_and_transmit();

    static void comms_uarte_handler(nrfx_uarte_event_t const *p_event, void *p_context);
    void on_uarte_event(nrfx_uarte_event_t const *p_event);

    static void timer_event_handler(nrf_timer_event_t event_type, void* p_context);
    void on_timer_event(nrf_timer_event_t event_type);
    
    void timed_loop_10ms();
    CREATE_STATIC_WRAPPER(comms_uarte_master_1wire, timed_loop_10ms, void);
};
