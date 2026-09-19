/**
 * Copyright (c) 2025, Ommo Technologies
 * 
 * All rights reserved.
 *
 * Background/charts/documentation: https://docs.google.com/document/d/1Iu4lMgjhMhUoje3RGIvziKPsUy9EXHeXdjMf4TNDk6k/edit?usp=sharing
 */

#include "comms_uarte_master_1wire.hpp"

#include <array>

#include "nrf_gpio.h"
#include "nrfx_ppi.h"

#include "ommo_config.h"
#include "port_master.hpp"
#include "timestamp_timer.hpp"


#define TIMER_CHANNEL_STARTTX        NRF_TIMER_CC_CHANNEL0
#define TIMER_EVENT_STARTTX       NRF_TIMER_EVENT_COMPARE0

#define TIMER_CHANNEL_RX_TIMEOUT     NRF_TIMER_CC_CHANNEL1
#define TIMER_EVENT_RX_TIMEOUT    NRF_TIMER_EVENT_COMPARE1

#define HANDSHAKE_LIMIT_TO_DISCONNECT ((US_TO_TICKS_16MHZ(100000) + COMMS_1WIRE_MAX_PERIOD_TICKS - 1) / COMMS_1WIRE_MAX_PERIOD_TICKS)

#ifdef OMMO_DEBUG_HANDSHAKE_PIN
#define DEBUG_TOGGLE_PIN(PIN) nrf_gpio_pin_toggle(PIN)
#else
#define DEBUG_TOGGLE_PIN(PIN)
#endif


// Note that pre-defined packets must not be const, so that they are placed in .data/RAM.
// EasyDMA only works from RAM.
static comms_1wire_timestamp_packet_t initiate_handshake_packet =
{
    .timestamp = COMMS_1WIRE_TIMESTAMP_DURING_HANDSHAKE,
    .flags = COMMS_1WIRE_FLAGS_HANDSHAKE | COMMS_1WIRE_FLAGS_PING,
};

static comms_1wire_master_tx_data_packet_t tx_test_packet =
{
    .data = { COMMS_1WIRE_TEST_DATA },
    .flags = 0,
    .length = 0,
};


// TODO update this module to use trace.h
#undef TRACE
#ifdef DEBUG_NRF

comms_uarte_master_1wire::trace_event_t *comms_uarte_master_1wire::trace(trace_event_type_t type)
{
    size_t ret_index = (trace_index++) % std::size(trace_events);
    trace_event_t *ret = &trace_events[ret_index];
    ret->type = type;
    ret->state = state;
    return ret;
}

#define TRACE(TYPE) trace(TYPE)
#else
#define TRACE(TYPE)
#endif


void comms_uarte_master_1wire::tx_at_trigger(const void *buffer, size_t length, tx_trigger_t trigger)
{
    NRFX_ASSERT(!uarte_resource->rx_in_progress());
    NRFX_ASSERT(!uarte_resource->tx_in_progress());

    TRACE(TRACE_TX);

    nrfx_timer_compare_int_disable(timer, TIMER_CHANNEL_RX_TIMEOUT);
    nrf_timer_event_clear(timer->p_reg, TIMER_EVENT_RX_TIMEOUT);

    uarte_resource->switch_to_tx_only();

    if (trigger != TX_TRIGGER_TIMER)
        APP_ERROR_CHECK(nrfx_ppi_channel_disable(tx_data_ppi));

    APP_ERROR_CHECK(nrfx_uarte_tx_with_hold(uarte, (uint8_t*)buffer, length, trigger != TX_TRIGGER_NOW));

    if (trigger == TX_TRIGGER_TIMER)
    {
        nrf_timer_event_clear(timer->p_reg, TIMER_EVENT_STARTTX);
        APP_ERROR_CHECK(nrfx_ppi_channel_enable(tx_data_ppi));
    }
}


void comms_uarte_master_1wire::rx(void *buffer, size_t length)
{
    NRFX_ASSERT(!uarte_resource->rx_in_progress());
    NRFX_ASSERT(!uarte_resource->tx_in_progress());

    TRACE(TRACE_RX);

    APP_ERROR_CHECK(nrfx_ppi_channel_disable(tx_data_ppi));

    uarte_resource->switch_to_rx_only();
    uarte_resource->uarte_rx_flush_fifo();

    APP_ERROR_CHECK(nrfx_uarte_rx(uarte, (uint8_t*)buffer, length));

    nrfx_timer_compare_int_enable(timer, TIMER_CHANNEL_RX_TIMEOUT);
}


void comms_uarte_master_1wire::initiate_handshake()
{
    handshake_count++;

    state = STATE_TX_HANDSHAKE_REQUEST;

    if (app_connection_status)
        initiate_handshake_packet.flags |= COMMS_1WIRE_FLAGS_OPEN;
    else
        initiate_handshake_packet.flags &= ~COMMS_1WIRE_FLAGS_OPEN;

    tx_at_trigger(&initiate_handshake_packet, sizeof(initiate_handshake_packet), TX_TRIGGER_SYNCH);
}


void comms_uarte_master_1wire::verify_test_pattern(const comms_1wire_slave_tx_data_packet_t *packet)
{
#if TEST_PATTERN_MODE
    test_packet_count++;

    if (packet->length != 0)
        incorrect_test_length_count++;

    if (0 != memcmp(packet->data, tx_test_packet.data, sizeof(packet->data)))
        incorrect_test_data_count++;
#endif
}


void comms_uarte_master_1wire::on_disconnection()
{
    TRACE(TRACE_DISCONNECTED);
    nrfx_timer_compare_int_disable(timer, TIMER_CHANNEL_RX_TIMEOUT);
    nrf_uarte_txrx_pins_set(uarte->p_reg, NRF_UARTE_PSEL_DISCONNECTED, OMMOCOMM_UNUSED_PIN);
    nrf_gpio_cfg_input(rtx_pin, NRF_GPIO_PIN_NOPULL);
    state = STATE_DISCONNECTED;
}


/*static*/ void comms_uarte_master_1wire::comms_uarte_handler(nrfx_uarte_event_t const *p_event, void *p_context)
{
    ((comms_uarte_master_1wire*)p_context)->on_uarte_event(p_event);
}


void comms_uarte_master_1wire::on_uarte_event(nrfx_uarte_event_t const *p_event)
{
    if (p_event->type == NRFX_UARTE_EVT_ERROR)
        TRACE(TRACE_ERROR);
    else if (p_event->type == NRFX_UARTE_EVT_RX_DONE)
        TRACE(TRACE_RX_DONE);
    else if (p_event->type == NRFX_UARTE_EVT_TX_DONE)
        TRACE(TRACE_TX_DONE);

    if(state == STATE_ERROR)
        return;

    if(p_event->type == NRFX_UARTE_EVT_RX_DONE)
    {
        NRFX_ASSERT(!uarte_resource->tx_in_progress());
        NRFX_ASSERT(state >= STATE_FIRST_RX);

        // Transmitted a handshake packet?
        if(state == STATE_RX_HANDSHAKE_ACK)
        {
            verify_test_pattern(&rx_data);

            // If the slave responded with a 0-length data packet, then transition to normal
            // operation and start transmiting timestamp packets
            if(rx_data.length == 0)
            {
                connection_count++;
#ifdef DEBUG_NRF
                connection_duration = 0;
#endif
                state = STATE_TX_TIMESTAMP;
            }
            // Incorrect response, send another handshake packet
            else
            {
                state = STATE_TX_HANDSHAKE_REQUEST;
            }
        }
        // Attempting to terminate a receive operation?  RX_TIMEOUT typically means the DATA line
        // is disconnected.  RX_ABORT may be caused by noise on the DATA line, which has spring
        // contacts.
        else if(state == STATE_RX_TIMEOUT ||
                state == STATE_RX_ABORT)
        {
            // If there is no response from the slave after a while, assume the slave has
            // disconnected, and switch back to monitoring logic level
            if (handshake_count - handshake_count_at_disconnection >= HANDSHAKE_LIMIT_TO_DISCONNECT)
                on_disconnection();
            else
                state = STATE_TX_HANDSHAKE_REQUEST;
        }
        // Did we initiate a receive where the data is not to be relayed to the application?
        else if(state == STATE_RX_DROPPED)
        {
            if (tx_timestamp.flags & COMMS_1WIRE_FLAGS_PING)
                verify_test_pattern(&rx_data);
            state = STATE_TX_TIMESTAMP;
        }
        else /*if(state == COMMS_1WIRE_STATE_RX_DATA)*/
        {
            // If the master requested a ping, check that it is valid.  If not, initiate a new
            // handshake.
            if (tx_timestamp.flags & COMMS_1WIRE_FLAGS_PING)
            {
                if (rx_packet->length == 0)
                {
                    verify_test_pattern(rx_packet);
                    state = STATE_TX_TIMESTAMP;
                }
                else
                {
                    state = STATE_TX_HANDSHAKE_REQUEST;
                }
            }
            // The slave MUST respond, even when the slave application has no data to transmit to
            // the master.  If there is no real data to transmit, it will send a ping packet.
            else if (rx_packet->length == 0)
            {
#ifdef DEBUG_NRF
                rx_zero_length_count++;
#endif
                verify_test_pattern(rx_packet);
                state = STATE_TX_TIMESTAMP;
                // Do not reset rx_packet.  Continue attempting to receive into rx_packet.
            }
            // Otherwise, data successfully received, notify the application
            else
            {
                // Clear before calling the application.  The application must call rx within the
                // callback, if it wants to be able to losslessly/continuously receive data from
                // the slave.
                rx_packet = nullptr;
                rx_complete_callback();
                state = STATE_TX_TIMESTAMP;
            }
        }
    }
    else if(p_event->type == NRFX_UARTE_EVT_TX_DONE)
    {
        NRFX_ASSERT(!uarte_resource->rx_in_progress());
        NRFX_ASSERT(state < STATE_FIRST_RX);

        // Did master just transmit a timestamp packet?
        if(state == STATE_TX_TIMESTAMP)
        {
#if TEST_PATTERN_MODE
            test_pattern_tx_vs_rx = !test_pattern_tx_vs_rx;
#endif
            // Note that tx_timestamp is what was previously transmitted to the slave.  All
            // decisions here should be consistent with tx_timestamp, so that there isn't
            // confusion between the master and slave.
            if (tx_timestamp.flags & COMMS_1WIRE_FLAGS_MASTER_TX)
            {
                // Check that the application didn't call cancel_rxtx_blocking while transmitting
                // the timestamp packet
                if (tx_packet != nullptr)
                {
                    // See cancel_rxtx_blocking: the transition to this state means that the ISR
                    // owns tx_packet, and it must not be freed.
                    state = STATE_TX_DATA;
                }
                // Otherwise, either in test pattern mode, or the tx was cancelled while
                // transmitting the timestamp packet.  In the latter case, send a 0-length
                // packet to maintain synch.
                else
                {
                    state = STATE_TX_TEST_DATA;
                }
            }
            else
            {
                // Check that the application didn't call cancel_rxtx_blocking while transmitting
                // the timestamp packet
                if (rx_packet != nullptr)
                {
                    // See cancel_rxtx_blocking: the transition to this state means that the ISR
                    // owns rx_packet, and it must not be freed.
                    state = STATE_RX_DATA;
                }
                // Otherwise, either in test pattern mode, or the rx was cancelled while
                // transmitting the timestamp packet.  Either way, the slave was told to transmit.
                else
                {
                    state = STATE_RX_DROPPED;
                }
            }
        }
        // Did master just transmit a ping packet?
        else if(state == STATE_TX_TEST_DATA)
        {
            state = STATE_TX_TIMESTAMP;
        }
        // Did master just transmit a data packet?
        else if(state == STATE_TX_DATA)
        {
            debug_msiu_tx_packet.append(tx_packet->data, tx_packet->length);
            tx_packet = nullptr;
            tx_complete_callback();
            state = STATE_TX_TIMESTAMP;
        }
        // Initial state, only reached once.  This simply synchronizes the UARTE and timestamp
        // timer clocks to minimize errors.  After this state, start watching for a logic high on
        // the DATA line.
        else if(state == STATE_SYNC_CLOCKS)
        {
            on_disconnection();
        }
        // Otherwise, master just transmitted a handshake packet, so expect a ping response.
        else /*if(state == COMMS_1WIRE_STATE_TX_HANDSHAKE_REQUEST)*/
        {
            state = STATE_RX_HANDSHAKE_ACK;
        }
    }
    else /*if(p_event->type == NRFX_UARTE_EVT_ERROR)*/
    {
        if (state >= STATE_FIRST_RX)
        {
#ifdef DEBUG_NRF
            rx_error_count++;
#endif
            state = STATE_RX_ABORT;
        }
        else
        {
#ifdef DEBUG_NRF
            tx_error_count++;
#endif
            // Assume this is left over from a previous rx state.  If the current transmission does
            // not complete, the software watchdog will reset the state machine.
            return;
        }
    }

    // Update the hardware to perform the function of the new state
    switch(state)
    {
    case STATE_TX_TIMESTAMP:
#ifdef DEBUG_NRF
        connection_duration++;
        if (connection_duration > connection_max_duration)
            connection_max_duration = connection_duration;
#endif
        // If a timestamp value is not currently available, this will initiate a handshake
        check_ready_and_transmit();
        break;

    case STATE_DISCONNECTED:
        break;

    case STATE_TX_HANDSHAKE_REQUEST:
        DEBUG_TOGGLE_PIN(OMMO_DEBUG_HANDSHAKE_PIN);
        initiate_handshake();
        break;

    case STATE_RX_DATA:
        DEBUG_TOGGLE_PIN(OMMO_DEBUG_RX_NONPING_PIN);
        rx(rx_packet, sizeof(*rx_packet));
        break;

    case STATE_RX_ABORT:
        uarte_resource->rx_abort();
        uarte_resource->uarte_rx_flush_fifo();
        break;

    case STATE_RX_HANDSHAKE_ACK:
    case STATE_RX_DROPPED:
        DEBUG_TOGGLE_PIN(OMMO_DEBUG_HANDSHAKE_PIN);
        DEBUG_TOGGLE_PIN(OMMO_DEBUG_RX_NONPING_PIN);
        rx(&rx_data, sizeof(rx_data));
        break;

    case STATE_TX_DATA:
        DEBUG_TOGGLE_PIN(OMMO_DEBUG_TX_PIN);
        tx_at_trigger(tx_packet, sizeof(*tx_packet), TX_TRIGGER_TIMER);
        break;
    
    case STATE_TX_TEST_DATA:
        DEBUG_TOGGLE_PIN(OMMO_DEBUG_TX_PIN);
        DEBUG_TOGGLE_PIN(OMMO_DEBUG_RX_NONPING_PIN);
        // Triggered by timer via tx_data_ppi
        tx_at_trigger(&tx_test_packet, sizeof(tx_test_packet), TX_TRIGGER_TIMER);
        break;
    
    default:
        NRFX_ASSERT(!"unexpected state");
    }
}


/*static*/ void comms_uarte_master_1wire::timer_event_handler(nrf_timer_event_t event_type, void* p_context)
{
    ((comms_uarte_master_1wire*)p_context)->on_timer_event(event_type);
}

void comms_uarte_master_1wire::on_timer_event(nrf_timer_event_t event_type)
{
    if (event_type == TIMER_EVENT_RX_TIMEOUT)
    {
        TRACE(TRACE_TIMEOUT);

        if (state != STATE_ERROR)
        {
#ifdef DEBUG_NRF
            timeout_count++;
#endif
            state = STATE_RX_TIMEOUT;
            uarte_resource->rx_abort();  // triggers a RX_DONE event
            uarte_resource->uarte_rx_flush_fifo();
        }
    }
}


void comms_uarte_master_1wire::init(nrfx_timer_t *timer,
                                    app_timer_id_t const *app_timer_id,
                                    rx_complete_callback_t rx_complete_callback,
                                    tx_complete_callback_t tx_complete_callback,
                                    event_handler_t ev_handler,
                                    uint8_t rtx_pin,
                                    uint8_t isr_priority)
{
    this->timer = timer;
    this->app_timer_id = app_timer_id;
    this->rx_complete_callback = rx_complete_callback;
    this->tx_complete_callback = tx_complete_callback;
    this->ev_handler = ev_handler;
    this->rtx_pin = rtx_pin;

    state = STATE_SYNC_CLOCKS;  // reflects 1 byte sent in init
    last_connection_status = false;
    connection_delay_ms = 0;
    app_connection_status = false;

    rx_packet = nullptr;
    tx_packet = nullptr;

    disconnected_count = 0;
    handshake_count = 0;
    connection_count = 0;
    synch_count = 0;
    handshake_count_at_disconnection = 0;
    sw_watchdog_elapsed_ms = 0;

#ifdef DEBUG_NRF
    rx_error_count = 0;
    tx_error_count = 0;
    timeout_count = 0;
    rx_zero_length_count = 0;
    trace_index = 0;
    sw_watchdog_count = 0;
    connection_duration = 0;
    connection_max_duration = 0;
#endif
#if TEST_PATTERN_MODE
    test_pattern_tx_vs_rx = false;
    test_packet_count = 0;
    incorrect_test_length_count = 0;
    incorrect_test_data_count = 0;
#endif

#ifdef OMMO_DEBUG_HANDSHAKE_PIN
    nrf_gpio_pin_clear(OMMO_DEBUG_HANDSHAKE_PIN);
    nrf_gpio_cfg_output(OMMO_DEBUG_HANDSHAKE_PIN);

    nrf_gpio_pin_clear(OMMO_DEBUG_TX_PIN);
    nrf_gpio_cfg_output(OMMO_DEBUG_TX_PIN);

    nrf_gpio_pin_clear(OMMO_DEBUG_RX_NONPING_PIN);
    nrf_gpio_cfg_output(OMMO_DEBUG_RX_NONPING_PIN);
#endif

    //Init uarte for half duplex.  The master side is expected to have a 10k pull-down (ex. see 13891).
    //The MSIU will enable a 1k pull-up at boot.  The master initially uses the pin as GPIO and waits
    //for the MSIU pull-up to be active.
    nrfx_uarte_config_t m_uart0_drv_config = NRFX_UARTE_DEFAULT_CONFIG;
    m_uart0_drv_config.pselrxd = rtx_pin;
    m_uart0_drv_config.pseltxd = NRF_UARTE_PSEL_DISCONNECTED;
    m_uart0_drv_config.baudrate = NRF_UARTE_BAUDRATE_1000000;
    m_uart0_drv_config.interrupt_priority = isr_priority;
    APP_ERROR_CHECK(port_master_acquire_uarte_direct(&m_uart0_drv_config, comms_uarte_handler, this, &uarte_resource));
    uarte = uarte_resource->get_nrfx_uarte_instance();

    nrf_gpio_cfg_input(rtx_pin, NRF_GPIO_PIN_NOPULL);

    //Setup PPI for tx'ing sample timestamp
    APP_ERROR_CHECK(timestamp_add_task_on_timestamp_synch_event(uarte_resource->get_starttx_task()));

    //Create a timer for triggering data packet transmission and for rx timeout
    nrfx_timer_config_t timer_cfg = NRFX_TIMER_DEFAULT_CONFIG;
    timer_cfg.p_context = this;
    timer_cfg.interrupt_priority = isr_priority;
    APP_ERROR_CHECK(nrfx_timer_init(timer, &timer_cfg, timer_event_handler));
    nrfx_timer_enable(timer);

    //Reset the timer when the timestamp packet transmission begins
    APP_ERROR_CHECK(timestamp_add_task_on_timestamp_synch_event(nrfx_timer_task_address_get(timer, NRF_TIMER_TASK_CLEAR)));

    //Configure a channel to start data packet transmission
    nrfx_timer_compare(timer, TIMER_CHANNEL_STARTTX,
                       COMMS_1WIRE_TIMESTAMP_PACKET_TX_TICKS + COMMS_1WIRE_DATA_PACKET_DELAY_TICKS,
                       false);

    //Data packet transmission is triggered by the timer
    APP_ERROR_CHECK(nrfx_ppi_channel_alloc(&tx_data_ppi));
    APP_ERROR_CHECK(nrfx_ppi_channel_assign(tx_data_ppi, nrfx_timer_event_address_get(timer, TIMER_EVENT_STARTTX), uarte_resource->get_starttx_task()));

    //Configure receive timeout duration, from the synch event
    nrfx_timer_compare(timer, TIMER_CHANNEL_RX_TIMEOUT,
                       COMMS_1WIRE_TIMESTAMP_PACKET_TX_TICKS + COMMS_1WIRE_DATA_PACKET_DELAY_TICKS + COMMS_1WIRE_DATA_PACKET_TX_TICKS + COMMS_1WIRE_RX_TIMEOUT_TICKS,
                       false);

    //Align UARTE clock with timestamp clock, which also starts the state machine
    nrf_ppi_channel_t temp;
    APP_ERROR_CHECK(nrfx_ppi_channel_alloc(&temp));
    APP_ERROR_CHECK(nrfx_ppi_channel_assign(temp, uarte_resource->get_endtx_event(), timestamp_get_timer_clear_task()));
    APP_ERROR_CHECK(nrfx_ppi_channel_enable(temp));
    uint8_t data = 0xFF;
    tx_at_trigger(&data, 1, TX_TRIGGER_NOW);
    while (state == STATE_SYNC_CLOCKS)
        __WFI();

    //Reset temp PPI
    APP_ERROR_CHECK(nrfx_ppi_channel_disable(temp));
    APP_ERROR_CHECK(nrfx_ppi_channel_free(temp));

#ifdef OMMO_DEBUG_UARTE_SYNCH_TIME
    nrf_gpio_cfg_output(OMMO_DEBUG_UARTE_SYNCH_TIME);
    nrf_gpio_pin_write(OMMO_DEBUG_UARTE_SYNCH_TIME, 0);
#endif

    //Init 10ms timer for disconnection watchdog, etc.
    APP_ERROR_CHECK(app_timer_create(app_timer_id, APP_TIMER_MODE_REPEATED, timed_loop_10ms));
    APP_ERROR_CHECK(app_timer_start(*app_timer_id, APP_TIMER_TICKS(10), this));
}


// Called from timestamp ISR (priority 3)
/*static*/ void comms_uarte_master_1wire::execute_timestamp_synch(uint32_t sample_event_value)
{
    next_timestamp_synch_value = sample_event_value;
    next_timestamp_synch_value_ready = true;

#ifdef OMMO_DEBUG_UARTE_SYNCH_TIME
    nrf_gpio_pin_toggle(OMMO_DEBUG_UARTE_SYNCH_TIME);
#endif
}


// Called from UARTE ISR (priority 1)
void comms_uarte_master_1wire::check_ready_and_transmit()
{
    if(next_timestamp_synch_value_ready)
    {
        next_timestamp_synch_value_ready = false;
        synch_count++;

        tx_timestamp.timestamp = next_timestamp_synch_value;
        if (
#if TEST_PATTERN_MODE
            test_pattern_tx_vs_rx || 
#endif
            tx_packet != nullptr)
        {
            tx_timestamp.flags = COMMS_1WIRE_FLAGS_MASTER_TX;
        }
        else if (rx_packet != nullptr)
        {
            tx_timestamp.flags = 0;
        }
        else
        {
            tx_timestamp.flags = COMMS_1WIRE_FLAGS_PING;
        }
        if (app_connection_status)
            tx_timestamp.flags |= COMMS_1WIRE_FLAGS_OPEN;
        if (!timestamp_is_synch_lost())
            tx_timestamp.flags |= COMMS_1WIRE_FLAGS_EXT_SYNCH;
        tx_at_trigger(&tx_timestamp, sizeof(tx_timestamp), TX_TRIGGER_SYNCH);
    }
    else
    {
        initiate_handshake();
    }
}


void comms_uarte_master_1wire::event_queue_process(void)
{
    state_t current_state = state;  // latch volatile
    bool current_connection_status = current_state < STATE_FIRST_DISCONNECTED ||
                                     current_state > STATE_LAST_DISCONNECTED;
    if (current_connection_status != last_connection_status)
    {
        last_connection_status = current_connection_status;
        ev_handler(current_connection_status ? EVENT_CONNECTED : EVENT_DISCONNECTED);
    }

    if (state == STATE_DISCONNECTED)
    {
        if (nrf_gpio_pin_read(rtx_pin))
        {
            state = STATE_CONNECTION_DELAY;
            connection_delay_ms = 0;
        }
        else
        {
            disconnected_count++;
        }
    }
    else if (state == STATE_CONNECTION_DELAY)
    {
        if (!nrf_gpio_pin_read(rtx_pin))
        {
            state = STATE_DISCONNECTED;
            disconnected_count++;
        }
    }
}


void comms_uarte_master_1wire::timed_loop_10ms()
{
    sw_watchdog_elapsed_ms += 10;
    if (sw_watchdog_elapsed_ms >= 100)
    {
        sw_watchdog_elapsed_ms = 0;

        // Check that UARTE ISR is still firing
        if (last_disconnected_count == disconnected_count &&
            last_synch_count == synch_count &&
            last_handshake_count == handshake_count &&
            last_connection_count == connection_count &&
            state != STATE_DISCONNECTED)
        {
            TRACE(TRACE_SW_WATCHDOG);
            state = STATE_ERROR;
#ifdef DEBUG_NRF
            sw_watchdog_count++;
#endif
        }
        else
        {
            last_disconnected_count = disconnected_count;
            last_synch_count = synch_count;
            last_handshake_count = handshake_count;
            last_connection_count = connection_count;
        }
    }

    if (state == STATE_CONNECTION_DELAY)
    {
        connection_delay_ms += 10;
        if (connection_delay_ms >= 100)
        {
            disconnected_count = 0;
            handshake_count_at_disconnection = handshake_count;
            initiate_handshake();
        }
    }

    if (state == STATE_ERROR)
    {
        TRACE(TRACE_ERROR_RECOVERY);
        while (uarte_resource->rx_in_progress())
        {
            uarte_resource->rx_abort();
            uarte_resource->uarte_rx_flush_fifo();
        }
        while (uarte_resource->tx_in_progress())
            uarte_resource->tx_abort();
        nrfx_timer_compare_int_disable(timer, TIMER_CHANNEL_RX_TIMEOUT);
        if (state == STATE_ERROR)
        {
            // Synchronize state update with UARTE events
            NRFX_CRITICAL_SECTION_ENTER();
            initiate_handshake();
            NRFX_CRITICAL_SECTION_EXIT();
        }
    }
}


ret_code_t comms_uarte_master_1wire::receive_packet_async(comms_1wire_slave_tx_data_packet_t *packet)
{
    if (packet == nullptr)
        return NRF_ERROR_INVALID_PARAM;

    if (rx_packet != nullptr)
        return NRF_ERROR_BUSY;

    rx_packet = packet;
    return NRF_SUCCESS;
}


ret_code_t comms_uarte_master_1wire::send_packet_async(const comms_1wire_master_tx_data_packet_t *packet)
{
    if (packet == nullptr)
        return NRF_ERROR_INVALID_PARAM;

    if (tx_packet != nullptr)
        return NRF_ERROR_BUSY;

    debug_msiu_tx_req.append(packet->data, packet->length);

    tx_packet = packet;
    return NRF_SUCCESS;
}


bool comms_uarte_master_1wire::is_tx_pending()
{
    return !!tx_packet;
}


void comms_uarte_master_1wire::cancel_tx_blocking(void)
{
    // Assumes that this is running at a lower priority than the UART ISR
    tx_packet = nullptr;
    while (state == STATE_TX_DATA)
        __WFI();
}


void comms_uarte_master_1wire::cancel_rx_blocking(void)
{
    // Assumes that this is running at a lower priority than the UART ISR
    rx_packet = nullptr;
    while (state == STATE_RX_DATA)
        __WFI();
}


void comms_uarte_master_1wire::set_app_connection_status(bool app_connection_status)
{
    this->app_connection_status = app_connection_status;
}
