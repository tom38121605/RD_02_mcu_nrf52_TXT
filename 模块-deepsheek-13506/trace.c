/**
 * Copyright (c) 2019, Ommo Technologies
 * 
 * All rights reserved.
 * 
 * 
 */

#include "trace.h"

#include <string.h>

#include "app_error.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_nvmc.h"
#include "nrfx_gpiote.h"

#include "advanced_wdt.h"
#include "ommo_config.h"

#ifdef OMMO_TIMESTAMP_TIMER
#include "timestamp_timer.h"
#endif

extern uint8_t __start_heap;

#ifdef OMMO_TRACE_ENABLED

const bool trace_stop_when_full = false;  // Remove 'const' for development use
trace_state_t trace_state __attribute__((section(".trace_log"))) = {0};
static bool trace_reset_request __attribute__((section(".non_init")));

#ifdef OMMO_TRACE_STACK_CHECK
static size_t trace_lowest_stack_marker = SIZE_MAX;
#endif

#endif  // OMMO_TRACE_ENABLED


// Allow applications to override this, and allow it to be used when tracing is disabled
__WEAK uint32_t trace_get_timestamp()
{
#ifdef OMMO_TIMESTAMP_TIMER
    return timestamp_get_current_timestamp();
#else
    return 0;
#endif
}


#ifdef OMMO_TRACE_ENABLED

#ifdef OMMO_STOP_TRACE_INPUT_PIN

static void stop_trace_input_pin_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action, void *context)
{
    trace_stop();
}

#endif


static size_t trace_calc_count_available()
{
    return ((size_t)&__start_heap - (size_t)&trace_state.log[0]) / sizeof(trace_state.log[0]);
}


static void trace_reset_internal()
{
    // Note that this function can be called from ISRs via trace_reset,
    // so it needs to be very fast and cannot be O(trace_state.count).
    memset(&trace_state, 0, offsetof(trace_state_t, log));
    trace_state.signature = TRACE_SIGNATURE;
    size_t trace_count_available = trace_calc_count_available();
    trace_state.count = trace_count_available > UINT16_MAX ? UINT16_MAX : (uint16_t)trace_count_available;
    trace_state.log[trace_state.count - 1] = TRACE_TYPE_NOT_WRAPPED << TRACE_TYPE_SHIFT;
}


void trace_reset()
{
    NRFX_CRITICAL_SECTION_ENTER();
    trace_reset_internal();
    NRFX_CRITICAL_SECTION_EXIT();
}


static void trace_boot()
{
    // Do not reset trace_index or trace_log so that the log persists across reboots.
    // These signature and count checks will catch where the address changed or heap grew without a power cycle.
    size_t trace_count_available = trace_calc_count_available();
    if (trace_state.signature != TRACE_SIGNATURE || trace_state.count > trace_count_available)
        trace_reset_internal();

    // Programmer may manually request a reset via a debugger + restart
    if (trace_reset_request)
    {
        trace_reset_request = false;
        trace_reset_internal();
    }
}

// Add an entry to the start of the CRT initializer function list.  If tracing is enabled,
// this trace_boot will be called before entering main, ensuring that tracing is available
// as early as possible, even for other initializers.
typedef void init_function();
static init_function *trace_boot_crt_hook __attribute__((used, section(".ctors.00000"))) = &trace_boot;


void trace_init()
{
#ifdef OMMO_STOP_TRACE_INPUT_PIN
    if (!nrfx_gpiote_is_init())
        APP_ERROR_CHECK(nrfx_gpiote_init());

    nrfx_gpiote_in_config_t stop_trace_in_config = NRFX_GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
    APP_ERROR_CHECK(nrfx_gpiote_in_init(OMMO_STOP_TRACE_INPUT_PIN, &stop_trace_in_config, stop_trace_input_pin_handler));
    nrfx_gpiote_in_event_enable(OMMO_STOP_TRACE_INPUT_PIN, true);
#endif
#ifdef OMMO_STOP_TRACE_OUTPUT_TOGGLE_PIN
    nrf_gpio_pin_write(OMMO_STOP_TRACE_OUTPUT_TOGGLE_PIN, 0);
    nrf_gpio_cfg_output(OMMO_STOP_TRACE_OUTPUT_TOGGLE_PIN);
#elif defined(OMMO_STOP_TRACE_OUTPUT_PULSE_PIN)
    nrf_gpio_pin_write(OMMO_STOP_TRACE_OUTPUT_PULSE_PIN, 0);
    nrf_gpio_cfg_output(OMMO_STOP_TRACE_OUTPUT_PULSE_PIN);
#endif
}


bool trace_stop()
{
    bool stopped_transition = false;

    if (!trace_state.stopped)
    {
        NRFX_CRITICAL_SECTION_ENTER();
        if (!trace_state.stopped)
        {
            // Stop toggling the pin after the first trigger, to enable resetting trace_stopped on the
            // downstream device before resetting this device
#ifdef OMMO_STOP_TRACE_OUTPUT_TOGGLE_PIN
            nrf_gpio_pin_toggle(OMMO_STOP_TRACE_OUTPUT_TOGGLE_PIN);
#endif
#ifdef OMMO_STOP_TRACE_OUTPUT_PULSE_PIN
            nrf_gpio_pin_set(OMMO_STOP_TRACE_OUTPUT_PULSE_PIN);
            // This delay is inside the critical section to ensure that it is precisely 20us,
            // so that a scope can be configured to look for this specific pulse duration.
            nrf_delay_us(20);
            nrf_gpio_pin_clear(OMMO_STOP_TRACE_OUTPUT_PULSE_PIN);
#endif
            trace_state.stopped = stopped_transition = true;
        }

        NRFX_CRITICAL_SECTION_EXIT();
    }

    return stopped_transition;
}


uint32_t *trace_log_open()
{
#ifdef OMMO_TRACE_STACK_CHECK
    uint32_t stack_marker;
    APP_ERROR_CHECK_BOOL((size_t)&stack_marker >= (size_t)&__StackLimit);
    if (trace_lowest_stack_marker > (size_t)&stack_marker)
        trace_lowest_stack_marker = (size_t)&stack_marker;
#endif

    if (trace_state.stopped)
        return NULL;

    uint32_t index = trace_state.index;
    uint32_t *log = &trace_state.log[index];
    uint32_t count = trace_state.count;
    if (index < count - MAX_TRACE_RECORD_SIZE)
        return log;

    if (trace_stop_when_full)
    {
        trace_stop();
        return NULL;
    }

    if (index < count)
    {
        // Create a TRACE_TYPE_NONE entry that spans the rest of the log
        *log = 0;
        trace_state.log[count-1] = 0;
        trace_store_length(log, &trace_state.log[count]);
    }
    
    return &trace_state.log[0];
}


uint8_t *trace_copy(uint8_t *buf, size_t buf_size)
{
    const size_t LOG_ENTRY_SIZE = sizeof(trace_state.log[0]);
    
    // Check whether there is space to save any log entries
    size_t header_size = offsetof(trace_state_t, log);
    if (header_size + LOG_ENTRY_SIZE > buf_size)
        return NULL;

    // Validate trace index
    if (trace_state.index > trace_state.count)
        return NULL;

    // Copy entire trace log header, which will enable reconstructing the original buffer
    memcpy(buf, &trace_state, header_size);
    buf += header_size;
    buf_size -= header_size;

    // Copy most recent entries from [0..trace_state.index]
    size_t remaining_log_entries = buf_size / LOG_ENTRY_SIZE;
    size_t entries_to_copy = trace_state.index;
    if (entries_to_copy > remaining_log_entries)
        entries_to_copy = remaining_log_entries;
    size_t bytes_to_copy = entries_to_copy * LOG_ENTRY_SIZE;
    memcpy(buf, &trace_state.log[trace_state.index - entries_to_copy], bytes_to_copy);
    buf += bytes_to_copy;
    remaining_log_entries -= entries_to_copy;

    // Copy wrapped entries from [trace_state.index..TRACE_COUNT]
    entries_to_copy = trace_state.count - trace_state.index;
    if (entries_to_copy > remaining_log_entries)
        entries_to_copy = remaining_log_entries;
    bytes_to_copy = entries_to_copy * LOG_ENTRY_SIZE;
    memcpy(buf, &trace_state.log[trace_state.count - entries_to_copy], bytes_to_copy);
    buf += bytes_to_copy;

    return buf;
}


#endif  // OMMO_TRACE_ENABLED
