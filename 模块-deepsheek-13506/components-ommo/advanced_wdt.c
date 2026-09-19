/** ------------------------------------------------------------------------ **
 *
 * @file advanced_wdt.c
 * @date 01/17/2025
 * @author Dmytro Kryvyi
 *
 ** ------------------------------------------------------------------------ **/

/* Includes ----------------------------------------------------------------- */
#include <stddef.h>
#include "advanced_wdt.h"
#include "ommo_app_error.h"
#include "nrf_power.h"

/* Defines ------------------------------------------------------------------ */
/* Macros ------------------------------------------------------------------- */
/* Enums -------------------------------------------------------------------- */
/* Types -------------------------------------------------------------------- */
/* Variables ---------------------------------------------------------------- */

/** The advanced WDT context structure.
 *  @note This structure is placed in the non-initialized section.
 */
static struct
{
    cortex_basic_stack_frame_t wdt_stack_frame; /**< Stack frame for WDT IRQ handler. */
    advanced_wdt_handler_t     m_handler;       /**< Handler for WDT exception. */

    uint32_t wdt_correct_key; /**< Key indicating that the stack frame is correct. */
} m_wdt __attribute__((section(".non_init")));

/* Private functions -------------------------------------------------------- */

/**
 * @brief C compatible Handler for WDT IRQ called after naked IRQ handler.
 * @param stack_top Pointer to the stack frame.
 */
void handle_wdt(const cortex_basic_stack_frame_t * stack_top)
{
    if (nrf_wdt_event_check(NRF_WDT_EVENT_TIMEOUT))
    {
        NRF_BREAKPOINT_COND;

        m_wdt.wdt_stack_frame = *stack_top;
        m_wdt.wdt_correct_key = NRF_WDT_RR_VALUE;
        if (m_wdt.m_handler)
        {
            m_wdt.m_handler(&m_wdt.wdt_stack_frame);
        }
        nrf_wdt_event_clear(NRF_WDT_EVENT_TIMEOUT);
    }
}

/* Shared functions --------------------------------------------------------- */

ret_code_t advanced_wdt_init(
      const advanced_wdt_handler_t handler,
      const uint32_t               reload_value,
      const nrf_wdt_behaviour_t    behaviour)
{
    const uint64_t ticks = (reload_value * 32768ULL) / 1000;
    if (ticks > UINT32_MAX)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    m_wdt.m_handler = handler;
    nrf_wdt_behaviour_set(behaviour);
    nrf_wdt_reload_value_set(ticks);
    nrf_wdt_reload_request_enable(NRF_WDT_RR0);

    NVIC_SetPriority(WDT_IRQn, APP_IRQ_PRIORITY_HIGHEST);
    NVIC_EnableIRQ(WDT_IRQn);

    nrf_wdt_event_clear(NRF_WDT_EVENT_TIMEOUT);
    nrf_wdt_int_enable(NRF_WDT_INT_TIMEOUT_MASK);
    return NRF_SUCCESS;
}

ret_code_t advanced_wdt_start(void)
{
    m_wdt.wdt_correct_key = 0;
    nrf_wdt_task_trigger(NRF_WDT_TASK_START);
    return NRF_SUCCESS;
}

void advanced_wdt_feed(void)
{
    nrf_wdt_reload_request_set(NRF_WDT_RR0);
}

const cortex_basic_stack_frame_t * advanced_wdt_get_stack_frame(void)
{
    return m_wdt.wdt_correct_key == NRF_WDT_RR_VALUE ? &m_wdt.wdt_stack_frame : NULL;
}

/**
 * @brief Naked IRQ handler for WDT.
 * @note You cannot create any of local variables in this function.
 */
__attribute__((naked)) void nrfx_wdt_irq_handler(void)
{
    // Call the handler with the stack frame.  The WDT reset will
    // occur in 2 WDT clock cycles, about 61 microseconds.
    IRQ_STACK_FRAME_EXTRACT(handle_wdt);
}

void advanced_wdt_check_boot_and_log(void)
{
    //Check for watchdog reset and go to error handler if the dog was the cause
    if(nrf_power_resetreas_get() & NRF_POWER_RESETREAS_DOG_MASK)
    {
        nrf_power_resetreas_clear(NRF_POWER_RESETREAS_DOG_MASK);

        // Do we have a watchdog stack frame?
        const cortex_basic_stack_frame_t * affected_stack_frame
              = advanced_wdt_get_stack_frame();
        if (affected_stack_frame != NULL)
        {
            char msg[64] = { 0 };
            snprintf(
                  msg, sizeof(msg), "WDT: R0 %08X, R1 %08X, LR %08X, PC: %08X",
                  affected_stack_frame->r0, affected_stack_frame->r1, affected_stack_frame->lr,
                  affected_stack_frame->pc);
            OMMO_APP_ERROR_CHECK(NRF_ERROR_TIMEOUT, STRING(msg), 0);
        }

        APP_ERROR_HANDLER(NRF_ERROR_TIMEOUT);
    }
}