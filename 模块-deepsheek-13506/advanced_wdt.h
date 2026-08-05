/** ------------------------------------------------------------------------ **
 *
 * @file advanced_wdt.h
 * @date 01/17/2025
 * @author Dmytro Kryvyi
 *
 * @brief nRF52 compatible advanced watchdog timer with backtrace report
 *
 ** ------------------------------------------------------------------------ **/

#pragma once

/* Includes ----------------------------------------------------------------- */
#include <stdint.h>
#include "sdk_errors.h"
#include "nrf_wdt.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Defines ------------------------------------------------------------------ */
/* Macros ------------------------------------------------------------------- */

/**
 * @brief Extract stack frame from IRQ handler and pass it to the handler.
 * @note IRQ handler must be as naked function.
 *
 * @param __handler Handler function to pass stack frame.
 */
#define IRQ_STACK_FRAME_EXTRACT(__handler)                                                     \
    __asm volatile("tst lr, #4                    \n"                                          \
                   "ite eq                        \n"                                          \
                   "mrseq r0, msp                 \n"                                          \
                   "mrsne r0, psp                 \n"                                          \
                   " ldr r1, =%0                  \n"                                          \
                   " bx r1                        \n"                                          \
                   :                                                                           \
                   : "i"(__handler)                                                            \
                   : "r0", "r1");

/* Enums -------------------------------------------------------------------- */
/* Types -------------------------------------------------------------------- */

/**
 * @brief Basic stack frame structure.
 */
typedef struct
{
    uint32_t r0;  /**< Register 0. */
    uint32_t r1;  /**< Register 1. */
    uint32_t r2;  /**< Register 2. */
    uint32_t r3;  /**< Register 3. */
    uint32_t r12; /**< Register 12. */
    uint32_t lr;  /**< Link register. */
    uint32_t pc;  /**< Program counter. */
    uint32_t psr; /**< Program status register. */
} cortex_basic_stack_frame_t;

/**
 * @brief Advanced watchdog handler.
 * @param stack_top Pointer to the stack frame.
 */
typedef void (*advanced_wdt_handler_t)(const cortex_basic_stack_frame_t * stack_top);

/* Shared functions --------------------------------------------------------- */

/**
 * @brief Initialize the watchdog.
 *
 * @note On nRF52 the watchdog is clocked by the 32.768 kHz clock, and you have only 2 of 32768
 *       ticks after the timeout event to handle it. After that the watchdog will reset the
 *       device.
 *
 * @param handler The handler function to call on watchdog timeout.
 * @param reload_value The reload value in milliseconds.
 * @param behaviour The watchdog behaviour in sleep/halt mode.
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
ret_code_t advanced_wdt_init(
      advanced_wdt_handler_t handler, uint32_t reload_value, nrf_wdt_behaviour_t behaviour);

/**
 * @brief Start the watchdog.
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
ret_code_t advanced_wdt_start(void);

/**
 * @brief Feed the watchdog.
 */
void advanced_wdt_feed(void);

/**
 * @brief Return the stack frame from the watchdog timeout.
 * @return Pointer to the stack frame during the watchdog exception.
 */
const cortex_basic_stack_frame_t * advanced_wdt_get_stack_frame(void);

/**
 * @brief Check if the watchdog was the cause of the reset and log the stack frame.
 * @note This function should be called in the main function before the watchdog is initialized.
 */
void advanced_wdt_check_boot_and_log(void);

#ifdef __cplusplus
}
#endif