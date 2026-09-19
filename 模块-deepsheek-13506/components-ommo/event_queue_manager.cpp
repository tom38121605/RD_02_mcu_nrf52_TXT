/*
 */

#include "event_queue_manager.hpp"

#include "nrf.h"
#include "nrf_assert.h"
#include "sdk_errors.h"
#include "sdk_macros.h"

#include "advanced_wdt.h"
#include "sdk_config.h"
#include "trace.h"
#include "ommo_app_error.h"
#include "app_util_platform.h"

//Main event queue should be used in _any_ busy loop that may take longer than a few hundred us
//Some of the items in the main event queue are still high priority.  We need to maintain an execute time
//of <1000us per loop.
event_queue_manager main_event_queue;

void ommo_main_evt_q_execute_once()
{
    main_event_queue.execute_once();
}

void ommo_main_evt_q_execute_once_w_sleep()
{
    main_event_queue.execute_once_with_sleep();
}

ret_code_t event_queue_manager::register_task_impl(void *context,
                                                    event_queue_task_pending_context pending_func,
                                                    event_queue_task_process_context process_func,
                                                    event_queue_task_acquire acquire_func,
                                                    event_queue_task_release release_func,
                                                    bool main_loop_only)
{
    VERIFY_TRUE(num_registered_functions < EVENT_QUEUE_MAX_FUNCTIONS, NRF_ERROR_NO_MEM);
    VERIFY_TRUE(process_func != nullptr, NRF_ERROR_INVALID_PARAM);

    // NOTE: Pending callbacks may be polled without their process callback
    //       running immediately (for example during sleep checks), so they
    //       must leave any "work pending" indicators intact.
    const uint16_t i = num_registered_functions;
    function_context[i]     = context;
    process_functions[i]    = (void*)process_func;
    pending_functions[i]    = (void*)pending_func;
    acquire_functions[i]    = (void*)acquire_func;
    release_functions[i]    = (void*)release_func;
    main_loop_only_flags[i] = main_loop_only;
    num_registered_functions++;

    return NRF_SUCCESS;
}

ret_code_t event_queue_manager::register_task(void *context,
                                               event_queue_task_pending_context pending_func,
                                               event_queue_task_process_context process_func,
                                               event_queue_task_acquire acquire_func,
                                               event_queue_task_release release_func,
                                               bool main_loop_only)
{
    VERIFY_TRUE(context != nullptr, NRF_ERROR_INVALID_PARAM);
    return register_task_impl(context, pending_func, process_func, acquire_func, release_func, main_loop_only);
}

ret_code_t event_queue_manager::register_task(void *context,
                                               event_queue_task_process_context process_func,
                                               event_queue_task_acquire acquire_func,
                                               event_queue_task_release release_func,
                                               bool main_loop_only)
{
    VERIFY_TRUE(context != nullptr, NRF_ERROR_INVALID_PARAM);
    return register_task_impl(context, nullptr, process_func, acquire_func, release_func, main_loop_only);
}

ret_code_t event_queue_manager::register_task(event_queue_task_pending pending_func,
                                               event_queue_task_process process_func,
                                               event_queue_task_acquire acquire_func,
                                               event_queue_task_release release_func,
                                               bool main_loop_only)
{
    return register_task_impl(nullptr,
                              (event_queue_task_pending_context)pending_func,
                              (event_queue_task_process_context)process_func,
                              acquire_func, release_func, main_loop_only);
}

ret_code_t event_queue_manager::register_task(event_queue_task_process process_func, bool main_loop_only)
{
    return register_task_impl(nullptr, nullptr,
                              (event_queue_task_process_context)process_func,
                              nullptr, nullptr, main_loop_only);
}

void event_queue_manager::execute_once() { execute_once_impl(false); }
void event_queue_manager::execute_once_as_main() { execute_once_impl(true); }
void event_queue_manager::execute_once_with_sleep() { execute_once_with_sleep_impl(false); }
void event_queue_manager::execute_once_with_sleep_as_main() { execute_once_with_sleep_impl(true); }

void event_queue_manager::execute_once_impl(bool in_main)
{
    // Disable event queue execution during fault handling.  IRQ handlers may fault, and the fault
    // handler calls functions (ex. for writing to flash) which call this function.
    if(in_ommo_app_error_handler)
        return;

    OMMO_APP_ERROR_CHECK(current_int_priority_get() == APP_IRQ_PRIORITY_THREAD ? NRF_SUCCESS : NRF_ERROR_FORBIDDEN, STRING("event_queue called from IRQ"), 0);

    for(uint16_t i=0; i<num_registered_functions; i++)
    {
        // Skip main-loop-only tasks when called from a nested/busy-wait context.
        if(main_loop_only_flags[i] && !in_main) continue;

        // Prevent re-entrant execution: if this process function is already on the call stack (somewhere below), skip it.
        const uint32_t bit = (1u << i);
        if(process_executing_mask & bit) continue;

        // Set the executing bit before pending/acquire so a nested execute_once_impl
        // cannot re-enter this task while we are evaluating it.
        process_executing_mask |= bit;

        // Check pending — null pending means always run.
        if(pending_functions[i] != nullptr)
        {
            bool pending = (function_context[i] == nullptr)
                ? ((event_queue_task_pending)pending_functions[i])()
                : ((event_queue_task_pending_context)pending_functions[i])(function_context[i]);

            if(!pending)
            {
                process_executing_mask &= ~bit;
                continue;
            }
        }

        // Try to acquire resources — null acquire means always allowed.
        if(acquire_functions[i] != nullptr)
        {
            bool acquired = ((event_queue_task_acquire)acquire_functions[i])();
            if(!acquired)
            {
                process_executing_mask &= ~bit; // resource unavailable — skip; release is not called
                continue;
            }
        }

#ifdef TRACE_OMIT_ADVANCED_WDT
        if (process_functions[i] != (void*)&advanced_wdt_feed)
#endif
        {
            TRACE_WITH_ARGS_2(TRACE_TYPE_TIME_FLASHADDR_U16, TRACE_LOCATION_EVENT_QUEUE_PROCESSING, (uint32_t)(size_t)process_functions[i], i);
        }

        if(function_context[i] == nullptr) ((event_queue_task_process)process_functions[i])();
        else ((event_queue_task_process_context)process_functions[i])(function_context[i]);

        process_executing_mask &= ~bit;

        if(release_functions[i] != nullptr)
            ((event_queue_task_release)release_functions[i])();
    }
}

/* This pattern ensures that an earlier work func doesn't get flagged
 * while executing a later work unit and then we go to sleep witout
 * executing the first work unit. */
void event_queue_manager::execute_once_with_sleep_impl(bool in_main)
{
    execute_once_impl(in_main);

    NRFX_CRITICAL_SECTION_ENTER();
    //Check to see if _any_ registered event queue has work to do
    bool work_present = false;
    for(uint16_t i=0; i<num_registered_functions && !work_present; i++)
    {
        if(main_loop_only_flags[i] && !in_main) 
            continue; // can't run here; don't let it prevent sleep

        if(pending_functions[i] != nullptr) //null pending functions do not prevent sleep
        {
            if(function_context[i] == nullptr) work_present |= ((event_queue_task_pending)pending_functions[i])();
            else work_present |= ((event_queue_task_pending_context)pending_functions[i])(function_context[i]);
        }
    }
    if(!work_present)
        __WFI();
    NRFX_CRITICAL_SECTION_EXIT();
}

void event_queue_manager::execute_forever()
{
    while(true)
        execute_once_impl(true);
}

void event_queue_manager::execute_forever_with_sleep()
{
    while(true)
        execute_once_with_sleep_impl(true);
}

void event_queue_manager::reset()
{
    num_registered_functions = 0;
    process_executing_mask = 0;
    for(uint16_t i=0; i<EVENT_QUEUE_MAX_FUNCTIONS; i++)
    {
        acquire_functions[i]    = nullptr;
        release_functions[i]    = nullptr;
        main_loop_only_flags[i] = false;
    }
}
