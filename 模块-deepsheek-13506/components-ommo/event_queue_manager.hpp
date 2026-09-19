#pragma once

#include <stdbool.h>
#include <stdarg.h>
#include <stdint.h>

#include "nrfx.h"
#include "sdk_errors.h"

#include "ommo_config.h"
#include "nrfx_timer.h"

// --------------------------------------------------------------------------
// C-Compatible Interface (Visible to both C and C++)
// --------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif

void ommo_main_evt_q_execute_once();
void ommo_main_evt_q_execute_once_w_sleep();

#ifdef __cplusplus
}
#endif


// --------------------------------------------------------------------------
// C++ Only Interface (Hidden from C)
// --------------------------------------------------------------------------
#ifdef __cplusplus

// Only support 32 functions
#define EVENT_QUEUE_MAX_FUNCTIONS 32

static_assert(EVENT_QUEUE_MAX_FUNCTIONS <= 32,
              "event_queue_manager supports at most 32 functions.");

//Function prototypes
typedef void (*event_queue_task_process_context)(void *context);
typedef bool (*event_queue_task_pending_context)(void *context);
typedef void (*event_queue_task_process)();
typedef bool (*event_queue_task_pending)();
typedef bool (*event_queue_task_acquire)();
typedef void (*event_queue_task_release)();

// Main event queue — should be used in any busy loop expected to run longer than a few hundred microseconds.
// Some queued tasks are still high priority, so the loop should maintain an execution time
// of under ~1000 µs per iteration to ensure timely processing.
class event_queue_manager;
extern event_queue_manager main_event_queue;

/**
 * @brief Manages and executes a list of tasks, with optional task-pending functions.
 *
 * The event_queue_manager executes tasks either unconditionally or based on each task's pending state.
 *
 * A task can be defined in one of two ways:
 * - As a simple always-run task, executed in the order it was added.
 * - As a conditional task with both a @c task_pending() and a @c task_process() function.
 *
 * When a task supports a "pending" concept, it must be split into separate @c task_pending() and
 * @c task_process() functions. This ensures the processor does not enter sleep mode while a task still
 * requires service. (See Note 1.)
 *
 * Both @c task_pending() and @c task_process() may optionally take a @c (void* context) parameter, but they
 * must have matching signatures—either both use the context or neither does. If a context parameter is used,
 * it must not be @c nullptr.
 *
 * @note
 * Note 1: Example scenario illustrating why pending and process functions are separate.
 *
 * Imagine two tasks are in the queue, and @c execute_with_sleep() is called:
 * 1. The processor calls @c task_pending() for Task 1 → returns false.
 * 2. The processor calls @c task_pending() for Task 2 → returns true, so @c task_process() for Task 2 runs.
 * 3. While Task 2 is executing, an interrupt updates Task 1, making its @c task_pending() return true.
 * 4. When Task 2 finishes, the processor might attempt to sleep, even though Task 1 now requires service.
 *
 * The event_queue_manager prevents this issue by separating the pending check from the processing step,
 * ensuring that all newly-ready tasks are serviced before the system enters sleep mode.
 */
class event_queue_manager
{
    public:
        event_queue_manager() :
            num_registered_functions(0),
            process_executing_mask(0)
        {}

        //Functions with context signature
        // NOTE: Pending callbacks may be invoked speculatively (e.g. to check
        //       for work before sleeping) without their paired process
        //       function running, so they must not clear or mutate the state
        //       that indicates work is pending.
        ret_code_t register_task(void *context,
                                 event_queue_task_pending_context pending_func,
                                 event_queue_task_process_context process_func,
                                 event_queue_task_acquire acquire_func = nullptr,
                                 event_queue_task_release release_func = nullptr,
                                 bool main_loop_only = false);
        ret_code_t register_task(void *context,
                                 event_queue_task_process_context process_func,
                                 event_queue_task_acquire acquire_func = nullptr,
                                 event_queue_task_release release_func = nullptr,
                                 bool main_loop_only = false);

        //Functions without parameters
        ret_code_t register_task(event_queue_task_pending pending_func,
                                 event_queue_task_process process_func,
                                 event_queue_task_acquire acquire_func = nullptr,
                                 event_queue_task_release release_func = nullptr,
                                 bool main_loop_only = false);
        ret_code_t register_task(event_queue_task_process process_func, bool main_loop_only = false);

        void execute_once();
        void execute_once_as_main();            // like execute_once() but runs main_loop_only tasks too
        void execute_once_with_sleep();
        void execute_once_with_sleep_as_main(); // like execute_once_with_sleep() but runs main_loop_only tasks too
        void execute_forever();
        void execute_forever_with_sleep();
        void reset();

    private:
        void execute_once_impl(bool in_main);
        void execute_once_with_sleep_impl(bool in_main);
        ret_code_t register_task_impl(void *context,
                                      event_queue_task_pending_context pending_func,
                                      event_queue_task_process_context process_func,
                                      event_queue_task_acquire acquire_func,
                                      event_queue_task_release release_func,
                                      bool main_loop_only);
        void *function_context[EVENT_QUEUE_MAX_FUNCTIONS];
        void *process_functions[EVENT_QUEUE_MAX_FUNCTIONS];
        void *pending_functions[EVENT_QUEUE_MAX_FUNCTIONS];
        void *acquire_functions[EVENT_QUEUE_MAX_FUNCTIONS];
        void *release_functions[EVENT_QUEUE_MAX_FUNCTIONS];
        bool main_loop_only_flags[EVENT_QUEUE_MAX_FUNCTIONS];
        uint16_t num_registered_functions;
        uint32_t process_executing_mask; // Bit i == 1 means process_functions[i] is currently executing somewhere
};

#endif // __cplusplus
