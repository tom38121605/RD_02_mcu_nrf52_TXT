/** ------------------------------------------------------------------------ **
 * @file uarte_basic.hpp
 * @date 09/16/2025
 *
 * @brief UARTE basic driver class implementing comm_device.
 *        TX then (optional) RX sequencing; blocking and async variants.
 ** ------------------------------------------------------------------------ **/
#pragma once

/* Includes ----------------------------------------------------------------- */
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "nrfx_uarte.h"
#include "comm_device.hpp"

/* Classes ------------------------------------------------------------------ */
class uarte_basic : public comm_device
{
    public:
        /**
         * @param p_instance Pointer to the UARTE instance; must outlive this object.
         * @param p_cfg Optional initial config; defaults to NRFX UARTE defaults.
         */
        explicit uarte_basic(const nrfx_uarte_t * p_instance,
                             const nrfx_uarte_config_t * p_cfg = nullptr);
        ~uarte_basic() override;

        uint32_t get_startrx_task() const;
        uint32_t get_starttx_task() const;
        uint32_t get_endrx_event() const;
        uint32_t get_endtx_event() const;
        uint32_t rx_abort() const;
        uint32_t tx_abort() const;
        bool rx_in_progress() const;
        bool tx_in_progress() const;
        void uarte_rx_flush_fifo() const;
        uint32_t set_txrx_pins_set(uint32_t pseltxd, uint32_t pselrxd);
        void switch_to_tx_only();
        void switch_to_rx_only();
        void switch_to_normal_trx();

        const nrfx_uarte_t *get_nrfx_uarte_instance() const;

        /** (Re)configure the UARTE. If already acquired, it is safely uninit+reinit. */
        ret_code_t reconfigure(const nrfx_uarte_config_t * p_cfg, nrfx_uarte_event_handler_t event_handler = nullptr, void *p_context = nullptr);

        /** Acquire/init the UARTE if not already acquired. */
        ret_code_t acquire(const nrfx_uarte_config_t * p_cfg, nrfx_uarte_event_handler_t event_handler = nullptr, void *p_context = nullptr);

        ret_code_t rx_variable(uint8_t *data, uint16_t max_size, uint16_t *received_size, uint32_t timeout);
        /* comm_device overrides ------------------------------------------------- */
        [[nodiscard]] bool is_acquired() const override;
        ret_code_t release() override;

        /** Blocking TX (if provided) then RX (if provided). */
        ret_code_t trx(const uint8_t * p_tx_buffer, size_t tx_length,
                       uint8_t * p_rx_buffer, size_t rx_length,
                       comm_device_trx_flags_t flags = COMM_DEVICE_FLAGS_NONE) override;

        /** Non-blocking TX (if provided) then RX (if provided); handler on completion. */
        ret_code_t trx(const uint8_t * p_tx_buffer, size_t tx_length,
                       uint8_t * p_rx_buffer, size_t rx_length,
                       comm_device_event_handler_t handler,
                       void * p_context = nullptr,
                       comm_device_trx_flags_t flags = COMM_DEVICE_FLAGS_NONE) override;

        [[nodiscard]] bool is_trx_completed() const override;

    private:
        /* State ---------------------------------------------------------------- */
        const nrfx_uarte_t * uarte_instance;
        nrfx_uarte_config_t  uarte_config;
        uint8_t rtc_timeout_channel = 0xFF;

        enum : uint8_t {
            UARTEB_ACQ_NONE = 0,
            UARTEB_ACQ_NORMAL,
            UARTEB_ACQ_DIRECT
        } acq = UARTEB_ACQ_NONE;

        enum : uint8_t {
            UARTEB_STEP_IDLE = 0,
            UARTEB_STEP_TX,
            UARTEB_STEP_RX,
            UARTEB_STEP_VARIABLE_RX
        } step = UARTEB_STEP_IDLE;

        /* Async user hooks */
        volatile comm_device_event_handler_t evt_handler = nullptr;
        void * evt_context = nullptr;

        /* In-flight buffers (owned by caller) */
        const uint8_t * cur_tx = nullptr;
        size_t          cur_tx_len = 0;
        uint8_t *       cur_rx = nullptr;
        size_t          cur_rx_len = 0;

        /* Handlers ------------------------------------------------------------- */
        static void static_uarte_basic_handler(nrfx_uarte_event_t const * p_event, void * p_context);
        static void static_timeout_handler(void * p_context);

        /* Helpers --------------------------------------------------------------- */
        void start_next_step(bool hold_next_step = false);   // advance TX->RX->IDLE
        static uint64_t get_pin_mask(const nrfx_uarte_config_t * p_cfg);
        void setup_pins(uint32_t pseltxd, uint32_t pselrxd);
};
