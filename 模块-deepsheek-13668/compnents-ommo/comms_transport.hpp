#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "sdk_errors.h"

typedef void (*comms_transport_on_tx_complete)(size_t actual_tx_length, ret_code_t err_code, const void *p_context);
typedef void (*comms_transport_on_rx_complete)(size_t actual_rx_length, bool failed, const void *p_context);

/**
 * @brief Abstract interface for stream-based transport implementations.
 *
 * The comms_transport_interface defines a standard transmit/receive API and
 * completion callback mechanism, allowing higher-level classes (such as
 * comms_packetizer) to operate over any underlying transport. Implementations
 * translate between this common interface and a specific low-level link (e.g.,
 * USB, UART, etc.), enabling conversion of byte streams to and from
 * application-level packets.
 */
class comms_transport_interface
{
    public:
        comms_transport_interface() : rx_complete_callback(nullptr), tx_complete_callback(nullptr), p_context(nullptr) {}
        virtual ~comms_transport_interface() = 0;
        void set_callbacks(comms_transport_on_rx_complete rx_cb, comms_transport_on_tx_complete tx_cb, void *p_context_g);

        //Interface functions
        virtual bool is_connected() = 0;
        virtual ret_code_t comms_transport_tx(uint8_t *p_tx_buffer, size_t tx_length, bool expedite_tx) = 0;
        virtual ret_code_t comms_transport_rx(uint8_t *p_rx_buffer, size_t max_rx_length) = 0;

    protected:
        comms_transport_on_rx_complete rx_complete_callback;
        comms_transport_on_tx_complete tx_complete_callback;
        void    *p_context;
};
