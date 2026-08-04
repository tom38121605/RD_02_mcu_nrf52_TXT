#pragma once

#include <stdarg.h>

#include "sdk_errors.h"
#include "nrfx.h"

#include "comms_transport.hpp"
#include "ommo_config.h"
#include "event_queue_manager.hpp"
#include "ommo_macros.h"

#if !defined(USB_SERIAL_RX_BUFFER_SIZE) && !defined(USB_SERIAL_TX_BUFFER_SIZE)
#define USB_SERIAL_TX_BUFFER_SIZE 16384
#define USB_SERIAL_RX_BUFFER_SIZE (3 * 1024)
#endif
#ifndef USB_SERIAL_RX_RAW_BUFFER_SIZE
#define USB_SERIAL_RX_RAW_BUFFER_SIZE 512
#endif

#define RTC_CHANNEL_UNINIT_VALUE    0xFF
#define RTC_TIMER                   (flush_timeout_channel != RTC_CHANNEL_UNINIT_VALUE)

typedef void (*comms_packetizer_packet_ready_handler)(uint8_t data[], uint16_t length, const void *p_context);

/**
 * @brief Handles packetization of data for transport over a stream-based interface.
 *
 * The comms_packetizer class provides a high-level interface for converting 
 * between application-level packet data and the underlying byte-stream transport 
 * provided by any class implementing comms_transport_interface. It handles both 
 * outbound and inbound data processing, including packet assembly, packet 
 * parsing, and optional timed flushing of partially filled packets.
 * 
 * Outbound Packet Assembly
 *   Accepts write requests from the system in the form of small packets or data 
 *   fragments.
 *   Accumulates these fragments into larger packets for efficient transmission.
 *   Passes the assembled packets to a transport object (via 
 *   comms_transport_interface) for sending.
 *   Supports both blocking and non-blocking send modes.
 *   Supports reserving space at the start of the buffer for a packet header.
 * 
 * Inbound Packet Processing
 *   Reads raw stream data from the transport interface.
 *   Decodes and splits the incoming byte stream into discrete packets.
 *   Verifies packet integrity (e.g., CRC checks) and reports errors if detected.
 *   Invokes the user-provided comms_packetizer_packet_ready_handler callback for 
 *   each successfully received packet, passing along the data and any context 
 *   pointer.
 * 
 * Flush Timeout Mechanism (Optional)
 *   When enabled, starts a timer to ensure that any partially filled outbound 
 *   packet is sent after a maximum of 500 ms, even if the packet has not reached 
 *   its minimum size threshold.
 *   This prevents excessive latency when data is generated infrequently, ensuring 
 *   timely delivery over the transport.
 */
 
typedef enum
{
    TRANSPORT_FLAG_NONE = 0,
    TRANSPORT_FLAG_IN_PROGRESS = 1,
    TRANSPORT_FLAG_FLUSH = 2
} transport_flags_t;

class comms_packetizer
{
    public:
        void enable_packet_reading();
        void disable_packet_reading();

        ret_code_t send_ack_blocking(const uint8_t return_code, const bool block);
        ret_code_t send_ack(const uint8_t return_code);
        ret_code_t send_command_packet_blocking(const uint8_t * buffer, const size_t length, const bool block);
        ret_code_t send_command_packet(const uint8_t * buffer, size_t length);
        ret_code_t write(const uint8_t * buffer, const uint16_t length);
        ret_code_t write_data_packet_vargslist(const size_t data_length, va_list valist);     
        inline ret_code_t write_data_packet_vargs(size_t data_length, ...)
        {
            va_list args;
            va_start(args, data_length);
            ret_code_t rvalue = write_data_packet_vargslist(data_length, args);
            va_end(args);
            return rvalue;
        }

        ret_code_t write_now(const uint8_t * buffer, const uint16_t length, const bool block);
        ret_code_t enable_flush_timeout();

        // Reserves space at the start of the buffer for a packet header, to be filled in by the transport.
        void set_tx_header_size(uint16_t reserved_size);

        void init(comms_transport_interface *transport, const comms_packetizer_packet_ready_handler packet_ready_callback, const void *p_context);
        ret_code_t add_tasks_to_event_queue(event_queue_manager *event_queue);

    private:    
        static void static_transport_read_complete(size_t actual_rx_length, bool failed, const void *p_context);
        static void static_transport_write_complete(size_t actual_tx_length, ret_code_t err_code, const void *p_context);
        static void static_on_flush_timeout(void * p_context);
        void transport_read_complete(size_t actual_rx_length, bool failed);
        void transport_write_complete(size_t actual_tx_length, ret_code_t err_code);
        void setup_next_rx();
        void setup_next_tx();
        void on_tx_complete(ret_code_t err_code);
        void initiate_rx();
        void initiate_tx();
        ret_code_t flush_tx_buffers(bool block);

        //Event queue functions
        bool event_queue_start_tx_pending();
        void event_queue_start_tx_process();
        CREATE_STATIC_WRAPPER(comms_packetizer, event_queue_start_tx_pending, bool);
        CREATE_STATIC_WRAPPER(comms_packetizer, event_queue_start_tx_process, void);

        bool event_queue_start_rx_pending();
        void event_queue_start_rx_process();
        CREATE_STATIC_WRAPPER(comms_packetizer, event_queue_start_rx_pending, bool);
        CREATE_STATIC_WRAPPER(comms_packetizer, event_queue_start_rx_process, void);
                
        bool event_queue_rx_error_recovery_pending();
        void event_queue_rx_error_recovery_process();
        CREATE_STATIC_WRAPPER(comms_packetizer, event_queue_rx_error_recovery_pending, bool);
        CREATE_STATIC_WRAPPER(comms_packetizer, event_queue_rx_error_recovery_process, void);
                
        bool event_queue_tx_error_recovery_pending();
        void event_queue_tx_error_recovery_process();
        CREATE_STATIC_WRAPPER(comms_packetizer, event_queue_tx_error_recovery_pending, bool);
        CREATE_STATIC_WRAPPER(comms_packetizer, event_queue_tx_error_recovery_process, void);
                
        inline void clear_tx_buffer()
        {
            tx_buffer_active_top = tx_buffer_base;
        }

        inline void clear_cobs_rx_buffer()
        {        
            NRFX_CRITICAL_SECTION_ENTER();
            rx_cobs_buffer_top = 0;
            rx_cobs_buffer_bottom = 0;
            rx_cobs_buffer_in_use = 0;
            rx_data_ready = false;
            rx_failed = false;
            NRFX_CRITICAL_SECTION_EXIT();
        }

        //Transport layer interface
        comms_transport_interface *transport_device;

        // TX packet status
        uint32_t          tx_lost_packets;
        ret_code_t        tx_err_code;
        volatile uint8_t  flush_flag;
        uint8_t           tx_retry_count;
        bool              tx_failed;
        bool              in_comms_transport_tx;
        
        // RX packet status
        bool              in_comms_transport_rx;
        bool              rx_data_ready;
        bool              rx_failed;
        uint32_t          rx_lost_packets;

        // Buffers
        volatile uint16_t tx_buffer_base;
        volatile uint16_t tx_buffer_active_top;
        uint8_t           tx_buffer_primary[USB_SERIAL_TX_BUFFER_SIZE];
        uint8_t           tx_buffer_secondary[USB_SERIAL_TX_BUFFER_SIZE];
        uint8_t *         tx_buffer_active;

        // Transport in progress state
        volatile uint8_t  tx_buffer_transport_flags;  // bit mask of transport_flags_t
        uint8_t *         tx_buffer_transport;
        uint8_t *         tx_buffer_transport_end;

        volatile uint16_t rx_cobs_buffer_in_use;  // event_queue_process/main reads below here
        uint16_t          rx_cobs_buffer_bottom;  // event_queue_process resumes reads here
        volatile uint16_t rx_cobs_buffer_top;  // transport_read_complete/ISR writes above here
        uint8_t           rx_raw_buffer[USB_SERIAL_RX_RAW_BUFFER_SIZE];
        uint8_t           rx_cobs_buffer[USB_SERIAL_RX_BUFFER_SIZE];

        // Optional flush timer to ensure data is sent over transport layer in maximum time
        uint8_t    flush_timeout_channel;

        //Callback to main
        comms_packetizer_packet_ready_handler  packet_handler;
        const void *packet_handler_context;
};




