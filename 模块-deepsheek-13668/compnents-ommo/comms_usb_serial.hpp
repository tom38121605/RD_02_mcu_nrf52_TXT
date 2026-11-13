#pragma once

#include "USB_CDC.h"
#include "comms_packetizer.hpp"
#include "comms_transport.hpp"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    USB_SERIAL_STATE_DETACHED, /*!< USB is not connected to the host. */
    USB_SERIAL_STATE_ATTACHED, /*!< USB is connected to the host, but CDC ACM is not in use. */
    USB_SERIAL_STATE_OPENED,
    USB_SERIAL_INTERNAL_STATE_RECONNECTING,  /*!< USB is connected to the host and port was open, but signal was lost */

    USB_SERIAL_STATE_FIRST_OPENED = USB_SERIAL_STATE_OPENED,  /*!< States at/above this value are/were in an open state. */
} usb_serial_state_t;

typedef enum
{
    USB_SERIAL_EVENT_NONE,
    USB_SERIAL_EVENT_DETACHED,  /*!< USB is not connected to the host. */
    USB_SERIAL_EVENT_ATTACHED,  /*!< USB is connected to the host, but CDC ACM is not in use. */
    USB_SERIAL_EVENT_OPENED,    /*!< USB serial port is opened. */
    USB_SERIAL_EVENT_CLOSED,    /*!< USB serial port is closed. */
} usb_serial_event_t;

typedef void (*usb_serial_event_handler_t)(usb_serial_event_t event);

typedef enum
{
    USB_SERIAL_DTR_STATE_OFF      = 0,  // OFF/ON states match Segger driver's use of USB_CDC_CONTROL_LINE_STATE::DTR
    USB_SERIAL_DTR_STATE_ON       = 1,
    USB_SERIAL_DTR_STATE_DETACHED = 2,  // Did not receive DTR=OFF from host before connection loss
} usb_serial_dtr_state_t;

/**
 * @brief USB CDC transport implementation with integrated packetization.
 *
 * The comms_usb_serial class implements comms_transport_interface to provide
 * a transport layer over a USB CDC (serial) connection, and inherits from
 * comms_packetizer to handle packet assembly and parsing. It enables sending
 * and receiving application-level packets over USB serial, and reports USB CDC
 * connection state changes (e.g., OPENED, CLOSED, SUSPENDED) via a user-defined
 * event callback. This class abstracts low-level USB CDC operations while
 * providing a consistent interface for higher-level communication code.
 */
class comms_usb_serial : public comms_packetizer, private comms_transport_interface
{
    public:
        void init(const comms_packetizer_packet_ready_handler packet_ready_handler, const usb_serial_event_handler_t ev_callback);
        usb_serial_state_t get_state(void);
        ret_code_t add_tasks_to_event_queue(event_queue_manager *event_queue);

    private:
        // Static functions
        static void static_usb_cdc_on_write_cplt(USB_ASYNC_IO_CONTEXT * pIOContext);
        static void static_usb_cdc_on_read_cplt(USB_ASYNC_IO_CONTEXT * pIOContext);
        static void static_usb_on_state_change(void * pContext, uint8_t new_state);
        static void static_usb_cdc_on_line_state_change(USB_CDC_CONTROL_LINE_STATE * state, const void *pContext);
        static void static_on_reconnect_timeout(void * pContext);

        // Private functions
        void determine_usb_state(uint8_t usb_state_mask);
        bool event_queue_state_change_pending();
        void event_queue_state_change_process();
        CREATE_STATIC_WRAPPER(comms_usb_serial, event_queue_state_change_pending, bool);
        CREATE_STATIC_WRAPPER(comms_usb_serial, event_queue_state_change_process, void);

        //Transport layer functions
        virtual bool is_connected();
        virtual ret_code_t comms_transport_tx(uint8_t *p_tx_buffer, size_t tx_length, bool expedite_tx);
        virtual ret_code_t comms_transport_rx(uint8_t *p_rx_buffer, size_t max_rx_length);

        // Private variables
        volatile usb_serial_state_t usb_current_state;
        volatile usb_serial_state_t usb_target_state;
        volatile usb_serial_dtr_state_t usb_cdc_dtr_state;
        uint8_t reconnect_timeout_channel;
        
        usb_serial_event_handler_t usb_cdc_event_handler;
        
        USB_CDC_HANDLE       usb_cdc_handle;
        USB_HOOK             usb_state_hook;
        uint8_t              usb_cdc_raw_buffer[USB_SERIAL_RX_BUFFER_SIZE];

        // Transport layer variables
        USB_ASYNC_IO_CONTEXT        read_io_ctx;
        USB_ASYNC_IO_CONTEXT        write_io_ctx;
};

#ifdef __cplusplus
}
#endif
