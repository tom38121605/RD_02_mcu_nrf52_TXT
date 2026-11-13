#pragma once

#include "comms_usb_serial.hpp"
#include "usbd_main.hpp"

#ifdef __cplusplus
extern "C" {
#endif

extern comms_usb_serial usb_serial;

inline void usb_serial_init(comms_packetizer_packet_ready_handler br_callback, usb_serial_event_handler_t callback)
{
    usb_serial.init(br_callback, callback);
}

inline ret_code_t usb_serial_add_tasks_to_event_queue(event_queue_manager *event_queue)
{
    return usb_serial.add_tasks_to_event_queue(event_queue);
}

inline usb_serial_state_t usb_serial_get_state(void)
{
    return usb_serial.get_state();
}

inline void usb_serial_uninit(void)
{
    usbd_main_uninit();
}

inline ret_code_t usb_serial_enable_flush_timeout()
{
    return usb_serial.enable_flush_timeout();
}

inline void usb_serial_send_ack_blocking(uint8_t return_code, bool block)
{
    usb_serial.send_ack_blocking(return_code, block);
}

inline void usb_serial_send_ack(uint8_t return_code)
{
    usb_serial.send_ack(return_code);
}

inline void usb_serial_send_command_packet_blocking(const uint8_t * buffer, size_t length, bool block)
{
    usb_serial.send_command_packet_blocking(buffer, length, block);
}

inline void usb_serial_send_command_packet(const uint8_t * buffer, size_t length)
{
    usb_serial.send_command_packet(buffer, length);
}

inline ret_code_t usb_serial_write(const uint8_t * buffer, uint16_t length)
{
    return usb_serial.write(buffer, length);
}

inline ret_code_t usb_serial_write_data_packet_vargs(size_t data_length, ...)
{
    va_list args;
    va_start(args, data_length);
    ret_code_t rvalue = usb_serial.write_data_packet_vargslist(data_length, args);
    va_end(args);
    return rvalue;
}

inline ret_code_t usb_serial_write_now(const uint8_t * buffer, uint16_t length, bool block)
{
    return usb_serial.write_now(buffer, length, block);
}

#ifdef __cplusplus
}
#endif
