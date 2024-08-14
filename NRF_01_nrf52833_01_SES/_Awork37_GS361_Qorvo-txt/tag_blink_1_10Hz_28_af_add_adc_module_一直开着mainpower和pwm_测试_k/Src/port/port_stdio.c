/**
* @file       port_stdio.c
*
* @brief      HW specific functions for standard IO interface
*
* @attention  Copyright 2018-2019 (c) DecaWave Ltd, Dublin, Ireland.
*             All rights reserved.
*
* @author     Decawave
*
* This file contains target specific implementations of functions used by the
* production test program for reading from the standard input and writing to the
* standard output. This standard I/O can be a UART peripheral, Segger RTT,
* semihosting, an LCD+keyboard,... As long a it can handle sending a
* buffer of data, sending a character without blocking during transmission and
* it can read characters.
*
*/


#include <stdint.h>

#include "port_stdio.h"

void (*handle_data)(uint8_t data) = NULL;
uint8_t static uart_tx_fifo_empty = 0 ;
uint8_t static uart_rx_fifo_rx = 0 ;
uint8_t c ;

int debug_en = 1;

/*! ----------------------------------------------------------------------------
 * @fn uart_event_handler
 * @brief Uart Event Handler
 *
 * This handler is called when the Uart interrupt is raised
 * Control flags are set based on two events:
 * - Uart RX fifo ready (data was received)
 * - Uart Tx fifo empty
 *
 * @param[in] Pointer to the uart event object
 */
app_uart_event_handler_t uart_event_handler(app_uart_evt_t * p_event)
{
    if(p_event->evt_type == APP_UART_DATA_READY)
    {
        uart_rx_fifo_rx = 1 ;
    }
    
    if(p_event->evt_type == APP_UART_TX_EMPTY)
    {
        uart_tx_fifo_empty = 1;
    } 
}

/*! ----------------------------------------------------------------------------
 * @fn port_stdio_write
 * @brief Write/transmit data to standard output
 *
 * This can be blocking or non-blocking.
 * If a non-blocking function is used, care must be taken to ensure the data is
 * copied to a TX buffer to ensure it's kept in memory until the data is
 * written and this function should wait (blocking or non-blocking) for previous
 * transmissions to complete.
 *
 * @param[in] data Pointer to data buffer
 * @param[in] len Length of data to be written
 * @return Number of bytes written or -1 if an error occurred
 */
int port_stdio_write_trunc(const uint8_t *data, int len)
{
    int i ;
    
    for(i=0;i<len;i++)
    {
        app_uart_put(*(data+i)) ;
    }

    while(!uart_tx_fifo_empty)
    {
        __WFE();
    }

    uart_tx_fifo_empty = 0 ;
    return len;
}

#define TRUNC_STR_SIZE      128
inline int port_stdio_write(const uint8_t *data, int len)
{
    int  len_tbd = len;
    int  len_done = 0;

    while(len_tbd > TRUNC_STR_SIZE)
    {
        port_stdio_write_trunc(data+len_done, TRUNC_STR_SIZE);
        len_tbd -= TRUNC_STR_SIZE;
        len_done += TRUNC_STR_SIZE;
    }
    port_stdio_write_trunc(data+len_done, len_tbd);
    len_done += len_tbd;
    
    return len_done;
}

/*! ----------------------------------------------------------------------------
 * @fn port_stdio_putc
 * @brief Write/transmit a single character/byte non-blocking
 *
 * This function should not block while the byte is transmitted but can block if
 * a previous transmit operation is still ongoing.
 *
 * @param[in] c Character/byte to transmit
 * @return 0 on success or -1 if an error occurred
 */
inline int port_stdio_putc(uint8_t c)
{
    uint32_t err_code = app_uart_put(c) ;
    return (err_code==NRF_SUCCESS ? 0 : -1);
}

/*! ----------------------------------------------------------------------------
 * @fn port_stdio_read
 * @brief Receive/read a character/byte of data from standard input
 *
 * This can be implemented blocking or non-blocking. Note that the
 * data_handler function might transmit data using the port_stdio_write
 * function. When using async reception methods care must be taken to ensure
 * transmission can occur while the reception is active.
 *
 * @param data_handler Callback to pass received byte to
 * @return The amount of bytes received or -1 if an error occurred
 */
int port_stdio_read(void (*data_handler)(uint8_t data))
{
    uint8_t rx_stat = 0 ;

    handle_data = data_handler;

    if(uart_rx_fifo_rx)
    {
        uart_rx_fifo_rx = 0;
        while(app_uart_get(&c)==NRF_SUCCESS)
        {
            handle_data(c);
            rx_stat = 1 ;
        }
    }
    return rx_stat;
}
