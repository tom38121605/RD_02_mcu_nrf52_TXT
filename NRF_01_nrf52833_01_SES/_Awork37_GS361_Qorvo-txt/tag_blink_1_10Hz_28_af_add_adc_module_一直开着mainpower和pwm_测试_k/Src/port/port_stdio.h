/*! ----------------------------------------------------------------------------
 * @file      port_stdio.h
 *
 * @brief     HW specific functions for standard IO interface
 *
 * @author    DecaWave
 *
 * @attention Copyright 2017-2019 (c) DecaWave Ltd, Dublin, Ireland.
 *            All rights reserved.
 */

#ifndef _PORT_STDIO_H_
#define _PORT_STDIO_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "nrf_uart.h"
#include "app_uart.h"
#include "port.h"

#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */

extern int debug_en;

app_uart_event_handler_t uart_event_handler(app_uart_evt_t * p_event);

/*! ----------------------------------------------------------------------------
 * @fn port_stdio_write
 * @brief Write/transmit data to standard output
 *
 * @param[in] data Pointer to data buffer
 * @param[in] len Length of data to be written
 * @return Number of bytes written or -1 if an error occurred
 */
int port_stdio_write(const uint8_t *data, int len);

#define port_stdio_transmit port_stdio_write

#define port_dbg_print(data,len) (debug_en ? port_stdio_transmit(data,len): 0)
/*! ----------------------------------------------------------------------------
 * @fn port_stdio_putc
 * @brief Write/transmit a single character/byte non-blocking
 *
 * This function should not block while the byte is transmitted but can block if
 * a previous transmit operation is still ongoing.
 *
 * @param[in] c Character/byte to transmit
 * return 0 on success or -1 if an error occurred
 */
int port_stdio_putc(uint8_t c);

/*! ----------------------------------------------------------------------------
 * @fn port_stdio_read
 * @brief Receive/read a character/byte of data from standard input
 *
 * @param data_handler Callback to pass received byte to
 * \return The amount of bytes received or -1 if an error occurred
 */
int port_stdio_read(void (*data_handler)(uint8_t data));
#define port_stdio_receive port_stdio_read

#ifdef __cplusplus
}
#endif

#endif /* _PORT_STDIO_H_ */
