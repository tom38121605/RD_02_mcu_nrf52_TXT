/*! ----------------------------------------------------------------------------
 * @file    deca_spi.h
 * @brief   SPI access functions
 *
 * @attention
 *
 * Copyright 2015-2019 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */

#ifndef _PORT_SPI_H_
#define _PORT_SPI_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <deca_types.h>
#include <nrfx_spim.h>
#include "port.h"

/*! ----------------------------------------------------------------------------
 * @fn port_spi_set_slowrate
 * @brief Set SPI clock to slow rate (<3MHz)
 * 
 * Set SPI clock to a slow (<3 MHz) rate allowing SPI communication before
 * CLKPLL is locked.
 */
void port_spi_set_slowrate(void);

/*! ----------------------------------------------------------------------------
 * @fn    port_set_dw1000_fastrate
 * @brief Set SPI clock to fast rate (<20MHz)
 * 
 * Set SPI clock to a fast (<20 MHz) rate allowing faster SPI communication when
 * CLKPLL is locked, making higher data throughput possible.
 */
void port_spi_set_fastrate(void);

/*! ----------------------------------------------------------------------------
 * @fn port_spi_write()
 * @brief Write data to DW1000 over SPI bus
 *
 * Low level abstract function to write to the SPI
 * Takes two separate byte buffers for write header and write data
 * 
 * \return 0 for success or -1 if an error occured
 */
int port_spi_write(uint16      headerLength,
                   const uint8 *headerBuffer,
                   uint16      bodyLength,
                   const uint8 *bodyBuffer);

/**---------------------------------------
 * Function: writetospiwithcrc()
 *
 * Low level abstract function to write to the SPI when SPI CRC mode is used
 * header + data + CRC8 byte
 * returns 0 for success, or -1 for error
 */
int port_spi_writewithcrc(
                uint16_t      headerLength,
                const uint8_t *headerBuffer,
                uint16_t      bodyLength,
                const uint8_t *bodyBuffer,
                uint8_t       crc8);

/*! ----------------------------------------------------------------------------
 * @fn readfromspi()
 * @brief
 * Low level abstract function to read from the SPI
 * Takes two separate byte buffers for write header and read data.
 *
 * \return The offset in read buffer where first byte of read data may be found
 * when read succeeded or -1 if an error occured
 */
int port_spi_read(uint16      headerLength,
                  const uint8 *headerBuffer,
                  uint16      readLength,
                  uint8       *readBuffer);

#ifdef __cplusplus
}
#endif

#endif /* _PORT_SPI_H_ */
