/*! ----------------------------------------------------------------------------
 * @file    deca_spi.c
 * @brief   Platform specific SPI functions used by deca_driver.
 *
 * Platform specific SPI functions used by deca_driver, defined in
 * deca_driver/deca_device_api.h
 *
 * @attention
 *
 * Copyright 2020 (c) DecaWave Ltd, Dublin, Ireland.
 * All rights reserved.
 *
 * @author DecaWave
 */

#include "port_spi.h"
#include "deca_device_api.h"

inline int writetospi(uint16 headerLength,
                      const uint8 *headerBuffer,
                      uint16 bodylength,
                      const uint8 *bodyBuffer)
{
    return port_spi_write(headerLength, headerBuffer, bodylength, bodyBuffer);
}

inline int readfromspi(uint16 headerLength,
                       /*const*/ uint8 *headerBuffer,
                       uint16 readlength,
                       uint8 *readBuffer)
{
    return port_spi_read(headerLength, headerBuffer, readlength, readBuffer);
}

int writetospiwithcrc(
                uint16_t      headerLength,
                const uint8_t *headerBuffer,
                uint16_t      bodyLength,
                const uint8_t *bodyBuffer,
                uint8_t       crc8)
{
    return port_spi_writewithcrc(headerLength, headerBuffer, bodyLength, bodyBuffer, crc8);
} // end writetospiwithcrc()


