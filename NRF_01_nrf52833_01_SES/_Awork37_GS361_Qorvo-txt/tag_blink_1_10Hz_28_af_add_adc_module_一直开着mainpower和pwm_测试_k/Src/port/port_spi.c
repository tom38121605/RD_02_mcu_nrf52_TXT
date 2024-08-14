/*! ----------------------------------------------------------------------------
 * @file    port_spi.c
 * @brief   SPI access functions
 *
 * @attention
 *
 * Copyright 2015-2020 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */

#include "deca_device_api.h"
#include "port_spi.h"
#include <string.h>

#define RX_BUFFER_LENGTH 256

extern const nrfx_spim_t spi ;
static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */

typedef enum{
  SPI_UNINIT = 0,
  SPI_SLOW,
  SPI_FAST
} spi_stat_e;
static spi_stat_e spi_stat = SPI_UNINIT; //0=uninit; 1=slow; 2=fast

/*! ----------------------------------------------------------------------------
 * @fn spim_event_handler
 * @brief Spim event handler
 * 
 * This handler is called when the SPIM interrupt is raised
 * SPIM event is raised at the end of succesful spi Xfer
 *
 * @param[in] Pointer to the SPIM event object
 * @param[in] Pointer to the SPIM context object
 */
void spim_event_handler(nrfx_spim_evt_t const * p_event,
                       void *                  p_context)
{
    spi_xfer_done = true;
}

/*! ----------------------------------------------------------------------------
 * @fn port_spi_set_slowrate
 * @brief Set SPI clock to slow rate (<3MHz)
 */
void port_spi_set_slowrate(void)
{
    if(spi_stat == SPI_SLOW)
    {
        return;
    }

    if(spi_stat == SPI_FAST)
    {
        nrfx_spim_uninit(&spi);
        spi_stat = SPI_UNINIT;
    }

    nrfx_spim_config_t spi_config = NRFX_SPIM_DEFAULT_CONFIG;
    spi_config.frequency      = NRF_SPIM_FREQ_1M;
    spi_config.ss_pin         = NRFX_SPIM_PIN_NOT_USED;
    spi_config.miso_pin       = SPI0_CONFIG_MISO_PIN;
    spi_config.mosi_pin       = SPI0_CONFIG_MOSI_PIN;
    spi_config.sck_pin        = SPI0_CONFIG_SCK_PIN;
    spi_config.ss_active_high = false;
    APP_ERROR_CHECK(nrfx_spim_init(&spi, &spi_config, spim_event_handler, NULL));
    spi_stat = SPI_SLOW;
}

/*! ----------------------------------------------------------------------------
 * @fn    port_set_dw1000_fastrate
 * @brief Set SPI clock to fast rate (<20MHz)
 */
void port_spi_set_fastrate(void)
{
    if(spi_stat == SPI_SLOW)
    {
        return;
    }

    if(spi_stat == SPI_FAST)
    {
        nrfx_spim_uninit(&spi);
        spi_stat = SPI_UNINIT;
    }

    nrfx_spim_config_t spi_config = NRFX_SPIM_DEFAULT_CONFIG;
    spi_config.frequency      = NRF_SPIM_FREQ_8M;
    spi_config.ss_pin         = NRFX_SPIM_PIN_NOT_USED;
    spi_config.miso_pin       = SPI0_CONFIG_MISO_PIN;
    spi_config.mosi_pin       = SPI0_CONFIG_MOSI_PIN;
    spi_config.sck_pin        = SPI0_CONFIG_SCK_PIN;
    spi_config.ss_active_high = false;
    APP_ERROR_CHECK(nrfx_spim_init(&spi, &spi_config, spim_event_handler, NULL));
    spi_stat = SPI_FAST;
}

/*! ----------------------------------------------------------------------------
 * @fn port_spi_write()
 * @brief Write data to DW1000 over SPI bus
 *
 * Low level abstract function to write to the SPI
 * Takes two separate byte buffers for write header and write data
 *
 * \return 0 for success or -1 if an error occurred
 */
int port_spi_write(uint16      headerLength,
                   const uint8 *headerBuffer,
                   uint16      bodyLength,
                   const uint8 *bodyBuffer)
{
    nrfx_err_t err_code;
    decaIrqStatus_t  stat;

    static uint8_t  m_tx_buf[RX_BUFFER_LENGTH];             /**< TX buffer. */
    static uint8_t  m_rx_buf[RX_BUFFER_LENGTH];             /**< RX buffer. */
    static uint8_t m_length;// = sizeof(m_tx_buf);       /**< Transfer length. */

    stat = decamutexon();

    memmove(m_tx_buf, headerBuffer, headerLength);
    memmove(m_tx_buf + headerLength , bodyBuffer, bodyLength);
    m_length = headerLength + bodyLength ;

    nrf_gpio_pin_clear(SPI_CS_PIN);
    spi_xfer_done = false;
    nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_TRX(m_tx_buf, m_length, m_rx_buf, m_length);
    
    if((err_code = nrfx_spim_xfer(&spi,&xfer_desc,0 )) != NRFX_SUCCESS)
    {
        decamutexoff(stat);
        return -1;
    }

    while (!spi_xfer_done)
    {
        __WFE();
    }

    nrf_gpio_pin_set(SPI_CS_PIN);
    decamutexoff(stat);
    return 0;
}

/*! ----------------------------------------------------------------------------
 * @fn readfromspi()
 * @brief
 * Low level abstract function to read from the SPI
 * Takes two separate byte buffers for write header and read data.
 *
 * \return The offset in read buffer where first byte of read data may be found
 * when read succeeded or -1 if an error occurred
 */
int port_spi_read(uint16      headerLength,
                  const uint8 *headerBuffer,
                  uint16      readLength,
                  uint8       *readBuffer)
{
    nrfx_err_t err_code;
    decaIrqStatus_t  stat;
    stat = decamutexon();

    static uint8_t  m_tx_buf[RX_BUFFER_LENGTH];             /**< TX buffer. */
    static uint8_t  m_rx_buf[RX_BUFFER_LENGTH];             /**< RX buffer. */
    static uint8_t m_length;// = sizeof(m_tx_buf);       /**< Transfer length. */
     
    memmove(m_tx_buf, headerBuffer, headerLength);
    memset(m_tx_buf + headerLength , 0x00, readLength);
    m_length = headerLength + readLength ;

    nrf_gpio_pin_clear(SPI_CS_PIN);
    spi_xfer_done = false;
    
    nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_TRX(m_tx_buf, m_length, m_rx_buf, m_length);

    if((err_code = nrfx_spim_xfer(&spi, &xfer_desc, 0)) != NRFX_SUCCESS)
    {
        decamutexoff(stat);
        return -1;
    }

    while (!spi_xfer_done)
    {
        __WFE();
    }
    
    memmove(readBuffer, m_rx_buf + headerLength, readLength);

    nrf_gpio_pin_set(SPI_CS_PIN);
    decamutexoff(stat);
    return 0;
}

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
                uint8_t       crc8)
{
    nrfx_err_t err_code;
    decaIrqStatus_t  stat;

    static uint8_t  m_tx_buf[RX_BUFFER_LENGTH];             /**< TX buffer. */
    static uint8_t  m_rx_buf[RX_BUFFER_LENGTH];             /**< RX buffer. */
    static uint8_t m_length;// = sizeof(m_tx_buf);       /**< Transfer length. */

    stat = decamutexon();

    memmove(m_tx_buf, headerBuffer, headerLength);
    memmove(m_tx_buf + headerLength , bodyBuffer, bodyLength);
    m_tx_buf[headerLength + bodyLength] = crc8;
    m_length = headerLength + bodyLength + 1;

    nrf_gpio_pin_clear(SPI_CS_PIN);
    spi_xfer_done = false;
    nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_TRX(m_tx_buf, m_length, m_rx_buf, m_length);
    
    if((err_code = nrfx_spim_xfer(&spi,&xfer_desc,0 )) != NRFX_SUCCESS)
    {
        decamutexoff(stat);
        return -1;
    }

    while (!spi_xfer_done)
    {
        __WFE();
    }

    nrf_gpio_pin_set(SPI_CS_PIN);
    decamutexoff(stat);
    return 0;
}

