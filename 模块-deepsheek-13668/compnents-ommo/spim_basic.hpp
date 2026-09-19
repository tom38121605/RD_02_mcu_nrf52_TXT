/** ------------------------------------------------------------------------ **
 * @file spim_basic.hpp
 * @date 05/12/2025
 *
 * @brief SPIM basic driver class, providing basic SPI communication.
 *        There are two modes of operation:
 *        - Normal mode, where the internal buffer is used for reception and CS
 *          pin is additionally used for acquisition.
 *        - Direct mode, where the provided buffer is directly used for
 *          transmission and reception, the CS pin is not used for acquisition
 *          and user should ensure its availability.
 *
 *        User may provide a custom event handler and context to be called.
 *        Class overrides the comm_device interface and provides transaction methods.
 *
 ** ------------------------------------------------------------------------ **/
#pragma once

/* Includes ----------------------------------------------------------------- */

#include <stdint.h>
#include "ommo_config.h"
#include "nrfx_spim.h"
#include "comm_device.hpp"

/* Defines ------------------------------------------------------------------ */
#ifndef SPIM_MAX_XFER_SIZE
#define SPIM_MAX_XFER_SIZE 64 /*!< Maximum size of the IO transmit buffer. */
#endif

/* Macros ------------------------------------------------------------------- */
/* Enums -------------------------------------------------------------------- */
/* Types -------------------------------------------------------------------- */
/* Classes ------------------------------------------------------------------ */

class spim_basic : public comm_device
{
  public:
    /**
     * @param p_instance Pointer to the SPIM instance, must be kept alive during the lifetime of this object.
     * @param p_cfg Pointer to the SPIM configuration. If nullptr, default configuration is used.
     */
    explicit spim_basic(const nrfx_spim_t * p_instance, const nrfx_spim_config_t * p_cfg = nullptr);
    ~spim_basic() override;

    /**
     * @brief Configure the SPIM instance with the given configuration. Does not relay on acquire state, may be called at any time.
     *        The CLS and MOSI pins are configured as output with high drive strength.
     *
     * @param p_cfg Pointer to the SPIM configuration. If nullptr, default or previous configuration will be used.
     * @param direct Direct acquisition mode. If true, the SS pin is not used and the DMA uses the provided buffers
     *        directly for transmission and reception.
     */
    ret_code_t reconfigure(const nrfx_spim_config_t * p_cfg = nullptr, bool direct = false);

    /** @brief Wrapper for reconfigure() method, which acquires the SPIM instance before reconfiguration. */
    ret_code_t acquire(const nrfx_spim_config_t * p_cfg = nullptr, bool direct = false);

    /** @brief Returns the start task for the SPIM instance. */
    [[nodiscard]] uint32_t get_start_task() const;

    /** @brief Triggers the START task on the underlying spim instance. Can be used to trigger trx setup with the flag COMM_DEVICE_FLAGS_HOLD */
    void trigger_start_task();

    void clear_repeated_transfer();

    /* Comm device override methods */
    ret_code_t release() override;
    ret_code_t trx(const uint8_t * p_tx_buffer, size_t tx_length, uint8_t *p_rx_buffer, size_t rx_length,
                   comm_device_trx_flags_t flags = COMM_DEVICE_FLAGS_NONE) override;
    ret_code_t trx(const uint8_t *p_tx_buffer, size_t tx_length, uint8_t *p_rx_buffer, size_t rx_length, comm_device_event_handler_t handler,
                   void * p_context = nullptr, comm_device_trx_flags_t flags = COMM_DEVICE_FLAGS_NONE) override;
    [[nodiscard]] bool is_acquired() const override;
    [[nodiscard]] bool is_trx_completed() const override;


  protected:
    volatile bool trx_complete = true; /*!< Indicates if the transaction is complete. */
    bool is_repeated = false;
    volatile enum {
        SPIMB_ACQ_NONE,   /*!< Not acquired. */
        SPIMB_ACQ_NORMAL, /*!< Acquired. The internal buffer is used for reception. */
        SPIMB_ACQ_DIRECT, /*!< Acquired. The used buffer is directly used for transmission and reception. */
    } acq = SPIMB_ACQ_NONE; /*!< Indicates the mode of acquisition. */

  private:
    const nrfx_spim_t *spim_instance;
    nrfx_spim_config_t spim_config;
    uint8_t io_buffer[SPIM_MAX_XFER_SIZE]; /*!< Buffer for the transaction. */

    volatile comm_device_event_handler_t evt_handler = nullptr; // User event handler.
    void *evt_context = nullptr; // User event context, passed as argument.
    void *evt_rx_buffer = nullptr; // User buffer to copy shifted RX data into after SPI RX finished

    /** @brief Static SPIM event handler. */
    static void spim_handler(const nrfx_spim_evt_t * p_event, void * p_context);

    /** @brief Cast our transaction flags to NRFx lib format. */
    static uint32_t cast_flags_to_nrf(comm_device_trx_flags_t flags);

    /**
     * @brief Returns the pin mask for the given SPIM configuration.
     * @note The CS pin is not included in the mask if direct acquisition is used.
     */
    static uint64_t get_pin_mask(const nrfx_spim_config_t * p_cfg, bool direct);
};

/* Shared variables --------------------------------------------------------- */
/* Shared functions --------------------------------------------------------- */