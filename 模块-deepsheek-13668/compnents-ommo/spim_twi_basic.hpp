/** ------------------------------------------------------------------------ **
 *
 * @file spim_twi_basic.hpp
 * @date 05/26/2025
 *
 * @brief SPIM and TWI basic mutable communication device class.
 *        Uses spim_basic and twi_basic as underlying communication devices.
 *        Due to HW limitations, only one of the two can be acquired at a time
 *        as these two devices share the same peripheral.
 *
 * @note The pins are not acquired in direct mode, so the user must ensure
 *       that the pins are not used by other devices.
 ** ------------------------------------------------------------------------ **/
#pragma once

/* Includes ----------------------------------------------------------------- */
#include "comm_device.hpp"
#include "spim_basic.hpp"
#include "twi_basic.hpp"

/* Defines ------------------------------------------------------------------ */
/* Macros ------------------------------------------------------------------- */
/* Enums -------------------------------------------------------------------- */
/* Types -------------------------------------------------------------------- */

/** @brief SPIM TWI basic modes of operation. */
typedef enum
{
    SWIMW_ACQUIRE_STATE_NONE, /*!< No device acquired. */
    SWIMW_ACQUIRE_STATE_TWI, /*!< TWI mode acquired. */
    SWIMW_ACQUIRE_STATE_SPI /*!< SPI mode acquired. */
} spim_twi_basic_mode_t;

/* Shared variables --------------------------------------------------------- */
/* Shared functions --------------------------------------------------------- */

class spim_twi_basic: public comm_device
{
  public:
    explicit spim_twi_basic(const nrfx_spim_t * spim_instance, const nrfx_twim_t * twi_instance)
    : spi_basic_instance(spim_instance)
    , twi_basic_instance(twi_instance)
    {
    }

    ~spim_twi_basic() override
    {
        if (active_device)
        {
            active_device->release();
            active_device = nullptr;
            acquire_state = SWIMW_ACQUIRE_STATE_NONE;
        }
    }

    [[nodiscard]] spim_twi_basic_mode_t get_acquire_state() const
    {
        return acquire_state;
    }

    ret_code_t init()
    {
        ret_code_t ret = NRF_ERROR_INVALID_STATE;
        if (acquire_state == SWIMW_ACQUIRE_STATE_NONE)
        {
            ret = twi_basic_instance.init();
        }
        return ret;
    }
    
    ret_code_t acquire_spim(const nrfx_spim_config_t * p_cfg = nullptr, bool direct = false)
    {
        ret_code_t ret = NRF_ERROR_INVALID_STATE;
        switch (acquire_state)
        {
            case SWIMW_ACQUIRE_STATE_SPI:
                ret = spi_basic_instance.reconfigure(p_cfg, direct);
                break;

            case SWIMW_ACQUIRE_STATE_TWI:
                acquire_state = SWIMW_ACQUIRE_STATE_NONE;
                active_device = nullptr;
                (void) twi_basic_instance.release();
                [[fallthrough]];

            case SWIMW_ACQUIRE_STATE_NONE:
                ret = spi_basic_instance.acquire(p_cfg, direct);
                if (ret == NRF_SUCCESS)
                {
                    active_device = &spi_basic_instance;
                    acquire_state = SWIMW_ACQUIRE_STATE_SPI;
                }
                break;

            default:
                break;
        }

        return ret;
    }

    ret_code_t acquire_twi(const twi_basic_config_t * p_cfg = nullptr, bool direct = false)
    {
        ret_code_t ret = NRF_ERROR_INVALID_STATE;
        switch (acquire_state)
        {
            case SWIMW_ACQUIRE_STATE_TWI:
                ret = twi_basic_instance.reconfigure(p_cfg, direct);
                break;

            case SWIMW_ACQUIRE_STATE_SPI:
                acquire_state = SWIMW_ACQUIRE_STATE_NONE;
                active_device = nullptr;
                (void) spi_basic_instance.release();
                [[fallthrough]];

            case SWIMW_ACQUIRE_STATE_NONE:
                ret = twi_basic_instance.acquire(p_cfg, direct);
                if (ret == NRF_SUCCESS)
                {
                    active_device = &twi_basic_instance;
                    acquire_state = SWIMW_ACQUIRE_STATE_TWI;
                }
                break;

            default:
                break;
        }
        return ret;
    }

    [[nodiscard]] uint32_t get_start_task() const
    {
        switch (acquire_state)
        {
            case SWIMW_ACQUIRE_STATE_SPI:
                return spi_basic_instance.get_start_task();

            default:
                return (uint32_t) NULL;
        }

        //Unreachable
    }

    [[nodiscard]] bool is_acquired() const override
    {
        return (acquire_state != SWIMW_ACQUIRE_STATE_NONE);
    }

    [[nodiscard]] bool is_trx_completed() const override
    {
        if (active_device)
        {
            return active_device->is_trx_completed();
        }
        return true;
    }

    ret_code_t release() override
    {
        if (active_device)
        {
            ret_code_t result = active_device->release();
            active_device = nullptr;
            acquire_state = SWIMW_ACQUIRE_STATE_NONE;
            return result;
        }
        return NRF_ERROR_INVALID_STATE;
    }

    ret_code_t trx(const uint8_t *p_tx_buffer, size_t tx_length, uint8_t *p_rx_buffer,
                   size_t rx_length, comm_device_trx_flags_t flags = COMM_DEVICE_FLAGS_NONE) override
    {
        if (active_device)
        {
            return active_device->trx(p_tx_buffer, tx_length, p_rx_buffer, rx_length, flags);
        }
        return NRF_ERROR_INVALID_STATE;
    }

    ret_code_t trx(const uint8_t *p_tx_buffer, size_t tx_length, uint8_t *p_rx_buffer,
                   size_t rx_length, comm_device_event_handler_t handler, void *p_context = nullptr,
                   comm_device_trx_flags_t flags = COMM_DEVICE_FLAGS_NONE) override
    {
        if (active_device)
        {
            return active_device->trx(p_tx_buffer, tx_length, p_rx_buffer, rx_length, handler, p_context, flags);
        }
        return NRF_ERROR_INVALID_STATE;
    }

  protected:
    spim_basic spi_basic_instance;
    twi_basic twi_basic_instance;

    comm_device* active_device = nullptr;
    volatile spim_twi_basic_mode_t acquire_state = SWIMW_ACQUIRE_STATE_NONE;
};

