#include "spim_basic.hpp"
#include "port_master.hpp"
#include "trace.h"
#include "utils.hpp"
#include "event_queue_manager.hpp"

void spim_basic::spim_handler(const nrfx_spim_evt_t * p_event, void * p_context)
{
    UNUSED_PARAMETER(p_event);
    spim_basic * p_this  = static_cast<spim_basic *>(p_context);
    OMMO_TRACE_ISR_ENTER_CALLBACK(TRACE_ISR_SPI_BASIC, p_this->evt_handler);
    p_this->trx_complete = true;
    if (!p_this->is_repeated)
        nrf_spim_disable(p_this->spim_instance->p_reg);

    // Copy the received data to the provided buffer
    if (p_this->acq == SPIMB_ACQ_NORMAL)
    {
        // Copy rx data into output buffer (skipping rx during tx)
        memcpy(p_this->evt_rx_buffer, p_this->io_buffer + p_event->xfer_desc.tx_length,
               p_event->xfer_desc.rx_length - p_event->xfer_desc.tx_length);
    }

    // Fire event handle if specified
    comm_device_event_handler_t handler = p_this->evt_handler;
    if (handler)
        handler(p_this, COMM_DEVICE_TRX_RESULT_SUCCESS, p_this->evt_context);

    OMMO_TRACE_ISR_EXIT_CALLBACK(TRACE_ISR_SPI_BASIC, handler);
}

uint32_t spim_basic::cast_flags_to_nrf(const comm_device_trx_flags_t flags)
{
    uint32_t nrf_flags = 0;

    if (flags & COMM_DEVICE_FLAGS_HOLD)
    {
        nrf_flags |= NRFX_SPIM_FLAG_HOLD_XFER;
    }
    if (flags & COMM_DEVICE_FLAGS_REPEATED)
    {
        nrf_flags |= NRFX_SPIM_FLAG_REPEATED_XFER;
    }

    return nrf_flags;
}

uint64_t spim_basic::get_pin_mask(const nrfx_spim_config_t * p_cfg, bool direct)
{
    uint64_t pin_mask = 0;

    if (p_cfg->miso_pin != NRFX_SPIM_PIN_NOT_USED)
    {
        pin_mask |= PORT_PIN_TO_MASK(p_cfg->miso_pin);
    }

    if (p_cfg->mosi_pin != NRFX_SPIM_PIN_NOT_USED)
    {
        pin_mask |= PORT_PIN_TO_MASK(p_cfg->mosi_pin);
    }

    if (!direct && p_cfg->ss_pin != NRFX_SPIM_PIN_NOT_USED)
    {
        pin_mask |= PORT_PIN_TO_MASK(p_cfg->ss_pin);
    }

    pin_mask |= PORT_PIN_TO_MASK(p_cfg->sck_pin);
    return pin_mask;
}

spim_basic::spim_basic(const nrfx_spim_t * p_instance, const nrfx_spim_config_t * p_cfg)
: spim_instance(p_instance)
, spim_config(p_cfg ? *p_cfg : (nrfx_spim_config_t)NRFX_SPIM_DEFAULT_CONFIG)
{
}

spim_basic::~spim_basic()
{
    (void) spim_basic::release();
}

ret_code_t spim_basic::reconfigure(const nrfx_spim_config_t * p_cfg, bool direct)
{
    // Uninitialize the SPIM instance and release pins if already acquired
    if (acq != SPIMB_ACQ_NONE)
    {
        port_master_release_pins(get_pin_mask(&spim_config, acq == SPIMB_ACQ_DIRECT));
        nrfx_spim_uninit(spim_instance);
    }

    const nrfx_spim_config_t * spim_cfg = p_cfg ? p_cfg : &spim_config;
    if (port_master_acquire_pins(get_pin_mask(spim_cfg, direct)) != NRF_SUCCESS)
    {
        return NRF_ERROR_NOT_FOUND;
    }

    // Сannot be not used(NRFX_SPIM_PIN_NOT_USED), or spi will not function correctly
    if (spim_cfg->sck_pin == NRFX_SPIM_PIN_NOT_USED)
    {
        return NRF_ERROR_INVALID_PARAM; // Ensure SCK pin is connected
    }

    const ret_code_t ret = nrfx_spim_init(spim_instance, spim_cfg, spim_handler, this);
    if (ret == NRF_SUCCESS)
    {
        // Disable the SPIM instance to prevent unintended transfers
        nrf_spim_disable(spim_instance->p_reg);

        // Copy the configuration to the internal buffer
        if (p_cfg)
        {
            spim_config = *p_cfg;
        }

        port_master_configure_high_speed_output(spim_config.sck_pin);
        port_master_configure_high_speed_output(spim_config.mosi_pin);
        nrf_gpio_pin_set(spim_config.mosi_pin); //Default to high while not in use
        acq = direct ? SPIMB_ACQ_DIRECT : SPIMB_ACQ_NORMAL;
    }
    else
    {
        acq = SPIMB_ACQ_NONE;
    }
    return ret;
}


ret_code_t spim_basic::acquire(const nrfx_spim_config_t * p_cfg, bool direct)
{
    if (acq != SPIMB_ACQ_NONE)
    {
        return NRF_ERROR_BUSY;
    }

    return reconfigure(p_cfg, direct);
}

[[nodiscard]] uint32_t spim_basic::get_start_task() const
{
    return nrfx_spim_start_task_get(spim_instance);
}

void spim_basic::trigger_start_task()
{
    nrf_spim_task_trigger(spim_instance->p_reg, NRF_SPIM_TASK_START);
}

void spim_basic::clear_repeated_transfer()
{
    NRFX_ASSERT(is_repeated);
    nrfx_spim_abort(spim_instance);
    is_repeated = false;
    nrf_spim_disable(spim_instance->p_reg);
}

ret_code_t spim_basic::release()
{
    if (acq == SPIMB_ACQ_NONE)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    nrfx_spim_uninit(spim_instance);
    port_master_release_pins(get_pin_mask(&spim_config, acq == SPIMB_ACQ_DIRECT));
    acq = SPIMB_ACQ_NONE;
    return NRF_SUCCESS;
}

ret_code_t spim_basic::trx(const uint8_t * p_tx_buffer, const size_t tx_length, uint8_t * p_rx_buffer, const size_t rx_length, const comm_device_trx_flags_t flags)
{
    evt_handler = nullptr;
    trx_complete = false;
    is_repeated = !!(flags & COMM_DEVICE_FLAGS_REPEATED);

    switch (acq)
    {
        case SPIMB_ACQ_NORMAL:
        {
            if (rx_length + tx_length > SPIM_MAX_XFER_SIZE)
            {
                return NRF_ERROR_INVALID_LENGTH;
            }

            //Save output buffer to copy response to after SPI TRX
            evt_rx_buffer = p_rx_buffer;

            // Setup transfer
            nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_TRX(p_tx_buffer, tx_length, io_buffer, tx_length + rx_length);
            nrf_spim_enable(spim_instance->p_reg);
            const ret_code_t error_code = nrfx_spim_xfer(spim_instance, &xfer_desc, cast_flags_to_nrf(flags));
            while (error_code == NRF_SUCCESS && !trx_complete)
            {
                main_event_queue.execute_once_with_sleep();
            }

            return error_code;
        }

        case SPIMB_ACQ_DIRECT:
        {
            const nrfx_spim_xfer_desc_t xfer = NRFX_SPIM_XFER_TRX(p_tx_buffer, tx_length, p_rx_buffer, rx_length);
            nrf_spim_enable(spim_instance->p_reg);
            const ret_code_t error_code = nrfx_spim_xfer(spim_instance, &xfer, cast_flags_to_nrf(flags));
            while (error_code == NRF_SUCCESS && !trx_complete)
            {
                main_event_queue.execute_once_with_sleep();
            }

            return error_code;
        }

        default:
            return NRF_ERROR_INVALID_STATE;
    }
}

ret_code_t spim_basic::trx(const uint8_t * p_tx_buffer, const size_t tx_length, uint8_t * p_rx_buffer, const size_t rx_length,
                           const comm_device_event_handler_t handler, void * p_context, const comm_device_trx_flags_t flags)
{
    evt_handler = handler;
    evt_context = p_context;
    trx_complete = false;
    is_repeated = !!(flags & COMM_DEVICE_FLAGS_REPEATED);

    switch (acq)
    {
        case SPIMB_ACQ_NORMAL:
        {
            if (rx_length + tx_length > SPIM_MAX_XFER_SIZE)
            {
                return NRF_ERROR_INVALID_LENGTH;
            }

            //Save output buffer to copy response to after SPI TRX
            evt_rx_buffer = p_rx_buffer;

            // Setup transfer
            nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_TRX(p_tx_buffer, tx_length, io_buffer, tx_length + rx_length);
            nrf_spim_enable(spim_instance->p_reg);
            return nrfx_spim_xfer(spim_instance, &xfer_desc, cast_flags_to_nrf(flags));
        }

        case SPIMB_ACQ_DIRECT:
        {
            const nrfx_spim_xfer_desc_t xfer = NRFX_SPIM_XFER_TRX(p_tx_buffer, tx_length, p_rx_buffer, rx_length);
            nrf_spim_enable(spim_instance->p_reg);
            return nrfx_spim_xfer(spim_instance, &xfer, cast_flags_to_nrf(flags));
        }

        default:
            return NRF_ERROR_INVALID_STATE;
    }
}

[[nodiscard]] bool spim_basic::is_acquired() const
{
    return acq != SPIMB_ACQ_NONE;
}

[[nodiscard]] bool spim_basic::is_trx_completed() const
{
    return trx_complete;
}
