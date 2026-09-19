#include "uarte_basic.hpp"
#include "port_master.hpp"
#include "timestamp_timer.hpp"
#include "sdk_macros.h"

#include "nrfx_uarte.h"


/* ---- NRFX ISR Callback --------------------------------------------------- */

void uarte_basic::static_uarte_basic_handler(nrfx_uarte_event_t const * p_event, void * p_context)
{
    uarte_basic * self = static_cast<uarte_basic*>(p_context);

    switch (p_event->type)
    {
        case NRFX_UARTE_EVT_TX_DONE:
            self->start_next_step();       // chain to RX or DONE
            break;

        case NRFX_UARTE_EVT_RX_DONE:
            self->start_next_step();       // after RX we finish
            break;

        case NRFX_UARTE_EVT_ERROR:
        default:
            self->step = UARTEB_STEP_IDLE;
            if (self->evt_handler) self->evt_handler(self, COMM_DEVICE_TRX_RESULT_ERROR, self->evt_context);
            break;
    }
}

void uarte_basic::static_timeout_handler(void * p_context)
{
    nrfx_uarte_rx_abort(((uarte_basic*)p_context)->uarte_instance);
}

/* ---- Static helpers ----------------------------------------------------- */

uint64_t uarte_basic::get_pin_mask(const nrfx_uarte_config_t * p_cfg)
{
    uint64_t mask = 0;
    if (!p_cfg) return 0;
    if (p_cfg->pseltxd != NRF_UARTE_PSEL_DISCONNECTED) mask |= PORT_PIN_TO_MASK(p_cfg->pseltxd);
    if (p_cfg->pselrxd != NRF_UARTE_PSEL_DISCONNECTED) mask |= PORT_PIN_TO_MASK(p_cfg->pselrxd);
    return mask;
}

/* ---- Lifecycle ---------------------------------------------------------- */

uarte_basic::uarte_basic(const nrfx_uarte_t * p_instance, const nrfx_uarte_config_t * p_cfg)
: uarte_instance(p_instance)
, uarte_config(p_cfg ? *p_cfg : (nrfx_uarte_config_t)NRFX_UARTE_DEFAULT_CONFIG)
{
}

uarte_basic::~uarte_basic()
{
    (void) release();
}

uint32_t uarte_basic::get_startrx_task() const
{
    VERIFY_TRUE(acq != UARTEB_ACQ_NONE, NRF_ERROR_INVALID_STATE);

    return nrfx_uarte_task_address_get(uarte_instance, NRF_UARTE_TASK_STARTRX);
}

uint32_t uarte_basic::get_starttx_task() const
{
    VERIFY_TRUE(acq != UARTEB_ACQ_NONE, NRF_ERROR_INVALID_STATE);

    return nrfx_uarte_task_address_get(uarte_instance, NRF_UARTE_TASK_STARTTX);
}

uint32_t uarte_basic::get_endrx_event() const
{
    VERIFY_TRUE(acq != UARTEB_ACQ_NONE, NRF_ERROR_INVALID_STATE);

    return nrfx_uarte_event_address_get(uarte_instance, NRF_UARTE_EVENT_ENDRX);
}

uint32_t uarte_basic::get_endtx_event() const
{
    VERIFY_TRUE(acq != UARTEB_ACQ_NONE, NRF_ERROR_INVALID_STATE);

    return nrfx_uarte_event_address_get(uarte_instance, NRF_UARTE_EVENT_ENDTX);
}

uint32_t uarte_basic::rx_abort() const
{
    VERIFY_TRUE(acq != UARTEB_ACQ_NONE, NRF_ERROR_INVALID_STATE);

    nrfx_uarte_rx_abort(uarte_instance);
    return NRF_SUCCESS;
}

uint32_t uarte_basic::tx_abort() const
{
    VERIFY_TRUE(acq != UARTEB_ACQ_NONE, NRF_ERROR_INVALID_STATE);

    nrfx_uarte_tx_abort(uarte_instance);
    return NRF_SUCCESS;
}

bool uarte_basic::rx_in_progress() const
{
    VERIFY_TRUE(acq != UARTEB_ACQ_NONE, NRF_ERROR_INVALID_STATE);

    return nrfx_uarte_rx_in_progress(uarte_instance);
}

bool uarte_basic::tx_in_progress() const
{
    VERIFY_TRUE(acq != UARTEB_ACQ_NONE, NRF_ERROR_INVALID_STATE);

    return nrfx_uarte_tx_in_progress(uarte_instance);
}

void uarte_basic::uarte_rx_flush_fifo() const
{
    /* The FIFO may contain some data, we need to flush it before initiating the RX. */
    if (nrf_uarte_event_check(uarte_instance->p_reg, NRF_UARTE_EVENT_RXDRDY))
    {
        // Per nRF PS, set RXD.MAXCNT > 4
        uint8_t empty_buff[8];
        nrf_uarte_rx_buffer_set(uarte_instance->p_reg, empty_buff, sizeof(empty_buff));

        /* Clear any pending events before flushing. */
        nrf_uarte_event_clear(uarte_instance->p_reg, NRF_UARTE_EVENT_RXTO);
        nrf_uarte_event_clear(uarte_instance->p_reg, NRF_UARTE_EVENT_ENDRX);

        nrf_uarte_task_trigger(uarte_instance->p_reg, NRF_UARTE_TASK_FLUSHRX);

        // Wait for the RX to be flushed (RX timeout or RX end).
        // Per nRF PS: "The UARTE will generate the ENDRX event after completing the FLUSHRX
        // task even if the RX FIFO was empty or if the RX buffer does not get filled up"
        // Evidence strongly suggests that is incorrect.
        // Per nRF PS: "The UARTE is able to receive up to four bytes after the STOPRX task
        // has been triggered"
        // Assume at most 3 loop iterations per microsecond, 10us/byte at 1Mbps
        // 10 us/byte * 4 bytes * 3 iterations/us = 120 iterations minimum (150 to be conservative)
        uint32_t iter_count = 0;
        while (!nrf_uarte_event_check(uarte_instance->p_reg, NRF_UARTE_EVENT_RXTO) &&
               !nrf_uarte_event_check(uarte_instance->p_reg, NRF_UARTE_EVENT_ENDRX) &&
               iter_count < 150)
        {
            iter_count++;
        }

        /* Clear the RXDRDY event after RX has been flushed. */
        nrf_uarte_event_clear(uarte_instance->p_reg, NRF_UARTE_EVENT_RXDRDY);
    }

    /* Error may occur during the GPIO configuration. */
    nrf_uarte_event_clear(uarte_instance->p_reg, NRF_UARTE_EVENT_ERROR);
}

void uarte_basic::setup_pins(uint32_t pseltxd, uint32_t pselrxd)
{
    //Set the new TX pin to strong push pull
    if (uarte_config.pseltxd != NRF_UARTE_PSEL_DISCONNECTED)
    {
        nrf_gpio_pin_write(uarte_config.pseltxd, 1);
        port_master_configure_high_speed_output(uarte_config.pseltxd);
    }

    //Set rx pin to input
    if (uarte_config.pselrxd != NRF_UARTE_PSEL_DISCONNECTED)
    {
        nrf_gpio_cfg_input(uarte_config.pselrxd, NRF_GPIO_PIN_PULLUP);
    }
}

/* @brief Sets the rx/tx pins if they are available
 */          
uint32_t uarte_basic::set_txrx_pins_set(uint32_t pseltxd, uint32_t pselrxd)
{
    //Release existing pins
    port_master_release_pins(get_pin_mask(&uarte_config));

    //Attempt to acquire new pins
    nrfx_uarte_config_t new_config = uarte_config;
    new_config.pselrxd = pselrxd;
    new_config.pseltxd = pseltxd;
    ret_code_t err_code = port_master_acquire_pins(get_pin_mask(&new_config));
    if(err_code != NRF_SUCCESS)
    {
        port_master_acquire_pins(get_pin_mask(&uarte_config)); //Re-acquire old pins
        return err_code;
    }

    setup_pins(pseltxd, pselrxd);

    //Set underlying pins
    nrf_uarte_txrx_pins_set(uarte_instance->p_reg, pseltxd, pselrxd);

    return NRF_SUCCESS;
}

/* @brief switch_to_tx_only sets the TX_PIN to p_cfg_pselrxd and the RX_PIN to p_cfg.pseltxd
 */          
void uarte_basic::switch_to_tx_only()
{
    if(acq == UARTEB_ACQ_NONE)
        return;

    //Set the future RX pin high so that we don't get a UARTE error
    if (uarte_config.pseltxd != NRF_UARTE_PSEL_DISCONNECTED)
    {
        nrf_gpio_cfg_output(uarte_config.pseltxd);
        nrf_gpio_pin_write(uarte_config.pseltxd, 1);
    }

    //If we use NRF_UARTE_PSEL_DISCONNECTED on RX the UARTE will generate a BREAK error condition
    nrf_uarte_txrx_pins_set(uarte_instance->p_reg, uarte_config.pselrxd, uarte_config.pseltxd);

    // Setup rx pin for tx, strong push pull
    setup_pins(uarte_config.pselrxd, NRF_UARTE_PSEL_DISCONNECTED);
}

/* @brief switch_to_rx_only disables the uarte TX PIN, and set's the uarte RX pin to the pin specified in uarte_config.pselrxd
 */
void uarte_basic::switch_to_rx_only()
{
    if(acq == UARTEB_ACQ_NONE)
        return;

    // Setup rx pin for rx
    setup_pins(NRF_UARTE_PSEL_DISCONNECTED, uarte_config.pselrxd);
    
    nrf_uarte_txrx_pins_set(uarte_instance->p_reg, NRF_UARTE_PSEL_DISCONNECTED, uarte_config.pselrxd);
}

/* @brief switch_to_normal_trx sets the uarte TX PIN to uarte_config.pseltxd
 *        and set's the uarte RX pin to the pin specified in uarte_config.pselrxd
 */
void uarte_basic::switch_to_normal_trx()
{
    if(acq == UARTEB_ACQ_NONE)
        return;

    setup_pins(uarte_config.pseltxd, uarte_config.pselrxd);

    nrf_uarte_txrx_pins_set(uarte_instance->p_reg, uarte_config.pseltxd, uarte_config.pselrxd);
}
  
const nrfx_uarte_t *uarte_basic::get_nrfx_uarte_instance() const
{
    if(acq != UARTEB_ACQ_DIRECT)
        return nullptr;
    else
        return uarte_instance;
}

ret_code_t uarte_basic::reconfigure(const nrfx_uarte_config_t * p_cfg, nrfx_uarte_event_handler_t event_handler, void *p_context)
{
    VERIFY_TRUE(p_cfg != nullptr, NRF_ERROR_INVALID_PARAM);

    // If they specify a low level event handler, then we will be in direct mode and will not process comm_device calls 
    bool direct_mode = (event_handler != nullptr);

    // If active, uninit and release pins first.
    if (acq != UARTEB_ACQ_NONE)
    {
        nrfx_uarte_uninit(uarte_instance);
        port_master_release_pins(get_pin_mask(&uarte_config));
        acq = UARTEB_ACQ_NONE;
    }

    // Save applied config
    uarte_config = *p_cfg;

    // Claim pins via port_master before init.
    ret_code_t err = port_master_acquire_pins(get_pin_mask(&uarte_config));
    if (err != NRF_SUCCESS)
    {
        return err;
    }

    // Init UARTE (with our ISR + context = this || their ISR + their context).
    if(direct_mode)
    {
        event_handler = event_handler;
        uarte_config.p_context = p_context;
    }
    else
    {
        event_handler = static_uarte_basic_handler;
        uarte_config.p_context = this;
    }
    err = nrfx_uarte_init(uarte_instance, &uarte_config, event_handler);
    if (err != NRF_SUCCESS)
    {
        port_master_release_pins(get_pin_mask(&uarte_config));
        return err;
    }

    // Configure state
    step = UARTEB_STEP_IDLE;
    acq = (direct_mode ? UARTEB_ACQ_DIRECT : UARTEB_ACQ_NORMAL);

    // Configure gpio's
    switch_to_normal_trx();
    return NRF_SUCCESS;
}

ret_code_t uarte_basic::acquire(const nrfx_uarte_config_t * p_cfg, nrfx_uarte_event_handler_t  event_handler, void *p_context)
{
    VERIFY_TRUE(acq == UARTEB_ACQ_NONE, NRF_ERROR_BUSY);

    return reconfigure(p_cfg, event_handler, p_context);
}
ret_code_t uarte_basic::release()
{
    VERIFY_TRUE(acq != UARTEB_ACQ_NONE, NRF_ERROR_INVALID_STATE);

    nrfx_uarte_uninit(uarte_instance);
    port_master_release_pins(get_pin_mask(&uarte_config));

    // Set rx pin to reset state(same as power-on reset)
    if(uarte_config.pselrxd != NRF_UARTE_PSEL_DISCONNECTED)
        //nrf_gpio_cfg_default(uarte_config.pselrxd);
        nrf_gpio_cfg_input(uarte_config.pselrxd, NRF_GPIO_PIN_PULLUP);  // HACK leave pull-up enabled
    if(uarte_config.pseltxd != NRF_UARTE_PSEL_DISCONNECTED)
        nrf_gpio_cfg_default(uarte_config.pseltxd);

    if(rtc_timeout_channel != 0xFF)
    {
        rtc_timer_release_timeout_channel(rtc_timeout_channel);
        rtc_timeout_channel == 0xFF;
    }

    acq = UARTEB_ACQ_NONE;
    step = UARTEB_STEP_IDLE;
    return NRF_SUCCESS;
}

bool uarte_basic::is_acquired() const
{
    return acq != UARTEB_ACQ_NONE;
}

bool uarte_basic::is_trx_completed() const
{
    return (step == UARTEB_STEP_IDLE);
}

/* ---- Blocking RX (RX amount varible with timeout) ----------------------- */
ret_code_t uarte_basic::rx_variable(uint8_t *data, uint16_t max_size,
                                         uint16_t *received_size, uint32_t timeout)
{
    VERIFY_TRUE(acq == UARTEB_ACQ_NORMAL, NRF_ERROR_INVALID_STATE);
    VERIFY_PARAM_NOT_NULL(data);

    //Grab an RTC channel
    if(rtc_timeout_channel == 0xFF)
        VERIFY_SUCCESS(rtc_timer_get_timeout_channel(&rtc_timeout_channel, static_timeout_handler, this));

    //Flush teh fifo
    uarte_rx_flush_fifo();

    //Start rx
    step = UARTEB_STEP_VARIABLE_RX;
    APP_ERROR_CHECK(rtc_timer_set_timeout(rtc_timeout_channel, MS_TO_RTC_TIMEOUT_GEN(timeout)));
    APP_ERROR_CHECK(nrfx_uarte_rx(uarte_instance, data, max_size));

    //Wait for timeout/finish
    while (step != UARTEB_STEP_IDLE)
        __WFI();

    //Stop timeout
    APP_ERROR_CHECK(rtc_timer_disable_timeout(rtc_timeout_channel));

    //Return data
    if(received_size)
        *received_size = nrf_uarte_rx_amount_get(uarte_instance->p_reg);

    return NRF_SUCCESS;
}

/* ---- Blocking TRX (TX then optional RX) --------------------------------- */
ret_code_t uarte_basic::trx(const uint8_t *p_tx_buffer, size_t tx_length,
                            uint8_t *p_rx_buffer, size_t rx_length,
                            comm_device_trx_flags_t flags)
{

    VERIFY_TRUE(acq == UARTEB_ACQ_NORMAL, NRF_ERROR_INVALID_STATE);

    evt_handler = nullptr;
    evt_context = nullptr;

    cur_tx     = p_tx_buffer;
    cur_tx_len = tx_length;
    cur_rx     = p_rx_buffer;
    cur_rx_len = rx_length;

    step = UARTEB_STEP_IDLE;
    start_next_step(flags & COMM_DEVICE_FLAGS_HOLD);   // kicks TX or RX or complete immediately

    while (step != UARTEB_STEP_IDLE)
        __WFI();

    return NRF_SUCCESS;
}

/* ---- Async TRX (TX then optional RX) ------------------------------------ */

ret_code_t uarte_basic::trx(const uint8_t * p_tx_buffer, size_t tx_length,
                            uint8_t * p_rx_buffer, size_t rx_length,
                            comm_device_event_handler_t handler, void * p_context,
                            comm_device_trx_flags_t flags)
{
    VERIFY_TRUE(acq == UARTEB_ACQ_NORMAL, NRF_ERROR_INVALID_STATE);

    evt_handler = handler;
    evt_context = p_context;

    cur_tx     = p_tx_buffer;
    cur_tx_len = tx_length;
    cur_rx     = p_rx_buffer;
    cur_rx_len = rx_length;

    step = UARTEB_STEP_IDLE;
    start_next_step(flags & COMM_DEVICE_FLAGS_HOLD);   // kicks TX or RX or complete immediately

    return NRF_SUCCESS;
}

void uarte_basic::start_next_step(bool hold_next_step)
{
    if (step == UARTEB_STEP_IDLE)
    {
        if (cur_tx && cur_tx_len)
        {
            step = UARTEB_STEP_TX;
            if (nrfx_uarte_tx_with_hold(uarte_instance, cur_tx, cur_tx_len, hold_next_step) != NRF_SUCCESS)
            {
                step = UARTEB_STEP_IDLE;
                if (evt_handler) evt_handler(this, COMM_DEVICE_TRX_RESULT_ERROR, evt_context);
            }
            return;
        }
        if (cur_rx && cur_rx_len)
        {
            step = UARTEB_STEP_RX;
            if (nrfx_uarte_rx_with_hold(uarte_instance, cur_rx, cur_rx_len, hold_next_step) != NRF_SUCCESS)
            {
                step = UARTEB_STEP_IDLE;
                if (evt_handler) evt_handler(this, COMM_DEVICE_TRX_RESULT_ERROR, evt_context);
            }
            return;
        }

        step = UARTEB_STEP_IDLE;
        if (evt_handler) evt_handler(this, COMM_DEVICE_TRX_RESULT_SUCCESS, evt_context);
        return;
    }

    if (step == UARTEB_STEP_TX)
    {
        if (cur_rx && cur_rx_len)
        {
            step = UARTEB_STEP_RX;
            if (nrfx_uarte_rx_with_hold(uarte_instance, cur_rx, cur_rx_len, hold_next_step) != NRF_SUCCESS)
            {
                step = UARTEB_STEP_IDLE;
                if (evt_handler) evt_handler(this, COMM_DEVICE_TRX_RESULT_ERROR, evt_context);
            }
            return;
        }
        step = UARTEB_STEP_IDLE;
        if (evt_handler) evt_handler(this, COMM_DEVICE_TRX_RESULT_SUCCESS, evt_context);
        return;
    }

    if (step == UARTEB_STEP_RX ||
        step == UARTEB_STEP_VARIABLE_RX )
    {
        step = UARTEB_STEP_IDLE;
        if (evt_handler) evt_handler(this, COMM_DEVICE_TRX_RESULT_SUCCESS, evt_context);
        return;
    }
}

