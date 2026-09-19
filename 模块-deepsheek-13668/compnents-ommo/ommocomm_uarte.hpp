/** ------------------------------------------------------------------------ **
 * @file ommocomm_uarte.hpp
 * @date 05/12/2025
 *
 * @brief Ommocomm UARTE Driver with SPI/TWI abstraction. Defines the `ommocomm_uarte` class,
 *        a communication interface implementing asynchronous UARTE transactions with support
 *        for SPI and I2C emulation over an abstract communication channel.
 *
 *       ## Key Features
 *         - Implements the `comm_device` interface for modular command-based communication.
 *         - Supports three acquisition modes: Direct UART, SPI, and TWI (I2C).
 *         - Provides automatic mode for sensor polling with event callback support.
 *         - Offers SPI and I2C transaction emulation via UARTE as backend transport.
 *
 ** ------------------------------------------------------------------------ **/

#pragma once

/* Includes ----------------------------------------------------------------- */
#include <stdint.h>

#include "nrf_ppi.h"
#include "nrf_timer.h"
#include "nrfx_ppi.h"

#include "comm_device.hpp"
#include "ommocomm_types.h"
#include "dfu_container.h"
#include "uarte_basic.hpp"

/* Defines ------------------------------------------------------------------ */

#ifndef OMMOCOMM_MAX_MOSI_PAYLOAD_LENGTH
#define OMMOCOMM_MAX_MOSI_PAYLOAD_LENGTH 256 /*!< Maximum size of the IO transmit buffer. */
#endif

#define OMMOCOMM_EEPROM_PAGE_SIZE               128
#define OMMOCOMM_MAX_MISSED_SENSOR_DATA_SAMPLES 5
#define OMMOCOMM_REBOOT_TIME                    200

#define OMMOCOMM_NUM_SS_PER_PORT                4
#define OMMOCOMM_NUM_I2C_BUSSES_PER_PORT        1

/* Macros ------------------------------------------------------------------- */
/* Enums -------------------------------------------------------------------- */
/* Types -------------------------------------------------------------------- */

/** @brief Ommocomm UARTE state machine states. */
typedef enum
{
    OMMOCOMM_STATE_IDLE, /*!< Idle state, no transaction in progress. */

    OMMOCOMM_STATE_MOSI_HEADER, /*!< Transmitting MOSI header. */
    OMMOCOMM_STATE_MOSI_HEADER_TO_PAYLOAD_DELAY, /*!< Delay after MOSI header before sending payload. */
    OMMOCOMM_STATE_MOSI_PAYLOAD, /*!< Transmitting MOSI payload. */

    OMMOCOMM_STATE_MISO_HEADER, /*!< Receiving MISO header. */
    OMMOCOMM_STATE_MISO_PAYLOAD, /*!< Receiving MISO payload. */
    OMMOCOMM_STATE_MISO_PAYLOAD_TO_IDLE_DELAY, /*!< Delay after MISO payload before going idle to allow coprocessor to prepare for next command. */

    OMMOCOMM_STATE_AUTO_MODE_TX,
    OMMOCOMM_STATE_AUTO_MODE_RX,
    OMMOCOMM_STATE_AUTO_MODE_STOPPING,
    //OMMOCOMM_STATE_DIRECT_UART_MODE,
    //OMMOCOMM_STATE_BARE_MODE,
} ommocomm_state_t;

/** @brief Ommocomm device types. */
typedef enum
{
    OMMOCOMM_EVENT_SUCCESS,
    OMMOCOMM_EVENT_TIMEOUT,
    OMMOCOMM_EVENT_ERROR
} ommocomm_event_t;

/** @brief Ommocomm acquisition modes. */
typedef enum
{
    OMMOCOMM_ACQMODE_IDLE,
    OMMOCOMM_ACQMODE_DIRECT,
    OMMOCOMM_ACQMODE_TWI,
    OMMOCOMM_ACQMODE_SPI
} ommocomm_acqmode_t;

/** @brief Ommocomm auto mode event handler type. */
typedef void (*ommocomm_auto_evt_handler_t)(ommocomm_event_t event, void *context);

/* Classes ------------------------------------------------------------------ */
class ommocomm_uarte : public comm_device
{
  public:
    ~ommocomm_uarte() override;

    //ret_code_t uart_cfg();
    //const nrfx_uarte_t *bare_get_instance() const;
    //nrf_timer_cc_channel_t bare_get_ts_capture_timer_channel() const;
    //nrf_ppi_channel_t bare_get_ts_capture_ppi_channel() const;
    //uint32_t bare_get_pin() const;

    //ret_code_t direct_uart_begin() override;
    //ret_code_t direct_uart_send(const uint8_t *data, uint16_t size) override;
    //ret_code_t direct_uart_receive(uint8_t *data, uint16_t size, uint16_t *received_size, uint32_t timeout) override;
    //ret_code_t direct_uart_end() override;

    ret_code_t send_command(mosi_header_t mosi_header_g, uint8_t *mosi_payload_g, miso_header_t *miso_header_g, uint8_t *miso_payload, uint8_t miso_payload_length_g);
    ret_code_t spi_trx(uint8_t port_ss_index, const uint8_t* tx_data, uint8_t tx_len, uint8_t *rx_data, uint8_t rx_len);
    ret_code_t i2c_trx(uint8_t i2c_bus_id, const uint8_t* tx_data, uint8_t tx_len, uint8_t *rx_data, uint8_t rx_len);

    ret_code_t automode_enable();
    ret_code_t automode_setup_synch_tx_and_hold(uint8_t sequence, uint8_t *data, uint8_t data_len, ommocomm_auto_evt_handler_t sample_cb, void *context);
    ret_code_t automode_send_stop_tx(ommocomm_auto_evt_handler_t sample_cb, void *context);
    ret_code_t automode_seq_clear(uint8_t seq_num);
    ret_code_t automode_seq_add(uint8_t seq_num, uint8_t *auto_command, uint8_t auto_command_len);

    ret_code_t get_synch_to_spi_tx_delay(uint32_t *delay);
    ret_code_t get_version(uint16_t *fw_version, ommocomm_device_type_t *device_type, uint16_t *device_id);
    ret_code_t get_wai(uint8_t *wai);
    ret_code_t enter_bootloader_mode();
    uint32_t check_fw_and_update(prog_stage_cb_t progress_callback = nullptr, void (*wdt_feed_ptr)() = nullptr);

    ret_code_t init();
    ret_code_t acquire(uarte_basic *uarte, ommocomm_acqmode_t mode, uint32_t uarte_trx_pin_g, uint8_t cs_index_twi_location = 0, uint8_t data_pull_enable_pin_g = 0xFF);
    [[nodiscard]] uint32_t get_start_task() const;

    /* Comm device override methods */
    ret_code_t release() override;
    ret_code_t trx(const uint8_t * p_tx_buffer, size_t tx_length, uint8_t *p_rx_buffer, size_t rx_length,
                   comm_device_trx_flags_t flags = COMM_DEVICE_FLAGS_NONE) override;
    ret_code_t trx(const uint8_t *p_tx_buffer, size_t tx_length, uint8_t *p_rx_buffer, size_t rx_length, comm_device_event_handler_t handler,
                   void * p_context = nullptr, comm_device_trx_flags_t flags = COMM_DEVICE_FLAGS_NONE) override;
    [[nodiscard]] bool is_acquired() const override;
    [[nodiscard]] bool is_trx_completed() const override;

  protected:
    volatile ommocomm_state_t current_state = OMMOCOMM_STATE_IDLE;
    volatile ommocomm_acqmode_t acquired_mode = OMMOCOMM_ACQMODE_IDLE;
    volatile bool uarte_rx_aborted = false;

  private:
    uarte_basic *uarte_instance = nullptr;
    nrfx_uarte_config_t uart_config = NRFX_UARTE_DEFAULT_CONFIG;
    mosi_header_t mosi_header{};
    uint8_t mosi_payload[OMMOCOMM_MAX_MOSI_PAYLOAD_LENGTH];
    miso_header_t *miso_header = nullptr;
    uint8_t *miso_payload = nullptr;
    uint8_t miso_payload_buffer_max_length = 0;
    uint8_t auto_mode_payload_length = 0;

    ommocomm_auto_evt_handler_t auto_evt_handler = nullptr; /*!< Callback for auto mode events. */
    void *auto_evt_ctx = nullptr; /*!< Context for auto mode events. */
    volatile bool auto_seq_executing = false;

    uint8_t acquired_ss_index_bus_location = 0;
    uint8_t rtc_timeout_channel;
    nrfx_atomic_flag_t timeout_executed = 0;

    //Data line pullup enable/disable line
    uint8_t data_pull_enable_pin = 0xFF;
    nrf_ppi_channel_t pull_enable_ppi;
    nrf_ppi_channel_t pull_disable_ppi;

    static void static_uarte_handler(nrfx_uarte_event_t const *p_event, void *p_context);
    static void static_timeout_handler(void * p_context);

    void timeout_handler();
    void uarte_handler(nrfx_uarte_event_t const *p_event);
    void setup_delay_state(ommocomm_state_t new_state);

    uint32_t uarte_rx_data(uint8_t* data, uint16_t size);
    uint32_t uarte_tx_data(uint8_t* data, uint16_t size, bool hold_xfer = false) const;
};
