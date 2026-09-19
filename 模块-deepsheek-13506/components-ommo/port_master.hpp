#pragma once

/* Includes ----------------------------------------------------------------- */
#include "ommo_config.h"
#include "nrf_gpio.h"
#include "utils.hpp"
#include "comm_device.hpp"
#include "spim_basic.hpp"
#include "spim_twi_basic.hpp"
#include "array_container.hpp"
#include "uarte_basic.hpp"
#include "ommo_macros.h"
#include "sdk_config.h"

#ifdef OMMOCOMM_ENABLED
#include "ommocomm_uarte.hpp"
#endif

/* Defines ------------------------------------------------------------------ */
#ifndef OMMO_PM_SPIM_TWI_PERIPHERALS_IDS
#define OMMO_PM_SPIM_TWI_PERIPHERALS_IDS
#endif
#ifndef OMMO_PM_SPI_PERIPHERALS_IDS
#define OMMO_PM_SPI_PERIPHERALS_IDS
#endif
#ifndef OMMO_PM_TWI_PERIPHERALS_IDS
#define OMMO_PM_TWI_PERIPHERALS_IDS
#endif
#ifndef OMMO_PM_UARTE_PERIPHERALS_IDS
#define OMMO_PM_UARTE_PERIPHERALS_IDS
#endif
#ifndef OMMO_PM_SPI_BUSSES
#define OMMO_PM_SPI_BUSSES
#endif
#ifndef OMMO_PM_SS_PINS
#define OMMO_PM_SS_PINS
#endif
#ifndef OMMO_PM_PORTS
#define OMMO_PM_PORTS
#endif

#define OMMO_PM_SPIM_TWI_BASIC_COUNT COUNT_ARGS(OMMO_PM_SPIM_TWI_PERIPHERALS_IDS)
#define OMMO_PM_SPIM_BASIC_COUNT     COUNT_ARGS(OMMO_PM_SPI_PERIPHERALS_IDS)
#define OMMO_PM_TWI_BASIC_COUNT      COUNT_ARGS(OMMO_PM_TWI_PERIPHERALS_IDS)
#define OMMO_PM_UARTE_COUNT          COUNT_ARGS(OMMO_PM_UARTE_PERIPHERALS_IDS)
#define OMMO_PM_SPI_BUSSES_COUNT     COUNT_ARGS(OMMO_PM_SPI_BUSSES)
#define OMMO_PM_SS_PINS_COUNT        COUNT_ARGS(OMMO_PM_SS_PINS)
#define OMMO_PM_PORTS_COUNT          COUNT_ARGS(OMMO_PM_PORTS)
#define OMMO_PM_MAX_PERIPHERALS_COUNT (OMMO_PM_SPIM_BASIC_COUNT + OMMO_PM_TWI_BASIC_COUNT + OMMO_PM_UARTE_COUNT + OMMO_PM_SPIM_TWI_BASIC_COUNT)

//TODO we need to include ommocomm IC's as well(something like  + OMMO_PM_OMMOCOMM_UARTE_COUNT * OMMO_PM_MAX_IC_PER_OMMOCOMM_PORT)
#ifndef OMMO_PM_MAX_ICS_PER_PORT
#define OMMO_PM_MAX_ICS_PER_PORT (OMMO_PM_MAX_SS_PINS_PER_BUS * OMMO_PM_MAX_SPI_BUSSES_PER_PORT + OMMO_PM_MAX_TWI_BUSSES_PER_PORT)
#endif

#ifndef OMMO_PM_MAX_TWI_BUSSES_PER_PORT
#define OMMO_PM_MAX_TWI_BUSSES_PER_PORT 0
#endif

#ifndef OMMO_PM_MAX_SPI_BUSSES_PER_PORT
#define OMMO_PM_MAX_SPI_BUSSES_PER_PORT 0
#endif

#ifndef OMMO_PM_MAX_SS_PINS_PER_BUS
#define OMMO_PM_MAX_SS_PINS_PER_BUS 0
#endif

#ifndef OMMO_PM_SPI_IRQ_PRIORITY
#define OMMO_PM_SPI_IRQ_PRIORITY NRFX_SPIM_DEFAULT_CONFIG_IRQ_PRIORITY
#endif

#ifndef OMMO_PM_TWI_IRQ_PRIORITY
#define OMMO_PM_TWI_IRQ_PRIORITY NRFX_TWIM_DEFAULT_CONFIG_IRQ_PRIORITY
#endif

/* Macros ------------------------------------------------------------------- */
/* Enums -------------------------------------------------------------------- */
/* Types -------------------------------------------------------------------- */

/** @brief TWI communication bus descriptor. */
typedef struct
{
    uint8_t scl_pin;
    uint8_t sda_pin;
    uint8_t  location;
} twi_bus_t;

/** @brief SPI communication bus descriptor. */
typedef struct
{
    uint8_t miso_pin = NRFX_SPIM_PIN_NOT_USED;
    uint8_t mosi_pin = NRFX_SPIM_PIN_NOT_USED;
    uint8_t sck_pin = NRFX_SPIM_PIN_NOT_USED;
    nrf_spim_frequency_t frequency = NRF_SPIM_FREQ_2M;
    nrf_spim_mode_t mode = NRF_SPIM_MODE_3;
    nrf_spim_bit_order_t bit_order = NRF_SPIM_BIT_ORDER_MSB_FIRST;
    uint8_t irq_priority = OMMO_PM_SPI_IRQ_PRIORITY;
    bool supports_sampling = true;
} spi_bus_t;

typedef uint8_t spi_bus_index_t;

/** @brief SPI bus slice descriptor, used to group SS pins for a specific SPI bus. */
typedef struct
{
    const uint8_t spi_bus_index; /**< Index of the SPI bus this slice belongs to, @ref spi_bus_t. */
    uint8_t       ss_index_list[OMMO_PM_MAX_SS_PINS_PER_BUS] = {};
    uint8_t       ss_index_count = 0;
} spi_bus_slice_t;

typedef uint8_t spi_slice_index_t;
typedef uint8_t spi_ss_index_t;

/** @brief Port descriptor, containing all communication buses and their configurations. */
typedef struct
{
    spi_bus_slice_t spi_buses[OMMO_PM_MAX_SPI_BUSSES_PER_PORT] = {}; /**< Array of SPI bus slices available on this port. */
    uint8_t         spi_count = 0; /**< Number of SPI bus slices available on this port. */
    twi_bus_t twi_buses[OMMO_PM_MAX_TWI_BUSSES_PER_PORT] = {}; /**< Array of TWI buses available on this port. */
    uint8_t   twi_count = 0; /**< Number of TWI buses available on this port. */

    uint8_t supports_ommocomm_eeprom_only_pin = 0xFF; /**< Pin number that supports OMMOCOMM eeprom only communication. If 0xFF, no such pin is available. */
    uint8_t supports_ommocomm_virtual_port_pin = 0xFF; /**< Pin number that supports OMMOCOMM virtual port communication. If 0xFF, no such pin is available. */
    uint8_t supports_ommocomm_pullup_enable_pin = 0xFF; /**< Pin number that enables or disables the pullup on all supported ommocomm pins. If 0xFF, no such pin is available. */
    bool flash_media_present = false; /**< True if flash media is available on this port. */
    bool supports_sampling = true; /**< Whether this port supports sampling. */
} port_t;

/** @brief comm_device_configs are optional OVERRIDE configs for the defaults.  0xFF is treated as a special value indicating do not overwrite. */
typedef struct
{
    //Combined configs
    uint64_t             frequency = 0xFF;
    //I2C configs
    uint8_t              scl_push_pull = 0xFF;
    //SPI configs
    uint8_t              ss_active_high = 0xFF;
    nrf_spim_mode_t      mode = (nrf_spim_mode_t)0xFF;
    nrf_spim_bit_order_t bit_order = (nrf_spim_bit_order_t)0xFF;

} comm_device_config_t;

/** @brief Bus-type-specific location identifier.  For SPI, this encodes bus number and SS index.  For TWI, this is simply bus number. */
typedef uint8_t bus_location_t;

/* Shared variables --------------------------------------------------------- */
// System port definition
extern const uint8_t  port_master_ss_pins[];
extern const spi_bus_t port_master_spi_busses[];
extern const port_t    port_master_ports[];

/* Shared functions --------------------------------------------------------- */
void port_master_init();

bool port_master_has_twi_bus(uint8_t port, uint8_t bus_location);

ret_code_t port_master_get_twi_bus_config(uint8_t port, uint8_t bus_location, twi_basic_config_t *config);
ret_code_t port_master_get_spi_bus_config(spi_bus_index_t spi_bus_index, nrfx_spim_config_t *p_out_config);

ret_code_t port_master_acquire_spim_direct(spi_bus_index_t spi_bus_index, spim_basic **spim_basic_ptr, comm_device_config_t *config = nullptr);
ret_code_t port_master_acquire_spim_twi_direct(spi_bus_index_t spi_bus_index, spim_twi_basic **spim_twi_basic_ptr, comm_device_config_t *config = nullptr);

#if OMMO_PM_UARTE_COUNT
ret_code_t port_master_acquire_uarte_direct(const nrfx_uarte_config_t * p_cfg, nrfx_uarte_event_handler_t  event_handler, void *p_context, uarte_basic **uarte_ptr);

//Should be acquired through standard acquire command
ret_code_t port_master_acquire_uarte(const nrfx_uarte_config_t * p_cfg, uarte_basic **uarte_ptr);
#endif // OMMO_PM_UARTE_COUNT

#ifdef OMMOCOMM_ENABLED
ret_code_t port_master_acquire_ommocomm_direct(uint8_t port, bool virtual_or_eeprom_only, ommocomm_uarte **ommocomm_uarte_ptr);
#endif

ret_code_t port_master_acquire(uint8_t port, ICBusType bus_type, uint8_t bus_location, comm_device ** comm_device_ptr, comm_device_config_t *config = nullptr);

ret_code_t port_master_acquire_pins(uint64_t pin_mask);
void       port_master_release_pins(uint64_t pin_mask);

static inline ret_code_t port_master_acquire_pin(uint8_t pin)
{
    return port_master_acquire_pins(PORT_PIN_TO_MASK(pin));
}

static inline void       port_master_release_pin(uint8_t pin)
{
    port_master_release_pins(PORT_PIN_TO_MASK(pin));
}

ret_code_t port_master_measure_and_calculate_i2c_MOSI_CS0_frequency();

// SS functions
void port_master_switch_ss(uint8_t new_ss_pin);
void port_master_ss_deselect_all();
void port_master_ss_select(ArrayContainer<uint8_t> & ss_index);

// Register a GPIO pin that is owned by GPIOTE task mode so SS operations use task triggers.
void port_master_register_gpiote_ss_pin(uint8_t gpio_pin);
void port_master_unregister_gpiote_ss_pin(uint8_t gpio_pin);

// GPIO helper functions
static inline void port_master_configure_high_speed_output(uint8_t pin)
{
    if (pin != 0xFF)
    {
        nrf_gpio_cfg(pin, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_H0H1, NRF_GPIO_PIN_NOSENSE);
    }
}

static inline bus_location_t port_master_encode_spi_bus_location(spi_slice_index_t slice_index, uint8_t ss_index = 0xFF)
{
    NRFX_ASSERT(slice_index < 16);
    NRFX_ASSERT(ss_index < 16 || ss_index == 0xFF);
    return (slice_index << 4) | (ss_index == 0xFF ? 0xF : ss_index);
}

static inline spi_slice_index_t port_master_decode_spi_slice_index(uint8_t spi_bus_location)
{
    return spi_bus_location >> 4;
}

static inline spi_ss_index_t port_master_decode_spi_ss_index(uint8_t spi_bus_location)
{
    uint8_t encoded_ss_index = spi_bus_location & 0xF;
    return encoded_ss_index == 0xF ? 0xFF : encoded_ss_index;
}
