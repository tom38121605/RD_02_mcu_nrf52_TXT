#include "port_master.hpp"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "sdk_common.h"
#include "nrf.h"
#include "sdk_config.h"
#include "app_timer.h"

#include "nrf_gpio.h"
#include "nrfx_gpiote.h"
#include "nrf_delay.h"
#include "nrfx_twim.h"
#include "nrfx_spim.h"
#include "nrfx_timer.h"
#include "nrfx_ppi.h"

#include "ommo_config.h"

#include "comm_device.hpp"
#include "twi_basic.hpp"
#include "spim_basic.hpp"
#include "uarte_basic.hpp"
#include "utils.hpp"
#include "spim_twi_basic.hpp"
#include "platform.h"
#include "array_container.hpp"

#ifdef OMMOCOMM_ENABLED
#include "ommocomm_uarte.hpp"
#endif

// System port definition
#ifndef OMMO_PM_SS_PINS
const uint8_t port_master_ss_pins[] = {};
#else
#define GPIO_DEF(arg) NRF_GPIO_PIN_MAP arg,
const uint8_t port_master_ss_pins[] = {FOR_EACH(GPIO_DEF, OMMO_PM_SS_PINS)};
#endif

#ifndef OMMO_PM_SPI_BUSSES
const spi_bus_t port_master_spi_busses[] = {};
#else
#define SPI_BUS_DEF(arg) {UTIL_OBSTRUCT arg },
const spi_bus_t port_master_spi_busses[] = {FOR_EACH(SPI_BUS_DEF, OMMO_PM_SPI_BUSSES)};
#endif

#ifndef OMMO_PM_PORTS
const port_t port_master_ports[] = {};
#else
#define PORT_DEF(arg) {UTIL_OBSTRUCT arg },
const port_t port_master_ports[] = {FOR_EACH(PORT_DEF, OMMO_PM_PORTS)};
#endif

//Private variables
#if OMMO_PM_SPIM_BASIC_COUNT
#define SPIM_INTST(id) NRFX_SPIM_INSTANCE(id),
#define SPIM_BASIC_INTST(index) spim_basic(&spi_peripheral[index]),
static const nrfx_spim_t spi_peripheral[] = {FOR_EACH(SPIM_INTST, OMMO_PM_SPI_PERIPHERALS_IDS) };
static spim_basic spim_basic_instances[] = { FOR_MACRO(COUNT_ARGS(OMMO_PM_SPI_PERIPHERALS_IDS), SPIM_BASIC_INTST) };
#endif

#if OMMO_PM_TWI_BASIC_COUNT
#define TWIM_INTST(id) NRFX_TWIM_INSTANCE(id),
#define TWIM_BASIC_INTST(index) twi_basic(&twi_peripheral[index]),
static const nrfx_twim_t twi_peripheral[] = { FOR_EACH(TWIM_INTST, OMMO_PM_TWI_PERIPHERALS_IDS) };
static twi_basic twi_basic_instances[] = { FOR_MACRO(COUNT_ARGS(OMMO_PM_TWI_PERIPHERALS_IDS), TWIM_BASIC_INTST) };
#endif

#if OMMO_PM_SPIM_TWI_BASIC_COUNT
#define SPIM_TWI_INTST_HELPER(_spim, _twi) \
    {                                      \
        .spim = NRFX_SPIM_INSTANCE(_spim), \
        .twi  = NRFX_TWIM_INSTANCE(_twi),  \
    },
#define SPIM_TWI_INTST(param) SPIM_TWI_INTST_HELPER param
#define SPIM_TWI_BASIC(index) spim_twi_basic(&spim_twi_peripheral[index].spim, &spim_twi_peripheral[index].twi),
static const struct
{
    nrfx_spim_t spim;
    nrfx_twim_t twi;
} spim_twi_peripheral[]                          = { FOR_EACH(SPIM_TWI_INTST, OMMO_PM_SPIM_TWI_PERIPHERALS_IDS) };
static spim_twi_basic spim_twi_basic_instances[] = { FOR_MACRO(COUNT_ARGS(OMMO_PM_SPIM_TWI_PERIPHERALS_IDS), SPIM_TWI_BASIC) };
#endif

#if OMMO_PM_UARTE_COUNT
#define UARTE_INTST(id) NRFX_UARTE_INSTANCE(id),
#define UARTE_BASIC_INTST(index) uarte_basic(&uarte_peripheral[index]),
static const nrfx_uarte_t uarte_peripheral[] = {FOR_EACH(UARTE_INTST, OMMO_PM_UARTE_PERIPHERALS_IDS)};
static uarte_basic uarte_basic_instances[] = { FOR_MACRO(COUNT_ARGS(OMMO_PM_UARTE_PERIPHERALS_IDS), UARTE_BASIC_INTST) };

#ifdef OMMOCOMM_ENABLED
static ommocomm_uarte ommocomm_instances[OMMO_PM_UARTE_COUNT];
#endif
#endif

static uint64_t pins_in_use_mask = 0x00;
static nrf_twim_frequency_t port_master_max_MOSI_CS0_I2C_freq = NRF_TWIM_FREQ_9p8K;
static bool port_master_initialized = false;

//Private functions
static void port_master_apply_comm_device_config(nrfx_spim_config_t *spim_config, comm_device_config_t *comm_dev_config);
static void port_master_apply_comm_device_config(twi_basic_config_t *twi_config, comm_device_config_t *comm_dev_config);
#ifdef OMMOCOMM_ENABLED
static ret_code_t port_master_acquire_ommocomm_internal(ommocomm_acqmode_t mode, uint8_t port, bool virtual_or_eeprom_only, uint8_t cs_index_twi_location, ommocomm_uarte ** ommocomm_uarte_ptr);
#endif

static void port_master_blank_timer_event_handler(nrf_timer_event_t event_type, void* p_context)
{
    UNUSED_PARAMETER(event_type);
    UNUSED_PARAMETER(p_context);
}

#ifdef OMMO_TEMP_TIMER

#pragma GCC push_options
#pragma GCC optimize ("O3") //We need constant time through here despite build mode
static inline void port_master_measure_start(nrfx_timer_t * timer, uint32_t pin)
{
    NRF_GPIO_Type * p_reg = nrf_gpio_pin_port_decode(&pin);

    NRFX_CRITICAL_SECTION_ENTER();
    // Optimized to minimize cycles between calls. This is the unoptimized code:
    // nrfx_timer_resume(timer);
    // nrf_gpio_pin_set(pin);

    *(volatile uint32_t *)((uint8_t *)timer->p_reg + (uint32_t)NRF_TIMER_TASK_START) = 0x1UL;
    p_reg->OUTSET = 1UL << pin;
    NRFX_CRITICAL_SECTION_EXIT();
}
#pragma GCC pop_options

static void port_master_measure_pin_rise_times(const uint8_t *pins, uint32_t *times, const uint8_t count)
{
    if (!nrfx_gpiote_is_init())
    {
        APP_ERROR_CHECK(nrfx_gpiote_init());
    }

    //Setup timer
    nrfx_timer_config_t timer_cfg = NRFX_TIMER_DEFAULT_CONFIG;
    nrfx_timer_t temp_timer = OMMO_TEMP_TIMER;
    APP_ERROR_CHECK(nrfx_timer_init(&temp_timer, &timer_cfg, port_master_blank_timer_event_handler));
    nrfx_timer_enable(&temp_timer);

    #ifdef TEST_PIN
    nrfx_gpiote_out_config_t gpiote_config_out;
    gpiote_config_out.action = NRF_GPIOTE_POLARITY_TOGGLE;
    gpiote_config_out.init_state = NRF_GPIOTE_INITIAL_VALUE_LOW;
    gpiote_config_out.task_pin = true;
    APP_ERROR_CHECK(nrfx_gpiote_out_init(TEST_PIN, &gpiote_config_out));
    nrfx_gpiote_out_task_enable(TEST_PIN);
    #endif

    #ifdef OMMO_SENSOR_I2C_PULLUP_EN
    nrf_gpio_pin_write(OMMO_SENSOR_I2C_PULLUP_EN, 0);
    #endif

    //Allocate ppi
    nrf_ppi_channel_t temp_ppi;
    APP_ERROR_CHECK(nrfx_ppi_channel_alloc(&temp_ppi));
    for(int i=0; i<count; i++)
    {
        //Set pin low
        nrf_gpio_pin_write(pins[i], 0);
        nrf_gpio_cfg(pins[i], NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_CONNECT, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_H0D1, NRF_GPIO_PIN_NOSENSE);
        nrf_delay_us(100);

        //Config GPIOTE to watch for rising edge
        nrfx_gpiote_in_config_t gpiote_config;
        gpiote_config.sense = NRF_GPIOTE_POLARITY_LOTOHI;
        gpiote_config.skip_gpio_setup = true;
        gpiote_config.hi_accuracy = false;  //Use PORT EVENT
        APP_ERROR_CHECK(nrfx_gpiote_in_init(pins[i], &gpiote_config, NULL));
        nrfx_gpiote_in_event_enable(pins[i], false); //This will set the pin to an input and allow it to rise

        //Stop timer on rising edge detection
        APP_ERROR_CHECK(nrfx_ppi_channel_assign(temp_ppi, nrfx_gpiote_in_event_addr_get(pins[i]), nrfx_timer_task_address_get(&temp_timer, NRF_TIMER_TASK_CAPTURE0)));
        #ifdef TEST_PIN
        APP_ERROR_CHECK(nrfx_ppi_channel_fork_assign(temp_ppi, nrfx_gpiote_out_task_addr_get(TEST_PIN)));
        #endif
        APP_ERROR_CHECK(nrfx_ppi_channel_enable(temp_ppi));

        //Clear timer
        nrfx_timer_pause(&temp_timer);
        nrfx_timer_clear(&temp_timer);

        //Measure rise time
        port_master_measure_start(&temp_timer, pins[i]);

        //Wait for signal to rise
        uint32_t count = 0;
        while(!nrf_gpio_pin_read(pins[i]) && nrfx_timer_capture(&temp_timer, NRF_TIMER_CC_CHANNEL1) < US_TO_TICKS_16MHZ(1000))
            count++;

        //Grab result
        times[i] = nrfx_timer_capture_get(&temp_timer, NRF_TIMER_CC_CHANNEL0);
        nrfx_timer_disable(&temp_timer);

        //Reset pin state
        nrfx_gpiote_in_uninit(pins[i]);
    }

    //Unallocate
    APP_ERROR_CHECK(nrfx_ppi_channel_free(temp_ppi));
    nrfx_timer_uninit(&temp_timer);

    #ifdef OMMO_SENSOR_I2C_PULLUP_EN
    nrf_gpio_pin_write(OMMO_SENSOR_I2C_PULLUP_EN, 1);
    #endif
}

ret_code_t port_master_measure_and_calculate_i2c_MOSI_CS0_frequency()
{
    // Measure possible SCL/SDA rise times
    uint32_t cs0_mosi_times[OMMO_PM_PORTS_COUNT * 2];
    uint8_t  cs0_mosi_pins[OMMO_PM_PORTS_COUNT * 2];
    uint8_t  num_i2c_pins_to_test = 0;

    for (const port_t & port_master_port : port_master_ports)
    {
        for (uint8_t bus = 0; bus < port_master_port.twi_count; bus++)
        {
            if (port_master_port.twi_buses[bus].location == IC_BUS_LOCATION_I2C_MOSI_CS0)
            {
                cs0_mosi_pins[num_i2c_pins_to_test++] = port_master_port.twi_buses[bus].scl_pin;
                cs0_mosi_pins[num_i2c_pins_to_test++] = port_master_port.twi_buses[bus].sda_pin;
            }
        }
    }

    if(num_i2c_pins_to_test > 0)
    {
        num_i2c_pins_to_test = remove_duplicates(cs0_mosi_pins, num_i2c_pins_to_test);
        port_master_measure_pin_rise_times(cs0_mosi_pins, cs0_mosi_times, num_i2c_pins_to_test);

        // Calculate i2c frequency setting
        uint32_t max_count = find_max(cs0_mosi_times, num_i2c_pins_to_test);
        uint32_t freq      = HFCLK_FREQ / (4 * (max_count + 1));

        // Save setting
        port_master_max_MOSI_CS0_I2C_freq = twi_basic::frequency_to_setting(freq);
    }

    return NRF_SUCCESS;
}
#endif

void port_master_init()
{
#if OMMO_PM_TWI_BASIC_COUNT
    for (twi_basic & basic: twi_basic_instances)
    {
        APP_ERROR_CHECK(basic.init());
    }
#endif

#if OMMO_PM_SPIM_TWI_BASIC_COUNT
    for (spim_twi_basic & basic: spim_twi_basic_instances)
    {
        APP_ERROR_CHECK(basic.init());
    }
#endif

#ifdef OMMOCOMM_ENABLED
    for (ommocomm_uarte & ommocomm: ommocomm_instances)
    {
        APP_ERROR_CHECK(ommocomm.init());
    }
#endif

    for (const uint8_t & pin: port_master_ss_pins)
    {
        nrf_gpio_pin_set(pin);
        port_master_configure_high_speed_output(pin);
    }

#ifdef OMMO_SENSOR_I2C_PULLUP_EN
    nrf_gpio_pin_write(OMMO_SENSOR_I2C_PULLUP_EN, 1);
    nrf_gpio_cfg(OMMO_SENSOR_I2C_PULLUP_EN, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT,
                  NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0D1, NRF_GPIO_PIN_NOSENSE);
#endif

    port_master_initialized = true;
}

bool port_master_has_twi_bus(uint8_t port, uint8_t bus_location)
{
    // Verify port
    if (port >= OMMO_PM_PORTS_COUNT)
        return false;

    // Find bus_location in i2c bus list
    for (uint8_t i = 0; i < port_master_ports[port].twi_count; i++)
    {
        if (port_master_ports[port].twi_buses[i].location == bus_location)
            return true;
    }

    return false;
}

ret_code_t port_master_get_twi_bus_config(uint8_t port, uint8_t bus_location, twi_basic_config_t * config)
{
    // Verify port
    if (port >= OMMO_PM_PORTS_COUNT)
        return NRF_ERROR_INVALID_DATA;

    // Clear existing config
    memset(config, 0, sizeof(twi_basic_config_t));

    // Find bus_location in i2c bus list
    for (uint8_t i = 0; i < port_master_ports[port].twi_count; i++)
    {
        if (port_master_ports[port].twi_buses[i].location == bus_location)
        {
            // MOSI/CS0 bus's have special configs
            if (bus_location == IC_BUS_LOCATION_I2C_MOSI_CS0)
            {
                config->scl_push_pull = true;
                config->frequency     = port_master_max_MOSI_CS0_I2C_freq;
            }
            else
            {
                config->frequency     = NRF_TWIM_FREQ_100K;
                config->scl_push_pull = false;
            }

            config->scl_pin = port_master_ports[port].twi_buses[i].scl_pin;
            config->sda_pin = port_master_ports[port].twi_buses[i].sda_pin;
            return NRF_SUCCESS;
        }
    }

    return NRF_ERROR_NOT_FOUND;
}

ret_code_t port_master_get_spi_bus_config(spi_bus_index_t spi_bus_index, nrfx_spim_config_t * p_out_config)
{
    if (spi_bus_index >= OMMO_PM_SPI_BUSSES_COUNT)
    {
        return NRF_ERROR_NOT_FOUND;
    }

    if (p_out_config == NULL)
    {
        return NRF_ERROR_NULL;
    }

    *p_out_config = (nrfx_spim_config_t)
    {
        .sck_pin        = port_master_spi_busses[spi_bus_index].sck_pin,
        .mosi_pin       = port_master_spi_busses[spi_bus_index].mosi_pin,
        .miso_pin       = port_master_spi_busses[spi_bus_index].miso_pin,
        .ss_pin         = NRFX_SPIM_PIN_NOT_USED,
        .ss_active_high = false,
        .irq_priority   = port_master_spi_busses[spi_bus_index].irq_priority,
        .orc            = 0xFF,
        .frequency      = port_master_spi_busses[spi_bus_index].frequency,
        .mode           = port_master_spi_busses[spi_bus_index].mode,
        .bit_order      = port_master_spi_busses[spi_bus_index].bit_order,
        NRFX_SPIM_DEFAULT_EXTENDED_CONFIG
    };

    return NRF_SUCCESS;
}

void port_master_apply_comm_device_config(nrfx_spim_config_t *spim_config, comm_device_config_t *comm_dev_config)
{
    if(comm_dev_config->frequency != 0xFF) spim_config->frequency = (nrf_spim_frequency_t)comm_dev_config->frequency;
    if(comm_dev_config->ss_active_high != 0xFF) spim_config->ss_active_high = comm_dev_config->ss_active_high;
    if(comm_dev_config->mode != 0xFF) spim_config->mode = comm_dev_config->mode;
    if(comm_dev_config->bit_order != 0xFF) spim_config->bit_order = comm_dev_config->bit_order;
}

void port_master_apply_comm_device_config(twi_basic_config_t *twi_config, comm_device_config_t *comm_dev_config)
{
    if(comm_dev_config->frequency != 0xFF) twi_config->frequency = (nrf_twim_frequency_t)comm_dev_config->frequency;
    if(comm_dev_config->scl_push_pull != 0xFF) twi_config->scl_push_pull = comm_dev_config->scl_push_pull;
}

ret_code_t port_master_acquire_spim_direct(spi_bus_index_t spi_bus_index, spim_basic ** spim_basic_ptr, comm_device_config_t *comm_dev_config)
{
#if OMMO_PM_SPIM_BASIC_COUNT
    nrfx_spim_config_t spim_config;
    VERIFY_SUCCESS(port_master_get_spi_bus_config(spi_bus_index, &spim_config));
    if(comm_dev_config != nullptr) port_master_apply_comm_device_config(&spim_config, comm_dev_config);

    // Find a spim_basic resource
    for (spim_basic & spim_instance : spim_basic_instances)
    {
        if (!spim_instance.is_acquired())
        {
            ret_code_t rvalue = spim_instance.acquire(&spim_config, true);
            if (rvalue == NRF_SUCCESS)
            {
                *spim_basic_ptr = &spim_instance;
            }
            return rvalue;
        }
    }
#endif

    return NRF_ERROR_NO_MEM;
}

ret_code_t port_master_acquire_spim_twi_direct(spi_bus_index_t spi_bus_index, spim_twi_basic ** spim_twi_basic_ptr, comm_device_config_t *comm_dev_config)
{
#if OMMO_PM_SPIM_TWI_BASIC_COUNT
    nrfx_spim_config_t spim_config;
    VERIFY_SUCCESS(port_master_get_spi_bus_config(spi_bus_index, &spim_config));
    if(comm_dev_config != nullptr) port_master_apply_comm_device_config(&spim_config, comm_dev_config);

    // Find a spim_basic resource
    for (spim_twi_basic & spim_twi_instance : spim_twi_basic_instances)
    {
        if (!spim_twi_instance.is_acquired())
        {
            ret_code_t rvalue = spim_twi_instance.acquire_spim(&spim_config, true);
            if (rvalue == NRF_SUCCESS)
            {
                *spim_twi_basic_ptr = &spim_twi_instance;
            }
            return rvalue;
        }
    }
#endif
    return NRF_ERROR_NO_MEM;
}

#ifdef OMMOCOMM_ENABLED

ret_code_t port_master_acquire_ommocomm_direct(uint8_t port, bool virtual_or_eeprom_only, ommocomm_uarte ** ommocomm_uarte_ptr)
{
    return port_master_acquire_ommocomm_internal(OMMOCOMM_ACQMODE_DIRECT, port, virtual_or_eeprom_only, 0, ommocomm_uarte_ptr);
}

ret_code_t port_master_acquire_ommocomm_internal(ommocomm_acqmode_t mode, uint8_t port, bool virtual_or_eeprom_only, uint8_t cs_index_twi_location, ommocomm_uarte ** ommocomm_uarte_ptr)
{
    if (port >= OMMO_PM_PORTS_COUNT)
        return NRF_ERROR_NOT_FOUND;

    uint32_t rtx_pin = (virtual_or_eeprom_only ?
                        port_master_ports[port].supports_ommocomm_virtual_port_pin :
                        port_master_ports[port].supports_ommocomm_eeprom_only_pin);

    if (rtx_pin == 0xFF)
        return NRF_ERROR_NOT_SUPPORTED;

    //Find a uarte and pair with an ommocomm instances
    for(uint8_t i=0; i<OMMO_PM_UARTE_COUNT; i++)
    {
        if (!uarte_basic_instances[i].is_acquired())
        {
            ret_code_t rvalue = ommocomm_instances[i].acquire(&uarte_basic_instances[i], mode, rtx_pin, cs_index_twi_location, port_master_ports[port].supports_ommocomm_pullup_enable_pin);
            if(rvalue == NRF_SUCCESS)
                *ommocomm_uarte_ptr = &ommocomm_instances[i];
            return rvalue;
        }
    }

    return NRF_ERROR_NO_MEM;
}
#endif

#if OMMO_PM_UARTE_COUNT

ret_code_t port_master_acquire_uarte_direct(const nrfx_uarte_config_t * p_cfg, nrfx_uarte_event_handler_t  event_handler, void *p_context, uarte_basic **uarte_ptr)
{
    //Find a uarte and pair with an ommocomm instances
    for(uint8_t i=0; i<OMMO_PM_UARTE_COUNT; i++)
    {
        if (!uarte_basic_instances[i].is_acquired())
        {
            ret_code_t rvalue = uarte_basic_instances[i].acquire(p_cfg, event_handler, p_context);
            if (rvalue == NRF_SUCCESS)
            {
                *uarte_ptr = &uarte_basic_instances[i];
            }
            return rvalue;
        }
    }

    return NRF_ERROR_NO_MEM;
}

ret_code_t port_master_acquire_uarte(const nrfx_uarte_config_t * p_cfg, uarte_basic **uarte_ptr)
{
    return port_master_acquire_uarte_direct(p_cfg, nullptr, nullptr, uarte_ptr);
}

#endif // OMMO_PM_UARTE_COUNT

ret_code_t port_master_acquire(uint8_t port, ICBusType bus_type, uint8_t bus_location_ss_line, comm_device ** comm_device_ptr, comm_device_config_t *comm_dev_config)
{
    VERIFY_TRUE(port_master_initialized, NRF_ERROR_MODULE_NOT_INITIALIZED);
    VERIFY_TRUE(port < OMMO_PM_PORTS_COUNT, NRF_ERROR_NOT_FOUND);

    switch (bus_type)
    {
        case IC_BUS_TYPE_SPI:
        {
            // Verify SPI bus exists
            spi_slice_index_t slice_index = port_master_decode_spi_slice_index(bus_location_ss_line);
            spi_ss_index_t ss_index = port_master_decode_spi_ss_index(bus_location_ss_line);
            if (slice_index >= port_master_ports[port].spi_count
                || (ss_index != 0xFF
                    && (ss_index >= port_master_ports[port].spi_buses[slice_index].ss_index_count
                        || port_master_ports[port].spi_buses[slice_index].ss_index_list[ss_index] >= NRFX_ARRAY_SIZE(port_master_ss_pins))))
            {
                return NRF_ERROR_NOT_FOUND;
            }

            spi_bus_index_t spi_bus_index = port_master_ports[port].spi_buses[slice_index].spi_bus_index;

            // Setup SPI config
            nrfx_spim_config_t config;
            VERIFY_SUCCESS(port_master_get_spi_bus_config(spi_bus_index, &config));
            if (ss_index != 0xFF)
                config.ss_pin = port_master_ss_pins[port_master_ports[port].spi_buses[slice_index].ss_index_list[ss_index]];
            if(comm_dev_config != nullptr) port_master_apply_comm_device_config(&config, comm_dev_config);

#if OMMO_PM_SPIM_BASIC_COUNT
            for (spim_basic & spim_instance : spim_basic_instances)
            {
                if (!spim_instance.is_acquired())
                {
                    ret_code_t rvalue = spim_instance.acquire(&config);
                    if (rvalue == NRF_SUCCESS)
                    {
                        *comm_device_ptr = &spim_instance;
                    }
                    return rvalue;
                }
            }
#endif // OMMO_PM_SPIM_BASIC_COUNT
#if OMMO_PM_SPIM_TWI_BASIC_COUNT
            for (spim_twi_basic & spim_twi_instance : spim_twi_basic_instances)
            {
                if (!spim_twi_instance.is_acquired())
                {
                    // Acquire the TWI instance
                    ret_code_t rvalue = spim_twi_instance.acquire_spim(&config);
                    if (rvalue == NRF_SUCCESS)
                    {
                        *comm_device_ptr = &spim_twi_instance;
                    }
                    return rvalue;
                }
            }
#endif // OMMO_PM_SPIM_TWI_BASIC_COUNT
            return NRF_ERROR_NO_MEM;
        }

        case IC_BUS_TYPE_I2C:
        {
            //Setup twi config
            twi_basic_config_t config;
            VERIFY_SUCCESS(port_master_get_twi_bus_config(port, bus_location_ss_line, &config));
            if(comm_dev_config != nullptr) port_master_apply_comm_device_config(&config, comm_dev_config);

#if OMMO_PM_TWI_BASIC_COUNT
            for (twi_basic & twi_instance : twi_basic_instances)
            {
                if (!twi_instance.is_acquired())
                {
                    // Acquire the TWI instance
                    ret_code_t ret = twi_instance.acquire(&config);
                    if (ret == NRF_SUCCESS)
                    {
                        *comm_device_ptr = &twi_instance;
                    }
                    return ret;
                }
            }
#endif // OMMO_PM_TWI_BASIC_COUNT
#if OMMO_PM_SPIM_TWI_BASIC_COUNT
            for (spim_twi_basic & spim_twi_instance : spim_twi_basic_instances)
            {
                if (!spim_twi_instance.is_acquired())
                {
                    // Acquire the TWI instance
                    ret_code_t ret = spim_twi_instance.acquire_twi(&config);
                    if (ret == NRF_SUCCESS)
                    {
                        *comm_device_ptr = &spim_twi_instance;
                    }
                    return ret;
                }
            }
#endif // OMMO_PM_SPIM_TWI_BASIC_COUNT
            return NRF_ERROR_NO_MEM;
        }

#ifdef OMMOCOMM_ENABLED
        case IC_BUS_TYPE_OMMOCOMM_I2C:
        case IC_BUS_TYPE_OMMOCOMM_SPI:
        case IC_BUS_TYPE_CUSTOM:
        {
            if (port_master_ports[port].supports_ommocomm_virtual_port_pin == 0xFF)
            {
                return NRF_ERROR_NOT_SUPPORTED;
            }

            // Verify pin is free
            ommocomm_acqmode_t mode   = bus_type == IC_BUS_TYPE_OMMOCOMM_I2C   ? OMMOCOMM_ACQMODE_TWI
                                        : bus_type == IC_BUS_TYPE_OMMOCOMM_SPI ? OMMOCOMM_ACQMODE_SPI
                                                                               : OMMOCOMM_ACQMODE_DIRECT;
            return port_master_acquire_ommocomm_internal(mode, port, true, bus_location_ss_line, (ommocomm_uarte**)comm_device_ptr);
        }
#endif

        default:
            return NRF_ERROR_NOT_SUPPORTED;
    }
}

ret_code_t port_master_acquire_pins(uint64_t pin_mask)
{
    if (pins_in_use_mask & pin_mask)
    {
        return NRF_ERROR_BUSY;
    }

    pins_in_use_mask |= pin_mask;
    return NRF_SUCCESS;
}

void port_master_release_pins(const uint64_t pin_mask)
{
    pins_in_use_mask &= (~pin_mask);
}

// Bitmask of GPIO pins currently owned by GPIOTE task mode. 0 = none (common case, no overhead).
static uint64_t s_gpiote_ss_pin_mask = 0;

void port_master_register_gpiote_ss_pin(uint8_t gpio_pin)
{
    s_gpiote_ss_pin_mask |= (1ULL << gpio_pin);
}

void port_master_unregister_gpiote_ss_pin(uint8_t gpio_pin)
{
    s_gpiote_ss_pin_mask &= ~(1ULL << gpio_pin);
}

static inline void ss_pin_set(uint8_t gpio_pin)
{
    if(s_gpiote_ss_pin_mask && (s_gpiote_ss_pin_mask & (1ULL << gpio_pin)))
        nrfx_gpiote_set_task_trigger(gpio_pin);
    else
        nrf_gpio_pin_set(gpio_pin);
}

static inline void ss_pin_clear(uint8_t gpio_pin)
{
    if(s_gpiote_ss_pin_mask && (s_gpiote_ss_pin_mask & (1ULL << gpio_pin)))
        nrfx_gpiote_clr_task_trigger(gpio_pin);
    else
        nrf_gpio_pin_clear(gpio_pin);
}

static volatile uint32_t active_ss_pin = 0xFF;
void port_master_switch_ss(uint8_t new_ss_pin)
{
    if(active_ss_pin == 0xFE)
    {
        // Multiselected state, deactivate all ss pins first
        for(const uint8_t & pin : port_master_ss_pins)
            ss_pin_set(pin);
    }
    else if(active_ss_pin != 0xFF)
    {
        // Single ss pins selected, deactivate current ss pin first
        ss_pin_set(port_master_ss_pins[active_ss_pin]);
    }

    active_ss_pin = new_ss_pin;
    if(new_ss_pin != 0xFF)
        ss_pin_clear(port_master_ss_pins[active_ss_pin]);
}

void port_master_ss_deselect_all()
{
    active_ss_pin = 0xFF;
    for(const uint8_t & pin : port_master_ss_pins)
        ss_pin_set(pin);
}

void port_master_ss_select(ArrayContainer<uint8_t> & ss_index)
{
    active_ss_pin = 0xFE;
    for(const uint8_t pin_index : ss_index)
        ss_pin_clear(port_master_ss_pins[pin_index]);
}