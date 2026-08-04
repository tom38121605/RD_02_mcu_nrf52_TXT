#pragma once
#include "ommo_fw.pb.h"
#include "ommo_macros.h"

//#define EMI_AUTO_SAMPLE_SENSOR
//#define EMI_RUN_WITHOUT_SENSOR
//#define SENSORS_MAG_CALIBRATION_ON
//#define RUN_WITHOUT_SYNC
//#define DISABLE_PROTECTION


//The sensor hotplug system does not detect changes to ommocomm devices during idle independent of whether the
//state machine is configured.  We rely on the main code ping to detect the hotplug events whenever the state
//machine is not fully running.  The state machine will still report the unplug of an ommcommm device while running
//even with this flag set.
#define HOT_PLUG_OFF 

//LED RGB scale value
#define RGB_RED_SCALE       1.0
#define RGB_GREEN_SCALE     1.0
#define RGB_BLUE_SCALE      1.0

//General temporary use timer
#define OMMO_TEMP_TIMER                   NRFX_TIMER_INSTANCE(2)

//General
#define HFCLK_FREQ  16000000
#define OMMO_BOOTLOADER_PRESENT
#define OMMO_APP_DFU_COMMANDS_ENABLED

//General pins
#define OMMO_LED_PWM                      NRFX_PWM_INSTANCE(0)
#define OMMO_LED_ACTIVE_LOW               false
#define OMMO_LED_RED                      NRF_GPIO_PIN_MAP(0,25)
#define OMMO_LED_GREEN                    NRF_GPIO_PIN_MAP(1,5)
#define OMMO_LED_BLUE                     NRF_GPIO_PIN_MAP(1,3)

#define OMMO_SENSOR_TO_VCON_PIN           NRF_GPIO_PIN_MAP(0,17)
#define OMMO_DATA_EN_PIN                  NRF_GPIO_PIN_MAP(0,6)
#define OMMOCOMM_UARTE_DATA_PIN           NRF_GPIO_PIN_MAP(0,7)

//Power
#define OMMO_POWER_ON_PIN                 NRF_GPIO_PIN_MAP(0,15)
#define OMMO_POWER_DCDC

//rtc timeout gen
#define OMMO_RTC_TIMEOUT_GEN_RTC          NRFX_RTC_INSTANCE(2)

// fuel gauge
#define OMMO_BQ27Z558
#define OMMO_FUEL_GAUGE_INT             NRF_GPIO_PIN_MAP(1,7)
#define OMMO_FUEL_GAUGE_PORT            0
#define OMMO_FUEL_GAUGE_BUS_LOCATION    IC_BUS_LOCATION_FIXED_0
#define GOLDEN_IMAGE_ZWD_CCGAIN_1P38

//charger
#define OMMO_BQ25188
#define OMMO_BQ25188_INT             NRF_GPIO_PIN_MAP(0,20)
#define OMMO_BQ25188_PORT            0
#define OMMO_BQ25188_BUS_LOCATION    IC_BUS_LOCATION_FIXED_0

//EEPROM AT25FF041A
#define OMMO_EEPROM_AT25_SPI_PERIPHERAL     NRFX_SPIM_INSTANCE(2)
#define OMMO_EEPROM_AT25_CS                 NRF_GPIO_PIN_MAP(1,9)  //TODO currently set as gpio with pullup to save power
#define OMMO_EEPROM_AT25_SCK                NRF_GPIO_PIN_MAP(0,0)
#define OMMO_EEPROM_AT25_MOSI               NRF_GPIO_PIN_MAP(0,27)
#define OMMO_EEPROM_AT25_MISO               NRF_GPIO_PIN_MAP(0,26)
//#define OMMO_PINS_TO_VCC                     {OMMO_EEPROM_AT25_CS} //Keep EEPROM deactivated

//Charge serial LED
#define OMMO_CHARGE_LED
#define OMMO_CHARGE_LED_NUM_LEDS            4
#define OMMO_CHARGE_LED_SPI_BUS_INDEX       1
#define OMMO_CHARGE_LED_INVERTED            true
#define OMMO_CHARGE_LED_RGB_BYTE_ORDER      SERIAL_LED_BYTE_ORDER_GRB
#define SERIAL_LED_MAX_LEDS                 OMMO_CHARGE_LED_NUM_LEDS

// Port master
#define OMMO_PM_SPIM_TWI_PERIPHERALS_IDS  (0,0)  // Use bundled peripheral ID enclosed in parentheses and separated by commas
#define OMMO_PM_SPI_PERIPHERALS_IDS       1      // Use peripheral ID separated by commas
#define OMMO_PM_UARTE_PERIPHERALS_IDS     0, 1   // Use peripheral ID separated by commas
#define OMMO_PM_SS_PINS (0,30), (0,29), (0,14)   // Use pin numbers enclosed in parentheses and separated by commas

#define OMMO_PM_MAX_SPI_BUSSES_PER_PORT 1
#define OMMO_PM_MAX_SS_PINS_PER_BUS     3
#define OMMO_PM_MAX_TWI_BUSSES_PER_PORT 2

#define OMMO_PM_SPI_IRQ_PRIORITY  3
#define OMMO_PM_TWI_IRQ_PRIORITY  3

#define OMMO_PM_SPI_BUSSES  ( .miso_pin = NRF_GPIO_PIN_MAP(0,23), .mosi_pin = NRF_GPIO_PIN_MAP(0,2), .sck_pin =NRF_GPIO_PIN_MAP(0,31) ), \
                            ( .miso_pin = NRFX_SPIM_PIN_NOT_USED, .mosi_pin = OMMOCOMM_UARTE_DATA_PIN, .sck_pin = NRF_GPIO_PIN_MAP(1,1), .supports_sampling = false)

#define OMMO_PM_PORTS                                                                                                                   \
    ( .spi_buses = {                                                                                                                    \
        { .spi_bus_index = 0, .ss_index_list = {0, 1, 2}, .ss_index_count = 3 },                                                        \
      },                                                                                                                                \
      .spi_count = 1,                                                                                                                   \
      .twi_buses = {                                                                                                                    \
        { .scl_pin = port_master_spi_busses[0].mosi_pin, .sda_pin = port_master_ss_pins[0], .location = IC_BUS_LOCATION_I2C_MOSI_CS0 }, \
        { .scl_pin = NRF_GPIO_PIN_MAP(1,02), .sda_pin = NRF_GPIO_PIN_MAP(0,22), .location = IC_BUS_LOCATION_FIXED_0 },                  \
      },                                                                                                                                \
      .twi_count = 2,                                                                                                                   \
      .flash_media_present = true,                                                                                                      \
    ),                                                                                                                                  \
    ( .supports_ommocomm_virtual_port_pin = OMMOCOMM_UARTE_DATA_PIN,                                                                    \
    )

//IMU IRQ pin
#define OMMO_IMU_IRQ_PIN                    NRF_GPIO_PIN_MAP(0,12)

//Power
#define OMMO_POWER_REQUIREMENTS             (POWER_REQ_USB | POWER_REQ_OMMOCOMM_SAMPLER | POWER_REQ_1WIRE)

//Ommocomm
#define OMMOCOMM_ENABLED
#define OMMO_HOURS_COUNTER_SUM_ENABLED
#define OMMOCOMM_IRQ_PRIORITY                2
#define OMMOCOMM_UNUSED_PIN                  NRF_GPIO_PIN_MAP(0,10)
#define OMMOCOMM_AUTOMODE
#define OMMOCOMM_1WIRE
#define OMMOCOMM_1WIRE_TIMER                 NRFX_TIMER_INSTANCE(0)

//Timestamp
#define OMMO_TIMESTAMP_TIMER                 NRFX_TIMER_INSTANCE(3) //Timers 3 & 4 have 6 CC registers
#define OMMO_TIMESTAMP_SYNCH_IN
#define OMMO_TIMESTAMP_SAMPLE_PERIOD         15984 //~1000hz, divisible by 3 for basestation and 16 for uart clock
#define OMMO_TIMESTAMP_SYNCH_PERIOD_MULT     4

//comms_esb
#define OMMO_COMMS_ESB_SYNCH_TIMER            NRFX_TIMER_INSTANCE(4) //Timers 3 & 4 have 6 CC registers
#define OMMO_COMMS_ESB_SYNCH_RTC              NRFX_RTC_INSTANCE(0)

//usb_serial.c
#define OMMO_USB_SERIAL_MIN_PACKET_SIZE   128
#define USB_SERIAL_TX_BUFFER_SIZE (8*1024)
#define USB_SERIAL_RX_BUFFER_SIZE (3*1024)

//usb_serial_num.c
#define OMMO_USB_COMMUNICATION_CLASS      COMMUNICATION_CLASS_TRACKING_DEVICE

//adc_monitor.c
#define OMMO_ADC_MONITOR
#define OMMO_ADC_MONITOR_PERIOD           250 //ms
#define OMMO_ADC_MONITOR_VIN              NRF_SAADC_INPUT_AIN3  //VBAT
#define OMMO_ADC_MONITOR_VIN_CUTOFF       ADC_MONITOR_VOLTAGE_TO_ADC(3.2*499/(499 + 1000))
#define OMMO_ADC_MONITOR_VIN_DEAD_BATTERY ADC_MONITOR_VOLTAGE_TO_ADC(2.0*499/(499 + 1000)) //Do not charge below 2.0V
#define OMMO_ADC_MONITOR_VCON             NRF_SAADC_INPUT_AIN2
#define OMMO_ADC_MONITOR_VCON_THRESHOLD   ADC_MONITOR_VOLTAGE_TO_ADC(4.0*499/(499 + 1000))
#define OMMO_ADC_MONITOR_VSYS             NRF_SAADC_INPUT_AIN2
#define OMMO_ADC_MONITOR_VSYS_THRESHOLD   ADC_MONITOR_VOLTAGE_TO_ADC(2.0*499/(499 + 1000))

//Tracing
#define OMMO_ESB_TRACE_ENABLED
//#define OMMO_1WIRE_TRACE_ENABLED
//#define OMMO_USB_STATE_TRACE_ENABLED
//#define OMMO_USB_DATA_TRACE_ENABLED
//#define OMMO_LED_DRIVER_TRACE_ENABLED

//DFU
// The definition of OMMO_DFU_CAPABILITY_MASK is parsed by build.py
#ifdef OMMO_DFU_EXTERNAL_FLASH_ONLY
#define OMMO_DFU_CAPABILITY_MASK          OMMO_MAKE_DFU_CAPABILITY_MASK(EXTERNAL_FLASH)
#else
#define OMMO_DFU_CAPABILITY_MASK          OMMO_MAKE_DFU_CAPABILITY_MASK(INTERNAL_FLASH, EXTERNAL_FLASH)
#define OMMO_APP_DPU_INTERNAL_FLASH_SUPPORTED
#endif
#define OMMO_EXTERNAL_FLASH_DFU_SIZE      0x40000
#define OMMO_INFER_DFU_CAPABILITY_MASK(VERSION) ( \
      (VERSION) < 7006 ? OMMO_MAKE_DFU_CAPABILITY_MASK(USB_ONLY)  /* earlier builds only supported DFU over USB serial */ \
    : (VERSION) < 7011 ? OMMO_MAKE_DFU_CAPABILITY_MASK(INTERNAL_FLASH)  /* 7006 introduced 1-wire and in-app DFU to internal flash */ \
    :                    OMMO_MAKE_DFU_CAPABILITY_MASK(INTERNAL_FLASH, EXTERNAL_FLASH))  /* 7011 introduced DFU to external flash */
