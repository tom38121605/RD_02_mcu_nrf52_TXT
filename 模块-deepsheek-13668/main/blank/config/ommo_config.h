#if !defined(OMMO_CONFIG)
#define OMMO_CONFIG

#include "ommo_fw.pb.h"

//#define RUN_WITHOUT_SYNC
//#define MAG_CALIBRATION_ON
//#define OMMO_ADC_DISABLE_OVERVOLTAGE

#define LED_ON(x) nrf_gpio_pin_clear(x);
#define LED_OFF(x) nrf_gpio_pin_set(x);
#define LED_TOGGLE(x) nrf_gpio_pin_toggle(x);

//General pins
#define OMMO_LED_PWM                      NRFX_PWM_INSTANCE(0)
#define OMMO_LED_ACTIVE_LOW               true
#define OMMO_LED_RED                      NRF_GPIO_PIN_MAP(0,22)
#define OMMO_LED_GREEN                    NRF_GPIO_PIN_MAP(0,20)
#define OMMO_LED_BLUE                     NRF_GPIO_PIN_MAP(0,24)

//We need to make sure COLOR_OPERATING_NORMALLY results in a FULL ON color to prevent noise
#define RGB_RED_SCALE       1.0
#define RGB_GREEN_SCALE     1.0
#define RGB_BLUE_SCALE      1.0

//General
#define HFCLK_FREQ  16000000
#define OMMO_BOOTLOADER_PRESENT

#define OMMO_SENSOR_MAX_SS_PER_PORT         1

//rtc timeout gen
#define OMMO_RTC_TIMEOUT_GEN_RTC          NRFX_RTC_INSTANCE(2)

//eeprom
#define OMMO_O_COM_PIN                       NRF_GPIO_PIN_MAP(1,7)

// Set/Rest defines
#define OMMO_ADC_SET_RESET_ENABLED           true
#define OMMO_ADC_SET_RESET_INA_PIN           NRF_GPIO_PIN_MAP(0,0)
#define OMMO_ADC_SET_RESET_INB_PIN           NRF_GPIO_PIN_MAP(0,31)
#define OMMO_ADC_SET_TO_RESET_TIME_US        1
//#define OMMO_ADC_SET_RESET_POLARITY_INVERTED true

//mux
// EN pin is tied to vdda(always on)
#define OMMO_ADC_MUX_A0_PIN     NRF_GPIO_PIN_MAP(1,9)
#define OMMO_ADC_MUX_A1_PIN     NRF_GPIO_PIN_MAP(0,9)

//vtran channel control
#define OMMO_IO_X_EN            NRF_GPIO_PIN_MAP(0,27)
#define OMMO_IO_Y_EN            NRF_GPIO_PIN_MAP(0,7)
#define OMMO_IO_Z_EN            NRF_GPIO_PIN_MAP(0,8)

#define OMMO_MAG_IDLE_BRIDGE_ON
#define OMMO_MAG_BRIDGE_OVERVOLT_COMP
#define OMMO_VBRIDGE_EN         NRF_GPIO_PIN_MAP(0,25)
#define OMMO_VBRIDGE_ANALOG     COMP_PSEL_PSEL_AnalogInput5

//adc sampler
#define OMMO_ADC_127L11
#define OMMO_ADC_SPI_CS_PIN      (1,12)
#define OMMO_ADC_SPI_MISO_PIN    NRF_GPIO_PIN_MAP(1,13)
#define OMMO_ADC_SPI_MOSI_PIN    NRF_GPIO_PIN_MAP(1,14)
#define OMMO_ADC_SPI_SCK_PIN     NRF_GPIO_PIN_MAP(1,10)
#define OMMO_ADC_BUS_TYPE        IC_BUS_TYPE_SPI
#define OMMO_ADC_PORT            0
#define OMMO_ADC_SS_INDEX        0


#define OMMO_ADC_DATA_READY_PIN  NRF_GPIO_PIN_MAP(1,15)
#define OMMO_ADC_START_PIN       NRF_GPIO_PIN_MAP(0,1)
#define OMMO_ADC_RESET_PIN       NRF_GPIO_PIN_MAP(1,11)

#define OMMO_ADC_SAMPLING_TIMER          NRFX_TIMER_INSTANCE(0)
#define OMMO_ADC_SAMPLING_TIMER_PRIORITY 0

#define OMMO_ADC_OVERVOLTAGE_TRIGGER_POINT 2.7

//adc config
#define ADC_SAADC_RBRIDGE_RSENSE_ADC_MODE     NRF_SAADC_MODE_SINGLE_ENDED
#define ADC_SAADC_RBRIDGE_RSENSE_ADC_PIN_P    NRF_SAADC_INPUT_AIN4
#define ADC_SAADC_RBRIDGE_RSENSE_ADC_PIN_N    NRF_SAADC_INPUT_DISABLED
#define ADC_SAADC_RBRIDGE_RSENSE_ADC_GAIN_REG NRF_SAADC_GAIN1_6
#define ADC_SAADC_RBRIDGE_RSENSE_ADC_GAIN     (1.0f/6.0f)
#define ADC_SAADC_RBRIDGE_RSENSE_ADC_VREF     (0.6f / ADC_SAADC_RBRIDGE_RSENSE_ADC_GAIN)

#define ADC_SAADC_RBRIDGE_RSENSE_GAIN_EXT     (500.0f)
#define ADC_SAADC_RBRIDGE_RSENSE_NOMINAL_V    (4.5f * 845.0f / (100000.0f + 845.0f) * ADC_SAADC_RBRIDGE_RSENSE_GAIN_EXT)
#define ADC_SAADC_RBRIDGE_RSENSE              (10.0f) // resistor value

#define ADC_SAADC_RBRIDGE_VBIAS_ADC_MODE      NRF_SAADC_MODE_DIFFERENTIAL
#define ADC_SAADC_RBRIDGE_VBIAS_ADC_PIN_P     NRF_SAADC_INPUT_AIN0
#define ADC_SAADC_RBRIDGE_VBIAS_ADC_PIN_N     NRF_SAADC_INPUT_AIN1
#define ADC_SAADC_RBRIDGE_VBIAS_ADC_GAIN_REG  NRF_SAADC_GAIN1
#define ADC_SAADC_RBRIDGE_VBIAS_ADC_GAIN      (1.0f)
#define ADC_SAADC_RBRIDGE_VBIAS_ADC_VREF      (0.6f / ADC_SAADC_RBRIDGE_VBIAS_ADC_GAIN)

#define ADC_SAADC_RBRIDGE_VBIAS_NOMINAL_V     (4.5f * 10000.0f / (10000.0f + 10000.0f))
#define ADC_SAADC_RBRIDGE_VBIAS_GAIN_EXT      (1.0f / 2.0f)
#define ADC_SAADC_RBRIDGE_VBIAS_ADC_GAIN      (1.0f)

// Port master
#define OMMO_PM_SPI_PERIPHERALS_IDS       0     // Use peripheral ID separated by commas
#define OMMO_PM_UARTE_PERIPHERALS_IDS     0, 1  // Use peripheral ID separated by commas
#define OMMO_PM_SS_PINS OMMO_ADC_SPI_CS_PIN     // Use pin numbers enclosed in parentheses and separated by commas

#define OMMO_PM_MAX_SPI_BUSSES_PER_PORT 1
#define OMMO_PM_MAX_SS_PINS_PER_BUS     1

#define OMMO_PM_SPI_IRQ_PRIORITY  1 //1 level lower than adc timer
#define OMMO_PM_SPI_BUSSES  ( .miso_pin = OMMO_ADC_SPI_MISO_PIN, .mosi_pin = OMMO_ADC_SPI_MOSI_PIN, .sck_pin = OMMO_ADC_SPI_SCK_PIN, \
                              .frequency = NRF_SPIM_FREQ_16M, .mode = NRF_SPIM_MODE_1 )

#define OMMO_PM_PORTS                                                                                \
    ( .spi_buses = {                                                                                 \
        { .spi_bus_index = 0, .ss_index_list = {0}, .ss_index_count = 1 },                           \
      },                                                                                             \
      .spi_count = 1,                                                                                \
      .flash_media_present = true,                                                                   \
    )

//Timestamp
#define OMMO_TIMESTAMP_TIMER                   NRFX_TIMER_INSTANCE(3) //Timers 3 & 4 have 6 CC registers
#define OMMO_TIMESTAMP_SYNCH_IN
#define OMMO_TIMESTAMP_SAMPLE_PERIOD           15984 //~1000hz, divisible by 3 for basestation and 16 for uart clock
#define OMMO_TIMESTAMP_SYNCH_PERIOD_MULT       4

//comms_uarte
#define OMMO_COMMS_UARTE_SYNCH_PIN            NRF_GPIO_PIN_MAP(0,6)

//comms_esb
#define OMMO_COMMS_ESB_SYNCH_TIMER            NRFX_TIMER_INSTANCE(4) //Timers 3 & 4 have 6 CC registers
#define OMMO_COMMS_ESB_SYNCH_RTC              NRFX_RTC_INSTANCE(0)
#define OMMO_ESB_TRACE_ENABLED

//Ommocomm
#define OMMOCOMM_ENABLED
#define OMMOCOMM_UARTE_IRQ_PRIORITY           3
#define OMMOCOMM_UNUSED_PIN                   NRF_GPIO_PIN_MAP(1,0)

//user_serial.c
#define OMMO_USB_SERIAL_MIN_PACKET_SIZE   64

//user_serial_num.c
#define OMMO_USB_COMMUNICATION_CLASS      COMMUNICATION_CLASS_TRACKING_DEVICE

//adc_monitor.c
//#define OMMO_ADC_MONITOR
#define OMMO_ADC_MONITOR_PERIOD         250 //ms
//#define OMMO_ADC_MONITOR_VIN              NRF_SAADC_INPUT_AIN6
//#define OMMO_ADC_MONITOR_VIN_CUTOFF       ADC_MONITOR_VOLTAGE_TO_ADC(3.0*499/(499 + 1000))
#define OMMO_ADC_MONITOR_USBC_CC1       NRF_SAADC_INPUT_AIN3
#define OMMO_ADC_MONITOR_USBC_CC2       NRF_SAADC_INPUT_AIN2

//#define OMMO_DEBUG_RX                   NRF_GPIO_PIN_MAP(0,12)
//#define OMMO_DEBUG_IO                   NRF_GPIO_PIN_MAP(0,26)

#define OMMO_BUS_OE_UC_PIN              NRF_GPIO_PIN_MAP(0,17)

////ADC Calibration information
//#define ADC_TIMESTAMP_OFFSET    (-ADC_TOTAL_FILTER_DELAY_CLK)

//#define OMMO_DEBUG_ESB_PRI0_ISR_PIN           OMMO_DEBUG_RX
//#define OMMO_DEBUG_ESB_PRI1_ISR_PIN           OMMO_DEBUG_RX
//#define OMMO_DEBUG_ESB_SYNCH_RX_PIN           OMMO_DEBUG_RX

//#define OMMO_DEBUG_ADC_PRI0_ISR_PIN           OMMO_DEBUG_IO
//#define OMMO_DEBUG_ADC_PRI1_ISR_PIN           OMMO_DEBUG_IO

#endif