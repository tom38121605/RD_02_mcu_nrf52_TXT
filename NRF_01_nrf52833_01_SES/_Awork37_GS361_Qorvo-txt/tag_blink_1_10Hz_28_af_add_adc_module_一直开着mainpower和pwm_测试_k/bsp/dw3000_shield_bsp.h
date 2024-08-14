/**
 * @file       custom_board.h.c
 *
 * @brief      Defines for pins, LEDs, buttons etc for the MDEK1001 board
 *
 * @author     Decawave Applications
 *
 * @attention  Copyright 2018 (c) DecaWave Ltd, Dublin, Ireland.
 *             All rights reserved.
 */
#ifndef CUSTOM_BOARD_H
#define CUSTOM_BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nrf_gpio.h"

#define DWM3000_C0_DEVID

#define DWM3000     1
#define C0_SHIELD   2
#define QORVO_TAG   3
#define BOARD_BSP   QORVO_TAG // DWM3000 // C0_SHIELD// DWM3000 // C0_SHIELD  //DWM3000 // C0_SHIELD //DWM3000

#if defined(BOARD_BSP) && (BOARD_BSP == DWM3000)
#define DWIC_RST      24//18
#define DW1000_IRQ    19

// LEDs definitions for MDEK1001
#define LEDS_NUMBER    4
#define LEDS_ACTIVE_STATE 0

#define LED_START      14
#define LED_1          30
#define LED_2          14
#define LED_3          22
#define LED_3          22
#define LED_4          31
#define LED_STOP       31

#define LEDS_LIST { LED_1, LED_2, LED_3, LED_4 }

#define BSP_LED_0      LED_1
#define BSP_LED_1      LED_2
#define BSP_LED_2      LED_3
#define BSP_LED_3      LED_4

//#define BSP_LED_0_MASK (1UL<<BSP_LED_0)
//#define BSP_LED_1_MASK (1UL<<BSP_LED_1)
//#define BSP_LED_2_MASK (1UL<<BSP_LED_2)
//#define BSP_LED_3_MASK (1UL<<BSP_LED_3)

//#define LEDS_MASK      (BSP_LED_0_MASK | BSP_LED_1_MASK | BSP_LED_2_MASK | BSP_LED_3_MASK)

#define LPS22HB_BR_CS_Pin     11

#define SPI_CS_PIN   17 /**< SPI CS Pin.*/

#define LSM6DSL_I2C_ADDR      0b1101011
#define LIS2MDL_I2C_ADDR      0b0011110
#define LPS22HB_I2C_ADDR      0b1011100

/* all LEDs are lit when GPIO is low */
#define LEDS_INV_MASK  LEDS_MASK

#define BUTTONS_NUMBER 1
#define BUTTONS_ACTIVE_STATE 0

#define BUTTON_START   2
#define BUTTON_1       2
#define BUTTON_STOP    2
#define BUTTON_PULL    NRF_GPIO_PIN_PULLUP

#define BUTTONS_LIST { BUTTON_1,  }

#define BSP_BUTTON_0   BUTTON_1
#define BSP_BUTTON_1   BUTTON_2
#define BSP_BUTTON_2   BUTTON_3
#define BSP_BUTTON_3   BUTTON_4

//#define BSP_BUTTON_0_MASK (1<<BSP_BUTTON_0)
//#define BSP_BUTTON_1_MASK (1<<BSP_BUTTON_1)
//#define BSP_BUTTON_2_MASK (1<<BSP_BUTTON_2)
//#define BSP_BUTTON_3_MASK (1<<BSP_BUTTON_3)

//#define BUTTONS_MASK   0x001E0000

#define RX_PIN_NUMBER  11
#define TX_PIN_NUMBER  5
#define CTS_PIN_NUMBER 7
#define RTS_PIN_NUMBER 6

#define READY_PIN       25

#define HWFC           true     

#define SPI0_CONFIG_SCK_PIN         16
#define SPI0_CONFIG_MOSI_PIN        20
#define SPI0_CONFIG_MISO_PIN        18
#define SPI0_CONFIG_IRQ_PRIORITY    APP_IRQ_PRIORITY_LOW

#define SPIS_MISO_PIN   28  // SPI MISO signal.
#define SPIS_CSN_PIN    12  // SPI CSN signal.
#define SPIS_MOSI_PIN   25  // SPI MOSI signal.
#define SPIS_SCK_PIN    29  // SPI SCK signal.

#define SPIM0_SCK_PIN   29  // SPI clock GPIO pin number.
#define SPIM0_MOSI_PIN  25  // SPI Master Out Slave In GPIO pin number.
#define SPIM0_MISO_PIN  28  // SPI Master In Slave Out GPIO pin number.
#define SPIM0_SS_PIN    12  // SPI Slave Select GPIO pin number.

#define SPIM1_SCK_PIN   2   // SPI clock GPIO pin number.
#define SPIM1_MOSI_PIN  3   // SPI Master Out Slave In GPIO pin number.
#define SPIM1_MISO_PIN  4   // SPI Master In Slave Out GPIO pin number.
#define SPIM1_SS_PIN    5   // SPI Slave Select GPIO pin number.

#define SPIM2_SCK_PIN   12  // SPI clock GPIO pin number.
#define SPIM2_MOSI_PIN  13  // SPI Master Out Slave In GPIO pin number.
#define SPIM2_MISO_PIN  14  // SPI Master In Slave Out GPIO pin number.
#define SPIM2_SS_PIN    15  // SPI Slave Select GPIO pin number.

// serialization APPLICATION board - temp. setup for running serialized MEMU tests
#define SER_APP_RX_PIN              23    // UART RX pin number.
#define SER_APP_TX_PIN              24    // UART TX pin number.
#define SER_APP_CTS_PIN             2     // UART Clear To Send pin number.
#define SER_APP_RTS_PIN             25    // UART Request To Send pin number.

#define SER_APP_SPIM0_SCK_PIN       27     // SPI clock GPIO pin number.
#define SER_APP_SPIM0_MOSI_PIN      2      // SPI Master Out Slave In GPIO pin number
#define SER_APP_SPIM0_MISO_PIN      26     // SPI Master In Slave Out GPIO pin number
#define SER_APP_SPIM0_SS_PIN        23     // SPI Slave Select GPIO pin number
#define SER_APP_SPIM0_RDY_PIN       25     // SPI READY GPIO pin number
#define SER_APP_SPIM0_REQ_PIN       24     // SPI REQUEST GPIO pin number

// serialization CONNECTIVITY board
#define SER_CON_RX_PIN              24    // UART RX pin number.
#define SER_CON_TX_PIN              23    // UART TX pin number.
#define SER_CON_CTS_PIN             25    // UART Clear To Send pin number. Not used if HWFC is set to false.
#define SER_CON_RTS_PIN             2     // UART Request To Send pin number. Not used if HWFC is set to false.


#define SER_CON_SPIS_SCK_PIN        27    // SPI SCK signal.
#define SER_CON_SPIS_MOSI_PIN       2     // SPI MOSI signal.
#define SER_CON_SPIS_MISO_PIN       26    // SPI MISO signal.
#define SER_CON_SPIS_CSN_PIN        23    // SPI CSN signal.
#define SER_CON_SPIS_RDY_PIN        25    // SPI READY GPIO pin number.
#define SER_CON_SPIS_REQ_PIN        24    // SPI REQUEST GPIO pin number.

#define SER_CONN_CHIP_RESET_PIN     11    // Pin used to reset connectivity chip

// Arduino board mappings
#define ARDUINO_SCL_PIN             27    // SCL signal pin
#define ARDUINO_SDA_PIN             26    // SDA signal pin
#define ARDUINO_AREF_PIN            2     // Aref pin
#define ARDUINO_13_PIN              25    // Digital pin 13
#define ARDUINO_12_PIN              24    // Digital pin 12
#define ARDUINO_11_PIN              23    // Digital pin 11
#define ARDUINO_10_PIN              22    // Digital pin 10
#define ARDUINO_9_PIN               20    // Digital pin 9
#define ARDUINO_8_PIN               19    // Digital pin 8

#define ARDUINO_7_PIN               18    // Digital pin 7
#define ARDUINO_6_PIN               17    // Digital pin 6
#define ARDUINO_5_PIN               16    // Digital pin 5
#define ARDUINO_4_PIN               15    // Digital pin 4
#define ARDUINO_3_PIN               14    // Digital pin 3
#define ARDUINO_2_PIN               13    // Digital pin 2
#define ARDUINO_1_PIN               12    // Digital pin 1
#define ARDUINO_0_PIN               11    // Digital pin 0

#define ARDUINO_A0_PIN              3     // Analog channel 0
#define ARDUINO_A1_PIN              4     // Analog channel 1
#define ARDUINO_A2_PIN              28    // Analog channel 2
#define ARDUINO_A3_PIN              29    // Analog channel 3
#define ARDUINO_A4_PIN              30    // Analog channel 4
#define ARDUINO_A5_PIN              31    // Analog channel 5

// Low frequency clock source to be used by the SoftDevice
#define NRF_CLOCK_LFCLKSRC      {.source        = NRF_CLOCK_LF_SRC_XTAL,            \
                                 .rc_ctiv       = 0,                                \
                                 .rc_temp_ctiv  = 0,                                \
                                 .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM}

#endif// DWM3000

#if defined(BOARD_BSP) && (BOARD_BSP == C0_SHIELD)
/* ENABLE_USB_PRINT Macro is uncommented then Segger RTT Print will be enabled*/
#define ENABLE_USB_PRINT

#define BOARD_PCA10056 // BOARD_PCA10056 //BOARD_PCA10040

#if defined(BOARD_PCA10040)
//#define NRF52832_XXAA
#include "pca10040.h"
#elif defined(BOARD_PCA10056)
//#define NRF52840_XXAA
#include "pca10056.h"
#endif

#define DW3000_RST_Pin      ARDUINO_7_PIN
#define DW3000_IRQ_Pin      ARDUINO_8_PIN
#define DW3000_WUP_Pin      ARDUINO_9_PIN

// SPI defs
#define DW3000_CS_Pin       ARDUINO_10_PIN
#define DW3000_CLK_Pin      ARDUINO_13_PIN  // DWM3000 shield SPIM1 sck connected to DW1000
#define DW3000_MOSI_Pin     ARDUINO_11_PIN  // DWM3000 shield SPIM1 mosi connected to DW1000
#define DW3000_MISO_Pin     ARDUINO_12_PIN  // DWM3000 shield SPIM1 miso connected to DW1000
#define DW3000_SPI_IRQ_PRIORITY APP_IRQ_PRIORITY_LOW // 

// UART symbolic constants
#define UART_0_TX_PIN       TX_PIN_NUMBER           // DWM1001 module pin 20, DEV board name RXD
#define UART_0_RX_PIN       RX_PIN_NUMBER           // DWM1001 module pin 18, DEV board name TXD
#define DW3000_RTS_PIN_NUM      UART_PIN_DISCONNECTED
#define DW3000_CTS_PIN_NUM      UART_PIN_DISCONNECTED

// compatibility definitions for software packages, naming convension issue
#define DW1000_RST            DW3000_RST_Pin 
#define DW1000_IRQ            DW3000_IRQ_Pin
//#define DW3000_WUP_Pin 
#define SPI_CS_PIN            DW3000_CS_Pin
#define SPI0_CONFIG_SCK_PIN   DW3000_CLK_Pin      
#define SPI0_CONFIG_MOSI_PIN  DW3000_MOSI_Pin     
#define SPI0_CONFIG_MISO_PIN  DW3000_MISO_Pin    
#endif// C0_SHIELD

#if defined(BOARD_BSP) && (BOARD_BSP == QORVO_TAG)

/* ENABLE_USB_PRINT Macro is uncommented then Segger RTT Print will be enabled*/
#define ENABLE_USB_PRINT

//#include "pca10100.h"

#define DW3000_RST_Pin        NRF_GPIO_PIN_MAP(1,9)
#define DW3000_IRQ_Pin        NRF_GPIO_PIN_MAP(0,15)
#define DW3000_WUP_Pin        UART_PIN_DISCONNECTED// gnd 

// SPI defs
#define DW3000_CS_Pin         NRF_GPIO_PIN_MAP(0,17)
#define DW3000_CLK_Pin        NRF_GPIO_PIN_MAP(0,2)  // DWM3000 shield SPIM1 sck connected to DW1000
#define DW3000_MOSI_Pin       NRF_GPIO_PIN_MAP(0,20)  // DWM3000 shield SPIM1 mosi connected to DW1000
#define DW3000_MISO_Pin       NRF_GPIO_PIN_MAP(0,3)  // DWM3000 shield SPIM1 miso connected to DW1000
#define DW3000_SPI_IRQ_PRIORITY APP_IRQ_PRIORITY_LOW // 

// UART symbolic constants
#define UART_0_TX_PIN         NRF_GPIO_PIN_MAP(0,5)           // DWM1001 module pin 20, DEV board name RXD
#define UART_0_RX_PIN         NRF_GPIO_PIN_MAP(0,11)           // DWM1001 module pin 18, DEV board name TXD
#define DW3000_RTS_PIN_NUM    UART_PIN_DISCONNECTED
#define DW3000_CTS_PIN_NUM    UART_PIN_DISCONNECTED

// I2C defs
#define QT_I2C_SCL            NRF_GPIO_PIN_MAP(0,28)
#define QT_I2C_SDA            NRF_GPIO_PIN_MAP(0,29)

// GPIO defs
#define IMU_IRQ               NRF_GPIO_PIN_MAP(0,4) 

#define BOOST_EN_PIN          NRF_GPIO_PIN_MAP(0,9) 
#define DW_PWRDN_PIN          NRF_GPIO_PIN_MAP(0,10) 

#define ADC1_PIN              NRF_GPIO_PIN_MAP(0,31) 
#define ADC2_PIN              NRF_GPIO_PIN_MAP(0,30) 

#define PWM0_PIN              NRF_GPIO_PIN_MAP(0,18) 

// compatibility definitions for software packages, naming convension issue
#define DWIC_RST              DW3000_RST_Pin 
#define DW1000_IRQ            DW3000_IRQ_Pin


//#define DW3000_WUP_Pin 
#define SPI_CS_PIN            DW3000_CS_Pin
#define SPI0_CONFIG_SCK_PIN   DW3000_CLK_Pin      
#define SPI0_CONFIG_MOSI_PIN  DW3000_MOSI_Pin     
#define SPI0_CONFIG_MISO_PIN  DW3000_MISO_Pin   
#endif// C0_SHIELD


#ifdef __cplusplus
}
#endif

#endif // CUSTOM_BOARD_H
