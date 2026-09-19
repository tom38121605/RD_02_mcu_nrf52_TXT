#pragma once

// Minimize dependencies to permit use in lowest-level libraries

#include <string.h>

#include "app_util.h"
#include "app_util_platform.h"
#include "nrf_assert.h"

#include "ommo_config.h"

#ifdef __cplusplus
extern "C" {
#endif

// Tracing is unconditionally enabled for ISRs.  The trace log is saved with a crash report.
// ISR tracing may be disabled during development by defining OMMO_ISR_TRACE_DISABLED.
#ifndef OMMO_BOOTLOADER_PROJECT
#define OMMO_TRACE_ENABLED
#endif

#ifdef OMMO_ESB_BS_TRACE_ENABLED
#define OMMO_ESB_TRACE_ENABLED
#endif

#ifdef OMMO_TRACE_ENABLED
#define TRACE(                 TRACE_TYPE, LOCATION)                                     trace          (TRACE_TYPE, LOCATION,             0,              0,              0,              0)
#define TRACE_PRI0_WITH_ARG(   TRACE_TYPE, LOCATION, ARG)                                trace_pri0     (TRACE_TYPE, LOCATION,             (size_t)(ARG),  0,              0,              0)
#define TRACE_WITH_ARG(        TRACE_TYPE, LOCATION, ARG)                                trace          (TRACE_TYPE, LOCATION,             (size_t)(ARG),  0,              0,              0)
#define TRACE_WITH_ARGS_2(     TRACE_TYPE, LOCATION, ARG1, ARG2)                         trace          (TRACE_TYPE, LOCATION,             (size_t)(ARG1), (size_t)(ARG2), 0,              0)
#define TRACE_WITH_ARGS_3(     TRACE_TYPE, LOCATION, ARG1, ARG2, ARG3)                   trace          (TRACE_TYPE, LOCATION,             (size_t)(ARG1), (size_t)(ARG2), (size_t)(ARG3), 0)
#define TRACE_WITH_ARGS_4(     TRACE_TYPE, LOCATION,             ARG1, ARG2, ARG3, ARG4) trace          (TRACE_TYPE, LOCATION,             (size_t)(ARG1), (size_t)(ARG2), (size_t)(ARG3), (size_t)(ARG4))
#define TRACE_WITH_TIME(       TRACE_TYPE, LOCATION, TRACE_TIME)                         trace_with_time(TRACE_TYPE, LOCATION, TRACE_TIME, 0,              0,              0,              0)
#define TRACE_WITH_TIME_ARG(   TRACE_TYPE, LOCATION, TRACE_TIME, ARG1)                   trace_with_time(TRACE_TYPE, LOCATION, TRACE_TIME, (size_t)(ARG1), 0,              0,              0)
#define TRACE_WITH_TIME_ARGS_2(TRACE_TYPE, LOCATION, TRACE_TIME, ARG1, ARG2)             trace_with_time(TRACE_TYPE, LOCATION, TRACE_TIME, (size_t)(ARG1), (size_t)(ARG2), 0,              0)
#define TRACE_WITH_TIME_ARGS_3(TRACE_TYPE, LOCATION, TRACE_TIME, ARG1, ARG2, ARG3)       trace_with_time(TRACE_TYPE, LOCATION, TRACE_TIME, (size_t)(ARG1), (size_t)(ARG2), (size_t)(ARG3), 0)
#else
#define TRACE(TRACE_TYPE, LOCATION) do {} while (0)
#define TRACE_PRI0_WITH_ARG(TRACE_TYPE, LOCATION, ARG) do {} while (0)
#define TRACE_WITH_ARG(TRACE_TYPE, LOCATION, ARG) do {} while (0)
#define TRACE_WITH_ARGS_2(TRACE_TYPE, LOCATION, ARG1, ARG2) do {} while (0)
#define TRACE_WITH_ARGS_3(TRACE_TYPE, LOCATION, ARG1, ARG2, ARG3) do {} while (0)
#define TRACE_WITH_ARGS_4(TRACE_TYPE, LOCATION, ARG1, ARG2, ARG3) do {} while (0)
#define TRACE_WITH_TIME(TRACE_TYPE, LOCATION, TRACE_TIME) do {} while (0)
#define TRACE_WITH_TIME_ARG(   TRACE_TYPE, LOCATION, TRACE_TIME, ARG1) do {} while (0)
#define TRACE_WITH_TIME_ARGS_2(TRACE_TYPE, LOCATION, TRACE_TIME, ARG1, ARG2) do {} while (0)
#define TRACE_WITH_TIME_ARGS_3(TRACE_TYPE, LOCATION, TRACE_TIME, ARG1, ARG2, ARG3) do {} while (0)
#endif

// Note that changing this requires a power cycle, because the log state is not cleared at boot
// On a base station, ISR + ESB tracing is <1000 entries per 4ms synch period
// On a MSIU in either DC or data mode, <300 per 4ms synch period
// To use all remaining RAM for tracing:
// - check that the stack size is correct
// - in flash_placement.xml, for the .heap section, change "place_from_segment_end" to "Yes"
// - in ommo_config.h, #define TRACE_COUNT (define it to have no value)
#ifndef TRACE_COUNT
#ifdef NRF52820_XXAA
#define TRACE_COUNT 500
#else
#define TRACE_COUNT 5000
#endif
#endif

#define MAX_TRACE_RECORD_SIZE 40
#define TRACE_LENGTH_BITS 6
STATIC_ASSERT((1 << TRACE_LENGTH_BITS) >= MAX_TRACE_RECORD_SIZE);

#define TRACE_TYPE_SHIFT 24
#define TRACE_LOCATION_SHIFT 16

// Note that DebugCodesToStrings parses enums in this file of this form:
//  typedef enum {
//    NAME = 0x123,
//  } foo;

typedef enum {
    TRACE_LOCATION_NONE                                      = 0x00,

    // ommo_esb.c
    TRACE_OMMO_ESB_CHANGE_STATE                              = 0x01,
    TRACE_OMMO_ESB_RX_FAILED                                 = 0x02,
    // unused TRACE_OMMO_ESB_RADIO_ISR_EXIT                            = 0x03,
    TRACE_OMMO_ESB_CALLING_ESB_CALLBACK                      = 0x04,
    // unused TRACE_OMMO_ESB_RX_STOP_ISR                               = 0x05,
    TRACE_OMMO_ESB_TRANSMIT_CALLED                           = 0x06,
    TRACE_OMMO_ESB_RECEIVE_CALLED                            = 0x07,
    // unused TRACE_OMMO_ESB_EVT_ISR                                   = 0x08,

    // comms_esb_basestation.cpp
    COMMS_ESB_BS_SET_SYNCH_CHANNEL                           = 0x10,
    COMMS_ESB_BS_PAGING_SIU                                  = 0x11,
    COMMS_ESB_BS_LISTENING_TX_SUCCESS                        = 0x12,
    COMMS_ESB_BS_LISTENING_RX_FAILURE                        = 0x13,
    COMMS_ESB_BS_LISTENING_RX_SUCCESS                        = 0x14,
    COMMS_ESB_BS_ENTERING_DC_STATE                           = 0x15,
    COMMS_ESB_BS_DC_SYNCH_TX_DATA_TX                         = 0x16,
    COMMS_ESB_BS_DC_TX_SUCCESS                               = 0x17,
    COMMS_ESB_BS_DC_SYNCH_TX_DATA_RX                         = 0x18,
    COMMS_ESB_BS_DC_RX_FAILURE                               = 0x19,
    COMMS_ESB_BS_DC_RX_SUCCESS                               = 0x1A,
    COMMS_ESB_BS_CHANNEL_TEST_RX_FAILURE                     = 0x1B,
    COMMS_ESB_BS_CHANNEL_TEST_RX_SUCCESS                     = 0x1C,
    COMMS_ESB_BS_RSSI_SIMPLE_FINISHED                        = 0x1D,
    COMMS_ESB_BS_RSSI_DEEP_FINISHED                          = 0x1E,
    COMMS_ESB_BS_DC_LOST                                     = 0x1F,
    COMMS_ESB_BS_ENTERING_LISTEN_STATE                       = 0x20,
    COMMS_ESB_BS_DC_STARTING_DATA_TX                         = 0x21,
    COMMS_ESB_BS_DC_STARTING_DATA_RX                         = 0x22,
    COMMS_ESB_BS_SYNCH_VALUE                                 = 0x23,
    COMMS_ESB_BS_DISCONNECT                                  = 0x24,
    COMMS_ESB_BS_PACKET_SEND_REQUESTED                       = 0x25,
    COMMS_ESB_BS_DC_RX_OLD_PACKET                            = 0x26,
    COMMS_ESB_BS_DC_BS_TO_SIU                                = 0x27,
    COMMS_ESB_BS_DC_SIU_TO_BS                                = 0x28,
    COMMS_ESB_BS_DC_IDLE                                     = 0x29,
    COMMS_ESB_BS_DC_BAD_PACKET                               = 0x2A,
    COMMS_ESB_BS_DC_START_BS_TO_SIU                          = 0x2B,
    COMMS_ESB_BS_DC_RELAY_PACKET_BUSY                        = 0x2C,
    COMMS_ESB_BS_PAGING_ENDED                                = 0x2D,

    // comms_esb_siu_wireless.cpp
    COMMS_ESB_SIU_DC_BEGIN_SYNCH_STATE                       = 0x30,
    COMMS_ESB_SIU_DC_SCAN_NEXT_CHANNEL                       = 0x31,
    COMMS_ESB_SIU_WATCH_RX_FAILURE                           = 0x32,
    COMMS_ESB_SIU_WATCH_RX_SUCCESS                           = 0x33,
    // unused COMMS_ESB_SIU_DC_TX_SUCCESS                              = 0x34,
    COMMS_ESB_SIU_DC_PAGE_SCAN_RX_FAILURE                    = 0x35,
    COMMS_ESB_SIU_DC_DATA_RX_FAILURE                         = 0x36,
    // unused COMMS_ESB_SIU_DC_SYNCH_RX_SUCCESS                        = 0x37,
    COMMS_ESB_SIU_SET_SYNCH_CHANNEL                          = 0x38,
    COMMS_ESB_SIU_DC_DATA_RX_SUCCESS                         = 0x39,
    COMMS_ESB_SIU_DC_STARTING_DATA_TX                        = 0x3A,
    COMMS_ESB_SIU_ESB_DC_ENTERING                            = 0x3B,
    COMMS_ESB_SIU_DC_SYNCH_RX_RECEIVED                       = 0x3C,
    COMMS_ESB_SIU_SD_ENTERING                                = 0x3D,
    COMMS_ESB_SIU_SD_TX_SUCCESS                              = 0x3E,
    COMMS_ESB_SIU_SD_RX_SUCCESS                              = 0x3F,
    COMMS_ESB_SIU_SD_RX_FAILURE                              = 0x40,
    COMMS_ESB_SIU_SD_GOOD_ID_FAILURE                         = 0x41,
    COMMS_ESB_SIU_SYNCH_VALUE                                = 0x42,
    COMMS_ESB_SIU_IDLE_ENTERING                              = 0x43,
    COMMS_ESB_SIU_ESB_ONLY_ENTERING                          = 0x44,
    COMMS_ESB_SIU_ESB_ONLY_SYNCH_RX_FAILURE                  = 0x45,
    COMMS_ESB_SIU_ESB_ONLY_SYNCH_RX_SUCCESS                  = 0x46,
    COMMS_ESB_SIU_ESB_ONLY_STARTING_SYNCH_RX                 = 0x47,
    COMMS_ESB_SIU_DC_UNKNOWN_PACKET                          = 0x48,
    COMMS_ESB_SIU_DC_PAGE_SCAN_RX_SUCCESS                    = 0x49,
    // unused COMMS_ESB_SIU_DC_SKIP_DATA_TX                            = 0x4A,
    COMMS_ESB_SIU_ENTERING_SCAN                              = 0x4B,
    // unused COMMS_ESB_SIU_SYNCH_ONLY_STATE                           = 0x4C,
    COMMS_ESB_SIU_DC_SCAN_STATE                              = 0x4D,
    COMMS_ESB_SIU_DC_SYNCH_RX_FAILURE                        = 0x4E,
    COMMS_ESB_SIU_DC_INITIATE_WATCH_STATE                    = 0x4F,
    COMMS_ESB_SIU_DC_WATCH_WAITING_FOR_DATA_RX_TO_SYNCH_RX   = 0x50,
    COMMS_ESB_SIU_DC_WATCH_WAITING_FOR_SYNCH_RX_TO_DATA_RX   = 0x51,
    COMMS_ESB_SIU_DC_STATE_SYNCH_RX_TO_DATA_TX               = 0x52,
    COMMS_ESB_SIU_DC_STATE_SYNCH_RX_TO_DATA_RX               = 0x53,
    COMMS_ESB_SIU_DC_STATE_DATA_TX_TO_SYNCH_RX               = 0x54,
    COMMS_ESB_SIU_DC_STATE_DATA_RX_TO_SYNCH_RX               = 0x55,
    COMMS_ESB_SIU_SD_STATE_SYNCH_RX_TO_DATA_TX               = 0x56,
    COMMS_ESB_SIU_SD_STATE_DATA_TX_TO_DATA_TX                = 0x57,
    COMMS_ESB_SIU_SD_STATE_DATA_TX_TO_SYNCH_RX               = 0x58,
    // unused COMMS_ESB_SIU_WAITING_FOR_DC_ACK_FAILURE                 = 0x59,
    COMMS_ESB_SIU_TEMPORARY_DISCONNECT                       = 0x5A,
    COMMS_ESB_SIU_TEMPORARY_DISCONNECT_COMPLETE              = 0x5B,
    COMMS_ESB_SIU_TEMPORARY_DISCONNECT_ABORTED               = 0x5C,
    COMMS_ESB_SIU_RECONNECT                                  = 0x5D,
    COMMS_ESB_SIU_RECONNECT_COMPLETE                         = 0x5E,
    COMMS_ESB_SIU_RECONNECT_TIMEOUT                          = 0x5F,
    COMMS_ESB_MAIN_RELAY_PACKET                              = 0x60,
    COMMS_ESB_MAIN_RELAY_PACKET_BUSY                         = 0x61,
    COMMS_ESB_SIU_DC_RECONNECT_SYNCH_STATE                   = 0x62,
    COMMS_ESB_SIU_DC_RECONNECT_INITIATE_WATCH_STATE          = 0x63,
    COMMS_ESB_SIU_DC_RECONNECT_DATA_RX_TO_SYNCH_RX           = 0x64,
    COMMS_ESB_SIU_DC_RECONNECT_SYNCH_RX_TO_DATA_RX           = 0x65,

    // Misc
    TRACE_LED_DRIVER_SET                                     = 0x73,
    FAULT_HANDLER                                            = 0x74,

    // comms_usb_serial.cpp
    TRACE_USB_SERIAL_WRITE                                   = 0x80,
    TRACE_USB_SERIAL_WRITE_COMPLETE                          = 0x81,
    TRACE_USB_SERIAL_READ                                    = 0x82,
    TRACE_USB_SERIAL_READ_COMPLETE                           = 0x83,
    TRACE_USB_STATE_CHANGE                                   = 0x84,

    // ommo_1wire_adapter/main.cpp
    TRACE_1WIRE_ADAPTER_READ                                 = 0xA0,
    TRACE_1WIRE_ADAPTER_READ_COMPLETE                        = 0xA1,
    TRACE_1WIRE_ADAPTER_WRITE                                = 0xA2,
    TRACE_1WIRE_ADAPTER_WRITE_COMPLETE                       = 0xA3,
    TRACE_1WIRE_ADAPTER_CONNECTION_STATE                     = 0xA4,

    // comms_uarte_siu_1wire.cpp and comms_uarte_master_1wire.cpp
    TRACE_1WIRE_TX                                           = 0xA5,
    TRACE_1WIRE_RX                                           = 0xA6,
    TRACE_1WIRE_RX_DONE                                      = 0xA7,
    TRACE_1WIRE_TX_DONE                                      = 0xA8,
    TRACE_1WIRE_ERROR                                        = 0xA9,
    TRACE_1WIRE_TIMEOUT                                      = 0xAA,
    TRACE_1WIRE_SW_WATCHDOG                                  = 0xAB,
    TRACE_1WIRE_ERROR_RECOVERY                               = 0xAC,
    TRACE_1WIRE_ERROR_RECOVERY_RX                            = 0xAD,
    TRACE_1WIRE_ERROR_RECOVERY_TX                            = 0xAE,
    TRACE_1WIRE_INJECTED_BAD_TIMESTAMP                       = 0xAF,
    TRACE_1WIRE_DISCONNECTED_BAD_TIMESTAMP                   = 0xB0,
    TRACE_1WIRE_DISCONNECTED_MASTER_INITIATED_HANDSHAKE      = 0xB1,
    TRACE_1WIRE_DISCONNECTED_SLAVE_INITIATING_HANDSHAKE      = 0xB2,
    TRACE_1WIRE_DISCONNECTED_RX_CALLBACK_ERROR               = 0xB3,
    TRACE_1WIRE_DISCONNECTED_RX_CANCELLED                    = 0xB4,
    TRACE_1WIRE_DISCONNECTED_RX_TIMEOUT                      = 0xB5,
    TRACE_1WIRE_DISCONNECTED_RX_ERROR                        = 0xB6,
    TRACE_1WIRE_DISCONNECTED_INITIAL_STATE                   = 0xB7,
    TRACE_1WIRE_DISCONNECTED_ERROR_RECOVERY                  = 0xB8,
    TRACE_1WIRE_CONNECTED                                    = 0xB9,

    // All location identifiers starting from 0xC0 are used in production software and MUST
    // remain stable for interpreting crash reports.

    TRACE_LOCATION_ISR_ENTER                                 = 0xC0,  // argument is trace_isr_type, with optional callback address
    TRACE_LOCATION_ISR_EXIT                                  = 0xC1,  // argument is trace_isr_type, with optional callback address
    TRACE_LOCATION_ISR_ACTION                                = 0xC2,  // argument is trace_isr_type, with optional callback address
    TRACE_LOCATION_EVENT_QUEUE_PROCESSING                    = 0xC3,  // event_queue_manager::execute_once, with callback address
} trace_location;

typedef enum {
    TRACE_TYPE_NONE                         = 0x00,
    TRACE_TYPE_TIME24_U26_U16_U8            = 0x01,
    TRACE_TYPE_TIME_DATA                    = 0x02,
    TRACE_TYPE_U26_U16                      = 0x03,
    TRACE_TYPE_TIME24_U32_U26_U16_U8        = 0x04,
    TRACE_TYPE_DATA_U26                     = 0x05,
    TRACE_TYPE_TIME_U26_U16                 = 0x06,
    TRACE_TYPE_TIME_FLASHADDR_U16           = 0x07,  // same encoding as U26_U16, but U26 is a flash address (up to 64M)
    TRACE_TYPE_TIME_U8                      = 0x08,
    TRACE_TYPE_TIME_FLASHADDR_U26_U16       = 0x09,
    TRACE_TYPE_NOT_WRAPPED                  = 0x0A,  // indicator that buffer has not wrapped yet
    TRACE_TYPE_U32_U26_U16                  = 0x0B,
    TRACE_TYPE_U10                          = 0x0C,

    // The following trace types use encodings above but describe how to interpret the trace args

#ifdef OMMO_ESB_TRACE_ENABLED
    TRACE_TYPE_ESB                          = 0x55,  // encoded as TRACE_TYPE_TIME24_U26_U16_U8
    TRACE_TYPE_ESB_DATA                     = 0x59,  // encoded as TRACE_TYPE_DATA_U26
#endif
#ifdef OMMO_ESB_BS_TRACE_ENABLED
    TRACE_TYPE_ESB_BS_DC                    = 0x58,  // encoded as TRACE_TIME24_U32_U26_U16_U8
#endif
#ifdef OMMO_1WIRE_ADAPTER_TRACE_ENABLED
    TRACE_TYPE_1WIRE                        = 0x5B,  // encoded as TRACE_TYPE_U26_U16
    TRACE_TYPE_1WIRE_SIU_DATA               = 0x5C,  // encoded as TRACE_TYPE_DATA_U26
    TRACE_TYPE_1WIRE_CDC_DATA               = 0x5D,  // encoded as TRACE_TYPE_DATA_U26
#endif

    // All trace types starting from 0xC0 are used in production software and MUST
    // remain stable for interpreting crash reports.

    TRACE_TYPE_ISR_ACTION                   = 0xC0,  // encoded as TRACE_TYPE_TIME_U8
    TRACE_TYPE_ISR_ACTION_ARG               = 0xC1,  // encoded as TRACE_TYPE_TIME_U26_U16 (U26 is the arg)
    TRACE_TYPE_ISR_ACTION_CALLBACK          = 0xC2,  // encoded as TRACE_TYPE_TIME_FLASHADDR_U16
    TRACE_TYPE_ISR_ACTION_CALLBACK_ARG      = 0xC3,  // encoded as TRACE_TYPE_TIME_FLASHADDR_U26_U16 (U26 is the arg)
} trace_type;

typedef enum {

    // All ISR identifiers starting from 0xC0 are used in production software and MUST 
    // remain stable for interpreting crash reports.
    //
    // The term "IRQ" refers to a function that is referenced directly by the vector table.
    // The term "ISR" refers to either an IRQ handler or a function that is called indirectly from
    // an IRQ handler.  In other words, from the perspective of a software module, it's a function
    // that is triggered somehow by an interrupt.
    //
    // ISRs are instrumented in three different ways:
    // - enter/exit events in the nRF IRQ handler, without specifying the final Ommo handler
    //   implementation
    // - enter/exit events in the nRF IRQ handler, with additional "action only" events that
    //   report the final Ommo handler implementation
    // - enter/exit events in an Ommo peripheral layer, specifying the final Ommo handler
    //   implementation
    //
    // Examples:
    // - Some peripherals (ex. timers) are assigned statically and used directly by a module, so
    //   the nRF IRQ routines directly call a known Ommo implementation.  No "action only" events
    //   are needed to clarify which Ommo functions handle the interrupt.  For performance reasons,
    //   the Ommo callback may be known at the ISR entry point but is not logged.  Interpreting the
    //   trace log requires understanding how that peripheral is used.
    // - Some peripherals (ex. app timer) uses a single peripheral instance/IRQ that can delegate
    //   to multiple callbacks.  The lowest level IRQ handlers simply report enter/exit events,
    //   and the IRQ uses "action only" events as callbacks are invoked.
    // - UARTE, SPI, and I2C are expected to be used via comm_device and port_master APIs.
    //   These are instrumented in the comm_device implementation to report the handler
    //   in the entry/exit events.  These events will not account for the time between the nRF IRQ
    //   handler and the comm_device implementation.

    TRACE_ISR_ESB                           = 0xC0,  // RADIO_IRQHandler
    TRACE_ISR_SWI                           = 0xC1,  // egu_irq_handler
    TRACE_ISR_WIRELESS                      = 0xC2,  // comms_esb_event_handler (is an ISR from the perspective of the wireless module, called via TRACE_ISR_ESB_SW)
    TRACE_ISR_POWER                         = 0xC3,  // nrfx_power_irq_handler
    TRACE_ISR_UARTE_OMMOCOMM                = 0xC4,  // ommocomm_uarte::static_uarte_handler (via nrfx_uarte_N_irq_handler)
    TRACE_ISR_UARTE_1WIRE                   = 0xC5,  // comms_uarte_siu_1wire::comms_uarte_handler (via nrfx_uarte_N_irq_handler)
    TRACE_ISR_SPI_BASIC                     = 0xC6,  // spim_basic::spim_handler (via nrfx_spim_N_irq_handler, delegates to callback)
    TRACE_ISR_TWI_BASIC                     = 0xC7,  // twi_basic::twim_event_handler (via nrfx_twim_N_irq_handler, delegates to callback)
    TRACE_ISR_ADC                           = 0xC8,  // adc_monitor_adc_callback (via nrfx_saadc_irq_handler)
    TRACE_ISR_TIMER0                        = 0xC9,  // nrfx_timer_0_irq_handler
    TRACE_ISR_TIMER1                        = 0xCA,  // nrfx_timer_1_irq_handler
    TRACE_ISR_TIMER2                        = 0xCB,  // nrfx_timer_2_irq_handler
    TRACE_ISR_TIMER3                        = 0xCC,  // nrfx_timer_3_irq_handler
    TRACE_ISR_TIMER4                        = 0xCD,  // nrfx_timer_4_irq_handler
    TRACE_ISR_RTC0                          = 0xCE,  // nrfx_rtc_0_irq_handler
    TRACE_ISR_RTC1                          = 0xCF,  // nrfx_rtc_1_irq_handler
    TRACE_ISR_RTC2                          = 0xD0,  // nrfx_rtc_2_irq_handler
    TRACE_ISR_APP_TIMER_RTC0                = 0xD1,  // drv_rtc_rtc_0_irq_handler
    TRACE_ISR_APP_TIMER_RTC1                = 0xD2,  // drv_rtc_rtc_1_irq_handler
    TRACE_ISR_APP_TIMER_RTC2                = 0xD3,  // drv_rtc_rtc_2_irq_handler
    TRACE_ISR_APP_TIMER                     = 0xD4,  // action only, timer_expire in app_timer2.c
    TRACE_ISR_RTC_TIMEOUT                   = 0xD5,  // action only, rtc_timer_handler (via irq_handler in nrfx_rtc.c, delegates to callback)
    TRACE_ISR_USB                           = 0xD6,  // USBD_IRQHandler
    TRACE_ISR_GPIOTE                        = 0xD7,  // nrfx_gpiote_irq_handler
    TRACE_ISR_LPCOMP                        = 0xD8,  // nrfx_comp_irq_handler
    TRACE_ISR_CLOCK                         = 0xD9,  // nrfx_clock_irq_handler
    TRACE_ISR_QSPI                          = 0xDA,  // nrfx_qspi_irq_handler
    TRACE_ISR_PWM0                          = 0xDB,  // nrfx_pwm_0_irq_handler
    TRACE_ISR_PWM1                          = 0xDC,  // nrfx_pwm_1_irq_handler
    TRACE_ISR_PWM2                          = 0xDD,  // nrfx_pwm_2_irq_handler
    TRACE_ISR_PWM3                          = 0xDE,  // nrfx_pwm_3_irq_handler
} trace_isr;

#define TRACE_SIGNATURE 0x63617254

// This structure is included in crash reports and is read via JTAG by DebugCodesToStrings.
// If the layout changes, the signature should be changed.
typedef struct {
    uint32_t signature;  // TRACE_SIGNATURE
    uint16_t count;
    bool stopped;
    volatile uint32_t index;

    // This field must be last, TRACE_COUNT may be defined to nothing.  Also see trace_boot and trace_copy.
    uint32_t log[TRACE_COUNT];
} trace_state_t;

extern trace_state_t trace_state;

void trace_init();
static inline void trace(trace_type type, trace_location location, size_t arg1, size_t arg2, size_t arg3, size_t arg4);
static inline void trace_pri0(trace_type type, trace_location location, size_t arg1, size_t arg2, size_t arg3, size_t arg4);
static inline void trace_with_time(trace_type type, trace_location location, uint32_t time, size_t arg1, size_t arg2, size_t arg3, size_t arg4);
static inline void trace_pri0_with_time(trace_type type, trace_location location, uint32_t time, size_t arg1, size_t arg2, size_t arg3, size_t arg4);
uint32_t *trace_log_open();
static inline uint32_t *trace_log_encode(uint32_t *log, trace_type type, trace_location location, uint32_t time, size_t arg1, size_t arg2, size_t arg3, size_t arg4);
static inline void trace_log_close(uint32_t *log_start, uint32_t *log_end);
bool trace_stop();
void trace_reset();
uint8_t *trace_copy(uint8_t *buf, size_t buf_size);

//Trace takes too much time in IRQs
#pragma GCC push_options
#pragma GCC optimize ("O3")

uint32_t trace_get_timestamp();

static inline __attribute__((always_inline)) uint32_t trace_store_length(uint32_t *current_record, uint32_t *next_record)
{
    uint32_t next_record_index = next_record - &trace_state.log[0];
    ASSERT(next_record_index <= trace_state.count);
    uint32_t length = next_record - current_record;
    ASSERT(length >= 1 && length <= MAX_TRACE_RECORD_SIZE);
    // Store slot count at the end of each record to enable iterating record headers from the end
    // of the buffer.  The record type is in the high 8 bits of the first slot, and the length is
    // in the low 6 bits of the last slot.  In theory a record can span a single slot, but
    // currently there are no such records.  Also note that if records at the end are never
    // utilized, a 0 entry is interpreted to have a record length of 1.
    next_record[-1] |= length - 1;
    return next_record_index;
}

#define TRACE_MAX_ARRAY_SIZE ((MAX_TRACE_RECORD_SIZE-3) * sizeof(uint32_t))  // record with an array may use up to 3 other slots

static inline __attribute__((always_inline)) uint32_t *trace_append_array(uint32_t *log, size_t array_addr, size_t *array_size_ptr, size_t max_size)
{
    size_t array_size = *array_size_ptr;

    ASSERT(max_size <= TRACE_MAX_ARRAY_SIZE);

    if (array_size > max_size)
    {
        array_size = max_size;
        *array_size_ptr = array_size;
    }

    if (array_addr != 0)
    {
        memcpy(log, (const void*)array_addr, array_size);
        log += (array_size + sizeof(uint32_t) - 1) / sizeof(uint32_t);
    }

    return log;
}

static inline __attribute__((always_inline)) void trace_log_close(uint32_t *log_start, uint32_t *log_end)
{
    trace_state.index = trace_store_length(log_start, log_end);
}

static inline __attribute__((always_inline)) uint32_t *trace_log_encode(uint32_t *log, trace_type type, trace_location location, uint32_t time, size_t arg1, size_t arg2, size_t arg3, size_t arg4)
{
    uint32_t *log_start;
    switch(type)
    {
#ifdef OMMO_ESB_TRACE_ENABLED
        case TRACE_TYPE_ESB_DATA:
#endif
#ifdef OMMO_1WIRE_ADAPTER_TRACE_ENABLED
        case TRACE_TYPE_1WIRE_CDC_DATA:
        case TRACE_TYPE_1WIRE_SIU_DATA:
#endif
        case TRACE_TYPE_DATA_U26:
            log_start = log;
            *log++ = ((uint32_t)type<<TRACE_TYPE_SHIFT) | ((uint32_t)location<<TRACE_LOCATION_SHIFT);  // validated length stored in low 16 bits, below
            log = trace_append_array(log, arg1, &arg2, arg3);
            *log_start |= arg2;
            *log++ = arg4 << TRACE_LENGTH_BITS;
            break;
#ifdef OMMO_ESB_BS_TRACE_ENABLED
        case TRACE_TYPE_ESB_BS_DC:
#endif
        case TRACE_TYPE_TIME24_U32_U26_U16_U8:
            *log++ = ((uint32_t)type<<TRACE_TYPE_SHIFT) | ((uint32_t)location<<TRACE_LOCATION_SHIFT) | (uint16_t)arg3;
            *log++ = (time<<8) | (uint8_t)arg4;
            *log++ = arg1;
            *log++ = arg2 << TRACE_LENGTH_BITS;
            break;
#ifdef OMMO_ESB_TRACE_ENABLED
        case TRACE_TYPE_ESB:
#endif
        case TRACE_TYPE_TIME24_U26_U16_U8:
            *log++ = ((uint32_t)type<<TRACE_TYPE_SHIFT) | ((uint32_t)location<<TRACE_LOCATION_SHIFT) | (uint16_t)arg2;
            *log++ = (time<<8) | (uint8_t)arg3;
            *log++ = (uint32_t)arg1 << TRACE_LENGTH_BITS;
            break;
        case TRACE_TYPE_TIME_DATA:
            log_start = log;
            *log++ = ((uint32_t)type<<TRACE_TYPE_SHIFT) | ((uint32_t)location<<TRACE_LOCATION_SHIFT);  // validated length stored in low 16 bits, below
            *log++ = time;
            log = trace_append_array(log, arg2, &arg1, TRACE_MAX_ARRAY_SIZE);
            *log_start |= (uint16_t)arg1;
            *log++ = 0;  // Provide a place to store the slot count
            break;
#ifdef OMMO_1WIRE_ADAPTER_TRACE_ENABLED
        case TRACE_TYPE_1WIRE:
#endif
        case TRACE_TYPE_U26_U16:
            *log++ = ((uint32_t)type<<TRACE_TYPE_SHIFT) | ((uint32_t)location<<TRACE_LOCATION_SHIFT) | (uint16_t)arg2;
            *log++ = (uint32_t)arg1 << TRACE_LENGTH_BITS;
            break;
        case TRACE_TYPE_U32_U26_U16:
            *log++ = ((uint32_t)type<<TRACE_TYPE_SHIFT) | ((uint32_t)location<<TRACE_LOCATION_SHIFT) | (uint16_t)arg3;
            *log++ = (uint32_t)arg1;
            *log++ = (uint32_t)arg2 << TRACE_LENGTH_BITS;
            break;
        case TRACE_TYPE_TIME_U26_U16:
        case TRACE_TYPE_TIME_FLASHADDR_U16:
        case TRACE_TYPE_ISR_ACTION_ARG:
        case TRACE_TYPE_ISR_ACTION_CALLBACK:
            *log++ = ((uint32_t)type<<TRACE_TYPE_SHIFT) | ((uint32_t)location<<TRACE_LOCATION_SHIFT) | (uint16_t)arg2;
            *log++ = time;
            *log++ = (uint32_t)arg1 << TRACE_LENGTH_BITS;
            break;
        case TRACE_TYPE_TIME_FLASHADDR_U26_U16:
        case TRACE_TYPE_ISR_ACTION_CALLBACK_ARG:
            *log++ = ((uint32_t)type<<TRACE_TYPE_SHIFT) | ((uint32_t)location<<TRACE_LOCATION_SHIFT) | (uint16_t)arg3;
            *log++ = time;
            *log++ = arg1;
            *log++ = (uint32_t)arg2 << TRACE_LENGTH_BITS;
            break;
        case TRACE_TYPE_TIME_U8:
        case TRACE_TYPE_ISR_ACTION:
            *log++ = ((uint32_t)type<<TRACE_TYPE_SHIFT) | ((uint32_t)location<<TRACE_LOCATION_SHIFT) | (uint16_t)(arg1<<8) | (uint8_t)(time>>24);
            *log++ = (uint32_t)time << TRACE_LENGTH_BITS;
            break;
        case TRACE_TYPE_U10:
            *log++ = ((uint32_t)type<<TRACE_TYPE_SHIFT) | ((uint32_t)location<<TRACE_LOCATION_SHIFT) | (uint16_t)(arg1<<TRACE_LENGTH_BITS);
            break;

        default:
            ASSERT(false);
            break;
    }

    return log;
}

static inline __attribute__((always_inline)) void trace_with_time(trace_type type, trace_location location, uint32_t time, size_t arg1, size_t arg2, size_t arg3, size_t arg4)
{
    CRITICAL_REGION_ENTER();
    {
        uint32_t *log = trace_log_open();
        if (log)
        {
            uint32_t *log_end = trace_log_encode(log, type, location, time, arg1, arg2, arg3, arg4);
            trace_log_close(log, log_end);
        }
    }
    CRITICAL_REGION_EXIT();
}

static inline __attribute__((always_inline)) void trace(trace_type type, trace_location location, size_t arg1, size_t arg2, size_t arg3, size_t arg4)
{
    CRITICAL_REGION_ENTER();
    {
        uint32_t *log = trace_log_open();
        if (log)
        {
            uint32_t *log_end = trace_log_encode(log, type, location, trace_get_timestamp(), arg1, arg2, arg3, arg4);
            trace_log_close(log, log_end);
        }
    }
    CRITICAL_REGION_EXIT();
}

static inline __attribute__((always_inline)) void trace_pri0_with_time(trace_type type, trace_location location, uint32_t time, size_t arg1, size_t arg2, size_t arg3, size_t arg4)
{
    uint32_t *log = trace_log_open();
    if (log)
    {
        uint32_t *log_end = trace_log_encode(log, type, location, time, arg1, arg2, arg3, arg4);
        trace_log_close(log, log_end);
    }
}

static inline __attribute__((always_inline)) void trace_pri0(trace_type type, trace_location location, size_t arg1, size_t arg2, size_t arg3, size_t arg4)
{
    trace_pri0_with_time(type, location, trace_get_timestamp(), arg1, arg2, arg3, arg4);
}

#pragma GCC pop_options

// Macros for instrumenting ISR entry, exit, and callbacks.  These make a call, so to minimize
// impact on code generation, they should typically be used at:
// - the beginning of a function
// - before a return/end of a function
// - call to another function
#ifndef OMMO_ISR_TRACE_DISABLED
#define OMMO_TRACE_ISR_ENTER(ISR_TYPE) TRACE_WITH_ARG(TRACE_TYPE_ISR_ACTION, TRACE_LOCATION_ISR_ENTER, ISR_TYPE)
#define OMMO_TRACE_ISR_EXIT(ISR_TYPE) TRACE_WITH_ARG(TRACE_TYPE_ISR_ACTION, TRACE_LOCATION_ISR_EXIT, ISR_TYPE)
#define OMMO_TRACE_ISR_ENTER_PRI0(ISR_TYPE) TRACE_PRI0_WITH_ARG(TRACE_TYPE_ISR_ACTION, TRACE_LOCATION_ISR_ENTER, ISR_TYPE)
#define OMMO_TRACE_ISR_EXIT_PRI0(ISR_TYPE) TRACE_PRI0_WITH_ARG(TRACE_TYPE_ISR_ACTION, TRACE_LOCATION_ISR_EXIT, ISR_TYPE)
#define OMMO_TRACE_ISR_ENTER_U16(ISR_TYPE, ARG) TRACE_WITH_ARGS_2(TRACE_TYPE_ISR_ACTION_ARG, TRACE_LOCATION_ISR_ENTER, ARG, ISR_TYPE)
#define OMMO_TRACE_ISR_EXIT_U16(ISR_TYPE, ARG) TRACE_WITH_ARGS_2(TRACE_TYPE_ISR_ACTION_ARG, TRACE_LOCATION_ISR_EXIT, ARG, ISR_TYPE)
#define OMMO_TRACE_ISR_ENTER_CALLBACK(ISR_TYPE, CALLBACK) TRACE_WITH_ARGS_2(TRACE_TYPE_ISR_ACTION_CALLBACK, TRACE_LOCATION_ISR_ENTER, CALLBACK, ISR_TYPE)
#define OMMO_TRACE_ISR_EXIT_CALLBACK(ISR_TYPE, CALLBACK) TRACE_WITH_ARGS_2(TRACE_TYPE_ISR_ACTION_CALLBACK, TRACE_LOCATION_ISR_EXIT, CALLBACK, ISR_TYPE)
#define OMMO_TRACE_ISR_EXIT_CALLBACK_ARG(ISR_TYPE, CALLBACK, ARG) TRACE_WITH_ARGS_3(TRACE_TYPE_ISR_ACTION_CALLBACK_ARG, TRACE_LOCATION_ISR_EXIT, CALLBACK, ARG, ISR_TYPE)
#define OMMO_TRACE_ISR_ACTION_CALLBACK(ISR_TYPE, CALLBACK) TRACE_WITH_ARGS_2(TRACE_TYPE_ISR_ACTION_CALLBACK, TRACE_LOCATION_ISR_ACTION, CALLBACK, ISR_TYPE)
#else
#define OMMO_TRACE_ISR_ENTER(ISR_TYPE) do { } while (0)
#define OMMO_TRACE_ISR_EXIT(ISR_TYPE) do { } while (0)
#define OMMO_TRACE_ISR_ENTER_PRI0(ISR_TYPE) do { } while (0)
#define OMMO_TRACE_ISR_EXIT_PRI0(ISR_TYPE) do { } while (0)
#define OMMO_TRACE_ISR_ENTER_U16(ISR_TYPE, ARG) do { } while (0)
#define OMMO_TRACE_ISR_EXIT_U16(ISR_TYPE, ARG) do { } while (0)
#define OMMO_TRACE_ISR_ENTER_CALLBACK(ISR_TYPE, CALLBACK) do { } while (0)
#define OMMO_TRACE_ISR_EXIT_CALLBACK(ISR_TYPE, CALLBACK) do { } while (0)
#define OMMO_TRACE_ISR_EXIT_CALLBACK_ARG(ISR_TYPE, CALLBACK, ARG) do { } while (0)
#define OMMO_TRACE_ISR_ACTION_CALLBACK(ISR_TYPE, CALLBACK) do { } while (0)
#endif

#ifdef __cplusplus
}
#endif
