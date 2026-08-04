#pragma once

#include "app_util.h"

#include "cobs.h"

#ifndef OMMO_USB_SERIAL_MIN_PACKET_SIZE
#define OMMO_USB_SERIAL_MIN_PACKET_SIZE   128
#endif

#define OMMO_DEFAULT_CHANNEL_RF_CHANNEL_ID  25

#define OMMO_TIMESTAMP_SYNCH_PERIOD   (OMMO_TIMESTAMP_SAMPLE_PERIOD*OMMO_TIMESTAMP_SYNCH_PERIOD_MULT)

#define COMMS_ESB_SD_COMMAND_PACKET_LENGTH_BYTES    sizeof(comms_esb_sd_command_packet)
#define COMMS_ESB_SD_COMMAND_PACKET_LENGTH_US       NRF_ESB_TX_TRX_TIME_US(COMMS_ESB_SD_COMMAND_PACKET_LENGTH_BYTES)
#define COMMS_ESB_SD_COMMAND_PACKET_LENGTH_TICKS    US_TO_TICKS_16MHZ(COMMS_ESB_SD_COMMAND_PACKET_LENGTH_US)

#define COMMS_ESB_PADDING_US            24

#define OMMO_ESB_MAX_PAYLOAD_WINDOW_US  (TICKS_16MHZ_TO_US(OMMO_TIMESTAMP_SYNCH_PERIOD) - \
                                       COMMS_ESB_SD_COMMAND_PACKET_LENGTH_US - \
                                       2*NRF_ESB_RXTX_RU_US - \
                                       2*NRF_ESB_RXTX_SD_US - \
                                       COMMS_ESB_PADDING_US - \
                                       COMMS_ESB_PADDING_US) //synch to data padding + Data to synch padding

#define OMMO_ESB_DC_MAX_MULTISYNCH_TRANSFER_PACKET_LENGTH 4096

#define COMMS_ESB_MAX_SINGLE_PACKET_LENGTH      MIN(NRF_ESB_MAX_PAYLOAD_IN_US_WINDOW(OMMO_ESB_MAX_PAYLOAD_WINDOW_US), NRF_ESB_MAX_PAYLOAD_LENGTH)

#define COMMS_ESB_MAX_MULTIPACKET_PAYLOAD_LENGTH        (COMMS_ESB_MAX_SINGLE_PACKET_LENGTH-2)
#define COMMS_ESB_MAX_MULTIPACKET_NUM_SPLITS_REQUIRED   CEIL_DIV(OMMO_ESB_MAX_PAYLOAD_WINDOW_US,NRF_ESB_TX_TRX_TIME_US(NRF_ESB_MAX_PAYLOAD_LENGTH))
#define COMMS_ESB_MAX_MULTIPACKET_PAYLOAD_US            (OMMO_ESB_MAX_PAYLOAD_WINDOW_US - COMMS_ESB_MAX_MULTIPACKET_NUM_SPLITS_REQUIRED*NRF_ESB_BITS_TO_US(2*8 + NRF_ESB_PACKET_OVERHEAD_BITS))
#define COMMS_ESB_MAX_MULTIPACKET_TOTAL_LENGTH          (NRF_ESB_MAX_BITS_IN_US_WINDOW(COMMS_ESB_MAX_MULTIPACKET_PAYLOAD_US)/8)

//Datapacket time calculation
#define COMMS_ESB_PACKET_TIME_MEGAPACKET_MAX_COBS_SIZE(x)    (COBS_CRC_POST0_MAX_LEN(x)*OMMO_TIMESTAMP_SYNCH_PERIOD_MULT)
#define COMMS_ESB_PACKET_TIME_SPLITS_REQUIRED(x)        ((x)>NRF_ESB_MAX_PAYLOAD_LENGTH ? CEIL_DIV((x), NRF_ESB_MAX_PAYLOAD_LENGTH-2) : 1)  
#define COMMS_ESB_PACKET_TIME_US(x)                     NRF_ESB_BITS_TO_US(NRF_ESB_PACKET_OVERHEAD_BITS*COMMS_ESB_PACKET_TIME_SPLITS_REQUIRED(COMMS_ESB_PACKET_TIME_MEGAPACKET_MAX_COBS_SIZE(x))+COMMS_ESB_PACKET_TIME_MEGAPACKET_MAX_COBS_SIZE(x)*8)

#define OMMO_BS_ADDR_0          {0x37, 0x73, 0xAD, 0x9C}
#define OMMO_BS_ADDR_1          {0x37, 0x73, 0xAD, 0x9C}
#define OMMO_BS_ADDR_PREFIX     {0x68, 0x11, 0x33, 0xFF, 0xAB, 0xF5, 0x21, 0x3D }

#define OMMO_BS_HW_CH_RF_SKIP       2
#define OMMO_BS_HW_CH_RF_MAX        100
#define OMMO_BS_HW_CH_ID_COUNT      (OMMO_BS_HW_CH_RF_MAX / OMMO_BS_HW_CH_RF_SKIP + 1) //0-50 inclusive

#define OMMO_BS_USABLE_CH_ID_MIN            1
#define OMMO_BS_USABLE_CH_ID_MAX            39
#define OMMO_BS_USABLE_CH_ID_COUNT          (OMMO_BS_USABLE_CH_ID_MAX - OMMO_BS_USABLE_CH_ID_MIN + 1) //OMMO_BS_USABLE_CH_ID_MIN-OMMO_BS_USABLE_CH_ID_MAX inclusive
#define OMMO_BS_USABLE_CH_ID_OFFSET(x, y)   ((((x)-OMMO_BS_USABLE_CH_ID_MIN+(y))%OMMO_BS_USABLE_CH_ID_COUNT)+OMMO_BS_USABLE_CH_ID_MIN)
#define OMMO_BS_USABLE_CH_ID_TO_RF(x)       ((x)*OMMO_BS_HW_CH_RF_SKIP)


#define OMMO_DATA_PIPE                     0x00
#define OMMO_MULTIPACKET_DATA_PIPE         0x01
#define OMMO_DIRECT_COMM_PIPE              0x02
#define OMMO_FW_COMMAND_PIPE               0x03
#define OMMO_USER_COMMAND_PIPE             0x04

#define OMMO_DATA_PIPE_MSK                 (0x01 << OMMO_DATA_PIPE)
#define OMMO_MULTIPACKET_DATA_PIPE_MSK     (0x01 << OMMO_MULTIPACKET_DATA_PIPE)
#define OMMO_DIRECT_COMM_PIPE_MSK          (0x01 << OMMO_DIRECT_COMM_PIPE)
#define OMMO_FW_COMMAND_PIPE_MSK           (0x01 << OMMO_FW_COMMAND_PIPE)
#define OMMO_USER_COMMAND_PIPE_MSK         (0x01 << OMMO_USER_COMMAND_PIPE)

#define OMMO_ALL_PIPES_MSK                 (OMMO_DATA_PIPE_MSK | OMMO_MULTIPACKET_DATA_PIPE_MSK | OMMO_DIRECT_COMM_PIPE_MSK | OMMO_FW_COMMAND_PIPE_MSK | OMMO_USER_COMMAND_PIPE_MSK)
#define OMMO_ALL_COMMAND_PIPES_MSK         (OMMO_FW_COMMAND_PIPE_MSK | OMMO_USER_COMMAND_PIPE_MSK)

#define MS_TO_TICKS_16MHZ(x) ((uint32_t)(16000*(x)))
#define US_TO_TICKS_16MHZ(x) ((uint32_t)(16*(x)))
#define TICKS_16MHZ_TO_US(x) ((uint32_t)((x+15)/16)) //CEIL
#define TICKS_16MHZ_TO_MS(x) ((uint32_t)((x+15999)/16000)) //CEIL

#define CHECK_NULL_PTR(ptr) \
    do { if ((ptr) == NULL) return NRF_ERROR_NULL; } while (0)

#define IF_SUCCESS_DO(err, func) \
    do \
    { \
        if ((err) == NRF_SUCCESS) \
        { \
            err = func; \
        } \
    } while (0)

#define FOR_MACRO_0(m)  // Empty macro for 0 iterations
#define FOR_MACRO_1(m)  m(0)
#define FOR_MACRO_2(m)  FOR_MACRO_1(m)  m(1)
#define FOR_MACRO_3(m)  FOR_MACRO_2(m)  m(2)
#define FOR_MACRO_4(m)  FOR_MACRO_3(m)  m(3)
#define FOR_MACRO_5(m)  FOR_MACRO_4(m)  m(4)
#define FOR_MACRO_6(m)  FOR_MACRO_5(m)  m(5)
#define FOR_MACRO_7(m)  FOR_MACRO_6(m)  m(6)
#define FOR_MACRO_8(m)  FOR_MACRO_7(m)  m(7)
#define FOR_MACRO_9(m)  FOR_MACRO_8(m)  m(8)
#define FOR_MACRO_10(m) FOR_MACRO_9(m)  m(9)
#define FOR_MACRO_11(m) FOR_MACRO_10(m) m(10)
#define FOR_MACRO_12(m) FOR_MACRO_11(m) m(11)
#define FOR_MACRO_13(m) FOR_MACRO_12(m) m(12)
#define FOR_MACRO_14(m) FOR_MACRO_13(m) m(13)
#define FOR_MACRO_15(m) FOR_MACRO_14(m) m(14)
#define FOR_MACRO_16(m) FOR_MACRO_15(m) m(15)
#define FOR_MACRO_17(m) FOR_MACRO_16(m) m(16)
#define FOR_MACRO_18(m) FOR_MACRO_17(m) m(17)
#define FOR_MACRO_19(m) FOR_MACRO_18(m) m(18)
#define FOR_MACRO_20(m) FOR_MACRO_19(m) m(19)
#define FOR_MACRO_21(m) FOR_MACRO_20(m) m(20)
#define FOR_MACRO_22(m) FOR_MACRO_21(m) m(21)
#define FOR_MACRO_23(m) FOR_MACRO_22(m) m(22)
#define FOR_MACRO_24(m) FOR_MACRO_23(m) m(23)
#define FOR_MACRO_25(m) FOR_MACRO_24(m) m(24)
#define FOR_MACRO_26(m) FOR_MACRO_25(m) m(25)
#define FOR_MACRO_27(m) FOR_MACRO_26(m) m(26)
#define FOR_MACRO_28(m) FOR_MACRO_27(m) m(27)
#define FOR_MACRO_29(m) FOR_MACRO_28(m) m(28)
#define FOR_MACRO_30(m) FOR_MACRO_29(m) m(29)
#define FOR_MACRO_31(m) FOR_MACRO_30(m) m(30)
#define FOR_MACRO_32(m) FOR_MACRO_31(m) m(31)
#define FOR_MACRO_EXPAND(n, m) UTIL_OBSTRUCT(FOR_MACRO_##n(m))
// Macro to iterate from 0 to n-1, calling a macro for each index. Works with up to 32 iterations.
// Usage:
// #define MACRO(index) do_something(index);
// FOR_MACRO(5, MACRO) // will expand to do_something(0); do_something(1); do_something(2); do_something(3); do_something(4);
#define FOR_MACRO(n, m)  FOR_MACRO_EXPAND(n, m)

#define COUNT_ARGS_IMPL(_0,_1,_2,_3,_4,_5,_6,_7,_8,_9,_10,_11,_12,_13,_14,_15,_16,_17,_18,_19,_20,_21,_22,_23,_24,_25,_26,_27,_28,_29,_30,_31,_32,N,...) N
#define COUNT_ARGS_EXPAND(...) COUNT_ARGS_IMPL(__VA_OPT__(dummy,) __VA_ARGS__,32,31,30,29,28,27,26,25,24,23,22,21,20,19,18,17,16,15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0)
// Macro to count the number of arguments passed to it. Works with up to 32 arguments.
// Usage:
// #define ARGS 1, 2, 3, 4
// COUNT_ARGS(ARGS) // will expand to 4
#define COUNT_ARGS(...) COUNT_ARGS_EXPAND(__VA_ARGS__)

#define ___TEST_COUNT_ARGS
STATIC_ASSERT(COUNT_ARGS(___TEST_COUNT_ARGS) == 0);
#undef ___TEST_COUNT_ARGS
#define ___TEST_COUNT_ARGS a
STATIC_ASSERT(COUNT_ARGS(___TEST_COUNT_ARGS) == 1);
#undef ___TEST_COUNT_ARGS
#define ___TEST_COUNT_ARGS a, b
STATIC_ASSERT(COUNT_ARGS(___TEST_COUNT_ARGS) == 2);
#undef ___TEST_COUNT_ARGS

#define FE_0(...)
#define FE_1(m,x1) m(x1)
#define FE_2(m,x1,x2) FE_1(m,x1) m(x2)
#define FE_3(m,x1,x2,x3) FE_2(m,x1,x2) m(x3)
#define FE_4(m,x1,x2,x3,x4) FE_3(m,x1,x2,x3) m(x4)
#define FE_5(m,x1,x2,x3,x4,x5) FE_4(m,x1,x2,x3,x4) m(x5)
#define FE_6(m,x1,x2,x3,x4,x5,x6) FE_5(m,x1,x2,x3,x4,x5) m(x6)
#define FE_7(m,x1,x2,x3,x4,x5,x6,x7) FE_6(m,x1,x2,x3,x4,x5,x6) m(x7)
#define FE_8(m,x1,x2,x3,x4,x5,x6,x7,x8) FE_7(m,x1,x2,x3,x4,x5,x6,x7) m(x8)
#define FE_9(m,x1,x2,x3,x4,x5,x6,x7,x8,x9) FE_8(m,x1,x2,x3,x4,x5,x6,x7,x8) m(x9)
#define FE_10(m,x1,x2,x3,x4,x5,x6,x7,x8,x9,x10) FE_9(m,x1,x2,x3,x4,x5,x6,x7,x8,x9) m(x10)
#define FE_11(m,x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11) FE_10(m,x1,x2,x3,x4,x5,x6,x7,x8,x9,x10) m(x11)
#define FE_12(m,x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12) FE_11(m,x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11) m(x12)
#define FE_13(m,x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13) FE_12(m,x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12) m(x13)
#define FE_14(m,x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14) FE_13(m,x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13) m(x14)
#define FE_15(m,x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15) FE_14(m,x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14) m(x15)
#define FE_16(m,x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15,x16) FE_15(m,x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15) m(x16)
#define FE_17(m,x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15,x16,x17) FE_16(m,x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15,x16) m(x17)
#define FE_18(m,x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15,x16,x17,x18) FE_17(m,x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15,x16,x17) m(x18)
#define FE_19(m,x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15,x16,x17,x18,x19) FE_18(m,x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15,x16,x17,x18) m(x19)
#define FE_20(m,x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15,x16,x17,x18,x19,x20) FE_19(m,x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15,x16,x17,x18,x19) m(x20)
#define FE_21(m,x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15,x16,x17,x18,x19,x20,x21) FE_20(m,x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15,x16,x17,x18,x19,x20) m(x21)
#define FE_22(m,x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15,x16,x17,x18,x19,x20,x21,x22) FE_21(m,x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15,x16,x17,x18,x19,x20,x21) m(x22)
#define FE_23(m,x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15,x16,x17,x18,x19,x20,x21,x22,x23) FE_22(m,x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15,x16,x17,x18,x19,x20,x21,x22) m(x23)
#define FE_24(m,x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15,x16,x17,x18,x19,x20,x21,x22,x23,x24) FE_23(m,x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15,x16,x17,x18,x19,x20,x21,x22,x23) m(x24)
#define FE_25(m,x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15,x16,x17,x18,x19,x20,x21,x22,x23,x24,x25) FE_24(m,x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15,x16,x17,x18,x19,x20,x21,x22,x23,x24) m(x25)
#define FE_26(m,x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15,x16,x17,x18,x19,x20,x21,x22,x23,x24,x25,x26) FE_25(m,x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15,x16,x17,x18,x19,x20,x21,x22,x23,x24,x25) m(x26)
#define FE_27(m,x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15,x16,x17,x18,x19,x20,x21,x22,x23,x24,x25,x26,x27) FE_26(m,x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15,x16,x17,x18,x19,x20,x21,x22,x23,x24,x25,x26) m(x27)
#define FE_28(m,x1,x2,x3,x4,x5,x6,x7,x8,x,x10,x11,x12,x13,x14,x15,x16,x17,x18,x19,x20,x21,x22,x23,x24,x25,x26,x27,x28) FE_27(m,x1,x2,x3,x4,x5,x6,x7,x8,x,x10,x11,x12,x13,x14,x15,x16,x17,x18,x19,x20,x21,x22,x23,x24,x25,x26,x27) m(x28)
#define FE_29(m,x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15,x16,x17,x18,x19,x20,x21,x22,x23,x24,x25,x26,x27,x28,x29) FE_28(m,x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15,x16,x17,x18,x19,x20,x21,x22,x23,x24,x25,x26,x27,x28) m(x29)
#define FE_30(m,x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15,x16,x17,x18,x19,x20,x21,x22,x23,x24,x25,x26,x27,x28,x29,x30) FE_29(m,x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15,x16,x17,x18,x19,x20,x21,x22,x23,x24,x25,x26,x27,x28,x29) m(x30)
#define FE_31(m,x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15,x16,x17,x18,x19,x20,x21,x22,x23,x24,x25,x26,x27,x28,x29,x30,x31) FE_30(m,x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15,x16,x17,x18,x19,x20,x21,x22,x23,x24,x25,x26,x27,x28,x29,x30) m(x31)
#define FE_32(m,x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15,x16,x17,x18,x19,x20,x21,x22,x23,x24,x25,x26,x27,x28,x29,x30,x31,x32) FE_31(m,x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15,x16,x17,x18,x19,x20,x21,x22,x23,x24,x25,x26,x27,x28,x29,x30,x31) m(x32)
#define GET_FE_MACRO(_0,_1,_2,_3,_4,_5,_6,_7,_8,_9,_10,_11,_12,_13,_14,_15,_16,_17,_18,_19,_20,_21,_22,_23,_24,_25,_26,_27,_28,_29,_30,_31,_32,NAME,...) NAME
#define FOR_EACH_EXPAND(macro, ...) GET_FE_MACRO(__VA_OPT__(dummy,) __VA_ARGS__,FE_32,FE_31,FE_30,FE_29,FE_28,FE_27,FE_26,FE_25,FE_24,FE_23,FE_22,FE_21,FE_20,FE_19,FE_18,FE_17,FE_16,FE_15,FE_14,FE_13,FE_12,FE_11,FE_10,FE_9,FE_8,FE_7,FE_6,FE_5,FE_4,FE_3,FE_2,FE_1,FE_0)(macro, __VA_ARGS__)
// Macro to iterate over a variable number of arguments, calling a macro for each argument with the index as the first parameter up to 8 arguments.
// Usage:
// #define MACRO(index) do_something(index);
// #define ARGS 0, 1, 2, 3, 4
// FOR_EACH(MACRO, ARGS) // will expand to do_something(0); do_something(1); do_something(2); do_something(3); do_something(4);
#define FOR_EACH(macro, ...)  FOR_EACH_EXPAND(macro, __VA_ARGS__)

#define ___TEST_FOR_EACH_MACRO(x) + x
#define ___TEST_FOR_EACH_INPUT
STATIC_ASSERT(123 FOR_EACH(___TEST_FOR_EACH_MACRO, ___TEST_FOR_EACH_INPUT) == 123);
#undef ___TEST_FOR_EACH_INPUT
#define ___TEST_FOR_EACH_INPUT 45
STATIC_ASSERT(123 FOR_EACH(___TEST_FOR_EACH_MACRO, ___TEST_FOR_EACH_INPUT) == 123 + 45);
#undef ___TEST_FOR_EACH_INPUT
#define ___TEST_FOR_EACH_INPUT 45, 67
STATIC_ASSERT(123 FOR_EACH(___TEST_FOR_EACH_MACRO, ___TEST_FOR_EACH_INPUT) == 123 + 45 + 67);
#undef ___TEST_FOR_EACH_INPUT
#undef ___TEST_FOR_EACH_MACRO

// Macro to remove the parentheses from a macro argument.
#define UTIL_OBSTRUCT(...) __VA_ARGS__

#define REPEAT2(x) {x;x;}
#define REPEAT3(x) {x;x;x;}
#define REPEAT4(x) {x;x;x;x;}
#define REPEAT5(x) {x;x;x;x;x;}
#define REPEAT6(x) {x;x;x;x;x;x;}
#define REPEAT7(x) {x;x;x;x;x;x;x;}
#define REPEAT8(x) {x;x;x;x;x;x;x;x;}
#define REPEAT9(x) {x;x;x;x;x;x;x;x;x;}
#define REPEAT10(x) {x;x;x;x;x;x;x;x;x;x;}
#define REPEAT11(x) {x;x;x;x;x;x;x;x;x;x;x;}
#define REPEAT12(x) {x;x;x;x;x;x;x;x;x;x;x;x;}
#define REPEAT13(x) {x;x;x;x;x;x;x;x;x;x;x;x;x;}
#define REPEAT14(x) {x;x;x;x;x;x;x;x;x;x;x;x;x;x;}
#define REPEAT15(x) {x;x;x;x;x;x;x;x;x;x;x;x;x;x;x;}
#define REPEAT17(x) {x;x;x;x;x;x;x;x;x;x;x;x;x;x;x;x;x;}
#define REPEAT31(x) {x;x;x;x;x;x;x;x;x;x;x;x;x;x;x;x;x;x;x;x;x;x;x;x;x;x;x;x;x;x;x;}
#define REPEAT32(x) {x;x;x;x;x;x;x;x;x;x;x;x;x;x;x;x;x;x;x;x;x;x;x;x;x;x;x;x;x;x;x;x;}
#define REPEAT255(x) {REPEAT32(x);REPEAT32(x);REPEAT32(x);REPEAT32(x);REPEAT32(x);REPEAT32(x);REPEAT32(x);REPEAT31(x)}

// Macro to create a static wrapper function for a class member function
#define CREATE_STATIC_WRAPPER(ClassName, MemberFunctionName, ReturnType, ...) \
  static inline ReturnType MemberFunctionName(void* obj, ##__VA_ARGS__)       \
  {                                                                           \
      return ((ClassName*)obj)->MemberFunctionName(__VA_ARGS__);              \
  }

#define PORT_PIN_TO_MASK(pin) (1ull<<(pin))

#define OMMO_MAKE_DFU_CAPABILITY_MASK_EXPAND(x) (1 << dfu_DFUCapability_##x) |
#define OMMO_MAKE_DFU_CAPABILITY_MASK(...) (FOR_EACH(OMMO_MAKE_DFU_CAPABILITY_MASK_EXPAND, __VA_ARGS__) 0)

#define FIELD_END_OFFSET(TYPE, MEMBER) (offsetof(TYPE, MEMBER) + sizeof(((TYPE*)0)->MEMBER))
