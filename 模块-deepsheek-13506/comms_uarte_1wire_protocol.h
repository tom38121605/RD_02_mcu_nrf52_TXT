#pragma once

#include <stdint.h>

#include "utils.hpp"


// A handshake packet is distinguished by setting COMMS_1WIRE_FLAGS_HANDSHAKE in the flags, and
// this special timestamp value.  The master will continue to send a handshake packet until the
// slave responds with a 0-length data packet.
#define COMMS_1WIRE_TIMESTAMP_DURING_HANDSHAKE 0xfffefdfc

// Maximum time between data packet and next timestamp packet.  On the master side, the timestamp
// packet is sent based on the synch timer/synch input.  On the slave side, this value can be used
// to estimate timestamp packet arrival.
#define COMMS_1WIRE_TIMESTAMP_PACKET_MAX_DELAY_US 470
#define COMMS_1WIRE_TIMESTAMP_PACKET_MAX_DELAY_TICKS US_TO_TICKS_16MHZ(COMMS_1WIRE_TIMESTAMP_PACKET_MAX_DELAY_US)

#define COMMS_1WIRE_FLAGS_HANDSHAKE 0x80  // distinguishes a handshake packet from a timestamp packet
#define COMMS_1WIRE_FLAGS_EXT_SYNCH 0x08  // master's timestamp is synchronized with an external synch input
#define COMMS_1WIRE_FLAGS_OPEN      0x04  // an application has opened a connection to the slave 
#define COMMS_1WIRE_FLAGS_PING      0x02  // slave should respond with 0-length data packet
#define COMMS_1WIRE_FLAGS_MASTER_TX 0x01  // if set, slave receives a data packet; if clear, slave transmits a data packet

// Timestamp packet, transmitted by master at ~250 Hz in sync with the master's timestamp clock
typedef __PACKED_STRUCT
{
    uint32_t timestamp;   // master's timestamp clock, little endian
    uint8_t flags;        // COMMS_1WIRE_FLAGS_*
} comms_1wire_timestamp_packet_t;

// Time to transmit a single byte, assuming 1Mbps, N81
#define COMMS_1WIRE_BYTE_TX_US 10
#define COMMS_1WIRE_BYTE_TX_TICKS US_TO_TICKS_16MHZ(COMMS_1WIRE_BYTE_TX_US)

#define COMMS_1WIRE_TIMESTAMP_PACKET_TX_TICKS (COMMS_1WIRE_BYTE_TX_TICKS * sizeof(comms_1wire_timestamp_packet_t))

// Delay between timestamp and data packets
#define COMMS_1WIRE_DATA_PACKET_DELAY_US 300
#define COMMS_1WIRE_DATA_PACKET_DELAY_TICKS US_TO_TICKS_16MHZ(COMMS_1WIRE_DATA_PACKET_DELAY_US)

// ~4ms between timestamp packets, 400 - 47 - 30 - 3 = 320 byte payload, which accommodates
// 5 USB packets
#define COMMS_1WIRE_DATA_PACKET_MAX_SIZE ((  4000 \
                                           - COMMS_1WIRE_TIMESTAMP_PACKET_MAX_DELAY_US \
                                           - COMMS_1WIRE_DATA_PACKET_DELAY_US \
                                           - 3 * COMMS_1WIRE_BYTE_TX_US) / COMMS_1WIRE_BYTE_TX_US)

typedef __PACKED_STRUCT
{
    uint8_t data[COMMS_1WIRE_DATA_PACKET_MAX_SIZE];
    uint8_t flags;  // reserved
    uint16_t length;  // little endian, at the end so that slave can append packets to a buffer via DMA
} comms_1wire_master_tx_data_packet_t;

typedef __PACKED_STRUCT
{
    uint16_t length;  // little endian, at the beginning so that slave can install length as packets are transmitted
    uint8_t flags;  // reserved
    uint8_t data[COMMS_1WIRE_DATA_PACKET_MAX_SIZE];
} comms_1wire_slave_tx_data_packet_t;

// Wire time of a data packet must be the same for either direction
STATIC_ASSERT(sizeof(comms_1wire_master_tx_data_packet_t) == sizeof(comms_1wire_slave_tx_data_packet_t));

#define COMMS_1WIRE_DATA_PACKET_TX_TICKS (COMMS_1WIRE_BYTE_TX_TICKS * sizeof(comms_1wire_master_tx_data_packet_t))

// Minimum time between timestamp or data packets = 1ms.  6000 series devices have 4ms synch
// periods, but the assumption is that packet throughput is more important than payload size,
// so 1-wire uses 1ms.
#define COMMS_1WIRE_MAX_PERIOD_TICKS (COMMS_1WIRE_TIMESTAMP_PACKET_TX_TICKS + \
                                      COMMS_1WIRE_DATA_PACKET_DELAY_TICKS + \
                                      COMMS_1WIRE_DATA_PACKET_TX_TICKS + \
                                      COMMS_1WIRE_TIMESTAMP_PACKET_MAX_DELAY_TICKS)

// A packet must complete within 20us beyond its normal transmission window
#define COMMS_1WIRE_RX_TIMEOUT_TICKS US_TO_TICKS_16MHZ(20)

// Data expected in a ping ack packet
#define __COMMS_1WIRE_TEST_DATA_PLUS_1(x) 1+x,
#define __COMMS_1WIRE_TEST_DATA_PLUS_33(x) 33+x,
#define __COMMS_1WIRE_TEST_DATA_PLUS_65(x) 65+x,
#define __COMMS_1WIRE_TEST_DATA_PLUS_97(x) 97+x,
#define __COMMS_1WIRE_TEST_DATA_PLUS_129(x) 129+x,
#define __COMMS_1WIRE_TEST_DATA_PLUS_161(x) 161+x,
#define __COMMS_1WIRE_TEST_DATA_PLUS_193(x) 193+x,
#define __COMMS_1WIRE_TEST_DATA_PLUS_225(x) 225+x,
#define __COMMS_1WIRE_TEST_DATA_254_MINUS(x) 254-x,
#define __COMMS_1WIRE_TEST_DATA_222_MINUS(x) 222-x,
#define __COMMS_1WIRE_TEST_DATA_190_MINUS(x) 190-x,
#define COMMS_1WIRE_TEST_DATA FOR_MACRO_32(__COMMS_1WIRE_TEST_DATA_PLUS_1)     /*   1.. 32   32       */ \
                              FOR_MACRO_32(__COMMS_1WIRE_TEST_DATA_PLUS_33)    /*  33.. 64 + 32 =  64 */ \
                              FOR_MACRO_32(__COMMS_1WIRE_TEST_DATA_PLUS_65)    /*  65.. 96 + 32 =  96 */ \
                              FOR_MACRO_32(__COMMS_1WIRE_TEST_DATA_PLUS_97)    /*  97..128 + 32 = 128 */ \
                              FOR_MACRO_32(__COMMS_1WIRE_TEST_DATA_PLUS_129)   /* 129..160 + 32 = 160 */ \
                              FOR_MACRO_32(__COMMS_1WIRE_TEST_DATA_PLUS_161)   /* 161..192 + 32 = 192 */ \
                              FOR_MACRO_32(__COMMS_1WIRE_TEST_DATA_PLUS_193)   /* 193..224 + 32 = 224 */ \
                              FOR_MACRO_31(__COMMS_1WIRE_TEST_DATA_PLUS_225)   /* 225..255 + 31 = 255 */ \
                              FOR_MACRO_32(__COMMS_1WIRE_TEST_DATA_254_MINUS)  /* 254..223 + 32 = 287 */ \
                              FOR_MACRO_32(__COMMS_1WIRE_TEST_DATA_222_MINUS)  /* 222..191 + 32 = 319 */ \
                              FOR_MACRO_1 (__COMMS_1WIRE_TEST_DATA_190_MINUS)  /* 190..190 +  1 = 320 */
STATIC_ASSERT(COMMS_1WIRE_DATA_PACKET_MAX_SIZE == 320);
