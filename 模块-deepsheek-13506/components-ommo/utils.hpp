#ifndef UTILS_HPP
#define UTILS_HPP

#include <array>
#include <cstring>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "nrf_wdt.h"
#include "nrf_delay.h"
#include "nrfx_timer.h"

#include "ommo_config.h"

#include "advanced_wdt.h"

#ifdef OMMOCOMM_1WIRE
#include "comms_packetizer.hpp"
#endif

#include "ommo_esb.h"
#include "ommo_fw.pb.h"
  
  typedef struct
  {
      uint32_t ts;
      uint32_t bs_uuid;
  }
  __attribute__((packed)) synch_payload;

  typedef union
  {
      uint8_t bytes[8];
      uint32_t ints[2];
      uint64_t long_val;
      synch_payload synch;
  } __attribute__((packed)) comms_esb_sd_command_payload;

  typedef enum
  {
      POWER_REQ_NONE = 1,
      POWER_REQ_USB = 2,
      POWER_REQ_NOT_BUTTON_SIGNAL = 4,
      POWER_REQ_SPI_SENSOR = 8,
      POWER_REQ_OMMOCOMM_SAMPLER = 16,
      //POWER_REQ_OMMOCOMM_CHARGER = 32,
      POWER_REQ_1WIRE = 64,
  } power_req_t;

  typedef struct
  {
      uint8_t command_code;
      uint8_t addr;
      comms_esb_sd_command_payload payload;
  } __attribute__((packed)) comms_esb_sd_command_packet;

  typedef enum
  {
      TRANSFER_STATE_IDLE,
      TRANSFER_STATE_SIU_TO_BS,
      TRANSFER_STATE_BS_TO_SIU
  } comms_esb_transfer_state;

  inline int32_t calculate_wrapped_delta(uint32_t first, uint32_t second, uint32_t period)
  {
  int64_t delta;

      delta = (int64_t)first - (int64_t)second;
      if(delta < -(int32_t)(period>>1))
          delta += period;
      if(delta > (int32_t)(period>>1))
          delta -= period;

      return (int32_t)delta;
  }

  inline int16_t calculate_wrapped_delta(uint16_t first, uint16_t second, uint16_t period)
  {
  int32_t delta;

      delta = (int32_t)first - (int32_t)second;
      if(delta < -(int16_t)(period>>1))
          delta += period;
      if(delta > (int16_t)(period>>1))
          delta -= period;

      return (int16_t)delta;
  }

  inline void copyUint8(unsigned char packetBuffer[], uint16_t& index, const unsigned char value)
  {
      packetBuffer[index] = value;
      index += 1;
  }

  inline void copyUint16_LE(unsigned char packetBuffer[], uint16_t& index, const uint16_t value)
  {
      memcpy(packetBuffer+index, &value, 2);
      index += 2;
  }

  inline void copyUint32_LE(unsigned char packetBuffer[], uint16_t& index, const uint32_t value)
  {
      memcpy(packetBuffer+index, &value, 4);
      index += 4;
  }

  inline void copyUint64_LE(unsigned char packetBuffer[], uint16_t& index, const uint64_t value)
  {
      memcpy(packetBuffer+index, &value, 8);
      index += 8;
  }

  inline void copyFloatIEEE754_32_LE(unsigned char packetBuffer[], uint16_t& index, const float value)
  {
      memcpy(packetBuffer+index, &value, 4);
      index += 4;
  }

  inline void copyFloatIEEE754_32_LE_array(uint8_t size, unsigned char packetBuffer[], uint16_t& index, const float *value)
  {
      memcpy(packetBuffer+index, value, 4*size);
      index += 4*size;
  }

  inline int32_t parseInt24_LE(unsigned char packetBuffer[], uint16_t &index)
  {
      int32_t rvalue;
      rvalue = (int32_t)((((uint32_t)packetBuffer[index + 2]) << 24) | (((uint32_t)packetBuffer[index + 1]) << 16) | (((uint32_t)packetBuffer[index]) << 8));
      rvalue = rvalue >> 8;
      index += 3;

      return rvalue;
  }

  inline int16_t saturate_int32_int16(int32_t val)
  {
      if(val > INT16_MAX)
          return INT16_MAX;
      else if(val < INT16_MIN)
          return INT16_MIN;
      else
          return (int16_t)val;
  }

  inline uint16_t fill_in_ack_packet(uint8_t buffer[], OmmoAck ack_code)
  {
    buffer[0] = OMMO_COMMAND_ACK;
    buffer[1] = (uint8_t)ack_code;

    return 2;
  }

  //General helper functions
  extern "C" void get_firmare_versions(uint32_t *bootloader_version_ptr, uint32_t *application_version_ptr);
  uint16_t general_process_packet_received(uint8_t data[], uint16_t packet_size, uint8_t response_buffer[], uint16_t response_buffer_size);
  void usb_serial_general_process_and_respond(uint8_t data[], uint16_t length, const void *p_context);
  void usb_serial_packet_received_crash_reboot(uint8_t data[], uint16_t length);

  void crash_reboot_check();
  void clocks_start();
  uint32_t convert_ommo_ack_code_to_nrf_error_code(uint8_t ack_code);
  OmmoAck convert_nrf_error_code_to_ommo_ack_code(uint32_t error_code);
  bool memallset(const void *ptr, uint8_t value, size_t len);
  uint32_t ommo_delay_ms(uint32_t delay_ms, uint32_t *continue_from_time = NULL);

  //Array helper functions
  bool is_in_array(uint8_t val, uint8_t *array, uint16_t array_len);
  bool is_in_array(uint32_t val, uint32_t *array, uint16_t array_len);
  uint16_t remove_duplicates(uint8_t *array, uint16_t array_len);
  uint32_t find_max(uint32_t *array, uint16_t array_len);
  uint32_t interpolate_table(const uint32_t *x_values, const uint32_t *y_values, const uint32_t table_len, uint32_t x);

  // Called by operations that may halt the processor or disable interrupts.
  // Applications may override the weak no-op implementation in utils.cpp.
  ret_code_t disrupt_realtime_operation(uint16_t timeout_ms = UINT16_MAX);
  void resume_realtime_operation();
  bool realtime_operation_requested();

#endif