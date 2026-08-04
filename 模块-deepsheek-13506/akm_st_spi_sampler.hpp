#pragma once

  #include "nrf_gpio.h"

  #include "ommo_config.h"
  #include "utils.hpp"
  #include "device_info_command_processor.hpp"
  #include "ommo_fw.pb.h"
  #include "ic_constants.h"
  #include "event_queue_manager.hpp"

// Enums
typedef enum
{
    STATE_MACHINE_STOPPED,
    SETUP_MAG_START_SAMPLE,
    WAITING_FOR_MAG_START_SAMPLE_COMPLETION,
    SETUP_NEXT_MAG_DATA_READ,
    WAITING_FOR_MAG_DATA_READ_COMPLETION,
    SETUP_NEXT_IMU_DATA_READ,
    WAITING_FOR_IMU_DATA_READ_COMPLETION,
    SETUP_NEXT_DATA_HOT_PLUG_CHECK,
    WAITING_FOR_DATA_MODE_HOT_PLUG_CHECK,
    SLEEP_WITH_IMU_PREPARE,

    CHECK_FOR_I2C_EVENT,
    SETUP_I2C_DATA_READ,
    WAITING_FOR_I2C_DATA_READ_COMPLETION,
    OMMOCOMM_DELAY_SETUP_MAG_SET_COMMAND,
    SETUP_MAG_SET_COMMAND,
    WAITING_FOR_MAG_SET_COMMAND,

    OMMOCOMM_DELAY_SETUP_STATE_MACHINE_RESTART,
    SETUP_STATE_MACHINE_RESTART,
    OMMOCOMM_DELAY_STATE_MACHINE_STOP,
    IDLE_HOT_PLUG_CHECK,
    PREPARED_FOR_START
} sensors_state_t;

  #define OMMO_SENSOR_I2C_TRX_INTERVAL    1000

  #define OMMO_SENSOR_SELFTEST_OVERSAMPLE 64
  #define OMMO_SENSOR_SPI_BUFFER_SIZE 32
#ifdef OMMOCOMM_ENABLED
  #define OMMO_SENSOR_MAG_SET_TIME US_TO_TICKS_16MHZ(700) //ATTENTION WAS 800 BEFORE OMMOCOMM!!!
#else
  #define OMMO_SENSOR_MAG_SET_TIME US_TO_TICKS_16MHZ(800)
#endif

#ifndef OMMO_SENSOR_NUM_FIXED_I2C_BUSES
  #define OMMO_SENSOR_NUM_I2C_BUS_LOCATIONS_SUPPORTED (2)
#else
  #define OMMO_SENSOR_NUM_I2C_BUS_LOCATIONS_SUPPORTED (2 + OMMO_SENSOR_NUM_FIXED_I2C_BUSES)
#endif

  #define SENSORS_MAX_EVENTS_CALLBACKS 3

  #define MAG_PRESENT(spi_ch, ss_ch) (ss_map[spi_ch][ss_ch] != 0 && ic_data[ss_map[spi_ch][ss_ch]].is_mag)
  #define IMU_PRESENT(spi_ch, ss_ch) (ss_map[spi_ch][ss_ch] != 0 && !ic_data[ss_map[spi_ch][ss_ch]].is_mag)

  #define IS_DISABLED(spi_ch, ss_ch) (ss_map[spi_ch][ss_ch] != 0 && ic_data[ss_map[spi_ch][ss_ch]].is_disabled)

  //2 bytes, 8 bits/byte, 16Mhz timestamp clock, 2Mhz SPI clock
  #ifndef OMMO_SENSOR_MEMSIC_MMC5983_8MHZ_SPI
  #define SENSOR_SPI_START_DELAY (2*8*16/2)
  #else
  #define SENSOR_SPI_START_DELAY (2*8*16/8)
  #endif

  //#define IMU_MAX_DATA_LENGTH 15
  #define MAX_DATA_READ_LENGTH (MAX(MMC5983_DATA_READ_LENGTH, MAX(IMU_DSX_DATA_LENGTH_BASE, IMU_42605_DATA_READ_LENGTH)))

  //Port pin count defines
  #define OMMO_SENSOR_NUM_PINS_PER_PORT (OMMO_PM_MAX_SPI_BUSSES_PER_PORT * (OMMO_PM_MAX_SS_PINS_PER_BUS+3))
  #define OMMO_SENSOR_TOTAL_PIN_COUNT (OMMO_PM_SPI_BUSSES_COUNT*3 + OMMO_PM_SS_PINS_COUNT)

  //Port pin test defines
  #define OMMO_SENSOR_PORT_PINS_TEST_MOSI_INDEX 0
  #define OMMO_SENSOR_PORT_PINS_TEST_CS0_INDEX  1
  #define OMMO_SENSOR_PORT_PINS_TEST_MISO_INDEX 2
  #define OMMO_SENSOR_PORT_PINS_TEST_SCK_INDEX  3

  #ifndef OMMO_SENSOR_EVENT_FIFO_LEN
  #define OMMO_SENSOR_EVENT_FIFO_LEN 4
  #endif

  //Sensor information dictionary
  typedef struct
  {
      uint8_t data_length;
      uint8_t data_offset;
      uint8_t data_read_tx[2];
      uint8_t data_read_tx_len;
      uint8_t data_read_rx_len;
      uint8_t data_output_index_offset; //0 when only 1 read command, used to express the sub offset for when there is more than 1 read command
  } ic_data_dictionary_read_command;

  typedef struct
  {
      uint8_t ic_type;
      bool is_disabled;
      bool is_mag;
      uint8_t num_read_commands;
      ic_data_dictionary_read_command read_command[4];
  } ic_data_dictionary_entry;

  //Port pin test defines
  #define PORT_TEST_BREAKOUT_SERIES_R_VALUE  0.0
  #define PORT_TEST_BREAKOUT_CAP_VALUE_UF    0.22
  typedef enum {
      PIN_STATUS_GOOD,
      PIN_STATUS_SHORTED_GND,
      PIN_STATUS_SHORTED_VDD,
      PIN_STATUS_OPEN,
      PIN_STATUS_INVALID,
      PIN_STATUS_SHORTED_NEIGHBOR
  } sensor_port_pin_status;
  const char* const sensor_port_test_pin_names[] = {"MOSI", "CS0", "MISO", "SCK", "CS1", "CS2", "CS3", "CS4","CS5", "CS6"};

  typedef enum
  {
    SENSORS_STOP_CMD_NONE = 0, /*!< The module will not be suspended. */
    SENSORS_STOP_CMD_SUSPEND, /*!< The module will be suspended and may resume on request only. */
    SENSORS_STOP_CMD_WAKE_ON_IMU, /*!< The module will be suspended, but may wake from sleep when IMU detects motion. */
  } sensors_stop_cmd_t;

  typedef enum
  {
    SENSORS_EVENT_NONE = 0,
    SENSORS_EVENT_WAKE_ON_IMU,
    SENSORS_EVENT_HOTPLUG_INSERT,
    SENSORS_EVENT_HOTPLUG_REMOVE,
    SENSORS_EVENT_DFU_STARTED,
    SENSORS_EVENT_DFU_FINISHED,
    SENSORS_STATE_MACHINE_CONFIGURED,
    SENSORS_STATE_MACHINE_UNCONFIGURED
  } sensors_event_t;

  //Callback changes to data and length, all data sequencing is done inside of this class
  typedef void (*sensors_callback_func_type)(uint8_t *data, uint16_t size, uint32_t timestamp, uint32_t timestamp_offset);
  typedef void (*sensors_event_callback_t)(sensors_event_t event);

  typedef void data_descriptor_callback_t(DeviceGroupDescriptorProto *dgdp, const void *context);

  //Public functions
  uint32_t sensors_start_state_machine();
  ret_code_t sensors_stop_state_machine(sensors_stop_cmd_t cmd = SENSORS_STOP_CMD_SUSPEND, bool blocking = true);
  uint8_t sensors_get_total_attached();
  void sensors_on_serial_closed();
  uint16_t sensors_process_packet_received(uint8_t data[], uint16_t data_length, uint8_t response_buffer[], uint16_t response_buffer_size);
  void sensors_set_data_descriptor_header_format(DataHeaderFormat dhf);
  uint16_t sensors_scan_bus_generate_data_descriptor(uint8_t packet_id_request_buffer[], uint16_t buffer_size, data_descriptor_callback_t *data_descriptor_callback = nullptr, const void *context = nullptr);
  bool sensors_hot_plugin_event_check();
  bool sensors_hot_unplug_event_check();
  uint8_t sensors_test(uint8_t port, device_info_storage_type *port_type, ICType *ic_type, uint8_t *ic_ss_index_bus_location, ICBusType *ic_bus_type);
  void sensor_port_pins_test(uint8_t port_num, sensor_port_pin_status* pin_test_result);
  //uint16_t sensors_read_self_test_info_physical_port(uint8_t port, uint8_t port_ss_index, uint16_t num_samples, uint8_t *output);
  void sensors_disable_outputs();
  void sensors_enable_outputs();  
  ret_code_t sensors_add_tasks_to_event_queue(event_queue_manager *event_queue);
  void sensors_start_idle_hot_plug_check();
  bool sensors_is_state_machine_ready_to_run();
  uint32_t sensors_add_event_callback(sensors_event_callback_t event_callback);
  void sensors_ommocomm_check_and_update_all_ports();
  bool sensors_are_resources_acquired();

  void sensors_init(sensors_callback_func_type sample_set_ready_callback_function);
