#pragma once

#include "ommo_config.h"
#include "utils.hpp"

#include "SoftI2C.hpp"
#include "device_info_command_processor.hpp"
#include "ic_constants.h"
typedef void (*adc_sensors_callback_func_type)(uint8_t *adc_data, uint16_t adc_data_size, uint32_t timestamp, uint32_t timestamp_offset);

//Public functions
void adc_sensors_init(adc_sensors_callback_func_type sample_set_ready_callback_function);
ret_code_t adc_sensors_enable();
void adc_sensors_disable();
void adc_fir_event_queue_process();
ret_code_t adc_sensors_dry_run_blocking(uint8_t dry_run_times = 10);
bool adc_sensors_is_state_machine_configured();
uint16_t adc_scan_bus_generate_data_descriptor(uint8_t packet_id_request_buffer[], uint16_t buffer_size);
uint16_t adc_process_packet_received(uint8_t data[], uint16_t data_length, uint8_t response_buffer[], uint16_t response_buffer_size);