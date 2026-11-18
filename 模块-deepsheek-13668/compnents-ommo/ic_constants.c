#include "ic_constants.h"
#include "ommo_config.h"

/* =============== 
 * MAG 
 * =============== 
 */

//mmc5983
#ifdef MEMSIC_BIPOLAR_EXCITATION
const int32_t ic_def_mmc5983_sensor_sample_delay[] = {MMC5983_X_SAMPLE_DELAY, MMC5983_Y_SAMPLE_DELAY, MMC5983_Z_SAMPLE_DELAY};
const DeviceInfoSensorProto ic_def_mmc5983_sensors[] = {
    {.sensor_class = SENSOR_CLASS_MAG, .num_sensor_channels = 3, .sensor_sample_delay_count = 3, .sensor_sample_delay = (int32_t*)ic_def_mmc5983_sensor_sample_delay, .saturation_limit = MMC5983_SATURATION_LIMIT, .nominal_scale = MMC5983_SCALE},
    {.sensor_class = SENSOR_CLASS_MAG_POLARITY, .num_sensor_channels = 1}};
    
const DeviceInfoICProto ic_def_mmc5983 = {.ic_type = IC_TYPE_MEMSIC_5983, .sensors_count = 2, .ic_data_format = IC_DATA_FORMAT_MMC5983_BIPOLAR, .sensors = (DeviceInfoSensorProto*)ic_def_mmc5983_sensors};
#else
const int32_t ic_def_mmc5983_sensor_sample_delay[] = {MMC5983_X_SAMPLE_DELAY, MMC5983_Y_SAMPLE_DELAY, MMC5983_Z_SAMPLE_DELAY};
const DeviceInfoSensorProto ic_def_mmc5983_sensors[] = {
    {.sensor_class = SENSOR_CLASS_MAG, .num_sensor_channels = 3, .sensor_sample_delay_count = 3, .sensor_sample_delay = (int32_t*)ic_def_mmc5983_sensor_sample_delay, .saturation_limit = MMC5983_SATURATION_LIMIT, .nominal_scale = MMC5983_SCALE}};
    
const DeviceInfoICProto ic_def_mmc5983 = {.ic_type = IC_TYPE_MEMSIC_5983, .sensors_count = 1, .ic_data_format = IC_DATA_FORMAT_MMC5983_RESET, .sensors = (DeviceInfoSensorProto*)ic_def_mmc5983_sensors};
#endif

//mmc5603
const int32_t ic_def_mmc5603_sensor_sample_delay[] = {MMC5603_X_SAMPLE_DELAY, MMC5603_Y_SAMPLE_DELAY, MMC5603_Z_SAMPLE_DELAY};
const DeviceInfoSensorProto ic_def_mmc5603_sensors[] = {
    {.sensor_class = SENSOR_CLASS_MAG, .num_sensor_channels = 3, .sensor_sample_delay_count = 3, .sensor_sample_delay = (int32_t*)ic_def_mmc5603_sensor_sample_delay, .saturation_limit = MMC5603_SATURATION_LIMIT, .nominal_scale = MMC5603_SCALE},
    {.sensor_class = SENSOR_CLASS_TEMP, .num_sensor_channels = 1, .nominal_scale = MMC5983_TEMP_SCALE, .nominal_offset = MMC5983_TEMP_OFFSET, .sensor_disabled = true},
    {.sensor_class = SENSOR_CLASS_DEBUG_6_I16, .num_sensor_channels = 6}};
const DeviceInfoICProto ic_def_mmc5603 = {.ic_type = IC_TYPE_MEMSIC_5603, .sensors_count = 3, .ic_data_format = IC_DATA_FORMAT_MMC5603, .sensors = (DeviceInfoSensorProto*)ic_def_mmc5603_sensors};

//tle493d
//OMMO_TLE493D_HIGH_SENSITIVITY
const int32_t tle493d_mag_sensor_sample_delay_high_sensitivity[] = {TLE493D_X_SAMPLE_START_DELAY_HIGH_SENSITIVITY, TLE493D_Y_SAMPLE_START_DELAY_HIGH_SENSITIVITY, TLE493D_Z_SAMPLE_START_DELAY_HIGH_SENSITIVITY};
const DeviceInfoSensorProto ic_def_tle493d_sensors_high_sensitivity[] = {
    {.sensor_class = SENSOR_CLASS_MAG, .num_sensor_channels = 3, .sensor_sample_delay_count = 3, .sensor_sample_delay = (int32_t*)tle493d_mag_sensor_sample_delay_high_sensitivity, .saturation_limit = TLE493D_SATURATION_LIMIT_HIGH_SENSITIVITY, .nominal_scale = TLE493D_SCALE_HIGH_SENSITIVITY}, 
    {.sensor_class = SENSOR_CLASS_TEMP, .num_sensor_channels = 1, .nominal_scale = 0.96, .nominal_offset = -258.2, .sensor_disabled = true}};
const DeviceInfoICProto ic_def_tle493d_high_sensitivity = {.ic_type = IC_TYPE_TLE493D, .sensors_count = 2, .ic_data_format = IC_DATA_FORMAT_TLE493D, .config_enable_high_sensitivity = true, .sensors = (DeviceInfoSensorProto*)ic_def_tle493d_sensors_high_sensitivity};
//NORMAL SENSITIVITY
const int32_t tle493d_mag_sensor_sample_delay_no_options[] = {TLE493D_X_SAMPLE_START_DELAY_NORMAL, TLE493D_Y_SAMPLE_START_DELAY_NORMAL, TLE493D_Z_SAMPLE_START_DELAY_NORMAL};
const DeviceInfoSensorProto ic_def_tle493d_sensors_no_options[] = {
    {.sensor_class = SENSOR_CLASS_MAG, .num_sensor_channels = 3, .sensor_sample_delay_count = 3, .sensor_sample_delay = (int32_t*)tle493d_mag_sensor_sample_delay_no_options, .saturation_limit = TLE493D_SATURATION_LIMIT_NORMAL, .nominal_scale = TLE493D_SCALE_NORMAL}, 
    {.sensor_class = SENSOR_CLASS_TEMP, .num_sensor_channels = 1, .nominal_scale = 0.96, .nominal_offset = -258.2}};
const DeviceInfoICProto ic_def_tle493d_no_options = {.ic_type = IC_TYPE_TLE493D, .sensors_count = 2, .ic_data_format = IC_DATA_FORMAT_TLE493D_W_TEMP, .sensors = (DeviceInfoSensorProto*)ic_def_tle493d_sensors_no_options};


/* =============== 
 * IMU 
 * =============== 
 */
//lsm6ds33
//const DeviceInfoSensorProto ic_def_lsm6ds33_sensors[] = {
//    {.sensor_class = SENSOR_CLASS_ACCEL, .num_sensor_channels = 3, .saturation_limit = 2, .nominal_scale = IMU_DSX_ACCEL_SCALE},
//    {.sensor_class = SENSOR_CLASS_GYRO, .num_sensor_channels = 3, .saturation_limit = 1000, .nominal_scale = IMU_DSX_GYRO_SCALE}};
//const DeviceInfoICProto ic_def_lsm6ds33 = {.ic_type = IC_TYPE_LSM6DS33, .sensors_count = 2, .ic_data_format = IC_DATA_FORMAT_LSM6DSX, .sensors = (DeviceInfoSensorProto*)ic_def_lsm6ds33_sensors};

//lsm6dsm
const DeviceInfoSensorProto ic_def_lsm6dsm_sensors_no_options[] = {
    {.sensor_class = SENSOR_CLASS_ACCEL, .num_sensor_channels = 3, .saturation_limit = 2, .nominal_scale = IMU_DSX_ACCEL_SCALE},
    {.sensor_class = SENSOR_CLASS_GYRO, .num_sensor_channels = 3, .saturation_limit = 1000, .nominal_scale = IMU_DSX_GYRO_SCALE},
    {.sensor_class = SENSOR_CLASS_TIMESTAMP_OFFSET, .num_sensor_channels = 1, .sensor_disabled = true},
    {.sensor_class = SENSOR_CLASS_TEMP, .num_sensor_channels = 1, .nominal_offset = IMU_DSX_TEMP_OFFSET, .nominal_scale = IMU_DSX_TEMP_SCALE, .sensor_disabled = true}};
    const DeviceInfoICProto ic_def_lsm6dsm_no_options = {.ic_type = IC_TYPE_LSM6DSM, .sensors_count = 4, .ic_data_format = IC_DATA_FORMAT_LSM6DSX, .sensors = (DeviceInfoSensorProto*)ic_def_lsm6dsm_sensors_no_options};

const DeviceInfoSensorProto ic_def_lsm6dsm_sensors_temp_enabled[] = {
    {.sensor_class = SENSOR_CLASS_ACCEL, .num_sensor_channels = 3, .saturation_limit = 2, .nominal_scale = IMU_DSX_ACCEL_SCALE},
    {.sensor_class = SENSOR_CLASS_GYRO, .num_sensor_channels = 3, .saturation_limit = 1000, .nominal_scale = IMU_DSX_GYRO_SCALE},
    {.sensor_class = SENSOR_CLASS_TIMESTAMP_OFFSET, .num_sensor_channels = 1, .sensor_disabled = true},
    {.sensor_class = SENSOR_CLASS_TEMP, .num_sensor_channels = 1, .nominal_offset = IMU_DSX_TEMP_OFFSET, .nominal_scale = IMU_DSX_TEMP_SCALE}};
    const DeviceInfoICProto ic_def_lsm6dsm_temp_enabled = {.ic_type = IC_TYPE_LSM6DSM, .sensors_count = 4, .ic_data_format = IC_DATA_FORMAT_LSM6DSX_W_TEMP, .config_enable_temp = true, .sensors = (DeviceInfoSensorProto*)ic_def_lsm6dsm_sensors_temp_enabled};

const DeviceInfoSensorProto ic_def_lsm6dsm_sensors_offset_enabled[] = {
    {.sensor_class = SENSOR_CLASS_ACCEL, .num_sensor_channels = 3, .saturation_limit = 2, .nominal_scale = IMU_DSX_ACCEL_SCALE},
    {.sensor_class = SENSOR_CLASS_GYRO, .num_sensor_channels = 3, .saturation_limit = 1000, .nominal_scale = IMU_DSX_GYRO_SCALE},
    {.sensor_class = SENSOR_CLASS_TIMESTAMP_OFFSET, .num_sensor_channels = 1, .sensor_disabled = true},
    {.sensor_class = SENSOR_CLASS_TEMP, .num_sensor_channels = 1, .nominal_offset = IMU_DSX_TEMP_OFFSET, .nominal_scale = IMU_DSX_TEMP_SCALE}};
    const DeviceInfoICProto ic_def_lsm6dsm_offset_enabled = {.ic_type = IC_TYPE_LSM6DSM, .sensors_count = 4, .ic_data_format = IC_DATA_FORMAT_LSM6DSX_W_OFFSET, .config_enable_offset = true, .sensors = (DeviceInfoSensorProto*)ic_def_lsm6dsm_sensors_offset_enabled};


//lsm6dsv
const DeviceInfoSensorProto ic_def_lsm6dsv_sensors_no_options[] = {
    {.sensor_class = SENSOR_CLASS_ACCEL, .num_sensor_channels = 3, .saturation_limit = 2, .nominal_scale = IMU_DSX_ACCEL_SCALE},
    {.sensor_class = SENSOR_CLASS_GYRO, .num_sensor_channels = 3, .saturation_limit = 1000, .nominal_scale = IMU_DSX_GYRO_SCALE},
    {.sensor_class = SENSOR_CLASS_TIMESTAMP_OFFSET, .num_sensor_channels = 1, .sensor_disabled = true},
    {.sensor_class = SENSOR_CLASS_TEMP, .num_sensor_channels = 1, .nominal_offset = IMU_DSX_TEMP_OFFSET, .nominal_scale = IMU_DSX_TEMP_SCALE, .sensor_disabled = true}};
    const DeviceInfoICProto ic_def_lsm6dsv_no_options = {.ic_type = IC_TYPE_LSM6DSV, .sensors_count = 4, .ic_data_format = IC_DATA_FORMAT_LSM6DSX, .sensors = (DeviceInfoSensorProto*)ic_def_lsm6dsv_sensors_no_options};


// icm42605
const DeviceInfoSensorProto ic_def_icm42605_sensors[] = {
    {.sensor_class = SENSOR_CLASS_ACCEL, .num_sensor_channels = 3, .nominal_scale = IMU_42605_ACCEL_SCALE},
    {.sensor_class = SENSOR_CLASS_GYRO, .num_sensor_channels = 3, .nominal_scale = IMU_42605_GYRO_SCALE},
    {.sensor_class = SENSOR_CLASS_TIMESTAMP_OFFSET, .num_sensor_channels = 1}};
const DeviceInfoICProto ic_def_icm42605 = {.ic_type = IC_TYPE_ICM_42605, .sensors_count = 3, .ic_data_format = IC_DATA_FORMAT_ICM42605, .sensors = (DeviceInfoSensorProto*)ic_def_icm42605_sensors};


/* =============== 
 * MISC 
 * =============== 
 */
//encoder
const DeviceInfoSensorProto ic_def_encoder_sensors[] = { 
    {.sensor_class = SENSOR_CLASS_TIMESTAMP_OFFSET, .num_sensor_channels = 1, .nominal_scale = 1.0},
    {.sensor_class = SENSOR_CLASS_ROTARY_ENCODER_POS, .num_sensor_channels = 1, .nominal_scale = KODENSHI1340_DEFAULT_PPR}, 
    {.sensor_class = SENSOR_CLASS_ROTARY_ENCODER_SPEED, .num_sensor_channels = 1, .nominal_scale = ENCODER_SPEED_SCALE}};
const DeviceInfoICProto ic_def_encoder = {.ic_type = IC_TYPE_KODENSHI1340, .encoder_ppr = KODENSHI1340_DEFAULT_PPR, .encoder_type = ENCODER_TYPE_BS_MECH, .nominal_rotation_rate_hz = A49406_SPD_DEFAULT, 
                                          .sensors_count = 3, .ic_data_format = IC_DATA_FORMAT_UI16_ENCODER_W_OFFSET, .sensors = (DeviceInfoSensorProto*)ic_def_encoder_sensors};             

//encoder cnc 
const DeviceInfoSensorProto ic_def_encoder_generic_sensors[] = {
    {.num_sensor_channels = 1, .sensor_disabled = true}, 
    {.num_sensor_channels = 1, .sensor_disabled = true}};                                                     
const DeviceInfoICProto ic_def_encoder_generic = {.ic_type = IC_TYPE_ENCODER_GENERIC, .sensors_count = 2, .ic_data_format = IC_DATA_FORMAT_OFF, .sensors = (DeviceInfoSensorProto*)ic_def_encoder_generic_sensors};   

//generic adc
const DeviceInfoSensorProto ic_def_generic_adc_sensors[] = {{.num_sensor_channels = 3}};
const DeviceInfoICProto ic_def_generic_adc = {.sensors_count = 1, .ic_type = IC_TYPE_ADS127L01_GENERIC, .ic_data_format = IC_DATA_FORMAT_ADS127L01, 
                                              .sensors = (DeviceInfoSensorProto*)ic_def_generic_adc_sensors}; 

//generic adc ads127l11
const DeviceInfoSensorProto ic_def_generic_adc_sensors_ads127l11[] = {
    {.sensor_class = SENSOR_CLASS_MAG, .num_sensor_channels = 3},
    {.sensor_class = SENSOR_CLASS_TEMP, .num_sensor_channels = 1}
    //,
    //{.sensor_class = SENSOR_CLASS_VOLTAGE, .num_sensor_channels = 1},
    //{.sensor_class = SENSOR_CLASS_CURRENT, .num_sensor_channels = 1}
    };
const DeviceInfoICProto ic_def_generic_adc_ads127l11 = {.sensors_count = 2, .ic_type = IC_TYPE_ADS127L11_GENERIC, .ic_data_format = IC_DATA_FORMAT_ADS127L11, 
                                                        .sensors = (DeviceInfoSensorProto*)ic_def_generic_adc_sensors_ads127l11}; 

//generic adc ads127l11 w/ debug data
const DeviceInfoSensorProto ic_def_adc_ads127l11_debug_sensors[] = {
    {.sensor_class = SENSOR_CLASS_MAG, .num_sensor_channels = 3},
    {.sensor_class = SENSOR_CLASS_TEMP, .num_sensor_channels = 1},
    {.sensor_class = SENSOR_CLASS_VOLTAGE, .num_sensor_channels = 3},
    {.sensor_class = SENSOR_CLASS_CURRENT, .num_sensor_channels = 3}
    };
const DeviceInfoICProto ic_def_adc_ads127l11_debug = {.sensors_count = 4, .ic_type = IC_TYPE_ADS127L11_GENERIC, .ic_data_format = IC_DATA_FORMAT_ADS127L11_DEBUG, 
                                                      .sensors = (DeviceInfoSensorProto*)ic_def_adc_ads127l11_debug_sensors}; 

//tmp126 temperature sensor
const DeviceInfoSensorProto ic_def_tmp126_sensors[] = {
    {.sensor_class = SENSOR_CLASS_TEMP, .num_sensor_channels = 1, .nominal_scale = 0.03125, .nominal_offset = 0}};
const DeviceInfoICProto ic_def_tmp126 = {.sensors_count = 1, .ic_type = IC_TYPE_TMP126, .ic_data_format = IC_DATA_FORMAT_TMP126,
                                                     .sensors = (DeviceInfoSensorProto*)ic_def_tmp126_sensors}; 

//tcs3472 color sensor
const DeviceInfoSensorProto ic_def_tcs3472_sensors[] = {{.sensor_class = SENSOR_CLASS_COLOR, .num_sensor_channels = 4}};
const DeviceInfoICProto ic_def_tcs3472 = {.sensors_count = 1, .ic_type = IC_TYPE_TCS3472, .ic_data_format = IC_DATA_FORMAT_TCS3472,
                                          .sensors = (DeviceInfoSensorProto*)ic_def_tcs3472_sensors};
                                                    
//hx711 adc (weight sensor)
const DeviceInfoSensorProto ic_def_hx711_sensors[] = {{.sensor_class = SENSOR_CLASS_WEIGHT, .num_sensor_channels = 1}};
const DeviceInfoICProto ic_def_hx711 = {.sensors_count = 1, .ic_type = IC_TYPE_HX711, .ic_data_format = IC_DATA_FORMAT_I32LE,
                                          .sensors = (DeviceInfoSensorProto*)ic_def_hx711_sensors};
//mfam fancy mag
const int32_t ic_def_mfam_sample_delay[] = {0};
const DeviceInfoSensorProto ic_def_mfam_sensors[] =
  {
      {.sensor_class = SENSOR_CLASS_MAG, .num_sensor_channels = 1, .nominal_scale = 50e-9, .sensor_sample_delay = (int32_t*)ic_def_mfam_sample_delay}, //Mag 0
      {.sensor_class = SENSOR_CLASS_MAG_STATUS, .num_sensor_channels = 1}, //Mag 0 status
      {.sensor_class = SENSOR_CLASS_MAG, .num_sensor_channels = 1, .nominal_scale = 50e-9, .sensor_sample_delay = (int32_t*)ic_def_mfam_sample_delay}, //Mag 1
      {.sensor_class = SENSOR_CLASS_MAG_STATUS, .num_sensor_channels = 1}, //Mag 1 status
      {.sensor_class = SENSOR_CLASS_SYSTEM_STATUS, .num_sensor_channels = 1}, //System status
      {.sensor_class = SENSOR_CLASS_UNKNOWN, .num_sensor_channels = 1}, //frame id
      {.sensor_class = SENSOR_CLASS_UNKNOWN, .num_sensor_channels = 4} //Aux channels
  };
const DeviceInfoICProto ic_def_mfam = {.sensors_count = 7, .ic_type = IC_TYPE_MFAM, .ic_data_format = IC_DATA_FORMAT_MFAM, 
                                       .tracking_unit_index = 1, .sensors = (DeviceInfoSensorProto*)ic_def_mfam_sensors}; 

//tmp1075n temperature sensor
const DeviceInfoSensorProto ic_def_tmp1075n_sensors[] = {
    {.sensor_class = SENSOR_CLASS_TEMP, .num_sensor_channels = 1, .nominal_scale = 0.0625, .nominal_offset = 0}};
const DeviceInfoICProto ic_def_tmp1075n = {.sensors_count = 1, .ic_type = IC_TYPE_TMP1075N, .ic_data_format = IC_DATA_FORMAT_TMP1075N,
                                                     .sensors = (DeviceInfoSensorProto*)ic_def_tmp1075n_sensors};

//tmp110 temperature sensor
const DeviceInfoSensorProto ic_def_tmp110_sensors[] = {
    {.sensor_class = SENSOR_CLASS_TEMP, .num_sensor_channels = 1, .nominal_scale = 0.0625, .nominal_offset = 0}};
const DeviceInfoICProto ic_def_tmp110 = {.sensors_count = 1, .ic_type = IC_TYPE_TMP110, .ic_data_format = IC_DATA_FORMAT_TMP1075N,
                                         .sensors = (DeviceInfoSensorProto*)ic_def_tmp110_sensors};

//analog button
const DeviceInfoSensorProto ic_def_analog_button_sensors[] = {{.sensor_class = SENSOR_CLASS_BUTTON, .num_sensor_channels = 1}};
const DeviceInfoICProto ic_def_analog_button = {.sensors_count = 1, .ic_type = IC_TYPE_BUTTON, .ic_data_format = IC_DATA_FORMAT_BUTTON_MAP,
                                                .sensors = (DeviceInfoSensorProto*)ic_def_analog_button_sensors}; 

//bq27z558 fuel gauge
const DeviceInfoSensorProto ic_def_bq27z558_sensors[] = {
    {.sensor_class = SENSOR_CLASS_BATTERY_SOC, .num_sensor_channels = 1, .nominal_scale = .01, .nominal_offset = 0},
    {.sensor_class = SENSOR_CLASS_CURRENT, .num_sensor_channels = 1, .nominal_scale = 1, .nominal_offset = 0},
    {.sensor_class = SENSOR_CLASS_BATTERY_REMAINING_CAPACITY, .num_sensor_channels = 1, .nominal_scale = 1, .nominal_offset = 0} };
const DeviceInfoICProto ic_def_bq27z558 = {.sensors_count = 3, .ic_type = IC_TYPE_BQ27Z558, .ic_data_format = IC_DATA_FORMAT_BQ27Z558,
                                           .sensors = (DeviceInfoSensorProto*)ic_def_bq27z558_sensors}; 

//battery charger tester sensor
const DeviceInfoSensorProto ic_def_batt_charger_tester_sensors[] = {
    {.sensor_class = SENSOR_CLASS_CHARGER_CHARGE_VOLTAGE, .num_sensor_channels = 1},
    {.sensor_class = SENSOR_CLASS_CHARGER_CHARGE_CURRENT, .num_sensor_channels = 1, .nominal_scale = 1e-3},
    };
const DeviceInfoICProto ic_def_batt_charger_tester = {.sensors_count = 2, .ic_type = IC_TYPE_CHARGER_TESTER, .ic_data_format = IC_DATA_FORMAT_CHARGER_TESTER,
                                                      .sensors = (DeviceInfoSensorProto*)ic_def_batt_charger_tester_sensors};

//master siu tester
const DeviceInfoICProto ic_def_master_siu_tester = {.sensors_count = 0, .ic_type = IC_TYPE_MASTER_SIU_TESTER};
