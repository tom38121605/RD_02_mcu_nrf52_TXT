#ifndef IC_CONSTANTS_H__
#define IC_CONSTANTS_H__

#include <stdint.h>
#include "ommo_fw.pb.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "ommo_config.h"

#ifndef US_TO_TICKS_16MHZ
#define US_TO_TICKS_16MHZ(x) ((uint32_t)(16*(x)))
#endif

/* =============== MEMSIC TEMP SENSOR CONSTANTS AND VALUES =============== */
//#define MEMSIC_BIPOLAR_EXCITATION

#define MMC5983_TEMP_SCALE   0.78431
#define MMC5983_TEMP_OFFSET  -75

/* =============== MMC5983 CONSTANTS AND VALUES =============== */

#define MMC5983_X_SAMPLE_DELAY 4850 //Experimentally determined
#define MMC5983_Y_SAMPLE_DELAY 4850
#define MMC5983_Z_SAMPLE_DELAY 4850

#define MMC5983_DATA_READ_LENGTH (MMC5983_DATA_LENGTH + MMC5983_DATA_OFFSET)
#define MMC5983_DATA_LENGTH 7
#define MMC5983_DATA_OFFSET 1

#define MMC5983_SCALE 6.104e-03 //uT/bit
#define MMC5983_SATURATION_LIMIT 700
//#define OMMO_SENSOR_MEMSIC_MMC5983_TEMP_COUNT 1000
#define MMC5983_READ_REG_MASK(x) (x | 0x80)

//MEMSIC registers  
#define MMC5983_XOUT_REG            0x00
#define MMC5983_TOUT_REG            0x07
#define MMC5983_STATUS_REG          0x08
#define MMC5983_CONTROL0_REG        0x09
#define MMC5983_CONTROL1_REG        0x0A
#define MMC5983_CONTROL2_REG        0x0B
#define MMC5983_CONTROL3_REG        0x0C
#define MMC5983_PRODUCT_ID_REG      0x2F

#define MMC5983_PRODUCT_ID_RESULT   0x30

#define MMC5983_UNLOCK_REG 0x0F
#define MMC5983_UNLOCK_VAL 0xE1

//MEMSIC values
#define MMC5983_CONTROL0_MAG_MEAS   0x01
#define MMC5983_CONTROL0_TEMP_MEAS  0x02
#define MMC5983_CONTROL0_INT_EN     0x04
#define MMC5983_CONTROL0_SET        0x08
#define MMC5983_CONTROL0_RESET      0x10
#define MMC5983_CONTROL0_INIT       0x00
#define MMC5983_CONTROL0_SET_CMD  (MMC5983_CONTROL0_RESET)
#define MMC5983_CONTROL0_START_CMD  (MMC5983_CONTROL0_MAG_MEAS)

#ifdef OMMO_SENSOR_MEMSIC_MMC5983_SET
  #define MMC5983_CONTROL0_SET_CMD    (MMC5983_CONTROL0_SET)
#else
  #define MMC5983_CONTROL0_SET_CMD    (MMC5983_CONTROL0_RESET) //RESET will invert the axis making them right handed
#endif

#define MMC5983_CONTROL1_CHIP_RESET 0x83
#define MMC5983_CONTROL1_BW_100     0x00
#define MMC5983_CONTROL1_BW_200     0x01
#define MMC5983_CONTROL1_BW_400     0x02
#define MMC5983_CONTROL1_BW_800     0x03
#define MMC5983_CONTROL1_INIT       MMC5983_CONTROL1_BW_800

#define MMC5983_CONTROL3_RESET      0x00
#define MMC5983_CONTROL3_ST_ENP     0x02
#define MMC5983_CONTROL3_ST_ENM     0x04



/* =============== MMC5603 CONSTANTS AND VALUES =============== */

//#define MMC5603_DATA_FORMAT 2     // IC_DATA_FORMAT_MMC5603
#define MMC5603_SCALE 6.10351547e-03
#define MMC5603_SATURATION_LIMIT 2800

#define MMC5603_X_SAMPLE_DELAY -11296 //Experimentally determined
#define MMC5603_Y_SAMPLE_DELAY -4016
#define MMC5603_Z_SAMPLE_DELAY -3984

#define MMC5603_DATA_LENGTH 9

//Mag registers
#define MMC5603_SLAVE_ADDR 0x30

#define MMC5603_XOUT_REG        0x00
#define MMC5603_TOUT_REG        0x09
#define MMC5603_STATUS_REG      0x18
#define MMC5603_CONTROL0_REG    0x1B
#define MMC5603_CONTROL1_REG    0x1C
#define MMC5603_CONTROL2_REG    0x1D
#define MMC5603_ODR_REG         0x1A
#define MMC5603_ID_REG          0x39

//Mag values
#define MMC5603_ID_RESP 0x10
#define MMC5603_SINGLE_MEASUREMENT 0x01
#define MMC5603_WIA1_RESULT 0x48



/* =============== TLE493D CONSTANTS AND VALUES =============== */

#define TLE493D_SCALE_HIGH_SENSITIVITY 64.935 //uT/bit
#define TLE493D_SATURATION_LIMIT_HIGH_SENSITIVITY 160000
#define TLE493D_X_SAMPLE_START_DELAY_HIGH_SENSITIVITY          US_TO_TICKS_16MHZ(56.0/2 + 122) //Measured, this includes the sampletime to sample start delay
#define TLE493D_Y_SAMPLE_START_DELAY_HIGH_SENSITIVITY          US_TO_TICKS_16MHZ(56.0 + 59.0/2 + 119) //Measured, this includes the sampletime to sample start delay
#define TLE493D_Z_SAMPLE_START_DELAY_HIGH_SENSITIVITY          US_TO_TICKS_16MHZ(56.0 + 59.0 + 59.0/2 + 110) //Measured, this includes the sampletime to sample start delay

#define TLE493D_SCALE_NORMAL 129.870 //uT/bit
#define TLE493D_SATURATION_LIMIT_NORMAL 160000
#define TLE493D_X_SAMPLE_START_DELAY_NORMAL          US_TO_TICKS_16MHZ(40.0/2) //Guess from datasheet, should add ~128 for the sampletime to sample start delay but not used
#define TLE493D_Y_SAMPLE_START_DELAY_NORMAL          US_TO_TICKS_16MHZ(40.0 + 43.0/2) //Guess from datasheet, should add ~128 for the sampletime to sample start delay but not used
#define TLE493D_Z_SAMPLE_START_DELAY_NORMAL          US_TO_TICKS_16MHZ(40.0 + 43.0 + 43.0/2) //Guess from datasheet, should add ~128 for the sampletime to sample start delay but not used

#define TLE493D_STARTUPDELAY                  600
#define TLE493D_RESETDELAY                    30

//TLE493D general
#define TLE493D_SLAVE_ADDR          0x35
#define TLE493D_HW_VER              0x09
#define TLE493D_DATA_READ_LENGTH    6
#define TLE493D_DATA_OUTPUT_LENGTH  8

//TLE493D registers
#define TLE493D_BX      0x00
#define TLE493D_BY      0x01
#define TLE493D_BZ      0x02
#define TLE493D_TEMP    0x03
#define TLE493D_BX2     0x04
#define TLE493D_TEMP2   0x05
#define TLE493D_DIAG    0x06
#define TLE493D_CONFIG  0x10
#define TLE493D_MOD1    0x11
#define TLE493D_MOD2    0x13
#define TLE493D_VER     0x16
#define TLE493D_NUM_REGS (TLE493D_VER+1)

//TLE493D bits
#define TLE493D_MOD1_FP     0x80
#define TLE493D_MOD1_PR     0x10
#define TLE493D_MOD1_CA     0x08
#define TLE493D_MOD1_INT    0x04
#define TLE493D_MOD1_MODE_LOW_POWER 0x00
#define TLE493D_MOD1_MODE_MASTER    0x01
#define TLE493D_MOD1_MODE_FAST      0x03

#define TLE493D_CONFIG_DT   0x80
#define TLE493D_CONFIG_AM   0x40
#define TLE493D_CONFIG_TRIG_BEFORE_MSB 0x10
#define TLE493D_CONFIG_TRIG_AFTER_REG5 0x20
#define TLE493D_CONFIG_X2   0x08
#define TLE493D_CONFIG_TL0  0x00
#define TLE493D_CONFIG_TL1  0x02
#define TLE493D_CONFIG_TL2  0x04
#define TLE493D_CONFIG_TL3  0x06
#define TLE493D_CONFIG_CP   0x01

#define TLE493D_WRITE_TRIG_AFTER_WRITE 0x20

//TLE493D values
#define TLE493D_CONFIG_VAL_HIGH_SENSITIVITY (TLE493D_CONFIG_DT | TLE493D_CONFIG_TRIG_AFTER_REG5 | TLE493D_CONFIG_X2 | TLE493D_CONFIG_TL0) //temp off, xyz on, trig after read, short range, no mag comp
#define TLE493D_CONFIG_VAL_NO_OPTIONS (0x00 | TLE493D_CONFIG_TRIG_AFTER_REG5 | TLE493D_CONFIG_TL0) //temp on, xyz on, trig after read, long range, no mag comp
#define TLE493D_MOD1_VAL (0x00 | TLE493D_MOD1_PR & ~(TLE493D_MOD1_CA) | TLE493D_MOD1_INT | TLE493D_MOD1_MODE_MASTER) //0x15 b'?001 0101  //1 byte protocol, collision avoidance on, interrupt off, master controlled mode


/* =============== IMU DSX CONSTANTS AND VALUES =============== */

//IMU constants
#define IMU_DSX_ACCEL_SCALE 6.1e-05
#define IMU_DSX_GYRO_SCALE 0.035
#define IMU_DSX_TEMP_OFFSET 25
#define IMU_DSX_TEMP_SCALE 3.906E-03
#define IMU_DSX_READ_REG_MASK(x) (x | 0x80)
#define IMU_DSX_WRITE_REG_MASK(x) (x & 0xFF)

#define IMU_DSX_DATA_LENGTH_BASE          12
#define IMU_DSX_DATA_LENGTH_ADDER_TSO     2
#define IMU_DSX_DATA_LENGTH_ADDER_TEMP    2

#define IMU_DSX_DATA_OFFSET             1

//IMU registers
// Shared by LSM6DS33, LSM6DSM, and LSM6DSV -- all use the same register addresses.
#define IMU_DSX_SLAVE_ADDR    0x6A

#define IMU_DSX_DRDY_PULSE_CFG  0x0B
#define IMU_DSX_INT1_CTRL       0x0D
#define IMU_DSX_CMD_WHO_AM_I    0x0F
#define IMU_DSX_CTRL1_XL        0x10
#define IMU_DSX_CTRL2_G         0x11
#define IMU_DSX_CTRL3_C         0x12
#define IMU_DSX_CTRL4_C         0x13
#define IMU_DSX_CTRL9_XL        0x18
#define IMU_DSX_CTRL10_C        0x19
#define IMU_DSX_MASTER_CONFIG   0x1A
#define IMU_DSX_OUTX_L_G        0x22  //gyro output start (6 bytes); accel follows immediately via IF_INC
#define IMU_DSX_OUTX_L_A        0x28  //accel output start (6 bytes)
#define OUT_TEMP_L              0x20  //temp, outx, outy, outz for both gyro and accel are consecutive

//IMU values
#define IMU_DSX_CTRL4_C_I2C_OFF     0x04
#define IMU_DS33_CTRL1_XL_INIT      0x80
#ifdef OMMO_IMU_HF
    #define IMU_DS33_CTRL2_G_INIT   0x98
#else
    #define IMU_DS33_CTRL2_G_INIT   0x88
#endif
#define IMU_DS33_CTRL3_C_INIT       0x44
#define IMU_DS33_CTRL9_XL_INIT      0x38
#define IMU_DS33_CTRL10_C_INIT      0x38
#define IMU_DS33_WHO_AM_I           0x69

#define IMU_DSM_CTRL1_XL_INIT       0x81
#ifdef OMMO_IMU_HF
    #define IMU_DSM_CTRL2_G_INIT    0x98
#else
    #define IMU_DSM_CTRL2_G_INIT    0x88
#endif
#define IMU_DSM_CTRL3_C_INIT        0x44
#define IMU_DSM_CTRL9_XL_INIT       0x00
#define IMU_DSM_CTRL10_C_INIT       0x00
#define IMU_DSM_WHO_AM_I            0x6A
#define IMU_DSX_INT1_CTRL_EN_G      0x02
#define IMU_DSX_INT1_CTRL_EN_XL     0x01
#define IMU_DSX_MASTER_CONFIG_DRDY_ON_INT1  0x80
#define IMU_DSX_DRDY_PULSE_CFG_PULSE_MODE   0x80

/* =============== LSM6DSV CONSTANTS AND VALUES =============== */
#define IMU_DSV_WHO_AM_I            0x70   // differs from LSM6DSM (0x6A); register address is the same

// CTRL1 (accel) / CTRL2 (gyro): bits[6:4]=OP_MODE, bits[3:0]=ODR
#define IMU_DSV_OP_MODE_ODR_TRIGGERED   0x03   // bits[6:4] = 011
#define IMU_DSV_ODR_CODE_960HZ          0x09   // bits[3:0]
#define IMU_DSV_CTRL1_INIT  ((IMU_DSV_OP_MODE_ODR_TRIGGERED << 4) | IMU_DSV_ODR_CODE_960HZ)  // 0x39
#define IMU_DSV_CTRL2_G_INIT  IMU_DSV_CTRL1_INIT
#define IMU_DSV_CTRL1_POWER_DOWN  0x00
#define IMU_DSV_CTRL2_POWER_DOWN  0x00

// CTRL3: BDU=1 (prevents mid-burst register update), IF_INC=1 (auto-increment multi-byte reads)
#define IMU_DSV_CTRL3_BDU     (1 << 6)
#define IMU_DSV_CTRL3_IF_INC  (1 << 2)
#define IMU_DSV_CTRL3_INIT    (IMU_DSV_CTRL3_BDU | IMU_DSV_CTRL3_IF_INC)  // 0x44

// CTRL4: INT2_IN_LH=1 for rising-edge sensitivity (DT0155 step 4). Datasheet omits this for ODR-trig;
// AN5922 s3.3 requires it with min 5us pulse width. Our pulse (PPI assert, ts-timer-cb deassert) = 13.5us.
#define IMU_DSV_CTRL4_DRDY_PULSED  (1 << 1)
#define IMU_DSV_CTRL4_INT2_IN_LH   (1 << 0)
#define IMU_DSV_CTRL4_INIT         (IMU_DSV_CTRL4_INT2_IN_LH)

// IF_CFG (0x03): disable I2C/I3C -- SPI-only. Not reset by SW_RESET, only by power-cycle.
#define IMU_DSX_IF_CFG_REG      0x03
#define IMU_DSV_IF_CFG_INIT     0x01

// ODR_TRIG_CFG (0x06): N=4 -> sensor produces 2*N=8 DRDYs per INT2 period
#define IMU_DSV_ODR_TRIG_CFG_REG             0x06
#define IMU_DSV_ODR_TRIG_N                   4
#define IMU_DSV_ODR_TRIG_CFG_INIT            IMU_DSV_ODR_TRIG_N
#define IMU_DSV_ODR_TRIG_SAMPLES_PER_PERIOD  (IMU_DSV_ODR_TRIG_N * 2)  // = 8

// INT1_CTRL: route gyro DRDY to INT1 only -- one clean pulse per sample period
#define IMU_DSV_INT1_DRDY_XL    (1 << 0)
#define IMU_DSV_INT1_DRDY_G     (1 << 1)
#define IMU_DSV_INT1_CTRL_INIT  IMU_DSV_INT1_DRDY_G  // 0x02

// CTRL6 (0x15): Gyro full-scale; bits[2:0]=FS_G, bits[7:6]=LPF1_G_BW (not applicable in ODR-trig mode)
#define IMU_DSV_CTRL6_G_REG     0x15
#define IMU_DSV_FS_G_125DPS     0x00   // ±125 dps
#define IMU_DSV_FS_G_250DPS     0x01   // ±250 dps
#define IMU_DSV_FS_G_500DPS     0x02   // ±500 dps
#define IMU_DSV_FS_G_1000DPS    0x03   // ±1000 dps
#define IMU_DSV_FS_G_2000DPS    0x04   // ±2000 dps
#define IMU_DSV_FS_G_4000DPS    0x07   // ±4000 dps
#define IMU_DSV_CTRL6_G_INIT    IMU_DSV_FS_G_1000DPS

// CTRL7 (0x16): Gyro filter config; default 0x00 (HPF off)
#define IMU_DSV_CTRL7_G_REG     0x16
#define IMU_DSV_CTRL7_G_INIT    0x00

// CTRL8 (0x17): Accel full-scale and LPF2 config; bits[2:1]=FS_XL, bits[7:5]=LPF2/HP BW
#define IMU_DSV_CTRL8_XL_REG    0x17
#define IMU_DSV_FS_XL_2G        0x00   // ±2g
#define IMU_DSV_FS_XL_4G        0x02   // ±4g
#define IMU_DSV_FS_XL_8G        0x04   // ±8g
#define IMU_DSV_FS_XL_16G       0x06   // ±16g
#define IMU_DSV_CTRL8_XL_INIT   IMU_DSV_FS_XL_2G   // ±2g, matches DSM; LPF2 and HPF disabled (bits[7:3]=0)

// CTRL9 (0x18): Accel HP/LPF2 enable; default 0x00 (LPF2 off, HPF off)
// Register address shared with IMU_DSX_CTRL9_XL (0x18); DSV fields differ from DSM.
#define IMU_DSV_CTRL9_XL_INIT   0x00



/* =============== IMU 42605 CONSTANTS AND VALUES =============== */

// IMU constants
#define IMU_42605_ACCEL_SCALE (8.0f / (1 << 15))
#define IMU_42605_GYRO_SCALE  (2000.0f / (1 << 15))
#define IMU_42605_ACCEL_WOM_THR(g) ((g >= 1.0f) ? 0xFF : (uint8_t)(255.0f * g))

#define IMU_42605_DATA_READ_LENGTH  15
#define IMU_42605_DATA_LENGTH       14
#define IMU_42605_DATA_OFFSET       1
#define IMU_42605_FSYNC_DATA_OFFSET (IMU_42605_DATA_OFFSET + 12)
#define IMU_42605_WHO_AM_I          0x42
#define IMU_42605_READ_REG_MASK(x)      (x | 0x80)

// Used as Delay command in a dispatching for loop
#define IMU_42605_CMD_DELAY 0xFF

// IMU registers
// Bank 0
#define IMU_42605_REG_DEVICE_CFG         0x11
#define IMU_42605_REG_INT_CONFIG         0x14
#define IMU_42605_REG_FIFO_CONFIG        0x16
#define IMU_42605_REG_ACCEL_DATA_X1      0x1F
#define IMU_42605_REG_INT_STATUS2        0x37
#define IMU_42605_SIGNAL_PATH_RESET      0x4B
#define IMU_42605_REG_INTF_CONFIG        0x4C
#define IMU_42605_REG_PWR_MGMT0          0x4E
#define IMU_42605_REG_GYRO_CONFIG0       0x4F
#define IMU_42605_REG_ACCEL_CONFIG0      0x50
#define IMU_42605_REG_GYRO_CONFIG1       0x51
#define IMU_42605_REG_GYRO_ACCEL_CONFIG0 0x52
#define IMU_42605_REG_ACCEL_CONFIG1      0x53
#define IMU_42605_REG_TMST_CONFIG        0x54
#define IMU_42605_REG_SMD_CONFIG         0x57
#define IMU_42605_REG_INT_SOURCE4        0x69
#define IMU_42605_REG_WHO_AM_I           0x75
#define IMU_42605_REG_BANK_SEL           0x76

// Bank 4
#define IMU_42605_REG_4_ACCEL_WOM_X_THR 0x4A
#define IMU_42605_REG_4_ACCEL_WOM_Y_THR 0x4B
#define IMU_42605_REG_4_ACCEL_WOM_Z_THR 0x4C

// Registers fields
#define IMU_42605_BANK_0_SEL 0x00
#define IMU_42605_BANK_1_SEL 0x01
#define IMU_42605_BANK_2_SEL 0x02
#define IMU_42605_BANK_3_SEL 0x03
#define IMU_42605_BANK_4_SEL 0x04

#define IMU_42605_DEVICE_CFG_RESET 0x01

#define IMU_42605_GYRO_CONFIG0_FS_SEL_16DPS   (0x07 << 5) /*!< 16DPS*/
#define IMU_42605_GYRO_CONFIG0_FS_SEL_31DPS   (0x06 << 5) /*!< 31DPS*/
#define IMU_42605_GYRO_CONFIG0_FS_SEL_62DPS   (0x05 << 5) /*!< 62DPS*/
#define IMU_42605_GYRO_CONFIG0_FS_SEL_125DPS  (0x04 << 5) /*!< 125DPS*/
#define IMU_42605_GYRO_CONFIG0_FS_SEL_250DPS  (0x03 << 5) /*!< 250DPS*/
#define IMU_42605_GYRO_CONFIG0_FS_SEL_500DPS  (0x02 << 5) /*!< 500DPS*/
#define IMU_42605_GYRO_CONFIG0_FS_SEL_1000DPS (0x01 << 5) /*!< 1000DPS*/
#define IMU_42605_GYRO_CONFIG0_FS_SEL_2000DPS (0x00 << 5) /*!< 2000DPS*/
#define IMU_42605_GYRO_CONFIG0_ODR_500HZ     0x0F        /*!< 500 Hz (2 ms)*/
#define IMU_42605_GYRO_CONFIG0_ODR_12_5HZ    0x0B        /*!< 12.5 Hz (80 ms)*/
#define IMU_42605_GYRO_CONFIG0_ODR_25HZ      0x0A        /*!< 25 Hz (40 ms)*/
#define IMU_42605_GYRO_CONFIG0_ODR_50HZ      0x09        /*!< 50 Hz (20 ms)*/
#define IMU_42605_GYRO_CONFIG0_ODR_100HZ     0x08        /*!< 100 Hz (10 ms)*/
#define IMU_42605_GYRO_CONFIG0_ODR_200HZ     0x07        /*!< 200 Hz (5 ms)*/
#define IMU_42605_GYRO_CONFIG0_ODR_1KHZ      0x06        /*!< 1 KHz (1 ms)*/
#define IMU_42605_GYRO_CONFIG0_ODR_2KHZ      0x05        /*!< 2 KHz (500 us)*/
#define IMU_42605_GYRO_CONFIG0_ODR_4KHZ      0x04        /*!< 4 KHz (250 us)*/
#define IMU_42605_GYRO_CONFIG0_ODR_8KHZ      0x03        /*!< 8 KHz (125 us)*/
#define IMU_42605_GYRO_CONFIG0_ODR_16KHZ     0x02        /*!< 16 KHz (62.5 us)*/
#define IMU_42605_GYRO_CONFIG0_ODR_32KHZ     0x01        /*!< 32 KHz (31.25 us)*/
#define IMU_42605_GYRO_CONFIG0_VALUE (IMU_42605_GYRO_CONFIG0_FS_SEL_2000DPS | IMU_42605_GYRO_CONFIG0_ODR_1KHZ)

#define IMU_42605_ACCEL_CONFIG0_FS_SEL_RESERVED (0x04 << 5)
#define IMU_42605_ACCEL_CONFIG0_FS_SEL_2g       (0x03 << 5) /*!< 2g*/
#define IMU_42605_ACCEL_CONFIG0_FS_SEL_4g       (0x02 << 5) /*!< 4g*/
#define IMU_42605_ACCEL_CONFIG0_FS_SEL_8g       (0x01 << 5) /*!< 8g*/
#define IMU_42605_ACCEL_CONFIG0_FS_SEL_16g      (0x00 << 5) /*!< 16g*/
#define IMU_42605_ACCEL_CONFIG0_ODR_500HZ      0xF         /*!< 500 Hz (2 ms)*/
#define IMU_42605_ACCEL_CONFIG0_ODR_1_5625HZ   0xE         /*!< 1.5625 Hz (640 ms)*/
#define IMU_42605_ACCEL_CONFIG0_ODR_3_125HZ    0xD         /*!< 3.125 Hz (320 ms)*/
#define IMU_42605_ACCEL_CONFIG0_ODR_6_25HZ     0xC         /*!< 6.25 Hz (160 ms)*/
#define IMU_42605_ACCEL_CONFIG0_ODR_12_5HZ     0xB         /*!< 12.5 Hz (80 ms)*/
#define IMU_42605_ACCEL_CONFIG0_ODR_25HZ       0xA         /*!< 25 Hz (40 ms)*/
#define IMU_42605_ACCEL_CONFIG0_ODR_50HZ       0x9         /*!< 50 Hz (20 ms)*/
#define IMU_42605_ACCEL_CONFIG0_ODR_100HZ      0x8         /*!< 100 Hz (10 ms)*/
#define IMU_42605_ACCEL_CONFIG0_ODR_200HZ      0x7         /*!< 200 Hz (5 ms)*/
#define IMU_42605_ACCEL_CONFIG0_ODR_1KHZ       0x6         /*!< 1 KHz (1 ms)*/
#define IMU_42605_ACCEL_CONFIG0_ODR_2KHZ       0x5         /*!< 2 KHz (500 us)*/
#define IMU_42605_ACCEL_CONFIG0_ODR_4KHZ       0x4         /*!< 4 KHz (250 us)*/
#define IMU_42605_ACCEL_CONFIG0_ODR_8KHZ       0x3         /*!< 8 KHz (125 us)*/
#define IMU_42605_ACCEL_CONFIG0_ODR_16KHZ      0x2         /*!< 16 KHz (62.5 us)*/
#define IMU_42605_ACCEL_CONFIG0_ODR_32KHZ      0x1         /*!< 32 KHz (31.25 us)*/
#define IMU_42605_ACCEL_CONFIG0_VALUE (IMU_42605_ACCEL_CONFIG0_FS_SEL_8g | IMU_42605_ACCEL_CONFIG0_ODR_1KHZ)

#define IMU_42605_GYRO_CONFIG1_GYRO_UI_FILT_ORD_1ST (0x00 << 2)
#define IMU_42605_GYRO_CONFIG1_GYRO_UI_FILT_ORD_2ND (0x01 << 2)
#define IMU_42605_GYRO_CONFIG1_GYRO_UI_FILT_ORD_3RD (0x02 << 2)
#define IMU_42605_GYRO_CONFIG1_GYRO_DEC2_M2_ORD_3RD 0x02
#define IMU_42605_GYRO_CONFIG1_VALUE (IMU_42605_GYRO_CONFIG1_GYRO_DEC2_M2_ORD_3RD | IMU_42605_GYRO_CONFIG1_GYRO_UI_FILT_ORD_2ND)


#define IMU_42605_ACCEL_CONFIG1_ACCEL_UI_FILT_ORD_1ST (0x00 << 3)
#define IMU_42605_ACCEL_CONFIG1_ACCEL_UI_FILT_ORD_2ND (0x01 << 3)
#define IMU_42605_ACCEL_CONFIG1_ACCEL_UI_FILT_ORD_3RD (0x02 << 3)
#define IMU_42605_ACCEL_CONFIG1_ACCEL_DEC2_M2_ORD_3RD 0x04
#define IMU_42605_ACCEL_CONFIG1_VALUE (IMU_42605_ACCEL_CONFIG1_ACCEL_DEC2_M2_ORD_3RD | IMU_42605_ACCEL_CONFIG1_ACCEL_UI_FILT_ORD_2ND)

#define IMU_42605_GYRO_ACCEL_CONFIG0_ACCEL_FILT_BW_40 (0x7 << 4)
#define IMU_42605_GYRO_ACCEL_CONFIG0_ACCEL_FILT_BW_20 (0x6 << 4)
#define IMU_42605_GYRO_ACCEL_CONFIG0_ACCEL_FILT_BW_16 (0x5 << 4)
#define IMU_42605_GYRO_ACCEL_CONFIG0_ACCEL_FILT_BW_10 (0x4 << 4)
#define IMU_42605_GYRO_ACCEL_CONFIG0_ACCEL_FILT_BW_8  (0x3 << 4)
#define IMU_42605_GYRO_ACCEL_CONFIG0_ACCEL_FILT_BW_5  (0x2 << 4)
#define IMU_42605_GYRO_ACCEL_CONFIG0_ACCEL_FILT_BW_4  (0x1 << 4) // 100hz
#define IMU_42605_GYRO_ACCEL_CONFIG0_ACCEL_FILT_BW_2  (0x0 << 4)
#define IMU_42605_GYRO_ACCEL_CONFIG0_GYRO_FILT_BW_40  0x07
#define IMU_42605_GYRO_ACCEL_CONFIG0_GYRO_FILT_BW_20  0x06
#define IMU_42605_GYRO_ACCEL_CONFIG0_GYRO_FILT_BW_16  0x05
#define IMU_42605_GYRO_ACCEL_CONFIG0_GYRO_FILT_BW_10  0x04
#define IMU_42605_GYRO_ACCEL_CONFIG0_GYRO_FILT_BW_8   0x03
#define IMU_42605_GYRO_ACCEL_CONFIG0_GYRO_FILT_BW_5   0x02
#define IMU_42605_GYRO_ACCEL_CONFIG0_GYRO_FILT_BW_4   0x01 // 100hz
#define IMU_42605_GYRO_ACCEL_CONFIG0_GYRO_FILT_BW_2   0x00
#define IMU_42605_GYRO_ACCEL_CONFIG0_VALUE (IMU_42605_GYRO_ACCEL_CONFIG0_ACCEL_FILT_BW_2 | IMU_42605_GYRO_ACCEL_CONFIG0_GYRO_FILT_BW_2)

#define IMU_42605_PWR_MGMT0_TEMP_DIS          (0x01 << 5)
#define IMU_42605_PWR_MGMT0_TEMP_EN           (0x00 << 5)
#define IMU_42605_PWR_MGMT0_IDLE_DIS          (0x01 << 4)
#define IMU_42605_PWR_MGMT0_IDLE_EN           (0x00 << 4)
#define IMU_42605_PWR_MGMT0_GYRO_MODE_LN      (0x03 << 2)
#define IMU_42605_PWR_MGMT0_GYRO_MODE_STANDBY (0x01 << 2)
#define IMU_42605_PWR_MGMT0_GYRO_MODE_OFF     (0x00 << 2)
#define IMU_42605_PWR_MGMT0_ACCEL_MODE_LN     0x03
#define IMU_42605_PWR_MGMT0_ACCEL_MODE_LP     0x02
#define IMU_42605_PWR_MGMT0_ACCEL_MODE_OFF    0x00
#define IMU_42605_PWR_MGMT0_VALUE (IMU_42605_PWR_MGMT0_TEMP_EN | IMU_42605_PWR_MGMT0_IDLE_DIS | IMU_42605_PWR_MGMT0_GYRO_MODE_LN | IMU_42605_PWR_MGMT0_ACCEL_MODE_LN)

#define IMU_42605_INT_CONFIG_INT2_LATCHED_MODE (0x01 << 5)
#define IMU_42605_INT_CONFIG_INT2_PUSH_PULL    (0x01 << 4)
#define IMU_42605_INT_CONFIG_INT2_ACTIVE_HIGH  (0x01 << 3)
#define IMU_42605_INT_CONFIG_VALUE (IMU_42605_INT_CONFIG_INT2_LATCHED_MODE | IMU_42605_INT_CONFIG_INT2_PUSH_PULL | IMU_42605_INT_CONFIG_INT2_ACTIVE_HIGH)

#define IMU_42605_INTF_CONFIG_FIFO_COUNT_ENDIAN_BIG   (0x01 << 5)
#define IMU_42605_INTF_CONFIG_SENSOR_DATA_ENDIAN_BIG  (0x01 << 4)
#define IMU_42605_INTF_CONFIG_UI_SIFS_CFG_DISABLE_I2C (0x03)
#define IMU_42605_INTF_CONFIG_VALUE (IMU_42605_INTF_CONFIG_FIFO_COUNT_ENDIAN_BIG | IMU_42605_INTF_CONFIG_SENSOR_DATA_ENDIAN_BIG | IMU_42605_INTF_CONFIG_UI_SIFS_CFG_DISABLE_I2C)


#define IMU_42605_INT_SOURCE4_WOM_Z_INT2_EN (0x01 << 2)
#define IMU_42605_INT_SOURCE4_WOM_Y_INT2_EN (0x01 << 1)
#define IMU_42605_INT_SOURCE4_WOM_X_INT2_EN (0x01 << 0)
#define IMU_42605_INT_SOURCE4_VALUE (IMU_42605_INT_SOURCE4_WOM_X_INT2_EN | IMU_42605_INT_SOURCE4_WOM_Y_INT2_EN | IMU_42605_INT_SOURCE4_WOM_Z_INT2_EN)

#define IMU_42605_SMD_CONFIG_MODE_DISABLED  0x00
#define IMU_42605_SMD_CONFIG_MODE_WOM       0x01
#define IMU_42605_SMD_CONFIG_MODE_SMD_SHORT 0x02
#define IMU_42605_SMD_CONFIG_MODE_SMD_LONG  0x03
#define IMU_42605_SMD_CONFIG_WOM_MODE (0x01 << 2)
#define IMU_42605_SMD_CONFIG_WOM_INT_MODE_ALL (0x01 << 3)
#define IMU_42605_SMD_CONFIG_VALUE (IMU_42605_SMD_CONFIG_WOM_MODE | IMU_42605_SMD_CONFIG_MODE_WOM)

#define IMU_42605_SIGNAL_PATH_RESET_FIFO_FLUSH       (0x01 << 1)
#define IMU_42605_SIGNAL_PATH_RESET_TMST_STROBE      (0x01 << 2)
#define IMU_42605_SIGNAL_PATH_RESET_ABORT_AND_RESET  (0x01 << 3)
#define IMU_42605_SIGNAL_PATH_RESET_DMP_MEM_RESET_EN (0x01 << 5)
#define IMU_42605_SIGNAL_PATH_RESET_DMP_INIT_EN      (0x01 << 6)

#define IMU_42605_FIFO_CONFIG_MODE_STOP_ON_FULL (0x02 << 6)
#define IMU_42605_FIFO_CONFIG_MODE_STREAM       (0x01 << 6)
#define IMU_42605_FIFO_CONFIG_MODE_BYPASS       (0x00 << 6)

#define IMU_42605_TMST_CONFIG_TMST_EN        0x01
#define IMU_42605_TMST_CONFIG_TMST_DIS       0x00
#define IMU_42605_TMST_CONFIG_TMST_FSYNC_EN  (0x01 << 1)
#define IMU_42605_TMST_CONFIG_TMST_FSYNC_DIS (0x00 << 1)
#define IMU_42605_TMST_DELTA_EN              (0x01 << 2)
#define IMU_42605_TMST_DELTA_DIS             (0x00 << 2)
#define IMU_42605_TMST_CONFIG_RESOL_16us     (0x01 << 3)
#define IMU_42605_TMST_CONFIG_RESOL_1us      (0x00 << 3)
#define IMU_42605_TMST_TO_REGS_EN            (0x01 << 4)
#define IMU_42605_TMST_TO_REGS_DIS           (0x00 << 4)

/* =============== A49406 MOTOR DRIVER CONSTANTS AND VALUES ============== */

#define A49406_SPD_DEFAULT          15.1

/* ================ ENCODER CONSTANTS AND VALUES ========================= */
//#define PI                  3.1415926
//#define ENCODER_DEFAULT_PPR 1340
//#define ENCODER_POS_SCALE   (ENCODER_DEFAULT_PPR/2/PI)
#define KODENSHI1340_DEFAULT_PPR 1340
#define ENCODER_SPEED_SCALE (1<<20)
#define ENCODER_DATA_LEN      6  // enc_tso(2) + enc_pos(2) + enc_speed(2)

/* =============== TEMP SENSOR TMP126 CONSTANTS AND VALUES =============== */

#define TMP126_DATA_READ_LEN    (TMP126_DATA_LEN + TMP126_DATA_OFFSET)
#define TMP126_DATA_LEN         2
#define TMP126_DATA_OFFSET      2

// Valid value: 0x1 ~ 0xA
#define TMP126_CMD_BYTE_CRC_DATA_LEN_SHIFT(X)  (X << 2)

#define TMP126_CMD_BYTE_CRC_ENABLE      (1 << 6)
#define TMP126_CMD_BYTE_AUTO_INCREMENT  (1 << 1)
#define TMP126_CMD_BYTE                 0x0

#define TMP126_READ_MASK   0x01  // LSB = 1 for Read
#define TMP126_WRITE_MASK  0xFE  // LSB = 0 for Write (bitwise AND)

#define TMP126_CMD_READ   (TMP126_CMD_BYTE | TMP126_READ_MASK)
#define TMP126_CMD_WRITE  (TMP126_CMD_BYTE & TMP126_WRITE_MASK)

// Reg
#define TMP126_TEMP_RESULT_REG  0x00
#define TMP126_CONFIG_REG       0x03
#define TMP126_TLOW_LIMIT_REG   0x05
#define TMP126_THIGH_LIMIT_REG  0x06
#define TMP126_DEVICE_ID_REG    0x0C

// Reg value
#define TMP126_CONFIG_RESET_MSB     0x01
#define TMP126_CONFIG_RESET_LSB     0x00

#define TMP126_CONFIG_AVG_ENABLED           (1 << 7)
#define TMP126_CONFIG_ONE_SHOT_MODE         (1 << 4)
#define TMP126_CONFIG_CONTINUOUS_MODE       (0 << 3)
#define TMP126_CONFIG_SHUTDOWN_MODE         (1 << 3)
#define TMP126_CONFIG_CONV_PERIOD_125MS     0x03
#define TMP126_CONFIG_CONV_PERIOD_250MS     0x04
#define TMP126_CONFIG_CONV_PERIOD_500MS     0x05
#define TMP126_CONFIG_CONV_PERIOD_1S        0x06

#define TMP126_CONFIG_VAL_MSB               0x00
#define TMP126_CONFIG_VAL_LSB               (TMP126_CONFIG_AVG_ENABLED | TMP126_CONFIG_CONTINUOUS_MODE | TMP126_CONFIG_CONV_PERIOD_125MS)

#define TMP126_DEVICE_ID_VAL_MASK_MSB    0x0F
#define TMP126_DEVICE_ID_VAL_MASK_LSB    0xFF
#define TMP126_DEVICE_ID_VAL_MSB         0x01
#define TMP126_DEVICE_ID_VAL_LSB         0x26

/* =============== SHARED REGISTER MAP: TI I2C TEMP SENSORS (TI_TMP) =============== */
/* Applies to TMP1075N, TMP110 */

#define TI_TMP_REG_TEMP  0x00
#define TI_TMP_REG_CFGR  0x01
#define TI_TMP_CFGR_POR  0xA060  // power-on reset value (LE uint16_t)

/* =============== TEMP SENSOR TMP1075N CONSTANTS AND VALUES =============== */

#define TMP1075N_ADDR 0x49

#define TMP1075N_DATA_READ_LENGTH (TMP1075N_DATA_LENGTH)
#define TMP1075N_DATA_LENGTH 2
#define TMP1075N_DATA_OFFSET 0

/* =============== TEMP SENSOR TMP110 CONSTANTS AND VALUES =============== */

#define TMP110_ADDR 0x48

#define TMP110_DATA_READ_LENGTH (TMP110_DATA_LENGTH)
#define TMP110_DATA_LENGTH 2
#define TMP110_DATA_OFFSET 0

/* =============== RGBC SENSOR TCS3472 CONSTANTS AND VALUES =============== */
/*
    *Interrupt feature is not included in the table below
    *Register reset value is 0's unless specified alone
    *Register value is the same on TCS3472 series
*/
#define TCS3472_SLAVE_ADDR 0x29

#define TCS3472_ENABLE             0x00
#define TCS3472_ENABLE_WEN         (0x1 << 3)             //wait period enable
#define TCS3472_ENABLE_AEN         (0x1 << 1)             //RGBC enable
#define TCS3472_ENABLE_PON         0x01                   //power on
#define TCS3472_ENABLE_VAL         (TCS3472_ENABLE_PON | TCS3472_ENABLE_AEN)
#define TCS3472_ENABLE_POWER_OFF   0

#define TCS3472_ATIME      0x01    //reset value: 0xFF (? not true in practice)
#define TCS3472_WTIME      0x03    //reset value: 0xFF (? not true in practice)
#define TCS3472_AILTL      0x04
#define TCS3472_AILTH      0x05
#define TCS3472_AIHTL      0x06
#define TCS3472_AIHTH      0x07
#define TCS3472_PERS       0x0C
#define TCS3472_CONFIG     0x0D
#define TCS3472_CONTROL    0x0F

#define TCS3472_ID           0x12   //reset value: ID
#define TCS3472_ID_RESULT    0x44   //34721, 34725
#define TCS3472_ID_RESULT_2  0x4D   //34723, 34727

#define TCS3472_STATUS     0x13
#define TCS3472_CDATAL     0x14
#define TCS3472_CDATAH     0x15
#define TCS3472_RDATAL     0x16
#define TCS3472_RDATAH     0x17
#define TCS3472_GDATAL     0x18
#define TCS3472_GDATAH     0x19
#define TCS3472_BDATAL     0x1A
#define TCS3472_BDATAH     0x1B

#define TCS3472_CMD_BIT                    0x80
#define TCS3472_REPEATED_BYTE_PROTOCOL     0x00
#define TCS3472_AUTO_INCREMENT_PROTOCOL    0x20
#define TCS3472_CMD_HEADER                 (TCS3472_CMD_BIT | TCS3472_AUTO_INCREMENT_PROTOCOL)


/* =============== ADC ADS127L11 CONSTANTS AND VALUES =============== */
#define ADS127L11_SPI_READ_LENGTH           3
#define ADS127L11_NUM_CHANNELS              4
#define ADS127L11_DATA_LENGTH               (4 * ADS127L11_NUM_CHANNELS)  //we pad an extra byte on each channel to make it a int32

#define ADS127L11_NUM_DEBUG_CHANNELS        6 // [x, y and z] * [current and voltage]
#define ADS127L11_DEBUG_DATA_LENGTH         (4 * ADS127L11_NUM_DEBUG_CHANNELS) 

#define ADS127L11_DIG_FILTER_SINC4_SAMPLE_DELAY 160.63f
#define ADS127L11_DIG_FILTER_SAMPLE_DELAY_US    ADS127L11_DIG_FILTER_SINC4_SAMPLE_DELAY

#define ADS127L11_MUX_SETTLING_TIME_US    60
#define ADS127L11_SAMPLE_WINDOW_US        180
#define ADS127L11_CHANNEL_PERIOD_US       (ADS127L11_MUX_SETTLING_TIME_US + ADS127L11_SAMPLE_WINDOW_US)

#define ADS127L11_X_CHANNEL_DELAY   (US_TO_TICKS_16MHZ(ADS127L11_MUX_SETTLING_TIME_US) + (US_TO_TICKS_16MHZ(ADS127L11_DIG_FILTER_SAMPLE_DELAY_US)/2))
#define ADS127L11_Y_CHANNEL_DELAY   (ADS127L11_X_CHANNEL_DELAY+US_TO_TICKS_16MHZ(ADS127L11_CHANNEL_PERIOD_US))
#define ADS127L11_Z_CHANNEL_DELAY   (ADS127L11_Y_CHANNEL_DELAY+US_TO_TICKS_16MHZ(ADS127L11_CHANNEL_PERIOD_US))

#define ADS127L11_RREG_CMD    0x40
#define ADS127L11_WREG_CMD    0x80

#define ADS127L11_ID_REG      0x00
#define ADS127L11_ID_REG_VAL  0x00

#define ADS127L11_CONFIG1_REG                 0x05
#define ADS127L11_CONFIG1_REG_H_REF_RNG       (1<<6)  //high voltage refernce range
#define ADS127L11_CONFIG1_REG_REFP_BUF        (1<<3)  //vref buffer
#define ADS127L11_CONFIG1_REG_VCM_OUTPUT      (1<<4)
#define ADS127L11_CONFIG1_VAL                 (ADS127L11_CONFIG1_REG_H_REF_RNG | ADS127L11_CONFIG1_REG_REFP_BUF | ADS127L11_CONFIG1_REG_VCM_OUTPUT)

#define ADS127L11_CONFIG2_REG                 0x06
#define ADS127L11_CONFIG2_REG_ONESHOT_MODE    (1<<3)  //adc only sample once after START
#define ADS127L11_CONFIG2_VAL                 ADS127L11_CONFIG2_REG_ONESHOT_MODE

#define ADS127L11_CONFIG3_REG                           0x07
#define ADS127L11_CONFIG3_REG_SIN4_OSR512               (0b01111)
#define ADS127L11_CONFIG3_REG_SIN4_OSR32_SINC1_OSR40    (0b10111)
#define ADS127L11_CONFIG3_VAL                           ADS127L11_CONFIG3_REG_SIN4_OSR512


/* =============== USB HUB FE2.1 CONSTANTS AND VALUES =============== */
#define EEPROM_AT24C02D_I2C_SLAVE_ADDR  (0xA0 >>1)
#define EEPROM_AT24C02D_PAGE_SIZE       8

#define USB_FE21_EEPROM_CONTENTS_LEN    32

#define USB_FE21_ID_L           0x00
#define USB_FE21_ID_H           0x01
#define USB_FE21_VENDOR_ID_L    0x02
#define USB_FE21_VENDOR_ID_H    0x03
#define USB_FE21_DEV_RELEASE_L  0x06
#define USB_FE21_DEV_RELEASE_H  0x07

#define USB_FE21_ID_L_VAL   0x40
#define USB_FE21_ID_H_VAL   0x1A

/* Reminders:
 * 1. To Change Config, increment Device Release number 
 * 2. When updating the serial number, remember to update the length as well
 */
#define USB_FE21_RELEASE_NUMBER_L 1
#define USB_FE21_RELEASE_NUMBER_H 0
#define USB_FE21_HUB_CONFIG { \
    0x40, 0x1A, /* 0x00 - 0x01: Low and High byte of check code */ \
    APP_USBD_VID & 0xFF, (APP_USBD_VID >> 8) & 0xFF, /* 0x02 - 0x03: Vendor ID (Low and High) */ \
    SIU_PID_13151_USB_HUB & 0xFF, (SIU_PID_13151_USB_HUB >> 8) & 0xFF, /* 0x04 - 0x05: Product ID (Low and High) */ \
    USB_FE21_RELEASE_NUMBER_L, USB_FE21_RELEASE_NUMBER_H, /* 0x06 - 0x07: Device Release (Low and High, BCD format) */ \
    'O', 'M', 'M', 'O', ' ', \
    'T', 'E', 'C', 'H', '\0', \
    '\0', '\0', '\0', '\0', '\0', '\0', /* 0x08 - 0x17: Device Serial Number(16 bytes) */ \
    0x0A,        /* 0x18: Length of Serial Number */ \
    0x00,        /* 0x19: Filling */ \
    0x07,        /* 0x1A: Number of Downstream Ports */ \
    0x00,        /* 0x1B: Filling */ \
    0x98,        /* 0x1C: Device Removable (Bit 0 is reserved to be 0) */ \
    0x00,        /* 0x1D: Filling */ \
    0x04,        /* 0x1E: Device Attributes (bit2: 0->200mA 1->500mA) */ \
    0x00         /* 0x1F: Checksum (must calculate the sum of all values from 0x00 to 0x1E) */ \
}

/* =============== FUEL GUAGE BQ27427 =============== */
//IC address
#define BQ27427_SLAVE_ADDR                 0x55

//Simplified commands
#define BQ27427_STATE_OF_CHARGE_RX_LEN    1
#define BQ27427_STATE_OF_CHARGE_COMMAND   BQ27427_STATE_OF_CHARGE_LOW
#define BQ27427_AVG_CURRENT_RX_LEN        1
#define BQ27427_AVG_CURRENT_COMMAND       BQ27427_AVG_CURRENT_LOW
#define BQ27427_REMAINING_CAP_RX_LEN      2
#define BQ27427_REMAINING_CAP_COMMAND     BQ27427_REMAINING_CAP_FILT_LOW

//Register
#define BQ27427_CONTROL_LOW                 0x00    
#define BQ27427_CONTROL_HIGH                0x01  
#define BQ27427_TEMP_LOW                    0x02    
#define BQ27427_TEMP_HIGH                   0x03
#define BQ27427_VOLTAGE_LOW                 0x04
#define BQ27427_VOLTAGE_HIGH                0x05
#define BQ27427_FLAGS_LOW                   0x06
#define BQ27427_FLAGS_HIGH                  0x07
#define BQ27427_NOM_AVAILABLE_CAP_LOW       0x08
#define BQ27427_NOM_AVAILABLE_CAP_HIGH      0x09
#define BQ27427_FULL_AVAILABLE_CAP_LOW      0x0A
#define BQ27427_FULL_AVAILABLE_CAP_HIGH     0x0B
#define BQ27427_REMAINING_CAP_LOW           0x0C
#define BQ27427_REMAINING_CAP_HIGH          0x0D
#define BQ27427_FULL_CHARGE_CAP_LOW         0x0E
#define BQ27427_FULL_CHARGE_CAP_HIGH        0x0F
#define BQ27427_AVG_CURRENT_LOW             0x10
#define BQ27427_AVG_CURRENT_HIGH            0x11

#define BQ27427_AVG_POWER_LOW               0x18
#define BQ27427_AVG_POWER_HIGH              0x19
#define BQ27427_STATE_OF_CHARGE_LOW         0x1C
#define BQ27427_STATE_OF_CHARGE_HIGH        0x1D
#define BQ27427_INT_TEMP_LOW                0x1E
#define BQ27427_INT_TEMP_HIGH               0x1F

#define BQ27427_REMAINING_CAP_UNFILT_LOW    0x28
#define BQ27427_REMAINING_CAP_UNFILT_HIGH   0x29
#define BQ27427_REMAINING_CAP_FILT_LOW      0x2A
#define BQ27427_REMAINING_CAP_FILT_HIGH     0x2B
#define BQ27427_FULL_CHARGE_UNFILT_CAP_LOW  0x2C
#define BQ27427_FULL_CHARGE_UNFILT_CAP_HIGH 0x2D
#define BQ27427_FULL_CHARGE_FILT_CAP_LOW    0x2E
#define BQ27427_FULL_CHARGE_FILT_CAP_HIGH   0x2F
#define BQ27427_STATE_OF_CHARGE_UNFILT_LOW  0x30
#define BQ27427_STATE_OF_CHARGE_UNFILT_HIGH 0x31

#define BQ27427_DATA_CLASS                  0x3E
#define BQ27427_DATA_BLOCK                  0x3F
#define BQ27427_BLOCK_DATA_START            0x40
#define BQ27427_BLOCK_DATA_END              0x5F
#define BQ27427_BLOCK_DATA_CHECKSUM         0x60
#define BQ27427_BLOCK_DATA_CONTROL          0x61

//Control sub command
#define BQ27427_CONTROL_STATUS              0x0000
#define BQ27427_CONTROL_DEVICE_TYPE         0x0001
#define BQ27427_CONTROL_FW_VERSION          0x0002
#define BQ27427_CONTROL_DM_CODE             0x0004
#define BQ27427_CONTROL_PREV_MACWRITE       0x0007
#define BQ27427_CONTROL_CHEM_ID             0x0008
#define BQ27427_CONTROL_BAT_INSERT          0x000C
#define BQ27427_CONTROL_BAT_REMOVE          0x000D
#define BQ27427_CONTROL_SET_CFGUPDATE       0x0013
#define BQ27427_CONTROL_SMOOTH_SYNC         0x0019
#define BQ27427_CONTROL_SHUTDOWN_ENABLE     0x001B
#define BQ27427_CONTROL_SHUTDOWN            0x001C
#define BQ27427_CONTROL_SEALED              0x0020
#define BQ27427_CONTROL_PULSE_SOC_INT       0x0023
#define BQ27427_CONTROL_CHAM_A              0x0030   //3230  4.35V
#define BQ27427_CONTROL_CHAM_B              0x0031   //1202  4.20V
#define BQ27427_CONTROL_CHAM_C              0x0032   //3142  4.40V
#define BQ27427_CONTROL_RESET               0x0041
#define BQ27427_CONTROL_SOFT_RESET          0x0042
#define BQ27427_CONTROL_UNSEAL              0x8000

#define BQ27427_DEVICE_TYPE_RESP            0x0427


/* =============== FUEL GUAGE BQ27Z558 =============== */
#define BQ27Z558_SLAVE_ADDR                0x55

#define BQ27Z558_REG_READ_RX_LEN           2

#define BQ27Z558_R_FW_VERSION_VAL          0x5815
#define BQ27Z558_BATT_ZWD_CAPACITY_MA      120

#define BQ27Z558_REG_TEMP                  0x06     /*!< Temperature in 0.1K. I2 */
#define BQ27Z558_REG_VOLTAGE               0x08     /*!< Voltage in mV. U2 */
#define BQ27Z558_REG_CURRENT               0x0C     /*!< Current in mA. I2 */
#define BQ27Z558_REG_REM_CAPACITY          0x10     /*!< Remaining capacity in mAh. U2 */
#define BQ27Z558_REG_FULL_CHARGE_CAPACITY  0x12     /*!< Full charge capacity in mAh. U2 */
#define BQ27Z558_REG_AVG_CUR               0x14     /*!< Average current in mA. I2 */
#define BQ27Z558_REG_AVG_TTE               0x16     /*!< Average time to empty in minutes. U2 */
#define BQ27Z558_REG_AVG_TTF               0x18     /*!< Average time to full in minutes. U2 */
#define BQ27Z558_REG_MAX_LOAD_CURRENT      0x1E     /*!< Maximum load current in mA. U2 */
#define BQ27Z558_REG_MAX_LOAD_TTE          0x20     /*!< Maximum load time to empty in minutes. U2 */
#define BQ27Z558_REG_AVERAGE_POWER         0x22     /*!< Average power in mW. I2 */
#define BQ27Z558_REG_BTP_DISCHR_SET        0x24     /*!< R/W Read/Update discharge set threshold for IRQ. And clear current IRQ status. U1 */
#define BQ27Z558_REG_BTP_CHR_SET           0x26     /*!< R/W Read/Update charge set threshold for IRQ. And clear current IRQ status. U1 */
#define BQ27Z558_REG_TEMP_INTERNAL         0x28     /*!< Internal temperature in 0.1K. I2 */
#define BQ27Z558_REG_CYCLE_COUNT           0x2A     /*!< Cycle count. U2 */
#define BQ27Z558_REG_STATE_RSOC            0x2C     /*!< State of Charge relative to remaining capacity. U1 */
#define BQ27Z558_REG_STATE_OF_HEALTH       0x2E     /*!< State of Health. U1 */
#define BQ27Z558_REG_DESIGN_CAPACITY       0x3C     /*!< Theoretical maximum pack capacity. */
#define BQ27Z558_REG_ALT_MANUF             0x3E     /*!< Manufacturer Access. */

#define BQ27Z558_MANUF_R_DEVICE_TYPE            0x01    /*!< Device Type. */
#define BQ27Z558_MANUF_R_FW_VERSION             0x02    /*!< Firmware Version. */
#define BQ27Z558_MANUF_R_HW_VERSION             0x03    /*!< Hardware Version. */
#define BQ27Z558_MANUF_R_IFCHECK_SUM            0x04    /*!< Interface Checksum. */
#define BQ27Z558_MANUF_R_STAT_DF_SIGN           0x05    /*!< Status Data Flash Signature. */
#define BQ27Z558_MANUF_R_CHEM_ID                0x06    /*!< Chemistry ID. */
#define BQ27Z558_MANUF_R_PREV_MAC_WRITE         0x07    /*!< Previous MAC Write. */
#define BQ27Z558_MANUF_R_STATIC_CHEM_DF_SIG     0x08    /*!< Static Chemistry Data Flash Signature. */
#define BQ27Z558_MANUF_R_ALL_DF_SIG             0x09    /*!< All Data Flash Signature. */
#define BQ27Z558_MANUF_R_SERIAL_NUMBER          0x4E    /*!< Serial Number. */
#define BQ27Z558_MANUF_R_OPERATION_STATUS       0x54    /*!< Operation status. */

#define BQ27Z558_MANUF_W_RESET                  0x12    /*!< Reset. */
#define BQ27Z558_MANUF_R_QMAX_DAY_U16           0x17    /*!< QMAX Day (unsigned 16-bit). */
#define BQ27Z558_MANUF_W_GAUGING_IT_EN          0x21    /*!< Gauging Impedance Track Enable. */

// charge discharge btp_int rsoc threshold
#define BQ27Z558_REG_BTP_DISCHR_SET_VALUE       10
#define BQ27Z558_REG_BTP_CHR_SET_VALUE          0xFF

// Op status(0x54) related value
#define BQ27Z558_OP_STATUS_A_SECURITY_MODE_BYTE_IDX     1
#define BQ27Z558_OP_STATUS_A_SECURITY_MODE_MASK         0x03
#define BQ27Z558_OP_STATUS_A_SECURITY_MODE_FULL_ACCESS  0x01
#define BQ27Z558_OP_STATUS_A_SECURITY_MODE_UNSEALED     0x02
#define BQ27Z558_OP_STATUS_A_SECURITY_MODE_SEALED       0x03

#define BQ27Z558_OP_STATUS_A_BTP_INT_BYTE_IDX           0
#define BQ27Z558_OP_STATUS_A_BTP_INT_BIT_MASK           0x80
#define BQ27Z558_OP_STATUS_A_BTP_INT_BIT_SET            0x80

#define BQ27Z558_OP_STATUS_B_RESET_INIT_BYTE_IDX        2
#define BQ27Z558_OP_STATUS_B_RESET_INIT_MASK            0x01
#define BQ27Z558_OP_STATUS_B_RESET_INIT_DONE            0x00    // inactive(0) - done, active(1) - ongoing

#define BQ27Z558_MANUF_R_GAUGING_STATUS         0x56    /*!< Gauging status + IT status. */
#define BQ27Z558_MANUF_R_MANUFACTURING_STATUS   0x57    /*!< Manufacturing. */

#define BQ27Z558_MANUF_R_DA_STATUS1             0x71    /*!< Battery data: 32 bytes. */
#define BQ27Z558_MANUF_R_DA_STATUS2             0x72    /*!< Battery data: 20 bytes. */
#define BQ27Z558_MANUF_R_IT_STATUS1             0x73    /*!< Impedance Track Status: 20 bytes. */
#define BQ27Z558_MANUF_R_IT_STATUS2             0x74    /*!< Impedance Track Status: 20 bytes. */
#define BQ27Z558_MANUF_R_IT_STATUS3             0x75    /*!< Impedance Track Status: 14 bytes. */

#define BQ27Z558_MANUF_R_CC_GAIN                0x4006


/* =============== CHARGER BQ25188 =============== */

// NOTE: 'field_val' must already be offset with the position before use.
//clear the field bits then assgin
#define SET_FIELD(reg_val, field_val, field_mask) (((reg_val) & ~(field_mask)) | ((field_val) & (field_mask)))

//100k or 400k
#define BQ25188_SLAVE_ADDR  0x6A

//Register
#define BQ25188_STAT0            0x00
#define BQ25188_STAT1            0x01
#define BQ25188_FLAG0            0x02
#define BQ25188_VBAT_CTRL        0x03
#define BQ25188_ICHG_CTRL        0x04   // Fast Charge Current Control
#define BQ25188_CHARGECTRL0      0x05
#define BQ25188_CHARGECTRL1      0x06   // Discharge limit(1A), 3 IRQs
#define BQ25188_IC_CTRL          0x07
#define BQ25188_TMR_ILIM         0x08
#define BQ25188_SHIP_RST         0x09
#define BQ25188_SYS_REG          0x0A   // i2c watchdog, dppm disable
#define BQ25188_TS_CONTROL       0x0B
#define BQ25188_MASK_ID          0x0C

#define BQ25188_STAT0_CHG_STAT_NOT_CHARGING 0x00
#define BQ25188_STAT0_CHG_STAT_CC           0x01
#define BQ25188_STAT0_CHG_STAT_CV           0x02
#define BQ25188_STAT0_CHG_STAT_CHARGE_DONE  0x03
#define BQ25188_STAT0_CHG_STAT_POS  5
#define BQ25188_STAT0_CHG_STAT_MASK (0b11 << BQ25188_STAT0_CHG_STAT_POS)

//battery target voltage
//create a func, use x - base / 10 to get val
#define BQ25188_VBAT_CTRL_BASE_MV   3500
#define BQ25188_VBAT_CTRL_VAL_MASK  0x7F
//#define BQ25188_VBAT_CTRL_VAL       70  //4.2V
#define BQ25188_VBAT_CTRL_VAL       95  //4.45V

#define BQ25188_ICHG_CTRL_CHG_DIS           0x1
//For ICHG <= 35mA = ICHGCODE +5mA 
//For ICHG > 35mA = 40+ ((ICHGCODE-31)*10)mA.
#define BQ25188_ICHG_CTRL_ICHG_VAL          37      //100mA
#define BQ25188_ICHG_CTRL_ICHG_VAL_MASK     0x7F
#define BQ25188_ICHG_CTRL_DIS_MASK          0x80
#define BQ25188_ICHG_CTRL_VAL               (BQ25188_ICHG_CTRL_ICHG_VAL & BQ25188_ICHG_CTRL_ICHG_VAL_MASK)

//NOTE ITERM changes when charge current changes
//ATTENTION Top bit needs to be 0(or write value will be random)
//          in datasheet one table says reserved
//          but other table says it is for faster charge enable. 
//          Treated as reserved for now(default 0)
#define BQ25188_CHARGECTRL0_IPRECHG_2X_TERM         (0x00 << 6) // precharge current
#define BQ25188_CHARGECTRL0_IPRECHG_1X_TERM         (0x01 << 6)
#define BQ25188_CHARGECTRL0_ITERM_DIS               (0x00 << 4)
#define BQ25188_CHARGECTRL0_ITERM_5PERCENT_ICHG     (0x01 << 4)
#define BQ25188_CHARGECTRL0_ITERM_10PERCENT_ICHG    (0x02 << 4)
#define BQ25188_CHARGECTRL0_ITERM_20PERCENT_ICHG    (0x03 << 4)
#define BQ25188_CHARGECTRL0_VINDPM_VBAT_PLUS_300MV  (0x00 << 2)
#define BQ25188_CHARGECTRL0_VINDPM_DIS              (0x03 << 2)
#define BQ25188_CHARGECTRL0_THERM_REG_DIS           (0x03 << 0) // thermal regulation
#define BQ25188_CHARGECTRL0_VAL                     (BQ25188_CHARGECTRL0_IPRECHG_1X_TERM | BQ25188_CHARGECTRL0_ITERM_10PERCENT_ICHG | BQ25188_CHARGECTRL0_VINDPM_VBAT_PLUS_300MV | BQ25188_CHARGECTRL0_THERM_REG_DIS)

#define BQ25188_CHARGECTRL1_IBAT_OCP_500MA   (0x00 << 6)
#define BQ25188_CHARGECTRL1_IBAT_OCP_1000MA  (0x01 << 6)
#define BQ25188_CHARGECTRL1_IBAT_OCP_1500MA  (0x02 << 6)
#define BQ25188_CHARGECTRL1_IBAT_OCP_3000MA  (0x03 << 6)

//Battery Undervoltage LockOut Threshold Falling (150mV Hist).
#define BQ25188_CHARGECTRL1_BUVLO_3V0   (0x02 << 3)
#define BQ25188_CHARGECTRL1_BUVLO_2V8   (0x03 << 3)
#define BQ25188_CHARGECTRL1_BUVLO_2V6   (0x04 << 3)
#define BQ25188_CHARGECTRL1_BUVLO_2V4   (0x05 << 3)
#define BQ25188_CHARGECTRL1_BUVLO_2V2   (0x06 << 3)
#define BQ25188_CHARGECTRL1_BUVLO_2V0   (0x07 << 3)

// Not included and use default value(0x00): 
// Recharge Voltage Threshold 100mV, 2XTMR_EN timer is not slowed at any time 
#define BQ25188_IC_CTRL_TS_EN                           (0x01 << 7) //Temperature sense enable (default)
#define BQ25188_IC_CTRL_TS_DIS                          (0x00 << 7) //Temperature sense disable
#define BQ25188_IC_CTRL_VLOWV_SEL_2V8                   (0x01 << 6) //Precharge voltage threshold 2.8V
#define BQ25188_IC_CTRL_VLOWV_SEL_3V                    (0x00 << 6) //Precharge voltage threshold 3V (default)
#define BQ25188_IC_CTRL_VRCH_0_100MV                    (0x00 << 5) //Recharge voltage threshold 
#define BQ25188_IC_CTRL_VRCH_0_200MV                    (0x00 << 5) //Recharge voltage threshold 
#define BQ25188_IC_CTRL_FAST_CHARGE_SAFETY_TIMER_3HR    (0x00 << 2)
#define BQ25188_IC_CTRL_FAST_CHARGE_SAFETY_TIMER_6HR    (0x01 << 2)
#define BQ25188_IC_CTRL_FAST_CHARGE_SAFETY_TIMER_12HR   (0x02 << 2)
#define BQ25188_IC_CTRL_FAST_CHARGE_SAFETY_TIMER_DIS    (0x03 << 2)
#define BQ25188_IC_CTRL_WATCHDOG_SEL_160S_DEFAULT_REG   (0x00 << 0)
#define BQ25188_IC_CTRL_WATCHDOG_SEL_160S_HW_RESET      (0x01 << 0)
#define BQ25188_IC_CTRL_WATCHDOG_SEL_40S_HW_RESET       (0x02 << 0)
#define BQ25188_IC_CTRL_WATCHDOG_SEL_DIS                (0x03 << 0)
#define BQ25188_IC_CTRL_VAL                             (BQ25188_IC_CTRL_TS_EN | BQ25188_IC_CTRL_VLOWV_SEL_3V | BQ25188_IC_CTRL_VRCH_0_200MV | BQ25188_IC_CTRL_FAST_CHARGE_SAFETY_TIMER_6HR | BQ25188_IC_CTRL_WATCHDOG_SEL_DIS)

#define BQ25188_CHG_STATUS_INT_DISABLE  (0x01 << 2)
#define BQ25188_CHG_STATUS_INT_ENABLE   (0x00 << 2)
#define BQ25188_ILIM_INT_DISABLE        (0x01 << 1)
#define BQ25188_ILIM_INT_ENABLE         (0x00 << 1)
#define BQ25188_ILIM_VINDPM_INT_DISABLE (0x01 << 0)
#define BQ25188_ILIM_VINDPM_INT_ENABLE  (0x01 << 0)

#define BQ25188_CHARGECTRL1_VAL         (BQ25188_CHARGECTRL1_IBAT_OCP_500MA | BQ25188_CHARGECTRL1_BUVLO_3V0 | BQ25188_CHG_STATUS_INT_ENABLE | BQ25188_ILIM_INT_DISABLE | BQ25188_ILIM_VINDPM_INT_DISABLE)

#define BQ25188_SHIP_RST_REG_RST            (0x01 << 7)
#define BQ25188_SHIP_RST_REG_EN_RST_SHIP    (0x02 << 5)    //ship mode enabled and hardware reset
#define BQ25188_SHIP_RST_REG_EN_PUSH        (0x01)  //TODO will it stop chip from software rst?
//no rst, no shipmode, disable push button
#define BQ25188_SHIP_RST_REG_VAL            0x00

#define BQ25188_SYS_REG_CTRL_BATT_TRACKING  (0x00 << 5)
#define BQ25188_SYS_REG_CTRL_4V4            (0x01 << 5)
#define BQ25188_SYS_REG_CTRL_4V5            (0x02 << 5) //default
#define BQ25188_SYS_REG_CTRL_4V6            (0x03 << 5)
#define BQ25188_SYS_REG_CTRL_4V7            (0x04 << 5)
#define BQ25188_SYS_REG_CTRL_4V8            (0x05 << 5)
#define BQ25188_SYS_REG_CTRL_4V9            (0x06 << 5)
#define BQ25188_SYS_REG_CTRL_PASS_THROUGH   (0x07 << 5)
#define BQ25188_SYS_REG_CTRL_MASK           (0x07 << 5)
#define BQ25188_SYS_REG_CTRL_VDPPM_EN       (0x00 << 0)
#define BQ25188_SYS_REG_CTRL_VDPPM_DIS      (0x01 << 0)
#define BQ25188_SYS_REG_CTRL_VAL            (BQ25188_SYS_REG_CTRL_PASS_THROUGH | BQ25188_SYS_REG_CTRL_VDPPM_DIS)

#define BQ25188_SYS_REG_SYS_MODE_VIN        (0x00 << 2) //default
#define BQ25188_SYS_REG_SYS_MODE_VBAT       (0x01 << 2)
#define BQ25188_SYS_REG_SYS_MODE_FLOAT      (0x02 << 2)
#define BQ25188_SYS_REG_SYS_MODE_PULLDOWN   (0x03 << 2)
#define BQ25188_SYS_REG_SYS_MODE_MASK       (0x03 << 2)

#define BQ25188_MASK_ID_REG_DISABLE_IRQ     0xF0
#define BQ25188_MASK_ID_REG_ID_VAL          0x04
#define BQ25188_MASK_ID_REG_ID_MASK         0x0F
#define BQ25188_MASK_ID_REG_ID_SETUP_VAL    (BQ25188_MASK_ID_REG_DISABLE_IRQ | BQ25188_MASK_ID_REG_ID_VAL)

/* =============== MISC =============== */

#define DEBUG_DATA_LENGTH 12



#ifdef __cplusplus
extern "C" {
#endif


/* =============== IC default profiles =============== */

/* MAG */
//mmc5983
extern const int32_t ic_def_mmc5983_timestampoffsets[];
extern const DeviceInfoSensorProto ic_def_mmc5983_sensors[];
extern const DeviceInfoICProto ic_def_mmc5983;

//mmc5603
extern const int32_t ic_def_mmc5603_timestampoffsets[];
extern const DeviceInfoSensorProto ic_def_mmc5603_sensors[];
extern const DeviceInfoICProto ic_def_mmc5603;

//tle493d
extern const int32_t tle493d_mag_timestampoffsets[];
//OMMO_TLE493D_HIGH_SENSITIVITY
extern const DeviceInfoSensorProto ic_def_tle493d_sensors_high_sensitivity[];
extern const DeviceInfoICProto ic_def_tle493d_high_sensitivity;
//NORMAL SENSITIVITY
extern const DeviceInfoSensorProto ic_def_tle493d_sensors_no_options[];
extern const DeviceInfoICProto ic_def_tle493d_no_options;

/* IMU */
//lsm6dsm
extern const DeviceInfoSensorProto ic_def_lsm6dsm_sensors_no_options[];
extern const DeviceInfoICProto ic_def_lsm6dsm_no_options;

extern const DeviceInfoSensorProto ic_def_lsm6dsm_sensors_temp_enabled[];
extern const DeviceInfoICProto ic_def_lsm6dsm_temp_enabled;

extern const DeviceInfoSensorProto ic_def_lsm6dsm_sensors_offset_enabled[];
extern const DeviceInfoICProto ic_def_lsm6dsm_offset_enabled;

//lsm6dsv
extern const DeviceInfoSensorProto ic_def_lsm6dsv_sensors_no_options[];
extern const DeviceInfoICProto ic_def_lsm6dsv_no_options;

//icm42605
extern const DeviceInfoSensorProto ic_def_icm42605_sensors[];
extern const DeviceInfoICProto ic_def_icm42605;


/* MISC */
//encoder
extern const DeviceInfoSensorProto ic_def_encoder_sensors[];
extern const DeviceInfoICProto ic_def_encoder;

//encoder cnc
extern const DeviceInfoSensorProto ic_def_encoder_generic_sensors[];
extern const DeviceInfoICProto ic_def_encoder_generic;

//generic adc
extern const DeviceInfoSensorProto ic_def_generic_adc_sensors[];
extern const DeviceInfoICProto ic_def_generic_adc;

//generic adc ads127l11
extern const int32_t ic_def_generic_adc_sensors_ads127l11_sample_delay[];
extern const DeviceInfoSensorProto ic_def_generic_adc_sensors_ads127l11[];
extern const DeviceInfoICProto ic_def_generic_adc_ads127l11;

extern const DeviceInfoSensorProto ic_def_adc_ads127l11_debug_sensors[];
extern const DeviceInfoICProto ic_def_adc_ads127l11_debug;

//tmp126 temp sensor
extern const DeviceInfoSensorProto ic_def_tmp126_sensors[];
extern const DeviceInfoICProto ic_def_tmp126;

//tcs3472 color sensor
extern const DeviceInfoSensorProto ic_def_tcs3472_sensors[];
extern const DeviceInfoICProto ic_def_tcs3472;

//hx711 adc sensor
extern const DeviceInfoSensorProto ic_def_hx711_sensors[];
extern const DeviceInfoICProto ic_def_hx711;

//mfam mag
extern const DeviceInfoSensorProto ic_def_mfam_sensors[];
extern const DeviceInfoICProto ic_def_mfam;

//tmp1075n temperature sensor
extern const DeviceInfoSensorProto ic_def_tmp1075n_sensors[];
extern const DeviceInfoICProto ic_def_tmp1075n;

//tmp110 temperature sensor
extern const DeviceInfoSensorProto ic_def_tmp110_sensors[];
extern const DeviceInfoICProto ic_def_tmp110;

//analog button
extern const DeviceInfoSensorProto ic_def_analog_button_sensors[];
extern const DeviceInfoICProto ic_def_analog_button;

//bq27z558 fuel gauge
extern const DeviceInfoSensorProto ic_def_bq27z558_sensors[];
extern const DeviceInfoICProto ic_def_bq27z558;

//battery charger tester sensor
extern const DeviceInfoSensorProto ic_def_batt_charger_tester_sensors[];
extern const DeviceInfoICProto ic_def_batt_charger_tester;

//master siu tester
extern const DeviceInfoICProto ic_def_master_siu_tester;

#ifdef __cplusplus
}
#endif


#endif // IC_CONSTANTS_H__