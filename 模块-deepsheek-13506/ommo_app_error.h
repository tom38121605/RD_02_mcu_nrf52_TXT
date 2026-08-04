/* Data structure */
/* []: Optional */
//+-------------------+---------------------+
//| Data              | Size(bytes)         |
//+-------------------+---------------------+
//| Rev#              | 1                   |
//| Err_code          | 2                   |
//| File_name_length  | 1                   |
//| File_name         | File_name_length    |
//| Line_num          | 2                   |
//| Epoch_time        | 4                   |
//| [Msg data type]   | 1                   |
//| [Msg data len]    | 2                   |
//| [Msg]             | [Msg data len]      |
//| Var1_dataType     | 1                   |
//| [Var1_arryLength] | arry: 2 / var: NA   |
//| Var1_data         | sizeof_data         |
//| Var2...           |                     |
//|                   |                     |
//+-------------------+---------------------+

#ifndef OMMO_APP_ERROR_H__
#define OMMO_APP_ERROR_H__

#include "ommo_fw.pb.h"
#include "nrf_dfu_types.h"

#ifdef __cplusplus

static CrashDataType TYPEID(bool x) {return CRASH_DATA_TYPE_BOOL;}
static CrashDataType TYPEID(char x) {return CRASH_DATA_TYPE_CHAR;}
static CrashDataType TYPEID(signed char x) {return CRASH_DATA_TYPE_SIGNED_CHAR;}
static CrashDataType TYPEID(unsigned char x) {return CRASH_DATA_TYPE_UNSIGNED_CHAR;}
static CrashDataType TYPEID(short int x) {return CRASH_DATA_TYPE_SHORT_INT;}
static CrashDataType TYPEID(unsigned short int x) {return CRASH_DATA_TYPE_UNSIGNED_SHORT_INT;}
static CrashDataType TYPEID(int x) {return CRASH_DATA_TYPE_INT;}
static CrashDataType TYPEID(unsigned int x) {return CRASH_DATA_TYPE_UNSIGNED_INT;}
static CrashDataType TYPEID(long int x) {return CRASH_DATA_TYPE_LONG_INT;}
static CrashDataType TYPEID(unsigned long int x) {return CRASH_DATA_TYPE_UNSIGNED_LONG_INT;}
static CrashDataType TYPEID(long long int x) {return CRASH_DATA_TYPE_LONG_LONG_INT;}
static CrashDataType TYPEID(unsigned long long int x) {return CRASH_DATA_TYPE_UNSIGNED_LONG_LONG_INT;}
static CrashDataType TYPEID(float x) {return CRASH_DATA_TYPE_FLOAT;}
static CrashDataType TYPEID(double x) {return CRASH_DATA_TYPE_DOUBLE;}
static CrashDataType TYPEID(long double x) {return CRASH_DATA_TYPE_LONG_DOUBLE;}
static CrashDataType TYPEID(int8_t * x) {return CRASH_DATA_TYPE_PTR_TO_INT8_T;}
static CrashDataType TYPEID(uint8_t * x) {return CRASH_DATA_TYPE_PTR_TO_UINT8_T;}
static CrashDataType TYPEID(int16_t * x) {return CRASH_DATA_TYPE_PTR_TO_INT16_T;}
static CrashDataType TYPEID(uint16_t * x) {return CRASH_DATA_TYPE_PTR_TO_UINT16_T;}
static CrashDataType TYPEID(int32_t * x) {return CRASH_DATA_TYPE_PTR_TO_INT32_T;}
static CrashDataType TYPEID(uint32_t * x) {return CRASH_DATA_TYPE_PTR_TO_UINT32_T;}
static CrashDataType TYPEID(int8_t const *) {return CRASH_DATA_TYPE_PTR_TO_INT8_T;}
static CrashDataType TYPEID(uint8_t const *) {return CRASH_DATA_TYPE_PTR_TO_UINT8_T;}
static CrashDataType TYPEID(int16_t const *) {return CRASH_DATA_TYPE_PTR_TO_INT16_T;}
static CrashDataType TYPEID(uint16_t const *) {return CRASH_DATA_TYPE_PTR_TO_UINT16_T;}
static CrashDataType TYPEID(int32_t const *) {return CRASH_DATA_TYPE_PTR_TO_INT32_T;}
static CrashDataType TYPEID(uint32_t const *) {return CRASH_DATA_TYPE_PTR_TO_UINT32_T;}
static CrashDataType TYPEID(void *) {return CRASH_DATA_TYPE_PTR_TO_UINT32_T;}
static CrashDataType TYPEID(void const *) {return CRASH_DATA_TYPE_PTR_TO_UINT32_T;}

extern "C" {

#else

#define TYPEID(x) _Generic((x), _Bool: CRASH_DATA_TYPE_BOOL, /* 1 */ \
    char: CRASH_DATA_TYPE_CHAR, \
    signed char: CRASH_DATA_TYPE_SIGNED_CHAR, \
    unsigned char: CRASH_DATA_TYPE_UNSIGNED_CHAR, \
    short int: CRASH_DATA_TYPE_SHORT_INT, /* 5 */ \
    unsigned short int: CRASH_DATA_TYPE_UNSIGNED_SHORT_INT, \
    int: CRASH_DATA_TYPE_INT, \
    unsigned int: CRASH_DATA_TYPE_UNSIGNED_INT, \
    long int: CRASH_DATA_TYPE_LONG_INT, \
    unsigned long int: CRASH_DATA_TYPE_UNSIGNED_LONG_INT, /* 10 */ \
    long long int: CRASH_DATA_TYPE_LONG_LONG_INT, \
    unsigned long long int: CRASH_DATA_TYPE_UNSIGNED_LONG_LONG_INT, \
    float: CRASH_DATA_TYPE_FLOAT, \
    double: CRASH_DATA_TYPE_DOUBLE, \
    long double: CRASH_DATA_TYPE_LONG_DOUBLE, /* 15 */ \
    int8_t *: CRASH_DATA_TYPE_PTR_TO_INT8_T,  \
    uint8_t *: CRASH_DATA_TYPE_PTR_TO_UINT8_T, \
    int16_t *: CRASH_DATA_TYPE_PTR_TO_INT16_T,  \
    uint16_t *: CRASH_DATA_TYPE_PTR_TO_UINT16_T, \
    int32_t *: CRASH_DATA_TYPE_PTR_TO_INT32_T, /* 20 */ \
    uint32_t *: CRASH_DATA_TYPE_PTR_TO_UINT32_T, \
    int8_t const *: CRASH_DATA_TYPE_PTR_TO_INT8_T, \
    uint8_t const *: CRASH_DATA_TYPE_PTR_TO_UINT8_T, \
    int16_t const *: CRASH_DATA_TYPE_PTR_TO_INT16_T,  \
    uint16_t const *: CRASH_DATA_TYPE_PTR_TO_UINT16_T, /* 25 */ \
    int32_t const *: CRASH_DATA_TYPE_PTR_TO_INT32_T,  \
    uint32_t const *: CRASH_DATA_TYPE_PTR_TO_UINT32_T,  \
    void *: CRASH_DATA_TYPE_PTR_TO_UINT32_T,  \
    void const *: CRASH_DATA_TYPE_PTR_TO_UINT32_T /* 29 */  )
#endif

#define OMMO_ERR_LOG_REV        1

#define FIRMWARE_CRASH_REBOOT   0x01
#define FAULT_HANDLER_REBOOT    0x02

#define OMMO_LOG_BUF_SIZE       1024

#define L(x)            TYPEID(x), x
#define L2(x, length)   TYPEID(x), x, length
//TODO investigate why _Generic("str") comes back with void const *
#define STRING(x)       CRASH_DATA_TYPE_STRING, x, sizeof(x)


/* Need a '0' variable at the end as an end sign */
#define OMMO_APP_ERROR_CHECK(ERR_CODE, ...)                                           \
  do {                                                                                \
    const uint32_t LOCAL_ERR_CODE = (ERR_CODE);                                       \
    if (LOCAL_ERR_CODE != NRF_SUCCESS) {                                              \
      ommo_app_error_handler((ERR_CODE), __LINE__, (uint8_t *)__FILE__, __VA_ARGS__); \
    }                                                                                 \
  } while (0)

extern uint8_t dfu_crash_report_buffer[CODE_PAGE_SIZE];

void ommo_app_error_handler(ret_code_t error_code, uint32_t line_num, const uint8_t *p_file_name, ...);
void ommo_app_error_fault_handler(uint32_t id, uint32_t pc, ret_code_t error_code, uint32_t line_num, const uint8_t *p_file_name);

void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info);

uint32_t ommo_app_error_flash_write(uint8_t *buffer, uint16_t buf_size);
uint32_t ommo_app_error_flash_read_decode(uint8_t *dst, size_t dst_size, size_t *return_length);

#ifdef __cplusplus
}
#endif

#endif // OMMO_APP_ERROR_H__