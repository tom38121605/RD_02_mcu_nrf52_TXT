#include "app_error.h"
#include "nordic_common.h"
#include "nrf_nvmc.h"
#include "sdk_errors.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_strerror.h"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "cobs.h"
#include "ommo_time_def.h"
#include "ommo_app_error.h"
#include "ommo_config.h"

// data page for crash report
uint8_t dfu_crash_report_buffer[CODE_PAGE_SIZE] __attribute__((section(".as_crash_storage_page"))) __attribute__((used));

static void copy_uint8(unsigned char packetBuffer[], uint16_t *index, const unsigned char value)
{
    packetBuffer[*index] = value;
    *index += 1;
}

static void copy_uint16(unsigned char packetBuffer[], uint16_t *index, const uint16_t value)
{
    memcpy(packetBuffer+*index, &value, 2);
    *index += 2;
}

static void copy_uint32(unsigned char packetBuffer[], uint16_t *index, const uint32_t value)
{
    memcpy(packetBuffer+*index, &value, 4);
    *index += 4;
}

static uint32_t copy_variable(uint8_t *buf, uint16_t *index, CrashDataType data_type, void *var, size_t size)
{
    if (*index + size + 4 <= OMMO_LOG_BUF_SIZE) // [data_type] + [cobs hdr] + [crc] + [ending 0]
    {
    
        // data_type
        copy_uint8(buf, index, data_type);

        // data
        memcpy(buf + *index, var, size);
        *index += size;
    
        return NRF_SUCCESS;
    }
    else
    {
        return NRF_ERROR_NO_MEM;
    }
}

static uint32_t copy_array(uint8_t *buf, uint16_t *index, CrashDataType data_type, void *arry, uint16_t length, size_t size)
{
  
    if (*index + size + 6 <= NRF_CODE_PAGE_SIZE) // [data_type] + [cobs hdr] + [crc] + [arry_length] + [ending 0]
    {
        // data_type
        copy_uint8(buf, index, data_type);

        // arry length (2 bytes)
        copy_uint16(buf, index, length);

        // data
        memcpy(buf + *index, arry, size);
        *index += size;
    
        return NRF_SUCCESS;
    }
    else
    {
        return NRF_ERROR_NO_MEM;
    }
}

void ommo_app_error_handler(ret_code_t error_code, uint32_t line_num, const uint8_t *p_file_name, ...)
{
    uint8_t ommo_log_buf[OMMO_LOG_BUF_SIZE];
    uint8_t ommo_log_buf_encode[OMMO_LOG_BUF_SIZE] = {0};
    uint16_t ommo_log_idx;

    va_list ap;

    void *temp;
    uint16_t var_size, var_idx;
    uint16_t arry_length;

    //Try not to crash while crashing :)
    __disable_irq();

    ommo_log_idx = 0;

    // Ommo err log rev
    copy_uint8(ommo_log_buf, &ommo_log_idx, OMMO_ERR_LOG_REV);

    // Program type added in rev 1
    copy_uint8(ommo_log_buf, &ommo_log_idx,
#if defined(OMMO_CODE_END) && defined(OMMO_BOOTLOADER_START_ADDR)
               OMMO_CODE_END < OMMO_BOOTLOADER_START_ADDR
#else
               1  // default to application (0 == boot loader)
#endif
               );

    // error_code
    copy_uint16(ommo_log_buf, &ommo_log_idx, error_code);

    // file_name
    if (p_file_name != NULL)
    {
        // File_name_length
        uint8_t file_length = strlen(p_file_name);
        copy_uint8(ommo_log_buf, &ommo_log_idx, file_length);

        // File_name
        memcpy(ommo_log_buf + ommo_log_idx, p_file_name, file_length);
        ommo_log_idx += file_length;
    }

    // Line num
    copy_uint16(ommo_log_buf, &ommo_log_idx, line_num);

    // Compile Epoch time
    copy_uint32(ommo_log_buf, &ommo_log_idx, DATE_TIME_UNIX);

#ifndef OMMO_NO_CRASH_REPORT_VARIABLES
    // handle the variables
    va_start(ap, p_file_name);

    while(true)
    {
        // Grab datatype
        CrashDataType data_type = (CrashDataType)va_arg(ap, int);

        //Continue until datatype returned is the 0 terminator
        if(data_type == 0)
            break;

        // read the data and push to ommo_log_buf
        switch (data_type)
        {
            case CRASH_DATA_TYPE_BOOL:
            {
              _Bool bl = (_Bool)va_arg(ap, int);
              copy_variable(ommo_log_buf, &ommo_log_idx, data_type, &bl, sizeof(_Bool));
              break;
            }
            case CRASH_DATA_TYPE_CHAR:
            {
              char ch = (char)va_arg(ap, int);
              copy_variable(ommo_log_buf, &ommo_log_idx, data_type, &ch, sizeof(char));
              break;
            }
            case CRASH_DATA_TYPE_SIGNED_CHAR:
            {
              signed char sgd_ch = (signed char)va_arg(ap, int);
              copy_variable(ommo_log_buf, &ommo_log_idx, data_type, &sgd_ch, sizeof(signed char));
              break;
            }
            case CRASH_DATA_TYPE_UNSIGNED_CHAR:
            {
              unsigned char u_c = (unsigned char)va_arg(ap, int);
              copy_variable(ommo_log_buf, &ommo_log_idx, data_type, &u_c, sizeof(unsigned char));
              break;
            }
            case CRASH_DATA_TYPE_SHORT_INT:
            {
              short int si = (short int)va_arg(ap, int);
              copy_variable(ommo_log_buf, &ommo_log_idx, data_type, &si, sizeof(short int));
              break;
            }
            case CRASH_DATA_TYPE_UNSIGNED_SHORT_INT:
            {
              unsigned short int u_si = (unsigned short int)va_arg(ap, int);
              copy_variable(ommo_log_buf, &ommo_log_idx, data_type, &u_si, sizeof(unsigned short int));
              break;
            }
            case CRASH_DATA_TYPE_INT:
            {
              int intg = va_arg(ap, int);
              copy_variable(ommo_log_buf, &ommo_log_idx, data_type, &intg, sizeof(int));
              break;
            }
            case CRASH_DATA_TYPE_UNSIGNED_INT:
            {
              unsigned int u_intg = (unsigned int)va_arg(ap, unsigned int);
              copy_variable(ommo_log_buf, &ommo_log_idx, data_type, &u_intg, sizeof(unsigned int));
              break;
            }
            case CRASH_DATA_TYPE_LONG_INT:
            {
              long int li = va_arg(ap, long int);
              copy_variable(ommo_log_buf, &ommo_log_idx, data_type, &li, sizeof(long int));
              break;
            }
            case CRASH_DATA_TYPE_UNSIGNED_LONG_INT:
            {
              unsigned long int u_li = va_arg(ap, unsigned long int);
              copy_variable(ommo_log_buf, &ommo_log_idx, data_type, &u_li, sizeof(unsigned long int));
              break;
            }
            case CRASH_DATA_TYPE_LONG_LONG_INT:
            {
              long long int lli = va_arg(ap, long long int);
              copy_variable(ommo_log_buf, &ommo_log_idx, data_type, &lli, sizeof(long long int));
              break;
            }
            case CRASH_DATA_TYPE_UNSIGNED_LONG_LONG_INT:
            {
              unsigned long long int u_lli = va_arg(ap, unsigned long long int);
              copy_variable(ommo_log_buf, &ommo_log_idx, data_type, &u_lli, sizeof(unsigned long long int));
              break;
            }
            case CRASH_DATA_TYPE_FLOAT:
            {
              float flt = (float)va_arg(ap, double);
              copy_variable(ommo_log_buf, &ommo_log_idx, data_type, &flt, sizeof(float));
              break;
            }
            case CRASH_DATA_TYPE_DOUBLE:
            {
              double dbl = va_arg(ap, double);
              copy_variable(ommo_log_buf, &ommo_log_idx, data_type, &dbl, sizeof(double));
              break;
            }
            case CRASH_DATA_TYPE_LONG_DOUBLE:
            {
              long double l_dbl = va_arg(ap, long double);
              copy_variable(ommo_log_buf, &ommo_log_idx, data_type, &l_dbl, sizeof(long double));
              break;
            }
            case CRASH_DATA_TYPE_PTR_TO_UINT8_T:
            case CRASH_DATA_TYPE_STRING:
            {
              temp = va_arg(ap, uint8_t *);
              arry_length = (uint16_t)va_arg(ap, int);
              var_size = arry_length;
              copy_array(ommo_log_buf, &ommo_log_idx, data_type, temp, arry_length, var_size);
              break;
            }
            case CRASH_DATA_TYPE_PTR_TO_UINT16_T:
            {
              temp = va_arg(ap, uint16_t *);
              arry_length = (uint16_t)va_arg(ap, int);
              var_size = arry_length * sizeof(uint16_t);
              copy_array(ommo_log_buf, &ommo_log_idx, data_type, temp, arry_length, var_size);
              break;
            }
            case CRASH_DATA_TYPE_PTR_TO_UINT32_T:
            {
              temp = va_arg(ap, uint32_t *);
              arry_length = (uint16_t)va_arg(ap, int);
              var_size = arry_length * sizeof(uint32_t); //Was <<3???
              copy_array(ommo_log_buf, &ommo_log_idx, data_type, temp, arry_length, var_size);
              break;
            }
            case CRASH_DATA_TYPE_PTR_TO_INT8_T:
            {
              temp = va_arg(ap, int8_t *);
              arry_length = (uint16_t)va_arg(ap, int);
              var_size = arry_length;
              copy_array(ommo_log_buf, &ommo_log_idx, data_type, temp, arry_length, var_size);
              break;
            }
            case CRASH_DATA_TYPE_PTR_TO_INT16_T:
            {
              temp = va_arg(ap, int16_t *);
              arry_length = (uint16_t)va_arg(ap, int);
              var_size = arry_length * sizeof(uint16_t);
              copy_array(ommo_log_buf, &ommo_log_idx, data_type, temp, arry_length, var_size);
              break;
            }
            case CRASH_DATA_TYPE_PTR_TO_INT32_T:
            {
              temp = va_arg(ap, int32_t *);
              arry_length = (uint16_t)va_arg(ap, int);
              var_size = arry_length * sizeof(uint32_t); //Was <<3???
              copy_array(ommo_log_buf, &ommo_log_idx, data_type, temp, arry_length, var_size);
              break;
            }
        }
    }

    va_end(ap);
#endif // !OMMO_NO_CRASH_REPORT_VARIABLES

    // encode the buf
    uint16_t encode_size = cobs_encode_crc_post0(ommo_log_buf, ommo_log_idx, ommo_log_buf_encode);

    // write to flash
    ommo_app_error_flash_write(ommo_log_buf_encode, encode_size);

    // to fit the variable to the fault handler
    error_info_t error_info =
        {
            .line_num = line_num,
            .p_file_name = p_file_name,
            .err_code = error_code,
        };

    // ommo_app_error_fault_handler(NRF_FAULT_ID_SDK_ERROR, 0, (uint32_t)(&error_info));
    ommo_app_error_fault_handler(NRF_FAULT_ID_SDK_ERROR, 0, error_code, line_num, p_file_name);

    UNUSED_VARIABLE(error_info);
}

/**
 * Function is implemented as weak so that it can be overwritten by custom application error handler
 * when needed.
 */
//__WEAK void ommo_app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
__WEAK void ommo_app_error_fault_handler(uint32_t id, uint32_t pc, ret_code_t error_code, uint32_t line_num, const uint8_t *p_file_name)
{
    __disable_irq();
    // NRF_LOG_FINAL_FLUSH();	//Junyu: Flush hangs. need to go back and check.

    NRF_LOG_ERROR("Fatal error");

    error_info_t info =
        {
            .line_num = line_num,
            .p_file_name = p_file_name,
            .err_code = error_code,
        };

    switch (id)
    {
        case NRF_FAULT_ID_SDK_ASSERT:
        {
            // assert_info_t * p_info = &info;
            error_info_t *p_info = &info;
            NRF_LOG_ERROR("ASSERTION FAILED at %s:%u",
                p_info->p_file_name,
                p_info->line_num);
            break;
        }

        case NRF_FAULT_ID_SDK_ERROR:
        {
            error_info_t *p_info = &info;
            NRF_LOG_ERROR("ERROR %u [%s] at %s:%u\r\nPC at: 0x%08x",
                p_info->err_code,
                nrf_strerror_get(p_info->err_code),
                p_info->p_file_name,
                p_info->line_num,
                pc);
            NRF_LOG_ERROR("End of error report");
            break;
        }

        default:
          NRF_LOG_ERROR("UNKNOWN FAULT at 0x%08X", pc);
          break;
    }

    NRF_BREAKPOINT_COND;

    // On assert, the system can only recover with a reset.
    // NRF_LOG_WARNING("System reset");
    // NVIC_SystemReset();

    app_error_save_and_stop(id, pc, (uint32_t)(&info));
}

/* Rewrite the Nordic weak func and make it as an entry to ommo err handler */
void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
  /* Redirect to ommo_err_handler if it is from app_error_handler_bare <- APP_ERROR_CHECK */
    if (id == NRF_FAULT_ID_SDK_ERROR)
    {
        error_info_t *ommo_info = (error_info_t *)info;

        // Entry to ommo_err_handler
        ommo_app_error_handler(ommo_info->err_code, ommo_info->line_num, ommo_info->p_file_name, 0);
    }

    /* Redirect to ommo_err_handler and store assert data
     * ASSERT(expr) -> func assert_nrf_callback
     */
    else if (id == NRF_FAULT_ID_SDK_ASSERT)
    {
        assert_info_t *ommo_info = (assert_info_t *)info;

        // Entry to ommo_err_handler
        ommo_app_error_handler(id, ommo_info->line_num, ommo_info->p_file_name, 0);
    }

    /* Original Nordic scheme */
    else
    {
      __disable_irq();
      NRF_LOG_FINAL_FLUSH();

#ifndef DEBUG
      NRF_LOG_ERROR("Fatal error");
 #else
      switch (id)
      {
#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
          case NRF_FAULT_ID_SD_ASSERT:
              NRF_LOG_ERROR("SOFTDEVICE: ASSERTION FAILED");
              break;

          case NRF_FAULT_ID_APP_MEMACC:
              NRF_LOG_ERROR("SOFTDEVICE: INVALID MEMORY ACCESS");
              break;
#endif
      case NRF_FAULT_ID_SDK_ASSERT:
      {
          assert_info_t *p_info = (assert_info_t *)info;
          NRF_LOG_ERROR("ASSERTION FAILED at %s:%u",
              p_info->p_file_name,
              p_info->line_num);
          break;
      }

      case NRF_FAULT_ID_SDK_ERROR:
      {
          error_info_t *p_info = (error_info_t *)info;
          NRF_LOG_ERROR("ERROR %u [%s] at %s:%u\r\nPC at: 0x%08x",
              p_info->err_code,
              nrf_strerror_get(p_info->err_code),
              p_info->p_file_name,
              p_info->line_num,
              pc);
          NRF_LOG_ERROR("End of error report");
          break;
      }

      default:
          NRF_LOG_ERROR("UNKNOWN FAULT at 0x%08X", pc);
          break;
      }
#endif

      NRF_BREAKPOINT_COND;
      // On assert, the system can only recover with a reset.

#ifndef DEBUG
      NRF_LOG_WARNING("System reset");
      NVIC_SystemReset();
#else
      app_error_save_and_stop(id, pc, info);
#endif // DEBUG
    }
}

uint32_t ommo_app_error_flash_write(uint8_t *buffer, uint16_t buf_size)
{
    uint32_t page_addr = (uint32_t)dfu_crash_report_buffer;

    if (buf_size > NRF_CODE_PAGE_SIZE)
      return NRF_ERROR_INVALID_DATA;

    nrf_nvmc_page_erase(page_addr);
    nrf_nvmc_write_bytes(page_addr, buffer, buf_size);

    return NRF_SUCCESS;
}

uint32_t ommo_app_error_flash_read_decode(uint8_t *dst, size_t dst_size, size_t *return_length)
{
    // decode data
    uint8_t crc;
    size_t input_length_used;
    uint8_t *page_addr = (uint8_t *)(dfu_crash_report_buffer);

    // decode data
    bool success = cobs_decode_crc(page_addr, OMMO_LOG_BUF_SIZE, dst, dst_size, &crc, &input_length_used, return_length);

    if(!success || crc != 0x00)
    {
        return NRF_ERROR_NOT_FOUND;
    }
  
    return NRF_SUCCESS;
}
