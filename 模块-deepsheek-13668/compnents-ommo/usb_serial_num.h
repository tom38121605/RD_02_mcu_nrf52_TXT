#ifndef USB_SERIAL_NUM
#define USB_SERIAL_NUM

#ifdef __cplusplus
extern "C" {
#endif

#define SERIAL_NUMBER_STRING_SIZE (10+1)

extern char g_extern_serial_number[SERIAL_NUMBER_STRING_SIZE];

void ommo_usbd_serial_num_generate(void);


#ifdef __cplusplus
}
#endif

#endif //USB_SERIAL_NUM
