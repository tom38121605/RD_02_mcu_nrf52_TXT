#include <stdio.h>
#include <string.h>

#include <stdint.h>
#include <inttypes.h>

#include "nrf.h"
#include "ommo_config.h"
#include "ommo_fw.pb.h"

#include "usb_serial_num.h"

//USB serial number string 'SIU_CLASS'+'SERIAL'
char g_extern_serial_number[SERIAL_NUMBER_STRING_SIZE];

char nibble_to_hex_char(uint8_t value)
{
    return value < 10 ? '0' + value : 'A' + value - 10;
}

void byte_to_hex_string(char *dst, uint8_t value)
{
    dst[0] = nibble_to_hex_char(value >> 4);
    dst[1] = nibble_to_hex_char(value & 0xf);
}

void ommo_usbd_serial_num_generate(void)
{
    byte_to_hex_string(&g_extern_serial_number[0], OMMO_USB_COMMUNICATION_CLASS);

    uint32_t device_id = NRF_FICR->DEVICEID[0];
    byte_to_hex_string(&g_extern_serial_number[2], (uint8_t)(device_id >> 24));
    byte_to_hex_string(&g_extern_serial_number[4], (uint8_t)(device_id >> 16));
    byte_to_hex_string(&g_extern_serial_number[6], (uint8_t)(device_id >>  8));
    byte_to_hex_string(&g_extern_serial_number[8], (uint8_t) device_id       );

    g_extern_serial_number[10] = '\0';
}
