/*! ----------------------------------------------------------------------------
 * @file production_main.c
 *
 * @brief Main loop for production pack
 *
 * @author DecaWave
 *
 * @attention Copyright 2019 (c) DecaWave Ltd, Dublin, Ireland.
 *            All rights reserved.
 */

#include <string.h>
#include <stdlib.h>

#include "deca_device_api.h"
#include "port.h"
#include "port_stdio.h"
#include "port_spi.h"
#include "port_irq.h"
#include "port_config.h"
#include "cmd_buffer.h"
#include "cmd_task.h"
#include "instance.h"
#include "production_uwb.h"
#include "production_main.h"
//#include "main.h"

static char str1[256];

/*! ----------------------------------------------------------------------------
 * @fn production_main
 * @brief Setup and main loop for production pack
 * 
 * Starts reading commands from stdio and processes them
 */
void production_main()
{
    char greetings0[] = "Init ... ";
    port_stdio_transmit((uint8_t *)greetings0, strlen(greetings0));

    cmd_task_init();
//    strcpy(str1,"port_reset_dwIC()\r\n");
//    port_stdio_write((uint8_t *)str1, strlen(str1));
    port_reset_dw1000();

//    strcpy(str1,"inittestapplication()\r\n");
//    port_stdio_write((uint8_t *)str1, strlen(str1));
    if (inittestapplication() < 0)
    {
        cmd_uwb_urd();
        return;
    }

    char greetings[] = "Done...\r\n";
    port_stdio_transmit((uint8_t *)greetings, strlen(greetings));

    if (port_stdio_receive(cmd_buffer_add_data) < 0)
    {
        return;
    }

    while (1)
    {
        cmd_task_process();
        cmd_buffer_process();
    }
}
