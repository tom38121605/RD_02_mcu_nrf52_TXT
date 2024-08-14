/*
 * @file       cmd_buffer.c
 *
 * @brief      Buffer storing commands and processing them when ready
 *
 * @author     Decawave
 *
 * @attention  Copyright 2018-2019 (c) DecaWave Ltd, Dublin, Ireland.
 *             All rights reserved.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "port_stdio.h"
#include "cmd.h"
#include "cmd_buffer.h"

#define BUFFER_SIZE (128)
static uint8_t buffer[BUFFER_SIZE];
static uint8_t buffer_repeat[BUFFER_SIZE];
static size_t buffer_len_repeat = 0;

static volatile bool command_ready = false;
static volatile bool command_repeat = false;

/*! ----------------------------------------------------------------------------
 * @fn cmd_buffer_add_data
 * @brief Add a byte of data to the command buffer
 * 
 * Adds a single byte of data to the command buffer, echos byte to stdio
 * 
 */
void cmd_buffer_add_data(uint8_t data)
{
    static size_t index_buffer = 0;

    if (command_ready)
    {
        // Main context is still in the process of sending data
        // ignore new data and backspaces
        return;
    }
    if ((data == '\b' || data == 0x7F) && index_buffer > 0)
    {
        // backspace or DEL, clear last character
        port_stdio_write((uint8_t*)"\b \b", 3);
        index_buffer--;
    }
    else if (index_buffer < BUFFER_SIZE)
    {
        if (data == '\n' || data == '\r')
        {
            port_stdio_write((uint8_t *)"\r\n", 2);
            if (index_buffer == 0 || buffer[index_buffer-1] == 0)
            {   
                //parse "empty" commands: repeat last command
                command_repeat = true;
                //return;
            }
            else
            {
                buffer_len_repeat = index_buffer + 1;
            }
            buffer[index_buffer] = 0;
            index_buffer = 0;
            command_ready = true;
        }
        else
        {
            port_stdio_putc(data);
            buffer[index_buffer] = data;
            index_buffer++;
        }
    }
}

/*! ----------------------------------------------------------------------------
 * @fn  cmd_buffer_process
 *
 * @brief Process commands in the command buffers
 * */
void cmd_buffer_process(void)
{
    if (port_stdio_read(cmd_buffer_add_data) < 1)
    {
        return;
    }
    if (command_ready)
    {
        if(command_repeat)
        {
            memcpy(buffer, buffer_repeat, buffer_len_repeat);
            command_repeat = false;
            cmd_parse(buffer);
        }
        else
        {
            memcpy(buffer_repeat, buffer, buffer_len_repeat);
            cmd_parse((char *)buffer); // parse and execute the command
        }
        command_ready = false;
    }
}
