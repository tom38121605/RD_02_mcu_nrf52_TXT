/*! ----------------------------------------------------------------------------
 * @file cmd.c
 *
 * @brief Command parser
 *
 * @author Decawave
 *
 * @attention Copyright 2018-2019 (c) DecaWave Ltd, Dublin, Ireland.
 *            All rights reserved.
 *
 */
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#include "cmd_fn.h"
#include "port_stdio.h"
#include "port_config.h"
#include "cmd.h"

//-----------------------------------------------------------------------------
// non-reentrant to reuse get_argc/get_arg_num in production code
#define MAX_NUM_ARGS    (13)
static char *txt;                    //static pointer to the "text" string
static char *txt_argv[MAX_NUM_ARGS]; //the arguments scanned by "get_argc" fn.

/*! ----------------------------------------------------------------------------
 * @brief get_argc will decompose string to few tokens.
 * It will destroy the current string and make it as a set of tokens in the
 * txt_argv array
 * @return The amount of arguments available
 */
int cmd_get_argc(void)
{
    int ret = 0;
    char *p = txt;

    memset(txt_argv, 0, sizeof(txt_argv));

    p = strtok(txt, " ");
    while (p) {
        if (ret < MAX_NUM_ARGS)
        {
            txt_argv[ret++] = p;
        }
        else
        {
            break;
        }
        p = strtok(NULL, " ");
    }

    return ret;
}

/*! ----------------------------------------------------------------------------
 * @brief this implementation of "get_arg_num" should be called after "get_argc"
 * @return -1 if wrong, but if the parameter is -1
 */
int cmd_get_arg_num(unsigned int i)
{
    int  ret = -1 ;

    if (txt_argv[i])
    {
        if (strncmp(txt_argv[i], "0X", 2) == 0)
        {
            ret = strtoul(txt_argv[i], NULL, 16);
        }
        else
        {
            ret = atoi(txt_argv[i]);
        }
    }

    return ret;
}

/*! ----------------------------------------------------------------------------
 * @brief this implementation of "get_arg" should be called after "get_argc"
 * @return the argument at index i
 * */
const char *cmd_get_arg(unsigned int i)
{
    return txt_argv[i];
}

/*
 * @brief "error" will be sent if error during parser or command execution returned error
 * */
static void cmd_on_error(const char *err)
{
    static const char* error = "error \r\n";
    port_stdio_write((uint8_t*)error, strlen(error));
    port_stdio_write((uint8_t*)err, strlen(err));
    port_stdio_write((uint8_t*)"\r\n", 2);
}

/*! ----------------------------------------------------------------------------
 * @fn    cmd_parse
 * @brief check if input text in known "COMMAND" and executes COMMAND
 */
void cmd_parse(char *text)
{
    char* cmd = text;
    char* args = NULL;
    int val = 0;

    while (*text) {
        if (args == NULL && *text == ' ')
        {
            // Split text in cmd and args so we can use cmd as string
            *text = 0;
            text++;
            while (*text && *text == ' ')
            {
                text++;
            }
            if (*text)
            {
                args = text;
                val = (int)strtol(args, &text, 10);
            }
        }
        else
        {
            *text = (char)toupper(*text);
            text++;
        }
    }

    int i = 0;
    const command_t* known_commands = cmd_fn_get_list();
    while (known_commands[i].name != NULL)
    {
        command_t* command = (command_t *) &known_commands[i];

        if ((strlen(cmd) == strlen(command->name)) &&
            (strcmp(cmd, command->name) == 0))
        {
            if (args && args > cmd)
            {
                // Glue cmd and args back together
                *(args - 1) = ' ';
            }
            txt = cmd;
            param_block_t *config = port_config_get();
            const char* ret = command->fn(txt, config, val);

            if (ret)
            {
                port_stdio_write((uint8_t*)ret, strlen(ret));
            }
            else
            {
                cmd_on_error(" function");
            }
        }
        i++;
    }
}
