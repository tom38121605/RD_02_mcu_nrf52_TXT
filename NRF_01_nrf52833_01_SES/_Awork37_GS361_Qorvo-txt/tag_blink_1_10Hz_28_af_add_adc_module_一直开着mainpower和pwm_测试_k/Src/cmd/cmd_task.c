/*
 * @file       cmd_task.c
 *
 * @brief      
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
#include "cmd_task.h"

//static volatile uint8_t rx_ongoing = 0;

static cmd_tasklist_t list;
//static volatile uint8_t curr_task;

//cmd_task_t tasks[MAX_TASK];

void cmd_task_init(void)
{
    //curr_task = TASK_UNAVAILABLE;
    list.total = 0;
    memset((uint8_t*)list.tasks, 0, sizeof(cmd_task_t)*MAX_TASK);

}


int cmd_task_add(task_cb_t cb)
{
    uint8_t i;
    if(list.total >= MAX_TASK)
    {
        return -1;
    }

    //check if already exist
    for(i = 0; i < list.total; i++) 
    {
        if (list.tasks[i].cb == cb)
        {
            return -1;
        }
    }

    //add to 
    list.tasks[list.total].cb = cb;
    list.tasks[list.total].tobeclear = 0;
    list.total += 1;
    
    return 0;
}

int cmd_task_rmv(task_cb_t cb)
{
    uint8_t i;
    for(i = 0; i < list.total; i++) 
    {
        if (list.tasks[i].cb == cb)
        {
            list.tasks[i].tobeclear = 1;
            return 0;
        }
    }
    return -1;
}

/*! ----------------------------------------------------------------------------
 * @fn  task_buffer_process
 *
 * @brief Process ongoing tasks
 * */
void cmd_task_process(void)
{
    uint8_t i;
    // to perform
    for(i = 0; i < list.total; i++) 
    {
        //curr_task = i;
        if (list.tasks[i].cb != NULL)
        {
            list.tasks[i].cb();
        }
    }

    // to clean up
    i = 0;
    while(i < list.total) 
    {
        if (list.tasks[i].tobeclear) 
        {
            list.total -= 1;
            memmove(&list.tasks[i], &list.tasks[i+1], (MAX_TASK-1-i)*sizeof(cmd_task_t));
        }
        else 
        {
            i++;
        }
    }
}
