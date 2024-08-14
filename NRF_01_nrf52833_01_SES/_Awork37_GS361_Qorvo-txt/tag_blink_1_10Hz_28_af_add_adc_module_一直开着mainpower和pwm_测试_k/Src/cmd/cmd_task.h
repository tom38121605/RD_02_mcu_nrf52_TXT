/*! ----------------------------------------------------------------------------
 * @file cmd_task.h
 *
 * @brief 
 *
 * @author Decawave
 *
 * @attention Copyright 2018-2019 (c) DecaWave Ltd, Dublin, Ireland.
 *            All rights reserved.
 */
#ifndef _CMD_TASK_H_
#define _CMD_TASK_H_

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_TASK  5
//#define TASK_UNAVAILABLE 0xFF


// Call-back type for tasks
typedef void (*task_cb_t)(void);

typedef struct{
    //volatile uint8_t flag;
    volatile uint8_t tobeclear;
    task_cb_t cb;
} cmd_task_t;

typedef struct{
    cmd_task_t tasks[MAX_TASK];
    uint8_t total;
} cmd_tasklist_t;


/*! ----------------------------------------------------------------------------
 * @fn  cmd_task_init
 *
 * @brief init the task processes. 
 * */
void cmd_task_init(void);

int cmd_task_add(task_cb_t cb);

int cmd_task_rmv(task_cb_t cb);

/*! ----------------------------------------------------------------------------
 * @fn  cmd_task_process
 *
 * @brief Process ongoing command tasks
 * */
void cmd_task_process(void);

#ifdef __cplusplus
}
#endif

#endif /* _CMD_TASK_H_ */
