/*! ----------------------------------------------------------------------------
 * @file cmd_fn.h
 *
 * @brief  header file for cmd_fn.c
 *
 * @author Decawave
 *
 * @attention Copyright 2018-2019 (c) DecaWave Ltd, Dublin, Ireland.
 *            All rights reserved.
 *
 */
#ifndef _CMD_FN_H_
#define _CMD_FN_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "config.h"

#define REG_FN(fn) const char *fn(char *text, param_block_t *pbss, int val)

#define CALL_FN(func_call)  do{                 \
            const char * ret = CMD_FN_RET_OK;   \
            check_return_t check;               \
            check = func_call() ;               \
            if (check != VALID){                \
                ret = CMD_FN_RET_ERR;           \
            }                                   \
            return ret;                         \
        } while(0)
#define REG_COMMON_FN(fn, func_call) REG_FN(fn) {CALL_FN(func_call);}

/* command table structure definition */
struct command_s
{
    const char *name;            /**< Command name string */
    REG_FN     ((*fn));
};
typedef struct command_s command_t;

/*! ----------------------------------------------------------------------------
 * @brief cmd_fn_get_list gets a list of supported commands
 * @return A {NULL, NULL} command_t* terminated list
 */
const command_t* cmd_fn_get_list(void);

#ifdef __cplusplus
}
#endif

#endif /* _CMD_FN_H_ */
