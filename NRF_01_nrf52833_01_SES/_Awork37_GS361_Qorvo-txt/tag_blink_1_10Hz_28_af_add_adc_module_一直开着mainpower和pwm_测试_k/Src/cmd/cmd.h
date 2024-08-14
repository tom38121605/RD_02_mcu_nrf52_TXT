/*! ----------------------------------------------------------------------------
 * @file cmd.h
 *
 * @brief  header file for cmd.c
 *
 * @author Decawave
 *
 * @attention Copyright 2018-2019 (c) DecaWave Ltd, Dublin, Ireland.
 *            All rights reserved.
 *
 */
#ifndef _CMD_H_
#define _CMD_H_

#ifdef __cplusplus
extern "C" {
#endif

/*! ----------------------------------------------------------------------------
 * @brief get_argc will decompose string to few tokens.
 * It will destroy the current string and make it as a set of tokens in the
 * txt_argv array
 * @return The amount of arguments availible
 */
int cmd_get_argc(void);

/*! ----------------------------------------------------------------------------
 * @brief this implementation of "get_arg_num" should be called after "get_argc"
 * @return -1 if wrong, but if the parameter is -1
 */
int cmd_get_arg_num(unsigned int i);

/*! ----------------------------------------------------------------------------
 * @brief this implementation of "get_arg" should be called after "get_argc"
 * @return the argument at index i
 * */
const char *cmd_get_arg(unsigned int i);

/*! ----------------------------------------------------------------------------
 * @fn    cmd_parse
 * @brief check if input text in known "COMMAND" and executes COMMAND
 */
void cmd_parse(char *text);

#ifdef __cplusplus
}
#endif

#endif /* _CMD_H_ */
