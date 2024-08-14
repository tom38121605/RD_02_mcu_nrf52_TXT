/*! ----------------------------------------------------------------------------
 * @file cmd_buffer.h
 *
 * @brief Double buffer storing commands and processing them when ready
 *
 * @author Decawave
 *
 * @attention Copyright 2018-2019 (c) DecaWave Ltd, Dublin, Ireland.
 *            All rights reserved.
 */
#ifndef _CMD_BUFFER_H_
#define _CMD_BUFFER_H_

#ifdef __cplusplus
extern "C" {
#endif

/*! ----------------------------------------------------------------------------
 * @fn cmd_buffer_add_data
 * @brief Add a byte of data to the command buffer
 * 
 * Adds a single byte of data to the command buffer and echos byte to stdio
 * 
 */
void cmd_buffer_add_data(uint8_t data);

/*! ----------------------------------------------------------------------------
 * @fn  cmd_buffer_process
 *
 * @brief Process commands in the command buffer
 */
void cmd_buffer_process(void);

/*! ----------------------------------------------------------------------------
 * @fn  task_buffer_process
 *
 * @brief Process ongoing tasks
 */
void cmd_task_process(void);

#ifdef __cplusplus
}
#endif

#endif /* _CMD_BUFFER_H_ */
