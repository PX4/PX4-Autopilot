/*******************************************************************************
 * Copyright 2025 ModalAI Inc.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 ******************************************************************************/


#ifndef MODAL_PIPE_SINK_H
#define MODAL_PIPE_SINK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <modal_pipe_common.h>
#include <modal_pipe_deprecated.h>

// Sensible limit on the number sinks a process can create
#define PIPE_SINK_MAX_CHANNELS  16

// Flags that can be passed to pipe_sink_create()
#define SINK_FLAG_EN_SIMPLE_HELPER      (1<<0) // must provide a buffer length on init
#define SINK_FLAG_EN_DEBUG_PRINTS       (1<<1)

/**
 * @brief      Creates a new named pipe at the specified path to act as a sink
 *             for other processes to send data into.
 *
 *             It is opened for blocking reads and its file descriptor is saved
 *             for reading by the user either directly or with the helper
 *             thread.
 *
 *             The easiest way to get data is to enable the helper read thread
 *             by setting the SINK_FLAG_EN_SIMPLE_HELPER bit in the flags
 *             argument. This thread will be started in the background by this
 *             function and will sit in a blocking read loop until the user
 *             calls pipe_sink_close_channel() or pipe_sink_close_all(). Each
 *             time data is available to read, the data will be passed by
 *             pointer to a user-defined callback function set with the
 *             pipe_sink_set_data_cb() function.
 *
 *             You will need to specify the size of the read buffer used by this
 *             helper thread with the buf_len argument. This length will be
 *             heavily dependent on the type and frequency of data you expect to
 *             receive. The buffer is dynamically allocated on the heap.
 *
 *             Alternatively, the user can disable the helper thread and read
 *             the pipe file descriptor manually. To retrieve this file
 *             descriptor, use the pipe_sink_get_fd() function.
 *
 * @param[in]  ch         channel to initialize
 * @param      path       path for the pipe to be created
 * @param[in]  flags      flags to configure optional features
 * @param[in]  pipe_size  The pipe size (recommended 1M which is large but safe)
 * @param[in]  buf_len    The buffer size (bytes) used by the read helper thread
 *
 * @return     0 on success, -1 on failure
 */
int pipe_sink_create(int ch, const char* path, int flags, int pipe_size, int buf_len);


/**
 * @brief      Assign a callback function to be called when new data is read by
 *             the read helper thread.
 *
 *             The user-defined callback should return void and take 3
 *             arguments: the channel the data was received on, a pointer to the
 *             data, and the number of bytes read.
 *
 *             Although callback functions can be assigned individually for each
 *             channel, having the channel argument allows a single callback to
 *             be assigned for all channels and still be able to differentiate
 *             which channel the message was received on.
 *
 *             Note that this callback will only get called if the read helper
 *             thread was enabled when the channel was initialized with
 *             pipe_sink_init_channel().
 *
 * @param[in]  ch    channel to assign the callback to
 * @param[in]  func  The function pointer
 *
 * @return     0 on success, -1 on failure
 */
typedef void sink_simple_cb(int ch, char* data, int bytes, void* context);
int pipe_sink_set_simple_cb(int ch, sink_simple_cb* cb, void* context);


/**
 * @brief      retrieve the data pipe file descriptor for the client to read
 *
 *             It is NOT recommended to use this function if the the read helper
 *             thread was enabled when the channel was initialized with
 *             pipe_sink_init_channel(). Note that this file descriptor will
 *             be closed on a call to pipe_sink_close_channel() or
 *             pipe_sink_close_all();
 *
 * @param[in]  ch    channel to fetch fd for
 *
 * @return     file descriptor
 */
int pipe_sink_get_fd(int ch);


/**
 * @brief      closes and deinitializes a single channel
 *
 *             If the read helper thread is enabled, this functions signals the
 *             thread to close. This also closes and deletes the pipe even if
 *             the client grabbed its file descriptor with the
 *             pipe_client_get_fd() function.
 *
 * @param[in]  ch    channel to close
 */
void pipe_sink_close(int ch);


/**
 * @brief      closes all open channels
 */
void pipe_sink_close_all(void);



#ifdef __cplusplus
}
#endif

#endif // MODAL_PIPE_SINK_H
