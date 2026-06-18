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



#ifndef MODAL_PIPE_CLIENT_H
#define MODAL_PIPE_CLIENT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <modal_pipe_common.h>
#include <modal_pipe_interfaces.h>
#include <modal_pipe_deprecated.h>

// Sensible limit on the number of pipes a client can connect to
#define PIPE_CLIENT_MAX_CHANNELS    128


// Flags that can be passed to pipe_client_open()
#define CLIENT_FLAG_EN_SIMPLE_HELPER        (1<<0) // must provide a buffer length on init
#define CLIENT_FLAG_EN_CAMERA_HELPER        (1<<1) // no need to choose a buffer length
#define CLIENT_FLAG_EN_POINT_CLOUD_HELPER   (1<<2) // must provide a buffer length on init
#define CLIENT_FLAG_EN_DEBUG_PRINTS         (1<<3) // turn on debug prints
#define CLIENT_FLAG_DISABLE_AUTO_RECONNECT  (1<<5) // disable auto-reconnect
#define CLIENT_FLAG_START_PAUSED            (1<<6) // claim and configure a pipe channel, but don't open it right away
#define CLIENT_FLAG_EN_ION_BUF_HELPER       (1<<7) 
#define CLIENT_FLAG_MANUAL_ION_BUF_RELEASE  (1<<8) // client will mark buf as unused

/**
 * @brief      Connect to a pipe server and optionally start a background helper
 *             thread to read it.
 *
 *             The topic string should ideally be the full path in the file
 *             system where the pipe server publishes: e.g. /run/mpa/imu1/
 *             However, you can provide the short topic name (e.g. "imu1") and
 *             this function will expand this to the full path assuming it is a
 *             topic located in /run/mpa/
 *
 *             If you are subscribing to a topic being published outside of
 *             /run/mpa/ (for example Android doesn't have a /run/ folder) then
 *             you must specify the entire path beginning at root "/".
 *
 *             First, this sends the client-specific "name" string to the
 *             server's request pipe at topic_dir/request. The server will
 *             respond by opening a new dedicated pipe for this client at the
 *             location topic_dir/name
 *
 *             This function waits up to a second for the server to make this
 *             pipe. It is then opened with the O_RDONLY for blocking reads and
 *             its file descriptor is saved for reading by the user either
 *             directly or with the helper thread.
 *
 *             The easiest way to get data is to enable either a simple or
 *             camera helper read thread by passing in either
 *             EN_PIPE_CLIENT_SIMPLE_HELPER or EN_PIPE_CLIENT_CAMERA_HELPER as
 *             flags. This thread will be started in the background by this
 *             function and will sit in a blocking read loop until the user
 *             calls pipe_client_close(). Each time data is available to read,
 *             the data will be passed by pointer to a user-defined callback
 *             function set with the pipe_client_set_data_cb() or
 *             pipe_client_set_camera_cb() function.
 *
 *             When using the simple data helper, you will need to specify the
 *             size of the read buffer used by this helper thread with the
 *             buf_len argument. This length will be heavily dependent on the
 *             type and frequency of data you expect to receive. The cameras
 *             helper thread will allocate just enough memory for one camera
 *             frame abd the buf_len argument is ignored.
 *
 *             Alternatively, the user can disable the helper thread by not
 *             passing an any of the helper flags and read the pipe file
 *             descriptor manually. To retrieve this file descriptor, use the
 *             pipe_client_get_fd() function.
 *
 *             Additionally, this opens the pipe_DIR/control pipe with the
 *             O_WRONLY flag if the server has enabled a control pipe for that
 *             channel. Its file descriptor is saved to the control_fd[ch]
 *             variable for the user to send control commands to the server. The
 *             client can use the pipe_send_control_cmd() function as a helper
 *             to do this.
 *
 *             Add the PIPE_CLIENT_OPEN_PAUSED flag to claim and initialize the
 *             channel for the given pipe name and location without actually
 *             opening the pipe. This saves the need for opening then pausing
 *             the pipe if the intention is to leave the pipe closed after
 *             initialization for later use.
 *
 *             Up to 128 channels can be opened to receive data from the same or
 *             multiple servers. This channel number has nothing to do with the
 *             server channel. It is only a way to index and differentiate
 *             between multiple open pipes.
 *
 * @param[in]  ch                channel to initialize (0-127)
 * @param      name_or_location  The name or location of the pipe
 * @param[in]  client_name       The client name
 * @param[in]  flags             flags to configure optional features
 * @param[in]  buf_len           The buffer size (bytes) used by the read helper
 *                               thread
 * @param      topic  base directory for the channel you wish to read from
 * @param      name   desired name of the dedicated pipe to be created
 *
 * @return     0 on success, -1 on failure
 */
int pipe_client_open(int ch, const char* name_or_location, const char* client_name, int flags, int buf_len);


/**
 * @brief      Checks all the channels and returns the first channel number not in use
 *
 * @return     channel number on success, -1 if no channels are available.
 */
int pipe_client_get_next_available_channel(void);


/**
 * @brief      reads the JSON info file for a specified open channel.
 *
 *             For reading the info for pipes without opening it, see
 *             pipe_get_info(const char* name_or_location, pipe_info_t* info)
 *             in modal_pipe_common.h
 *
 *             Note that info JSON files may contain extra data beyond what the
 *             pipe_info_t struct contains. Extra data is usually
 *             server-specific and needs to be parsed by the user. See
 *             pipe_get_info_json() and pipe_get_info_json_by_name() for more
 *             info on retreiving extra info data.
 *
 *             Note that this opens, reads, and closes a file from the file
 *             system. Therefore it should generally be used for one-offs or
 *             infrequent checks, don't call this hundreds of times per second.
 *
 * @param[in]  ch    opened pipe channel to retrieve the info for
 * @param[out] info  pointer to user's info struct
 *
 * @return     0 on success, -1 if the info file was not available or some other
 *             error was encountered.
 */
int pipe_client_get_info(int ch, pipe_info_t* info);


/**
 * @brief      Parses the JSON info file for a given open pipe into a cJSON data
 *             structure for user parsing.
 *
 *             For reading the json info for an unopened pipe, see
 *             pipe_get_info_json(const char* name_or_location) in
 *             modal_pipe_common.h
 *
 *             Note that this opens, reads, and closes a file from the file
 *             system. Therefore it should generally be used for one-offs or
 *             infrequent checks, don't call this hundreds of times per second.
 *
 *             You must remember to clean up the memory allocated for the cJSON
 *             data structure when you are done with it!! Use this:
 *
 *             cJSON_Delete(my_json_ptr);
 *
 * @param      ch    channel for the open pipe to read info from
 *
 * @return     a valid pointer on success, NULL if the info file was not
 *             available or some other error was encountered.
 */
cJSON* pipe_client_get_info_json(int ch);


/**
 * @brief      Fetch the number of bytes available to read from the pipe.
 *
 *
 *             This is useful to check to see if a client is falling behind and
 *             needs to dump data to catch up. E.g. when doing heavy processing
 *             on camera frames.
 *
 *             There is an equivalent function pipe_server_bytes_in_pipe() to
 *             allow servers to check this information too.
 *
 * @param[in]  ch    channel to check
 *
 * @return     # bytes available to read or -1 on error
 */
int pipe_client_bytes_in_pipe(int ch);


/**
 * @brief      Fetch the current size (capacity) of the pipe for a given
 *             channel.
 *
 *             Pipes usually default to 64k depending on your platform. This may
 *             be too small for some applications, particularly camera frames.
 *
 *             There is an equivalent function pipe_server_get_pipe_size() to
 *             allow servers to check this information too.
 *
 * @param[in]  ch    channel to check
 *
 * @return     size of pipe in bytes, or -1 on error
 */
int pipe_client_get_pipe_size(int ch);

/**
 * @brief      Set the size of a pipe for a given channel.
 *
 *             A client can (and should!) increase the pipe size in cases where
 *             they need more buffering. Servers may also increase the pipe size
 *             if they see fit.
 *
 *             Pipes behave like FIFO buffers and can store data until ready to
 *             be read by the client. Root can set the pipe size up to 256MiB on
 *             most Linux systems. Normal users can usually set the pipe size up
 *             to 1MiB, but processes usually run as root on VOXL
 *
 *             This function returns the new size of the pipe after attempting
 *             to set it to a new desired size. Usually the kernel will
 *             "round up" to the next exponent of 2 so you may get a pipe size
 *             that's larger than requested. If you request a size that's too
 *             large and the kernel rejects the request, then the pipe size will
 *             remain the same and that existing size will be returned. You
 *             should therefore error-check by seeing if the return value is <
 *             the desired size.
 *
 *             Since the kernel will round-up anyway, you might as well just
 *             request a nice even size, e.g. 1,2,4,8,16,32,64,128, or 256Mib.
 *
 *             e.g. to set the pipe size to 16MiB use size_bytes = 16*1024*1024;
 *
 * @param[in]  ch          channel to set
 * @param[in]  size_bytes  The desired pipe size in bytes
 *
 * @return     new size of the pipe or -1 on error.
 */
int pipe_client_set_pipe_size(int ch, int size_bytes);


/**
 * @brief      assign a callback function to be called when new data is read by
 *             the simple helper thread.
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
 *             Note that this callback will only get called if the simple read
 *             helper thread was enabled when the channel was initialized with
 *             pipe_client_open().
 *
 * @param[in]  ch    channel to assign the callback to (0-127)
 * @param[in]  func  The function pointer
 *
 * @return     0 on success, -1 on failure
 */
typedef void client_simple_cb(int ch, char* data, int bytes, void* context);
int pipe_client_set_simple_helper_cb(int ch, client_simple_cb* cb, void* context);


/**
 * @brief      assign a callback function to be called when new data is read by
 *             the camera helper thread.
 *
 *             The user-defined callback should return void and take 3
 *             arguments: the channel the data was received on, a metadata
 *             struct, and a pointer to the beginning of the frame data.
 *
 *             Although callback functions can be assigned individually for each
 *             channel, having the channel argument allows a single callback to
 *             be assigned for all channels and still be able to differentiate
 *             which channel the message was received on.
 *
 *             Note that this callback will only get called if the camera helper
 *             thread was enabled when the channel was initialized with
 *             pipe_client_open().
 *
 * @param[in]  ch    channel to assign the callback to (0-127)
 * @param[in]  func  The function pointer
 *
 * @return     0 on success, -1 on failure
 */
typedef void client_camera_cb(int ch, camera_image_metadata_t meta, char* frame, void* context);
int pipe_client_set_camera_helper_cb(int ch, client_camera_cb* cb, void* context);


/**
 * @brief      assign a callback function to be called when new data is read by
 *             the point cloud helper thread.
 *
 *             The data provided is in the format XYZXYZXYZ... where each point
 *             consists of 3 floats. The metadata struct will indicate the
 *             number of points, a timestamp, and which reference frame the data
 *             is in.
 *
 *             The user-defined callback should return void and take 3
 *             arguments: the channel the data was received on, a metadata
 *             struct, and a pointer to the beginning of the frame data.
 *
 *             Although callback functions can be assigned individually for each
 *             channel, having the channel argument allows a single callback to
 *             be assigned for all channels and still be able to differentiate
 *             which channel the message was received on.
 *
 *             Note that this callback will only get called if the point cloud
 *             helper thread was enabled when the channel was initialized with
 *             pipe_client_open().
 *
 * @param[in]  ch    channel to assign the callback to (0-127)
 * @param[in]  func  The function pointer
 *
 * @return     0 on success, -1 on failure
 */
typedef void client_pc_cb(int ch, point_cloud_metadata_t meta, void* data, void* context);
int pipe_client_set_point_cloud_helper_cb(int ch, client_pc_cb* cb, void* context);

#ifdef EN_ION_BUF
/**
 * @brief      assign a callback function to be called when new data is read by
 *             the camera helper thread.
 *
 *             The user-defined callback should return void and take 3
 *             arguments: the channel the data was received on, a metadata
 *             struct, and a pointer to the beginning of the frame data.
 *
 *             Although callback functions can be assigned individually for each
 *             channel, having the channel argument allows a single callback to
 *             be assigned for all channels and still be able to differentiate
 *             which channel the message was received on.
 *
 *             Note that this callback will only get called if the camera helper
 *             thread was enabled when the channel was initialized with
 *             pipe_client_open().
 *
 * @param[in]  ch    channel to assign the callback to (0-127)
 * @param[in]  func  The function pointer
 *
 * @return     0 on success, -1 on failure
 */
typedef void client_ion_buf_cb(int ch, mpa_ion_buf_t* data, void* context);
int pipe_client_set_ion_buf_helper_cb(int ch, client_ion_buf_cb* cb, void* context);
#endif

/**
 * @brief      Set what priority the helper thread will have once it is started.
 *
 *             Must be called before pipe_client_open(). If you want to change
 *             helper thread priorities after it has been started then use
 *             pipe_pthread_set_priority() from within the helper thread.
 *
 *             See THREAD_PRIORITY_* definitions in modal_start_stop.h, setting
 *             priority to 0 indicates the thread should use Linux pthread
 *             default scheduler and priority. Otherwise 1-99 sets the thread to
 *             use the Real-Time FIFO scheduler with specified priority.
 *
 *             This is an optional function call, the default behavior is to
 *             start the helper pthread with inherited properties from the
 *             calling process.
 *
 * @param[in]  ch        Channel to assign the thread
 * @param[in]  priority  The priority 0-99
 *
 * @return     0 on success, -1 on failure
 */
int pipe_client_set_helper_thread_priority(int ch, int priority);


/**
 * @brief      assign a callback function to be called when a connection is
 *             established with a server
 *
 *             This is really only useful in auto-reconnect mode to indicate
 *             when a connection has been re-established. however, in normal
 *             mode, this will still be called if the user sets the callback
 *             before initiating the connection and then a connection is
 *             established with pipe_client_open()
 *
 *             The user-defined callback should return void and take 2
 *             arguments: the channel number and an optional context pointer
 *
 *             Although callback functions can be assigned individually for each
 *             channel, having the channel argument allows a single callback to
 *             be assigned for all channels and still be able to differentiate
 *             which channel the message was received on.
 *
 * @param[in]  ch    channel to assign the callback to (0-127)
 * @param[in]  func  The function pointer
 *
 * @return     0 on success, -1 on failure
 */
typedef void client_connect_cb(int ch, void* context);
int pipe_client_set_connect_cb(int ch, client_connect_cb* cb, void* context);



/**
 * @brief      assign a callback function to be called when one of the helpers
 *             disconnects from the server
 *
 *             The user-defined callback should return void and take 2
 *             arguments: the channel number and an optional context pointer
 *
 *             Although callback functions can be assigned individually for each
 *             channel, having the channel argument allows a single callback to
 *             be assigned for all channels and still be able to differentiate
 *             which channel the message was received on.
 *
 *             Note that this callback will only get called if one of the read
 *             helpers was enabled when the channel was initialized.
 *             pipe_client_open().
 *
 * @param[in]  ch    channel to assign the callback to (0-127)
 * @param[in]  func  The function pointer
 *
 * @return     0 on success, -1 on failure
 */
typedef void client_disc_cb(int ch, void* context);
int pipe_client_set_disconnect_cb(int ch, client_disc_cb* cb, void* context);


/**
 * @brief      check if a channel is connected or not
 *
 *             This is mainly to be used with the EN_PIPE_CLIENT_AUTO_RECONNECT
 *             auto reconnect feature to check if the channel is connected or
 *             not without using the connect/disconnect callbacks.
 *
 * @param[in]  ch    channel to check (0-127)
 *
 * @return     1 if connected, 0 if not
 */
int pipe_client_is_connected(int ch);


/**
 * @brief      retrieve the FIFO's file descriptor for the client to read directly
 *
 *             It is NOT recommended to use this function if any of the helper threads
 *             thread were enabled when the pipe was opened with
 *             pipe_client_open(). Note that this file descriptor will
 *             be closed on a call to pipe_client_close() or
 *             pipe_client_close_all();
 *
 * @param[in]  ch    channel to fetch fd for (0-127)
 *
 * @return     file descriptor
 */
int pipe_client_get_fd(int ch);


/**
 * @brief      Send a control command string to the server while connected.
 *
 *             Available valid strings are server-dependent. For example the
 *             "set_pitmode_off" string can be sent to voxl-cpu-monitor via the
 *             cpu_stats2 pipe.
 *
 *             This function is to be used by a client that's actively connected
 *             to the pipe. For one-off commands to be sent without connecting
 *             to the pipe use pipe_send_control_cmd() in modal_pipe_common.h
 *             instead.
 *
 *             Note that creation of a control pipe is optional and determined
 *             by the server. If the server never enabled a control pipe then
 *             this function will return -1.
 *
 * @param[in]  ch           channel to send the command to (0-127)
 * @param      control_cmd  The control string
 *
 * @return     0 on success, -1 on failure
 */
int pipe_client_send_control_cmd(int ch, const void* control_cmd);


/**
 * @brief      Send data to the server through the control pipe while connected.
 *
 *             The data structure is server-dependent. This function differs
 *             from pipe_client_send_control_cmd in that the data does not have
 *             to be a string.
 *
 *             This function is to be used by a client that's actively connected
 *             to the pipe. For one-off commands to be sent without connecting
 *             to the pipe use pipe_send_control_cmd_bytes() in
 *             modal_pipe_common.h instead.
 *
 *             Note that creation of a control pipe is optional and determined
 *             by the server. If the server never enabled a control pipe then
 *             this function will return -1.
 *
 * @param[in]  ch     channel to send the command to (0-127)
 * @param      data   The data
 * @param[in]  bytes  The control string
 *
 * @return     0 on success, -1 on failure
 */
int pipe_client_send_control_cmd_bytes(int ch, const void* data, int bytes);


#ifdef EN_ION_BUF
/**
 * @brief      Sends buffer data back to the server that originally provided an
 *             ION buffer over MPA indicating to the server that the buffer is
 *             now unused by the the client and can safely be flagged as free to
 *             use again by the server.
 *
 * @param[in]  ch       { parameter_description }
 * @param      buf_ptr  The buffer pointer
 *
 * @return     0 on success, -1 on failure.
 */
int pipe_client_report_mpa_ion_buf_unused(int ch, mpa_ion_buf_t* buf_ptr);
#endif

/**
 * @brief      flush all unread data from a pipe
 *
 *             This should be used to discard old data or to get back in sync if
 *             a partial packet is read;
 *
 * @param[in]  ch    channel to flush
 *
 * @return     0 on success, PIPE_ERROR code on failure
 */
int pipe_client_flush(int ch);


/**
 * @brief      closes and deinitializes a single pipe connection
 *
 *             If a read helper thread is enabled, this functions signals the
 *             thread to close. This also closes any open file descriptors
 *             including the data fd even if the client grabbed it with the
 *             pipe_client_get_fd() function.
 *
 *             This function can be safely called from within a helper thread,
 *             but it will not return
 *
 * @param[in]  ch    channel to close (0-127)
 */
void pipe_client_close(int ch);


/**
 * @brief      resumes a single pipe connection that has been paused
 *
 *             This resumes the normal operation of the pipe client
 *
 * @param[in]  ch    channel to resume (0-127)
 */
int pipe_client_resume(int ch);


/**
 * @brief      closes a single pipe connection but does not deinitialize it
 *
 *             This will allow for the ceasing of the processing of data without
 *             cleaning all of the pipe's info, can be resumed with pipe_client_resume
 *
 *             If a read helper thread is enabled, this functions signals the
 *             thread to close. This also closes any open file descriptors
 *             including the data fd even if the client grabbed it with the
 *             pipe_client_get_fd() function.
 *
 *             This function can be safely called from within a helper thread,
 *             but it will not return
 *
 * @param[in]  ch    channel to pause (0-127)
 */
void pipe_client_pause(int ch);

/**
 * @brief      closes all open channels
 */
void pipe_client_close_all(void);





#ifdef __cplusplus
}
#endif

#endif // MODAL_PIPE_CLIENT_H
