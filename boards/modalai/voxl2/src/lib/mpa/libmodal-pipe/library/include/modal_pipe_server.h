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


#ifndef MODAL_PIPE_SERVER_H
#define MODAL_PIPE_SERVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <modal_pipe_common.h>
#include <modal_pipe_interfaces.h>

#ifdef EN_ION_BUF
#include <modal_pipe_buffers.h>
#endif

#include <modal_pipe_deprecated.h>
#include <modal_json.h>

// Sensible limit on the number of channels a server can advertize
#define PIPE_SERVER_MAX_CHANNELS        64
// Sensible limit on the number of clients that can connect to a single channel
#define PIPE_SERVER_MAX_CLIENTS_PER_CH  16

// Flags parsed to pipe_server_init_channel() to control behavior.
#define SERVER_FLAG_EN_CONTROL_PIPE     (1<<0)
#define SERVER_FLAG_EN_DEBUG_PRINTS     (1<<1)
//#define SERVER_FLAG_EN_INFO_PIPE      (1<<2) // moved to deprecated.h, don't use bit 2 anymore!!!!!
#define SERVER_FLAG_EN_ION_BUF          (1<<3)

// Possible states for a client
#define CLIENT_UNINITIALIZED    0   // client doesnt exist yet, no request has been made.
#define CLIENT_INITIALIZED      1   // new pipe has been made for the client, but no data has been transfered yet
#define CLIENT_CONNECTED        2   // client has connected and has opened the pipe for reading
#define CLIENT_DISCONNECTED     3   // Server tried to write to the client but client didn't have the pipe open for reading


////////////////////////////////////////////////////////////////////////////////
// Init and close functions
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief      Initialize an set up a single channel to advertize a specified
 *             topic.
 *
 *             The provided topic name can be a short name or a complete path.
 *
 *             A short name, e.g. "imu0", will be automatically expanded to a
 *             complete path "/run/mpa/imu0/" if the string does not begin with
 *             a "/". If you are on a platform such as Android that doesn't have
 *             a /run/ directory or you wish to put the topic somewhere else in
 *             the file system, then you can provide a complete path beginning
 *             and ending with "/".
 *
 *             This will create the complete topic directory including all
 *             necessary parent directories.
 *
 *             Up to 16 channels are supported for simultaneous use. This
 *             channel number has nothing to do with the client channel. It is
 *             only a way to index and differentiate between multiple open
 *             topics.
 *
 *             Within the complete topic directory this function will create a
 *             request pipe for clients to send requests to. This also starts a
 *             lightweight pthread to service the request pipe.
 *
 *             This will also create a "pid" pipe in the topic directory and a
 *             thread to service it. From that pipe, other users or clients can
 *             read the PID of the process that created the topic, allowing them
 *             to shut down or monitor the state of the topic. See
 *             modal-kill-pipe.
 *
 *             Extra features can be turned on/off with the flags field.
 *             Currently only one extra feature is supported: the control pipe:
 *
 *             SERVER_FLAG_EN_CONTROL_PIPE
 *
 *             By enabling the control pipe, an extra pipe is opened alongside
 *             the "request" pipe with the static name "control". Clients can
 *             send anything into this control pipe and the server can read the
 *             data via callback set by pipe_server_set_control_cb(). An example
 *             of this use case is the IMU calibrator app (a client) tells the
 *             voxl-imu-server to go into calibration mode by sending the
 *             "START_CALIBRATION" string into a control pipe.
 *
 *             SERVER_FLAG_EN_DEBUG_PRINTS
 *
 *             Enables extra debug info to be printed for debugging of the pipe
 *             interface.
 *
 * @param[in]  ch     channel to init
 * @param[in]  info   information describing the pipe
 * @param[in]  flags  extra feature flags
 *
 * @return     0 on success, -1 on failure
 */
int pipe_server_create(int ch, pipe_info_t info, int flags);

/**
 * @brief      Checks all the channels and returns the first channel number not in use
 *
 * @return     channel number on success, -1 if no channels are available.
 */
int pipe_server_get_next_available_channel(void);

/**
 * @brief      Fetches a pointer to the cJSON representation of the info JSON
 *             file.
 *
 *             Use this if you wish to add your own data to the info file, such
 *             as camera lens calibration data or similar.
 *
 *             first get a pointer to the json info with
 *             pipe_server_get_json_info_ptr, update it with your extra
 *             information, then call this to write to disk.
 *
 * @param[in]  ch    channel to set the info for
 *
 * @return     valid pointer on success, NULL on failure
 */
cJSON* pipe_server_get_info_json_ptr(int ch);


/**
 * @brief      rewrites the info json data to the info file
 *
 *             first get a pointer to the json info with
 *             pipe_server_get_json_info_ptr, update it with your extra
 *             information, then call this to write to disk.
 *
 * @param[in]  ch    channel to set the info for
 *
 * @return     0 on success, -1 on failure.
 */
int pipe_server_update_info(int ch);


/**
 * @brief      closes and deinitializes a single pipe
 *
 *             This removes the named pipes from the file system so clients are
 *             informed the server has closed. If a client want to reconnect to
 *             the server after this they must send a new request.
 *
 * @param[in]  ch    channel to close
 */
void pipe_server_close(int ch);


/**
 * @brief      closes and deinitializes all pipes
 *
 *             This removes the named pipes from the file system so clients are
 *             informed the server has closed. If a client want to reconnect to
 *             the server after this they must send a new request.
 */
void pipe_server_close_all(void);


////////////////////////////////////////////////////////////////////////////////
// Functions to write to all clients subcribed to a pipe
// Generally we publish to all clients on a channel. There is a per-client write
// function in the client-specific functions section.
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief      send data to all clients in one channel
 *
 * @param[in]  ch     channel to send to
 * @param[in]  data   pointer to data to send
 * @param[in]  bytes  number of byes to send
 *
 * @return     0 on success, -1 on failure
 */
int pipe_server_write(int ch, const void* data, int bytes);


/**
 * @brief      send data from multiple buffers to all clients in one channel
 *
 *             This is meant to reduce waste from memcopying data from multiple
 *             sources into contiguous memory before writing. For example when
 *             writing a list of time synced images or YUV images from a source
 *             with a gap between Y and UV data.
 *
 *             This will not write any data in the list unless there is space in
 *             the pipe for all data. That logic is done per-client.
 *
 * @param[in]  ch    channel to send to
 * @param[in]  nbuf  number of buffers to read from
 * @param      bufs  array of buffers
 * @param      lens  array of number of bytes to read from each buffer
 *
 * @return     0 on success, -1 on failure
 */
int pipe_server_write_list(int ch, int nbuf, const void** bufs, const size_t* lens);


/**
 * @brief      send camera metadata and frame data to all clients in one channel
 *
 *             This ensures each client gets exactly one metadata and one camera
 *             frame. The number of bytes read from the data pointer is pulled
 *             from the metadata struct.
 *
 *             Note that this can also be used to send stereo frames as long as
 *             the left and right images are in consecutive memory. This is
 *             possible because the size_bytes field in the
 *             camera_image_metata_t struct indicates the total size of both
 *             left and right camera frames combined. Only if the left and right
 *             frames are in separate non-consecutive buffers should you use
 *             pipe_server_send_stereo_frame_to_channel().
 *
 *             This also sets the magic number field in the metadata struct
 *             before sending so the user doesn't have to.
 *
 * @param[in]  ch    channel to send to
 * @param[in]  meta  The camera metadata
 * @param[in]  data  pointer to data to send
 *
 * @return     0 on success, -1 on failure
 */
int pipe_server_write_camera_frame(int ch, camera_image_metadata_t meta, const void* data);


/**
 * @brief      send camera metadata and stereo frame data to all clients in one
 *             channel
 *
 *             This ensures each client gets exactly one metadata and one camera
 *             frame. The number of bytes read from the data pointer is pulled
 *             from the metadata struct.
 *
 *             This is only to be used when the left and right stereo frames are
 *             in separate non-sequential memory buffers. If the left and right
 *             frames are in one continuous memory buffer than you can just use
 *             the normal pipe_server_send_camera_frame_to_channel() function by
 *             providing a pointer to the beginning of the buffer. That will
 *             assume the left frame precedes the right frame and will execute
 *             one write to the pipe instead of two. This is possible because
 *             the size_bytes field in the camera_image_metata_t struct
 *             indicates the total size of both left and right camera frames
 *             combined.
 *
 *             Note that when constructing the metadata struct, the size_bytes
 *             field should be the size of both left and right frames combined.
 *             size_bytes/2 will be read from each pointer and sent to the pipe.
 *
 *             This also sets the magic number field in the metadata struct
 *             before sending so the user doesn't have to.
 *
 * @param[in]  ch     channel to send to
 * @param[in]  meta   The camera metadata
 * @param      left   pointer to beginning of left frame
 * @param      right  pointer to beginning of right frame
 *
 * @return     0 on success, -1 on failure
 */
int pipe_server_write_stereo_frame(int ch, camera_image_metadata_t meta, const void* left, const void* right);


/**
 * @brief      send point cloud metadata and point data to all clients in one
 *             channel
 *
 *             This ensures each client gets exactly one metadata and one point
 *             cloud. The number of bytes read from the data pointer is
 *             3*sizeof(float)*meta.n_points
 *
 *             This also sets the magic number field in the metadata struct
 *             before sending so the user doesn't have to.
 *
 * @param[in]  ch    channel to send to
 * @param[in]  meta  The point cloud metadata
 * @param[in]  data  pointer to data to send
 *
 * @return     0 on success, -1 on failure
 */
int pipe_server_write_point_cloud(int ch, point_cloud_metadata_t meta, const void* data);


#ifdef EN_ION_BUF
/**
 * @brief      send ion buffer to all clients in one channel
 *
 *             This ensures each client gets its own fd to the ion buffer,
 *             the fd is comminicated to client through UNIX sockets 
 *
 *             This also sets the magic number field in the metadata struct
 *             before sending so the user doesn't have to.
 *
 * @param[in]  ch         channel to send to
 * @param[in]  ion_pool   The pool that the ion buffer is a part of
 * @param[in]  ion_buf    The ion buffer struct with relevant meta data
 *
 * @return     0 on success, -1 on failure
 */
int pipe_server_write_ion_buffer(int ch, mpa_ion_buf_pool_t* pool, mpa_ion_buf_t* ion_buf);
#endif

/**
 * @brief      send a string to a pipe
 *
 *             This could be done with the base pipe_server_write() function,
 *             but this saves the user from needing to call strlen.
 *
 *             This does send the null terminator at the end of the string so
 *             that strings can be differentiated at the other end of the pipe.
 *             It sends strlen()+1 bytes
 *
 * @param[in]  ch     channel to send to
 * @param[in]  string data to write
 *
 * @return     0 on success, -1 on failure
 */
int pipe_server_write_string(int ch, const char* string);


/**
 * @brief      fetch the number of clients currently connected to a particular
 *             channel. Use this to check if its worth spending CPU resources
 *             computing data for a pipe that has no clients to receive the
 *             data.
 *
 *             This includes clients that have sent a request but have not yet
 *             received their first packet.
 *
 * @param[in]  ch    channel to check
 *
 * @return     The total number of connected clients
 */
int pipe_server_get_num_clients(int ch);




////////////////////////////////////////////////////////////////////////////////
// Functions to set optional config such as callbacks and control pipe size
////////////////////////////////////////////////////////////////////////////////


/**
 * @brief      Update the control pipe size and read buffer from default.
 *
 *             Call this BEFORE creating a new pipe with pipe_server_create().
 *             You can't update or change these values after creating a pipe.
 *
 *             The default control pipe size is 64k and read buf size is 1k
 *             which should be more than sufficient for basic text control pipe
 *             commands. You should increase these values for high data rate
 *             applications such as receiving mavlink data through a control
 *             pipe.
 *
 *             For receiving fixed-length structs over a control pipe, make sure
 *             to make the read buf size a multiple of the struct size so that
 *             when the pipes fills up, the data is read out without splitting
 *             packets. You can also use the "pipe_validate_x" function in
 *             modal_pipe_interfaces.h
 *
 *             For example, for receiving mavlink messages over a control pipe,
 *             we suggest the same values from modal_pipe_interfaces.h:
 *
 *             MAVLINK_MESSAGE_T_RECOMMENDED_PIPE_SIZE
 *             MAVLINK_MESSAGE_T_RECOMMENDED_READ_BUF_SIZE
 *
 * @param[in]  ch             channel to set values for
 * @param[in]  pipe_size      The pipe size
 * @param[in]  read_buf_size  The read buffer size
 *
 * @return     0 on success, -1 on failure
 */
int pipe_server_set_control_pipe_size(int ch, int pipe_size, int read_buf_size);


/**
 * @brief      Set what priority the thread servigint he control pipe will have
 *             once it is started.
 *
 *             This is intended for use where the control pipe receives critical
 *             time-sensitive data such as in voxl-mavlink-server.
 *
 *             Must be called before pipe_server_create(). If you want to change
 *             helper thread priorities after it has been started then use
 *             pipe_pthread_set_priority() from within the helper thread.
 *
 *             See THREAD_PRIORITY_* definitions in modal_start_stop.h, setting
 *             priority to 0 indicates the thread should use Linux pthread
 *             default scheduler and priority. Otherwise 1-99 sets the thread to
 *             use the Real-Time FIFO scheduler with specified priority.
 *
 *             This is an optional function call, the default behavior is to
 *             start the control handler pthread with inherited properties from
 *             the calling process.
 *
 * @param[in]  ch        Channel to assign the thread
 * @param[in]  priority  The priority 0-99
 *
 * @return     0 on success, -1 on failure
 */
int pipe_server_set_control_thread_priority(int ch, int priority);


/**
 * @brief      assign a callback function to be called when a message is
 *             received on a control pipe
 *
 *             The user-defined callback should return void and take 3 arguments
 *
 *             - The channel the control message was received on. Although
 *               callback functions can be assigned individually for each
 *               channel, this allows a single callback to be assigned for all
 *               channels and still be able to differentiate which channel the
 *               message was received on.
 *
 *             - string as read from the control pipe
 *
 *             - number of bytes read fom the control pipe
 *
 * @param[in]  ch    channel to assign the callback to
 * @param[in]  func  The function pointer
 *
 * @return     0 on success, -1 on failure
 */
typedef void server_control_cb(int ch, char* string, int bytes, void* context);
int pipe_server_set_control_cb(int ch, server_control_cb* cb, void* context);


/**
 * @brief      Assign a callback function to be called when a new client is
 *             added automatically as a result of a request coming in on the
 *             request pipe.
 *
 *             This is entirely optional since a new data pipe will be created
 *             automatically before this callback is executed. This exists so
 *             that the server can be aware when new clients connect and
 *             initialize hardware as necessary.
 *
 *             The user-defined callback should return void and take 4 arguments
 *
 *             - The channel the request message was received on. Although
 *               callback functions can be assigned individually for each
 *               channel, this allows a single callback to be assigned for all
 *               channels and still be able to differentiate which channel the
 *               message was received on.
 *
 *             - string as read from the request pipe, typically the name of the
 *               client
 *
 *             - number of bytes read from the request pipe
 *
 *             - client id of the newly added client
 *
 * @param[in]  ch    channel to assign the callback to
 * @param[in]  func  The function pointer
 *
 * @return     0 on success, -1 on failure
 */
typedef void server_connect_cb(int ch, int client_id, char* name, void* context);
int pipe_server_set_connect_cb(int ch, server_connect_cb* cb, void* context);


/**
 * @brief      Assign a callback function to be called when a new client
 *             disconnects
 *
 *             This is entirely optional and only present if there server wishes
 *             to be aware of when client disconnect. Note that the server
 *             interface won't be aware that a client has disconnected until the
 *             server tries to write to it.
 *
 *             The user-defined callback should return void and take 3 arguments
 *
 *             - The channel the client disconnected from. Although callback
 *               functions can be assigned individually for each channel, this
 *               allows a single callback to be assigned for all channels and
 *               still be able to differentiate which channel the message was
 *               received on.
 *
 *             - client id that disconnected
 *
 *             - name of the client
 *
 * @param[in]  ch    channel to assign the callback to
 * @param[in]  func  The function pointer
 *
 * @return     0 on success, -1 on failure
 */
typedef void server_disconnect_cb(int ch, int client_id, char* name, void* context);
int pipe_server_set_disconnect_cb(int ch, server_disconnect_cb* cb, void* context);




////////////////////////////////////////////////////////////////////////////////
// Functions to interact with clients individually
//
// It is highly unlikely you will use any of these functions. They exist only to
// support hypothetical future special use cases, not general use!
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief      Manually add a new client and named pipe for sending data out.
 *
 *             This is usually called automatically when a new request
 *             comes in so the server doesn't have to do this themselves. This
 *             function is make available if the server needs to create a
 *             default data pipe that is open without the need for a client
 *             requesting it.
 *
 * @param[in]  ch    channel to add a dedicated pipe to
 * @param      name  The name of the pipe
 *
 * @return     ID number of newly added client, or -1 on failure.
 */
int pipe_server_add_client(int ch, const char* name);


/**
 * @brief      Fetch the number of bytes already in the pipe waiting to be read
 *             by the client.
 *
 *
 *             This is useful to check to see if a client is falling behind and
 *             needs to dump data to catch up. E.g. when doing heavy processing
 *             on camera frames.
 *
 *             There is an equivalent function pipe_client_bytes_in_pipe() to
 *             allow clients to check this information too.
 *
 * @param[in]  ch         channel to check
 * @param[in]  client_id  The client id
 *
 * @return     # bytes available to read or -1 on error
 */
int pipe_server_bytes_in_pipe(int ch, int client_id);


/**
 * @brief      Fetch the current size (capacity) of the pipe for a given channel
 *             and client.
 *
 *             You probably don't need to use this function! Set the default
 *             pipe size in the pipe_info_t struct when creating the pipe in the
 *             first place. This is only if you absolutely want to check to see
 *             what the pipe size for a specfic client is after they have
 *             subscribed to see if it changed.
 *
 * @param[in]  ch         channel to check
 * @param[in]  client_id  The client id
 *
 * @return     size of pipe in bytes, or -1 on error
 */
int pipe_server_get_pipe_size(int ch, int client_id);


/**
 * @brief      Set the size of a pipe for a particular client.
 *
 *             You probably don't need to use this function! Set the default
 *             pipe size in the pipe_info_t struct when creating the pipe in the
 *             first place. This is only if you absolutely want to dynamically
 *             change the pipe size of a specific client after they subscribe.
 *
 *             A client can (and should!) increase the pipe size in cases where
 *             they need more buffering. Servers may also increase the pipe size
 *             if they see fit. Generally servers should do this through the
 *             pipe_server_set_default_pipe_size(int ch) function instead.
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
 * @param[in]  client_id   The client id
 * @param[in]  size_bytes  The desired pipe size in bytes
 *
 * @return     new size of the pipe or -1 on error.
 */
int pipe_server_set_pipe_size(int ch, int client_id, int size_bytes);

/**
 * @brief      adds a list of available commands to the info cJSON that the users
 *             can see 
 *
 *             must be called after client is initialized
 *
 *
 * @param[in]  ch         channel to send to
 * @param[in]  commands   comma-separated list of available commands
 *
 * @return     0 on success, -1 on failure
 */

int pipe_server_set_available_control_commands(int ch, const char* commands);

/**
 * @brief      send data to just one specified client instead of to every client
 *             connected to the pipe
 *
 * @param[in]  ch         channel to send to
 * @param[in]  client_id  client to send to
 * @param[in]  data       pointer to data to send
 * @param[in]  bytes      number of byes to send
 *
 * @return     0 on success, -1 on failure
 */
int pipe_server_write_to_client(int ch, int client_id, const void* data, int bytes);


/**
 * @brief      check the state of a particular client
 *
 * @param[in]  ch         The channel to check
 * @param[in]  client_id  The client id to check
 *
 * @return     The pipe client state, see CLIENT_CONNECTED etc
 */
int pipe_server_get_client_state(int ch, int client_id);


/**
 * @brief      get the client id associated with a certain name
 *
 * @param[in]  ch    channel to check in
 * @param      name  The desired name
 *
 * @return     Returns the associate client id if it exists, otherwise -1
 */
int pipe_server_get_client_id_from_name(int ch, char* name);


/**
 * @brief      get the name associated with a particular client is
 *
 * @param[in]  ch         channel to check in
 * @param      client_id  The desired client id
 *
 * @return     Returns the associate name as a string if it exists, otherwise
 *             NULL
 */
char* pipe_server_get_client_name_from_id(int ch, int client_id);


/**
 * helper to suggest what pipe size to use when publishing image streams
 *
 * This is used by voxl-camera-server, voxl-uvc-server, voxl-replay, and other
 * services to figure out an appropriate pipe size. Camera server does not need
 * to allocate the same system memory for publishing VGA greyscale frames as it
 * does 4K color images.
 *
 * For stereo format this account for doubling of the image data, just provide
 * the width and height of a single image.
 *
 * Linux pipe sizes are always a power of two, this function rounds up to the
 * nearest power of two. Note this is not necessary, you can request a pipe size
 * that's not a power of two and the operating system will round up for you.
 *
 * @param[in]  format  see camera_image_metadata_t
 * @param[in]  width   The width
 * @param[in]  height  The height
 *
 * @return     recommended pipe size in bytes
 */
int pipe_suggest_cam_pipe_size(int16_t format, int width, int height);


#ifdef __cplusplus
}
#endif

#endif // MODAL_PIPE_SERVER_H
