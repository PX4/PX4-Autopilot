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


#ifndef MODAL_PIPE_DEPRECATED
#define MODAL_PIPE_DEPRECATED

#ifdef __cplusplus
extern "C" {
#endif

// #include <modal_pipe_common.h>
#include <modal_pipe_interfaces.h>


// old #defines, no reason to remove them
#define EN_PIPE_CLIENT_SIMPLE_HELPER		(1<<0) // must provide a buffer length on init
#define EN_PIPE_CLIENT_CAMERA_HELPER		(1<<1) // no need to choose a buffer length
#define EN_PIPE_CLIENT_POINT_CLOUD_HELPER	(1<<2) // must provide a buffer length on init
#define EN_PIPE_CLIENT_DEBUG_PRINTS			(1<<3) // turn on debug prints
#define EN_PIPE_CLIENT_AUTO_RECONNECT		(1<<4) // DEPRECATED, this is on by default now!
#define SERVER_FLAG_EN_INFO_PIPE			(1<<2) // Deprecated! info is always on now
#define EN_PIPE_SINK_SIMPLE_HELPER			(1<<0)
#define EN_PIPE_SINK_DEBUG_PRINTS			(1<<1)

/**
 * DEPRECATED!! please use pipe_server_create_pipe() instead!
 *
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
 *             SERVER_FLAG_EN_INFO_PIPE
 *
 *             Enables an extra "info" pipe where client can read static
 *             information. Set this information string with
 *             pipe_server_set_info_string().
 *
 * @param[in]  ch     channel to init (0-15)
 * @param[in]  topic  short topic name or complete path
 * @param[in]  flags  extra feature flags
 *
 * @return     0 on success, -1 on failure
 */
__attribute__((deprecated("\nPlease use pipe_server_create instead")))
int pipe_server_init_channel(int ch, const char* topic, int flags);


/**
 * DEPRECATED Please use pipe_server_close_all() instead
 *
 * @brief      closes and deinitializes all channels
 *
 *             This removes the named pipes from the file system so clients are
 *             informed the server has closed. If a client want to reconnect to
 *             the server after this they must send a new request.
 */
__attribute__((deprecated("\nPlease use pipe_server_close_all() instead")))
void pipe_server_close_all_channels(void);


/**
 * DEPRECATED, info files now contain JSON, use pipe_server_update_info_json() instead
 *
 * @brief      Set the string that will be made available by the info pipe.
 *
 *             Note: this string is meant for static data and should only be
 *             set once, it is possible to change after startup but you may
 *             encounter unexpected behavior
 *
 * @param[in]  ch        channel to set the string for.
 *
 * @param[in]  string    the string that will be saved to the channel
 *
 * @return     0 on success, -1 on failure.
 */
__attribute__((deprecated("\nPlease use pipe_server_update_info_json instead")))
int pipe_server_set_info_string(int ch, const char* string);


/**
 *DEPRECATED, please set the pipe size in the pipe_info_t struct when creating the pipe instead.
 *
 * @brief      set the default pipe size that will be created when a new client
 *             connects to the specified channel.
 *
 *             The linux kernel makes pipes 64k long by default which is too
 *             small for high-data applications such as sending cameras frames.
 *             Servers which send large data packets should create pipes for
 *             clients with a more reasonable size settable by this function.
 *             Pipes can be resized later on a per-client basis with
 *             pipe_server_set_pipe_size(int ch, int client_id);
 *
 *             Pipes usually default to 64k depending on your platform. This may
 *             be too small for some applications, particularly camera frames.
 *
 *             There is an equivalent function pipe_client_get_pipe_size() to
 *             allow clients to check this information too.
 *
 *             Usually the kernel will "round up" to the next exponent of 2 so
 *             you may get a pipe size that's larger than requested. If you
 *             request a size that's too large and the kernel rejects the
 *             request, then the pipe size will remain the same. The linux
 *             kernel seems to set a limit at 256MiB but you shouldn't need a
 *             pipe that big. 4MiB is reasonable for sending VGA image streams,
 *             16MiB is reasonable for sending 4k images.
 *
 *             Since the kernel will round-up anyway, you might as well just
 *             request a nice even size, e.g. 1,2,4,8,16,32,64,128, or 256Mib.
 *
 *             e.g. to set the pipe size to 16MiB use size_bytes = 16*1024*1024;
 *
 *             Set size_bytes=0 to disable setting the pipe size and let the
 *             kernel make pipes with the kernel default (usually 64k).
 *
 * @param[in]  ch    channel to set the default for.
 *
 * @return     0 on success, -1 on failure. Note that this function will succeed
 *             in setting the default to an unreasonable value, but this doens't
 *             mean the kernel will respect that number and let the pipes be
 *             resized as expected.
 */
__attribute__((deprecated("\nplease set the pipe size in the pipe_info_t struct when creating the pipe instead")))
int pipe_server_set_default_pipe_size(int ch, int size_bytes);


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
 * @param[in]  ch    channel to assign the callback to (0-15)
 * @param[in]  func  The function pointer
 *
 * @return     0 on success, -1 on failure
 */
typedef void server_request_cb(int ch, char* string, int bytes, int client_id, void* context);
__attribute__((deprecated("\nplease use pipe_server_set_connect_cb() instead")))
int pipe_server_set_request_cb(int ch, server_request_cb* cb, void* context);


/**
 * @brief      send data to all clients in one channel
 *
 * @param[in]  ch     channel to send to
 * @param[in]  data   pointer to data to send
 * @param[in]  bytes  number of byes to send
 *
 * @return     0 on success, -1 on failure
 */
__attribute__((deprecated("\nplease use pipe_server_write() instead")))
int pipe_server_send_to_channel(int ch, char* data, int bytes);


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
__attribute__((deprecated("\nplease use pipe_server_write_camera_frame() instead")))
int pipe_server_send_camera_frame_to_channel(int ch, camera_image_metadata_t meta, char* data);


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
__attribute__((deprecated("\nplease use pipe_server_write_stereo_frame() instead")))
int pipe_server_send_stereo_frame_to_channel(int ch, camera_image_metadata_t meta, char* left, char* right);


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
__attribute__((deprecated("\nplease use pipe_server_write_point_cloud() instead")))
int pipe_server_send_point_cloud_to_channel(int ch, point_cloud_metadata_t meta, float* data);



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
 *             Up to 16 channels can be opened to receive data from the same or
 *             multiple servers. This channel number has nothing to do with the
 *             server channel. It is only a way to index and differentiate
 *             between multiple open pipes.
 *
 * @param[in]  ch                channel to initialize (0-15)
 * @param      name_or_location  The name or location
 * @param      name              desired name of the dedicated pipe to be
 *                               created
 * @param[in]  flags             flags to configure optional features
 * @param[in]  buf_len           The buffer size (bytes) used by the read helper
 *                               thread
 * @param      topic  base directory for the channel you wish to read from
 *
 * @return     0 on success, -1 on failure
 */
__attribute__((deprecated("\nplease use pipe_client_open() instead")))
int pipe_client_init_channel(int ch, char* name_or_location, const char* client_name, int flags, int buf_len);

/**
 * @brief      print a human readable error returned by pipe_client_init_channel
 *
 *             pipe_client_init_channel fails silently so that clients can keep
 *             trying to init waiting for a server to come online or for a
 *             channel to free up without cluttering up the screen with error
 *             messages. If the user wants to print which error occured anyway
 *             then they can use this function. See the example
 *             modal-hello-client
 *
 * @param[in]  e     error returned from pipe_client_init_channel
 */
__attribute__((deprecated("\nplease use pipe_print_error() instead")))
void pipe_client_print_error(int e);


/**
 * @brief      take a partial (or full) pipe directory string, and write out the
 *             full and correct version
 *
 *             pipe_client_init_channel expects a full directory path ending in
 *             '/' containing the pipe channel it should subscribe to. For
 *             example, the path to the imu0 channel the voxl-imu-server
 *             publishes is: /run/mpa/imu0/
 *
 *             However, a user may just want to provide the short name "imu0" or
 *             "imu0/" as a command line argument to a program such as
 *             voxl-test-imu. In this case, it's up to voxl-test-imu to parse
 *             that command line argument and construct the full path. A user
 *             might also want to specify a full path to somewhere other than
 *             /run/mpa/. This helper function provides this function and can be
 *             used while parsing arguments as follows:
 *
 *             char pipe_path[128] pipe_client_contruct_full_path(optarg,
 *             pipe_path);
 *
 *             Examples of input > output behavior
 *
 *             imu0     > /run/mpa/imu0/ imu0/    > /run/mpa/imu0/ /foo/bar >
 *             /foo/bar/ /foo     > /foo/
 *
 *             This does not guarantee the path exists, it only formats the
 *             string. Try to init the channel with pipe_client_init_channel to
 *             see if it's active. This function may fail if the string pointers
 *             are NULL or an empty string is provided
 *
 * @param      in    input string
 * @param      out   pointer to pre-allocated memory to write the result to
 *
 * @return     0 on success, -1 on failure
 */
__attribute__((deprecated("\nplease use pipe_expand_location_string() instead")))
int pipe_client_construct_full_path(char* in, char* out);



/**
 * @brief      Requests up to buf_len bytes from the server's info pipe
 *             and stores it in buf.
 *
 * @param[in]  ch       channel to recieve from
 * @param      buf      The buffer
 * @param[in]  buf_len  The buffer length
 *
 * @return     number of bytes read or -1 on failure
 */
__attribute__((deprecated("\nplease use pipe_get_info_struct() or pipe_get_info_json() instead")))
int pipe_client_get_info_string(int ch, char* buf, int buf_len);


/**
 * @brief      closes and deinitializes a single pipe connection
 *
 *             If a read helper thread is enabled, this functions signals the
 *             thread to close. This also closes any open file descriptors
 *             including the data fd even if the client grabbed it with the
 *             pipe_client_get_fd() function.
 *
 * @param[in]  ch    channel to close (0-15)
 */
__attribute__((deprecated("\nplease use pipe_client_close() instead")))
void pipe_client_close_channel(int ch);


/**
 * @brief      closes and deinitializes a single pipe
 *
 *             This removes the named pipes from the file system so clients are
 *             informed the server has closed. If a client want to reconnect to
 *             the server after this they must send a new request.
 *
 * @param[in]  ch    channel to close
 */
__attribute__((deprecated("\nplease use pipe_server_close() instead")))
void pipe_server_close_channel(int ch);


/**
 * @brief      send data to just one specified client in one channel
 *
 * @param[in]  ch         channel to send to
 * @param[in]  client_id  client to send to
 * @param[in]  data       pointer to data to send
 * @param[in]  bytes      number of byes to send
 *
 * @return     0 on success, -1 on failure
 */
__attribute__((deprecated("\nplease use pipe_server_write_to_client() instead")))
int pipe_server_send_to_client(int ch, int client_id, char* data, int bytes);



/**
 * @brief      Use this to simultaneously validate that the data from a pipe
 *             contains valid imu data, find the number of valid packets
 *             contained in a single read from the pipe, and cast the raw data
 *             buffer as an imu_data_t* for easy access.
 *
 *             This does NOT copy any data and the user does not need to
 *             allocate an imu_data_t array separate from the pipe read buffer.
 *             The data can be read straight out of the pipe read buffer, much
 *             like reading data directly out of a mavlink_message_t message.
 *
 *             However, this does mean the user should finish processing this
 *             data before returning the pipe data callback which triggers a new
 *             read() from the pipe.
 *
 * @param[in]  data       pointer to pipe read data buffer
 * @param[in]  bytes      number of bytes read into that buffer
 * @param[out] n_packets  number of valid imu_data_t packets received
 *
 * @return     Returns the same data pointer provided by the first argument, but
 *             cast to an imu_data_t* struct for convenience. If there was an
 *             error then NULL is returned and n_packets is set to 0
 */
__attribute__((deprecated("\nPlease use pipe_validate_imu_data_t() instead")))
imu_data_t* modal_imu_validate_pipe_data(char* data, int bytes, int* n_packets);


/**
 * @brief      Use this to simultaneously validate that the bytes from a pipe
 *             contains valid data, find the number of valid packets
 *             contained in a single read from the pipe, and cast the raw data
 *             buffer as a pose_4dof_t* for easy access.
 *
 *             This does NOT copy any data and the user does not need to
 *             allocate a pose_4dof_t array separate from the pipe read buffer.
 *             The data can be read straight out of the pipe read buffer, much
 *             like reading data directly out of a mavlink_message_t message.
 *
 *             However, this does mean the user should finish processing this
 *             data before returning the pipe data callback which triggers a new
 *             read() from the pipe.
 *
 * @param[in]  data       pointer to pipe read data buffer
 * @param[in]  bytes      number of bytes read into that buffer
 * @param[out] n_packets  number of valid packets received
 *
 * @return     Returns the same data pointer provided by the first argument, but
 *             cast to an pose_4dof_t* struct for convenience. If there was an
 *             error then NULL is returned and n_packets is set to 0
 */
__attribute__((deprecated("\nPlease use pipe_validate_pose_4dof_t() instead")))
pose_4dof_t* modal_pose_4dof_validate_pipe_data(char* data, int bytes, int* n_packets);



/**
 * @brief      Use this to simultaneously validate that the bytes from a pipe
 *             contains valid data, find the number of valid packets
 *             contained in a single read from the pipe, and cast the raw data
 *             buffer as a pose_vel_6dof_t* for easy access.
 *
 *             This does NOT copy any data and the user does not need to
 *             allocate a pose_vel_6dof_t array separate from the pipe read buffer.
 *             The data can be read straight out of the pipe read buffer, much
 *             like reading data directly out of a mavlink_message_t message.
 *
 *             However, this does mean the user should finish processing this
 *             data before returning the pipe data callback which triggers a new
 *             read() from the pipe.
 *
 * @param[in]  data       pointer to pipe read data buffer
 * @param[in]  bytes      number of bytes read into that buffer
 * @param[out] n_packets  number of valid packets received
 *
 * @return     Returns the same data pointer provided by the first argument, but
 *             cast to an pose_vel_6dof_t* struct for convenience. If there was an
 *             error then NULL is returned and n_packets is set to 0
 */
__attribute__((deprecated("\nPlease use pipe_validate_pose_vel_6dof_t() instead")))
pose_vel_6dof_t* modal_pose_vel_6dof_validate_pipe_data(char* data, int bytes, int* n_packets);



/**
 * @brief      Use this to simultaneously validate that the bytes from a pipe
 *             contains valid data, find the number of valid packets
 *             contained in a single read from the pipe, and cast the raw data
 *             buffer as a vio_data_t* for easy access.
 *
 *             This does NOT copy any data and the user does not need to
 *             allocate a vio_data_t array separate from the pipe read buffer.
 *             The data can be read straight out of the pipe read buffer, much
 *             like reading data directly out of a mavlink_message_t message.
 *
 *             However, this does mean the user should finish processing this
 *             data before returning the pipe data callback which triggers a new
 *             read() from the pipe.
 *
 * @param[in]  data       pointer to pipe read data buffer
 * @param[in]  bytes      number of bytes read into that buffer
 * @param[out] n_packets  number of valid packets received
 *
 * @return     Returns the same data pointer provided by the first argument, but
 *             cast to an vio_data_t* struct for convenience. If there was an
 *             error then NULL is returned and n_packets is set to 0
 */
__attribute__((deprecated("\nPlease use pipe_validate_vio_data_t() instead")))
vio_data_t* modal_vio_validate_pipe_data(char* data, int bytes, int* n_packets);

__attribute__((deprecated("\nPlease use pipe_print_vio_state() instead")))
void modal_vio_print_state(int s);

__attribute__((deprecated("\nPlease use pipe_print_vio_error() instead")))
void modal_vio_print_error_code(int e);

/**
 * @brief      convert an image format id number to string
 *
 * For example IMAGE_FORMAT_RAW8 will return the string "RAW8"
 *
 * @param[in]  i     image format id, e.g. IMAGE_FORMAT_RAW8
 *
 * @return     const char8 string of the format
 */
__attribute__((deprecated("\nPlease use pipe_image_format_to_string() instead")))
const char* modal_image_format_name(int i);


/**
 * @brief      Creates a new named pipe at the specified path to act as a sink
 *             for other processes to send data into.
 *
 *             It is opened for blocking reads and its file descriptor is saved
 *             for reading by the user either directly or with the helper
 *             thread.
 *
 *             The easiest way to get data is to enable the helper read thread
 *             by setting the EN_PIPE_SINK_SIMPLE_HELPER bit in the flags
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
 * @param[in]  ch       channel to initialize
 * @param      path     path for the pipe to be created
 * @param[in]  flags    flags to configure optional features
 * @param[in]  buf_len  The buffer size (bytes) used by the read helper thread
 *
 * @return     0 on success, -1 on failure
 */
__attribute__((deprecated("\nPlease use pipe_sink_create() instead")))
int pipe_sink_init_channel(int ch, char* path, int flags, int buf_len);


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
__attribute__((deprecated("\nPlease use pipe_sink_close() instead")))
void pipe_sink_close_channel(int ch);


#ifdef __cplusplus
}
#endif

#endif // MODAL_PIPE_DEPRECATED
