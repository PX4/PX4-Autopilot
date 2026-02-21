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


#ifndef MODAL_PIPE_COMMON_H
#define MODAL_PIPE_COMMON_H

#include <cJSON.h>
#include <modal_pipe_defines.h>

/**
 * information describing a pipe. Servers use this to create a new pipe in the
 * file system. The data is available in json format in the info file, e.g.
 * /run/mpa/imu0/info for the client to read back along with any other json data
 * the server elects to put in there such as camera lens calibration.
 */
typedef struct pipe_info_t{
    char name[MODAL_PIPE_MAX_NAME_LEN];         ///< short name, e.g. "imu0"
    char location[MODAL_PIPE_MAX_DIR_LEN];      ///< this is the full pipe location, e.g. /run/mpa/imu0/
    char type[MODAL_PIPE_MAX_TYPE_LEN];         ///< examples: imu_data_t, mavlink_message_t, camera, point cloud
    char server_name[MODAL_PIPE_MAX_NAME_LEN];  ///< name of the server that created the pipe, e.g. voxl-imu-server
    int size_bytes;                             ///< FIFO size that server will create, when client subscribes, can be updated by client
    int server_pid;                             ///< process ID of the server that created the pipe
} pipe_info_t;

#define PIPE_INFO_INITIALIZER {\
    .name        = "unknown",\
    .location    = "unknown",\
    .type        = "unknown",\
    .server_name = "unknown",\
    .size_bytes  = MODAL_PIPE_DEFAULT_PIPE_SIZE,\
    .server_pid  = 0}


/**
 * @brief      print a human readable pipe error number
 *
 *             usually this is to check the return value of pipe_client_open()
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
void pipe_print_error(int e);



/**
 * @brief      Check if a pipe name or full location exists and can be opened.
 *
 *             This function can take either a pipe name (e.g. "imu0") or a
 *             complete location of a pipe (e.g. /run/mpa/imu0). If a pipe name
 *             is given, it will be assumed to live in /run/mpa/
 *
 *             Under the hood, this looks to see if the pipe location contains a
 *             request FIFO indicating the server that created the pipe is
 *             active and can receive requests to open the pipe by a client.
 *
 * @param      name_or_location  The name or location
 *
 * @return     1 if pipe exists, 0 otherwise.
 */
int pipe_exists(const char* name_or_location);



/**
 * @brief      Check if a pipe name or full location is of desired type.
 *
 *             We HIGHLY recommend checking if a pipe is of the correct type
 *             before opening it as your code will likely not know what to do
 *             with an unexpected data stream.
 *
 *             This function can take either a pipe name (e.g. "imu0") or a
 *             complete location of a pipe (e.g. /run/mpa/imu0). If a pipe name
 *             is given, it will be assumed to live in /run/mpa/
 *
 *             Note that type strings are limited to MODAL_PIPE_MAX_TYPE_LEN
 *             characters.
 *
 *             Note that this function returns 0 if a pipe doesn't exist OR if
 *             it is of the wrong type. If you wish to explicitly check if a
 *             pipe exists AND is of the desired type, you should use the
 *             pipe_exists() function first.
 *
 * @param      name_or_location  The name or location string of the desired pipe
 * @param      desired_type      The desired type
 *
 * @return     1 if the pipe type matches the desired type. 0 if the pipe is of
 *             a different type or doesn't exist.
 */
int pipe_is_type(const char* name_or_location, const char* desired_type);



/**
 * @brief      reads the JSON info file from a given pipe name into a
 *             pipe_info_t struct
 *
 *             If the name or location is valid and has an info file, the data
 *             will be parsed and the user's info struct will be populated with
 *             the info data read at the time.
 *
 *             If you've already opened the pipe you wish to fetch the info for,
 *             you can also use the pipe_client_get_info(int ch, pipe_info_t*
 *             info) function in modal_pipe_client.h
 *
 *             Note that info JSON files may contain extra data beyond what the
 *             pipe_info_t struct contains. Extra data is usually
 *             server-specific and needs to be parsed by the user. See
 *             pipe_get_info_json() for more info on retreiving extra info data.
 *
 *             This function can take either a pipe name (e.g. "imu0") or a
 *             complete location of a pipe (e.g. /run/mpa/imu0). If a pipe name
 *             is given, it will be assumed to live in /run/mpa/
 *
 *             Note that this opens, reads, and closes a file from the file
 *             system. Therefore it should generally be used for one-off or
 *             infrequent checks, don't call this hundreds of times per second.
 *
 * @param      name  The name or location string of the desired pipe
 * @param      info  pointer to user's info struct
 *
 * @return     0 on success, -1 if the info file was not available or some other
 *             error was encountered.
 */
int pipe_get_info(const char* name, pipe_info_t* info);


/**
 * @brief      reads the JSON info file from a given pipe name and parses it
 *             into a cJSON data structure for user parsing.
 *
 *             If the name or location is valid and has an info file, the data
 *             will be parsed and the user's info struct will be populated with
 *             the info data read at the time.
 *
 *             If you've already opened the pipe you wish to fetch the info for,
 *             you can also use the pipe_client_get_info_json(int ch) function
 *             in modal_pipe_client.h
 *
 *             Note that info JSON files may contain extra data beyond what the
 *             pipe_info_t struct contains. This is the function you should use
 *             to retrieve it.
 *
 *             This function can take either a pipe name (e.g. "imu0") or a
 *             complete location of a pipe (e.g. /run/mpa/imu0). If a pipe name
 *             is given, it will be assumed to live in /run/mpa/
 *
 *             Note that this opens, reads, and closes a file from the file
 *             system. Therefore it should generally be used for one-off or
 *             infrequent checks, don't call this hundreds of times per second.
 *
 *             You must remember to clean up the memory allocated for the cJSON
 *             data structure when you are done with it!! Use this:
 *
 *             cJSON_Delete(my_json_ptr);
 *
 * @param      name  The name or location string of the desired pipe
 *
 * @return     a valid pointer on success, NULL if the info file was not
 *             available or some other error was encountered.
 */
cJSON* pipe_get_info_json(const char* name);


/**
 * @brief      Take a pipe name, partial location path, or complete location
 *             path, and write out the full and correct pipe location path
 *             string to the topic directory.
 *
 *             This is not probably not necessary for users to use since
 *             pipe_client_open_pipe() uses this under the hood. You can give
 *             "imu0" to pipe_client_open_pipe and it will automatically expand
 *             the pipe name to the full pipe location /run/mpa/imu0/. However,
 *             if you want to do this expansion for yourself, this function is
 *             made available.
 *
 *             Examples of input > output behavior
 *
 *             - imu0     > /run/mpa/imu0/
 *             - imu0/    > /run/mpa/imu0/
 *             - /foo/bar > /foo/bar/
 *             - /foo     > /foo/
 *
 *             This does not guarantee the path exists, it only formats the
 *             string.
 *
 *             The output pointer should point to a memory location that can
 *             accomodate a string of length MODAL_PIPE_MAX_DIR_LEN
 *
 * @param      in    input string
 * @param      out   pointer to pre-allocated memory to write the result to
 *
 * @return     0 on success, -1 on failure
 */
int pipe_expand_location_string(const char* in, char* out);



/**
 * @brief      This function is used to safely shutdown the server responsible
 *             for creating a pipe directory and clean up any dangling pipes
 *             that may remain.
 *
 *             This is particularly useful for voxl-replay which can stop the
 *             camera and imu servers before publishing its own data to those
 *             same directories.
 *
 *             This will first send SIGINT to simulate ctrl-C and wait for the
 *             specified timeout (default 2 seconds). If the process does not
 *             exit gracefully in the timeout, it will be sent SIGKILL to force
 *             the process to stop with blind and barbaric fury.
 *
 *             If the process does not exit gracefully, it may leave the
 *             specified pipe dangling in the file system. If that were to
 *             occur, this will cleanup the pipe from the file system.
 *
 *             The command line utility modal_kill_pipe exists so you can
 *             quickly use this function without integrating into a custom
 *             program.
 *
 * @param[in]  dir        The pipe directory of which to stop the owner of
 * @param[in]  timeout_s  timeout period to wait for process to close cleanly,
 *                        must be >=0.1, 2.0 seconds is usually good.
 *
 * @return     return values:
 * - -4: invalid argument or other error
 * - -3: insufficient privileges to kill existing process
 * - -2: unreadable or invalid contents in the info pipe
 * - -1: existing process failed to close cleanly and had to be killed
 * - 0: No existing process was running
 * - 1: An existing process was running but it shut down cleanly.
 */
int pipe_kill_server_process(const char* name_or_location, float timeout_s);



/**
 * @brief      Send data to a server through the control pipe without needing to
 *             be connected as a client.
 *
 *             Available valid strings are server-dependent. For example the
 *             "set_pitmode_off" string can be sent to voxl-cpu-monitor via the
 *             cpu_stats2 pipe.
 *
 *             This function is to be used by a client that's not actively
 *             connected to the pipe. To send commands while connected as a
 *             client use pipe_client_send_control_cmd() in modal_pipe_client.h
 *             instead.
 *
 *             Note that creation of a control pipe is optional and determined
 *             by the server. If the server never enabled a control pipe then
 *             this function will return -1.
 *
 * @param[in]  pipe_name    The pipe name
 * @param      control_cmd  The control string
 * @param[in]  ch    channel to send the command to (0-127)
 *
 * @return     0 on success, -1 on failure
 */
int pipe_send_control_cmd(const char* pipe_name, const void* control_cmd);


/**
 * @brief      Send data to a server through the control pipe without needing to
 *             be connected as a client.
 *
 *             The data structure is are server-dependent. This function differs
 *             from pipe_send_control_cmd in that the data does not have to be a
 *             string.
 *
 *             This function is to be used by a client that's not actively
 *             connected to the pipe. To send commands while connected as a
 *             client use pipe_client_send_control_cmd_bytes() in
 *             modal_pipe_client.h instead.
 *
 *             Note that creation of a control pipe is optional and determined
 *             by the server. If the server never enabled a control pipe then
 *             this function will return -1.
 *
 * @param[in]  pipe_name  The pipe name
 * @param      data       The data
 * @param[in]  bytes      The control string
 * @param[in]  ch    channel to send the command to (0-127)
 *
 * @return     0 on success, -1 on failure
 */
int pipe_send_control_cmd_bytes(const char* pipe_name, const void* data, int bytes);

typedef struct {
    char source[MAX_SUB_MESSAGE_LEN];
    char submodule[MAX_SUB_MESSAGE_LEN];
    char level[MAX_SUB_MESSAGE_LEN];
    char message[MAX_MESSAGE_LEN];
} __attribute__((packed)) fault_t;

int write_fault_code(fault_t *fault);

#endif // MODAL_PIPE_COMMON_H
