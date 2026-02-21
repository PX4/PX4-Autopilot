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

#ifndef MODAL_PIPE_DEFINES_H
#define MODAL_PIPE_DEFINES_H


/**
 * This is the recommended location for pipes to be created. It is not enforced
 * or used anywhere in this library, it's just where we recommend putting pipes
 * for consistency. For example, the imu server would make two channels with two
 * directories in this base dir, one for each imu0 and imu1
 *
 * - /run/mpa/imu0/
 * - /run/mpa/imu1/
 *
 * The camera server would then share the base dir, making a new dir for each of
 * the cameras:
 *
 * - /run/mpa/tracking/
 * - /run/mpa/stereo/
 * - /run/mpa/hires/
 *
 * This base directory is chosen because it only exists in memory, not on the
 * disk. It is also NOT preserved between reboots, ensuring no remnants of old
 * pipes exist on startup after improper shutdown. It can also be mounted inside
 * of docker images to share pipe data in/out of dockers.
 */
#ifdef __ANDROID__
#warning Building for Android
#define MODAL_PIPE_DEFAULT_BASE_DIR "/data/data/com.modalai.sensors.server/mpa/"
#define MODAL_PIPE_MAX_DIR_LEN      128
#else
#define MODAL_PIPE_DEFAULT_BASE_DIR "/run/mpa/"
#define MODAL_PIPE_MAX_DIR_LEN      64
#endif // __ANDROID__

// Sensible limits of the length of directories and paths
#define MODAL_PIPE_MAX_NAME_LEN     32
#define MODAL_PIPE_MAX_PATH_LEN     (MODAL_PIPE_MAX_DIR_LEN + MODAL_PIPE_MAX_NAME_LEN)
#define MODAL_PIPE_MAX_TYPE_LEN     32

#define MODAL_PIPE_DEFAULT_PIPE_SIZE    (1024*1024) // 1Mb


// error codes used throughout the library
// mostly these are used by pipe_client_open()
#define PIPE_ERROR_OTHER                    -1
#define PIPE_ERROR_SERVER_NOT_AVAILABLE     -2
#define PIPE_ERROR_REACHED_MAX_NAME_INDEX   -3
#define PIPE_ERROR_FILE_IO                  -4
#define PIPE_ERROR_TIMEOUT                  -5
#define PIPE_ERROR_INVALID_ARG              -6
#define PIPE_ERROR_NOT_CONNECTED            -7
#define PIPE_ERROR_CTRL_NOT_AVAILABLE       -8
#define PIPE_ERROR_INFO_NOT_AVAILABLE       -9
#define PIPE_ERROR_CHANNEL_OOB              -10

#define MAX_SUB_MESSAGE_LEN 64
#define MAX_MESSAGE_LEN 256

#define FAULT_PATH	(MODAL_PIPE_DEFAULT_BASE_DIR "fault")

#endif // MODAL_PIPE_DEFINES_H