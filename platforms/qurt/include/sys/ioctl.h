/****************************************************************************
 * Copyright (c) 2015 Mark Charlebois. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once
/**
 * @file
 * The declarations in this file are released to DSPAL users and are used to
 * make file I/O call's with a device path.
 */

#include "sys/ioccom.h"
#include <sys/cdefs.h>

__BEGIN_DECLS

/**
 * Sends an IOCTL for a previous open bus/port device or file.
 *
 * @param fd
 * File descriptor returned from the open function.
 * @param request
 * The numeric value of the IOCTL defined for the particular bus/port openened.
 * In the case of files, see fcntl.h\n
 * In the case of devices, see the following header files for a list of
 * IOCTL options defined for certain type of bus/port device
 * - dev_fs_lib_serial.h
 * - dev_fs_lib_spi.h
 * - dev_fs_lib_gpio.h
 * - dev_fs_lib_i2c.h \n
 * @param argp
 * Parameter buffer defined for the particular IOCTL.
 * @return
 * - SUCCESS: The IOCTL was successfully processed.
 * TODO: List error codes for all bus/port types.
 */
int ioctl(int fd, int request, void *argp);

__END_DECLS
