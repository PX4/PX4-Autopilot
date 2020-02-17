/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

/**
 * @file console_buffer.h
 * This implements a simple console buffer to store the full bootup output.
 * It can be printed via the 'dmesg' utility.
 * The output of stdout is redirected to CONSOLE_BUFFER_DEVICE, which is stored
 * to a circular buffer and output to stderr, so that everything is still printed
 * to the original console.
 */

#include <px4_platform_common/px4_config.h>


#define CONSOLE_BUFFER_DEVICE "/dev/console_buf"

#ifdef BOARD_ENABLE_CONSOLE_BUFFER

__BEGIN_DECLS

/**
 * Initialize the console buffer: register the CONSOLE_BUFFER_DEVICE
 * @return 0 on success, <0 error otherwise
 */
int px4_console_buffer_init();

/**
 * Print content of the console buffer to stdout
 * @param follow if true keep waiting and print new content whenever the buffer
 *               is updated
 */
void px4_console_buffer_print(bool follow);

/**
 * Get the current used buffer size
 */
int px4_console_buffer_size();

/**
 * Read (chunks) of the console buffer.
 * Note that no lock is held between reading multiple chunks, so the buffer could get
 * updated meanwhile. Use px4_console_buffer_size() to read no more than expected.
 * @param buffer output buffer
 * @param buffer_length output buffer length
 * @param offset input and output argument for the offset. Initially set this to -1.
 * @return number of bytes written to the buffer (or <0 on error)
 */
int px4_console_buffer_read(char *buffer, int buffer_length, int *offset);

__END_DECLS

#else

static inline int px4_console_buffer_init()
{
	return 0;
}

static inline int px4_console_buffer_size()
{
	return 0;
}

static inline int px4_console_buffer_read(char *buffer, int buffer_length, int *offset)
{
	return 0;
}
#endif /* BOARD_ENABLE_CONSOLE_BUFFER */
