/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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

#include <stdint.h>
#include <unistd.h>

#define VIDEO_DEVICE_IMAGE_WIDTH 320
#define VIDEO_DEVICE_IMAGE_HEIGHT 240

struct frame_data {
	uint32_t timestamp;
	uint32_t seq;
	uint32_t bytes;
	uint32_t index;
	void *data;
};

struct buffer {
	void *start;
	size_t length;
};

class VideoDevice
{
public:
	VideoDevice(char const *dev_name, uint32_t n_buffers) :
		_fd(-1), _dev_name(dev_name), _n_buffers(n_buffers), _buffers(nullptr) {};

	~VideoDevice() = default;

	/// Start the device
	int start();

	/// Stop the device
	int stop();

	/// Print various infos
	int print_info();

	/// Non-blocking call to fetch an image. Returns 0 if the images was read, -1 on error
	/// and 1 no new image is ready.
	int get_frame(struct frame_data &frame);

	/// Return a frame when the data is not needed any more.
	int put_frame(struct frame_data &frame);

private:
	int _fd;
	const char *_dev_name;
	uint32_t _n_buffers;
	struct buffer *_buffers;

	int open_device();
	int close_device();
	int init_device();
	int init_buffers();
	int init_crop();
	int init_format();
	int start_capturing();
	int stop_capturing();
};
