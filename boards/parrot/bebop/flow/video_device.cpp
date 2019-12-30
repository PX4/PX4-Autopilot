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

/**
 * @file video_device.cpp
 *
 * Wrapper for a V4L2 device using the memory mapped interface.
 *
 */

#include "video_device.h"

#include <stdlib.h>

#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <errno.h>
#include <string.h>

#include <linux/videodev2.h>

#include <px4_platform_common/posix.h>

int VideoDevice::start()
{
	int ret = open_device();

	if (ret < 0) {
		return ret;
	}

	ret = init_device();

	if (ret < 0) {
		return ret;
	}

	return start_capturing();
}

int VideoDevice::stop()
{

	int result = stop_capturing();

	if (result < 0) {
		PX4_ERR("Error stop stream");
	}

	// Unmap buffers
	for (unsigned int i = 0; i < _n_buffers; ++i) {
		result = munmap(_buffers[i].start, _buffers[i].length);

		if (result < 0) {
			PX4_ERR("Error: Unmapping buffer");
		}
	}

	free(_buffers);

	result = close_device();

	if (result < 0) {
		return result;
	}

	return OK;
}

int VideoDevice::print_info()
{
	PX4_INFO("Device: %s", _dev_name);
	return 0;
}

int VideoDevice::open_device()
{
	struct stat st;

	// Check if device is usable
	int ret = stat(_dev_name, &st);

	if (ret < 0) {
		PX4_ERR("Cannot identify %s: %d", _dev_name, errno);
		return -EIO;
	}

	if (!S_ISCHR(st.st_mode)) {
		PX4_ERR("%s is no device", _dev_name);
		return -EIO;
	}

	// Open V4L2 device nonblocking
	_fd = open(_dev_name, O_RDWR | O_NONBLOCK);

	if (_fd < 0) {
		PX4_ERR("Unable to open %s", _dev_name);
		return -EIO;
	}

	return OK;
}

int VideoDevice::close_device()
{
	int ret = close(_fd);

	if (ret < 0) {
		PX4_ERR("Error closing %s", _dev_name);
		return -EIO;
	}

	return OK;
}

int VideoDevice::init_device()
{
	struct v4l2_capability cap {};

	int ret = ioctl(_fd, VIDIOC_QUERYCAP, &cap);

	if (ret < 0) {
		if (EINVAL == errno) {
			PX4_ERR("No V4L2 device: %s", _dev_name);
			return -EINVAL;

		} else {
			PX4_ERR("VIDIOC_QUERYCAP failed: %s", _dev_name);
			return -1;
		}
	}

	if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
		PX4_ERR("Device is no video capture device: %s", _dev_name);
		return -EIO;
	}

	ret = init_crop();

	if (ret < 0) {
		PX4_ERR("Error when setting image crop");
		return -1;
	}

	return init_buffers();
}

int VideoDevice::init_crop()
{
	struct v4l2_cropcap cropcap {};
	struct v4l2_crop crop {};

	cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	int ret = ioctl(_fd, VIDIOC_CROPCAP, &cropcap);

	if (ret < 0) {
		PX4_WARN("CROPCAP failed");
	}

	crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	crop.c = cropcap.defrect; // reset to default
	ret = ioctl(_fd, VIDIOC_S_CROP, &crop);

	if (ret < 0) {
		PX4_WARN("S_CROP failed");
	}

	return init_format();
}

int VideoDevice::init_format()
{
	usleep(10000);
	struct v4l2_format fmt {};

	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt.fmt.pix.width = VIDEO_DEVICE_IMAGE_WIDTH;
	fmt.fmt.pix.height = VIDEO_DEVICE_IMAGE_HEIGHT;
	fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_NV12;
	fmt.fmt.pix.colorspace = V4L2_COLORSPACE_REC709;

	int ret = ioctl(_fd, VIDIOC_S_FMT, &fmt);

	if (ret < 0) {
		PX4_ERR("Unable to set format");
		return -1;
	}

	const char *format_string;

	switch (fmt.fmt.pix.pixelformat) {
	case V4L2_PIX_FMT_YUYV:
		format_string = "YUYV";
		break;

	case V4L2_PIX_FMT_YVYU:
		format_string = "YVYU";
		break;

	case V4L2_PIX_FMT_NV12:
		format_string = "NV12";
		break;

	default:
		format_string = "None";
	}

	PX4_INFO("Set image format: %ux%u\n    Format: %s\n    Size: %u",
		 fmt.fmt.pix.width,
		 fmt.fmt.pix.height,
		 format_string,
		 fmt.fmt.pix.sizeimage);

	return OK;
}

int VideoDevice::init_buffers()
{
	struct v4l2_requestbuffers req {};

	req.count = _n_buffers;
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = V4L2_MEMORY_MMAP;

	int ret = ioctl(_fd, VIDIOC_REQBUFS, &req);

	if (ret < 0) {
		PX4_ERR("Unable to request buffers: %s", _dev_name);
		return -1;
	}

	_buffers = (struct buffer *) malloc(_n_buffers * sizeof(_buffers[0]));

	if (_buffers == nullptr) {
		PX4_ERR("Unable to allocate buffers");
		return -1;
	}

	for (unsigned int i = 0; i < _n_buffers; ++i) {
		struct v4l2_buffer buf {};

		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = i;

		ret = ioctl(_fd, VIDIOC_QUERYBUF, &buf);

		if (ret < 0) {
			PX4_ERR("Error QUERYBUF");
			return -1;
		}

		_buffers[i].length = buf.length;
		_buffers[i].start = mmap(nullptr, buf.length, PROT_READ | PROT_WRITE,
					 MAP_SHARED, _fd, buf.m.offset);

		if (_buffers[i].start == MAP_FAILED) {
			PX4_ERR("Out of memory");
			return -1;
		}
	}

	return OK;
}

int VideoDevice::start_capturing()
{
	for (unsigned int i = 0; i < _n_buffers; ++i) {
		struct v4l2_buffer buf {};

		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = i;

		int ret = ioctl(_fd, VIDIOC_QBUF, &buf);

		if (ret < 0) {
			PX4_ERR("Unable to queue buffer: %d", i);
			return -1;
		}
	}

	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	int ret = ioctl(_fd, VIDIOC_STREAMON, &type);

	if (ret < 0) {
		PX4_ERR("Unable to start streaming");
		return -1;
	}

	PX4_INFO("Streaming started: %s", _dev_name);
	return OK;
}

int VideoDevice::stop_capturing()
{
	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	int ret = ioctl(_fd, VIDIOC_STREAMOFF, &type);

	if (ret < 0) {
		PX4_ERR("Unable to stop streaming");
		return -1;
	}

	PX4_INFO("Streaming stopped: %s", _dev_name);
	return OK;
}

int VideoDevice::get_frame(struct frame_data &frame)
{
	struct v4l2_buffer buf {};

	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_MMAP;

	int ret = ioctl(_fd, VIDIOC_DQBUF, &buf);

	if (ret < 0) {
		if (errno == EAGAIN) {
			//PX4_INFO("No frame ready");
			return 1;

		} else if (errno == EIO) {
			PX4_INFO("EIO");
			return 1;

		} else {
			PX4_ERR("Buffer deque error");
			return -1;
		}
	}

	frame.data = _buffers[buf.index].start;
	frame.seq = buf.sequence;
	frame.timestamp = buf.timestamp.tv_sec * 1000000 + buf.timestamp.tv_usec;
	frame.bytes = buf.bytesused;
	frame.index = buf.index;

	return 0;
}

int VideoDevice::put_frame(struct frame_data &frame)
{
	struct v4l2_buffer buf {};

	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_MMAP;
	buf.index = frame.index;
	buf.length = _buffers[frame.index].length;

	int ret = ioctl(_fd, VIDIOC_QBUF, &buf);

	if (ret < 0) {
		PX4_ERR("Buffer deque error");
		return -1;
	}

	return OK;
}
