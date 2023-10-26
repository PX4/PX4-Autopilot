/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * @file uorb_publisher.hpp
 *
 * Defines generic, templatized uORB over Zenoh / ROS2
 *
 * @author Peter van der Perk <peter.vanderperk@nxp.com>
 */

#pragma once

#include "zenoh_publisher.hpp"
#include <uORB/Subscription.hpp>
#include <dds_serializer.h>

#define CDR_SAFETY_MARGIN 12

class uORB_Zenoh_Publisher : public Zenoh_Publisher
{
public:
	uORB_Zenoh_Publisher(const orb_metadata *meta, const uint32_t *ops) :
		Zenoh_Publisher(true),
		_uorb_meta{meta},
		_cdr_ops(ops)
	{
		_uorb_sub = orb_subscribe(meta);
	};

	~uORB_Zenoh_Publisher() override = default;

	// Update the uORB Subscription and broadcast a Zenoh ROS2 message
	virtual int8_t update() override
	{
		uint8_t data[_uorb_meta->o_size];
		orb_copy(_uorb_meta, _uorb_sub, data);

		uint8_t buf[_uorb_meta->o_size + 4 + CDR_SAFETY_MARGIN];
		memcpy(buf, ros2_header, sizeof(ros2_header));

		dds_ostream_t os;
		os.m_buffer = buf;
		os.m_index = (uint32_t)sizeof(ros2_header);
		os.m_size = (uint32_t)sizeof(ros2_header) + _uorb_meta->o_size + CDR_SAFETY_MARGIN;
		os.m_xcdr_version = DDSI_RTPS_CDR_ENC_VERSION_2;

		if (dds_stream_write(&os,
				     &dds_allocator,
				     (const char *)&data,
				     _cdr_ops)) {
			return publish((const uint8_t *)buf, os.m_size);

		} else {
			return _Z_ERR_MESSAGE_SERIALIZATION_FAILED;
		}
	};

	void setPollFD(px4_pollfd_struct_t *pfd)
	{
		pfd->fd = _uorb_sub;
		pfd->events = POLLIN;
	}

	void print()
	{
		printf("uORB %s -> ", _uorb_meta->o_name);
		Zenoh_Publisher::print();
	}

private:
	const orb_metadata *_uorb_meta;
	int _uorb_sub;
	const uint32_t *_cdr_ops;
};
