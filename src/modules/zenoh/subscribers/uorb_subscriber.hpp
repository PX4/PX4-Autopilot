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
 * @file uorb_subscriber.hpp
 *
 * Defines generic, templatized uORB over Zenoh / ROS2
 *
 * @author Peter van der Perk <peter.vanderperk@nxp.com>
 */

#pragma once

#include "zenoh_subscriber.hpp"
#include <uORB/topics/input_rc.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/actuator_outputs.h>

class uORB_Zenoh_Subscriber : public Zenoh_Subscriber
{
public:
	uORB_Zenoh_Subscriber(const orb_metadata *meta, const uint32_t *ops) :
		Zenoh_Subscriber(),
		_uorb_meta{meta},
		_cdr_ops(ops)
	{
		int instance = 0;
		_uorb_pub_handle = orb_advertise_multi(_uorb_meta, nullptr, &instance);
	};

	~uORB_Zenoh_Subscriber() override = default;

	// Update the uORB Subscription and broadcast a Zenoh ROS2 message
	void data_handler(const z_loaned_sample_t *sample)
	{
		char data[_uorb_meta->o_size];

		const z_loaned_bytes_t *payload = z_sample_payload(sample);
		size_t len = z_bytes_len(payload);

		dds_istream_t is = {.m_buffer = (unsigned char *)(payload), .m_size = static_cast<int>(len),
				    .m_index = 4, .m_xcdr_version = DDSI_RTPS_CDR_ENC_VERSION_2
				   };
		dds_stream_read(&is, data, &dds_allocator, _cdr_ops);

		// As long as we don't have timesynchronization between Zenoh nodes
		// we've to manually set the timestamp
		fix_timestamp(data);

		// ORB_ID::input_rc needs additional timestamp fixup
		if (static_cast<ORB_ID>(_uorb_meta->o_id) == ORB_ID::input_rc) {
			memcpy(&data[8], data, sizeof(hrt_abstime));
		}

		orb_publish(_uorb_meta, _uorb_pub_handle, &data);
	};

	void fix_timestamp(char *data)
	{
		hrt_abstime now = hrt_absolute_time();
		memcpy(data, &now, sizeof(hrt_abstime));
	}

	void print()
	{
		Zenoh_Subscriber::print("uORB", _uorb_meta->o_name);
	}

protected:
	// Default payload-size function -- can specialize in derived class
	size_t get_payload_size()
	{
		return _uorb_meta->o_size;
	}

private:
	const orb_metadata *_uorb_meta;
	orb_advert_t _uorb_pub_handle;
	const uint32_t *_cdr_ops;
};
