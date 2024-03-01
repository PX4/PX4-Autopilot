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
 * @file zenoh_subscriber.hpp
 *
 * Defines basic functionality of Zenoh subscriber class
 *
 * @author Peter van der Perk <peter.vanderperk@nxp.com>
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>

#include <lib/parameters/param.h>
#include <containers/List.hpp>
#include <zenoh-pico.h>

// CycloneDDS CDR Deserializer
#include <dds/cdr/dds_cdrstream.h>
#include <dds_serializer.h>

class Zenoh_Subscriber : public ListNode<Zenoh_Subscriber *>
{
public:
	Zenoh_Subscriber(bool rostopic = true);
	virtual ~Zenoh_Subscriber();

	virtual int declare_subscriber(z_session_t s, const char *keyexpr);

	virtual int undeclare_subscriber();

	virtual void data_handler(const z_sample_t *sample);

	virtual void print();

protected:
	virtual void  print(const char *type_string, const char *topic_string);

	z_owned_subscriber_t _sub;
	char _topic[60]; // The Topic name is somewhere is the Zenoh stack as well but no good api to fetch it.


	// Indicates ROS2 Topic namespace
	bool _rostopic;
	const char *_rt_prefix = "rt/";
	const size_t _rt_prefix_offset = 3; // "rt/" are 3 chars
};
