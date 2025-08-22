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
 * @file zenoh_publisher.hpp
 *
 * Defines basic functionality of Zenoh publisher class
 *
 * @author Peter van der Perk <peter.vanderperk@nxp.com>
 */

#pragma once

#include "../rmw_attachment.h"

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>

#include <lib/parameters/param.h>
#include <containers/List.hpp>
#include <zenoh-pico.h>

class Zenoh_Publisher : public ListNode<Zenoh_Publisher *>
{
public:
	Zenoh_Publisher();
	virtual ~Zenoh_Publisher();

	virtual int declare_publisher(z_owned_session_t s, const char *keyexpr, uint8_t *gid);

	virtual int undeclare_publisher();

	virtual int8_t update() = 0;

	virtual void print();

protected:
	int8_t publish(const uint8_t *, int size);

	z_owned_publisher_t _pub;
	RmwAttachment _attachment;
};
