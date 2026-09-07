/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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
 * @file Publication.cpp
 *
 * Out-of-line definitions of the type-independent PublicationBase machinery.
 * Keeping advertise()/publish() out of the header emits them once instead of
 * once per translation unit (PX4 links with bfd ld, no ICF, and no LTO).
 */

#include "Publication.hpp"
#include "PublicationMulti.hpp"

namespace uORB
{

PublicationBase::~PublicationBase()
{
	if (_handle != nullptr) {
		// don't automatically unadvertise queued publications (eg vehicle_command)
		if (Manager::orb_get_queue_size(_handle) == 1) {
			unadvertise();
		}
	}
}

bool PublicationBase::advertise()
{
	if (!advertised()) {
		_handle = orb_advertise(get_topic(), nullptr);
	}

	return advertised();
}

bool PublicationBase::publish(const void *data)
{
	if (!advertised()) {
		advertise();
	}

	return (Manager::orb_publish(get_topic(), _handle, data) == PX4_OK);
}

bool PublicationMultiBase::advertise()
{
	if (!advertised()) {
		int instance = 0;
		_handle = orb_advertise_multi(get_topic(), nullptr, &instance);
	}

	return advertised();
}

bool PublicationMultiBase::publish(const void *data)
{
	if (!advertised()) {
		advertise();
	}

	return (orb_publish(get_topic(), _handle, data) == PX4_OK);
}

int PublicationMultiBase::get_instance()
{
	// advertise if not already advertised
	if (advertise()) {
		return Manager::orb_get_instance(_handle);
	}

	return -1;
}

} // namespace uORB
