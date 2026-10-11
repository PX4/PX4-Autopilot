/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include <containers/IntrusiveSortedList.hpp>
#include <uavcan/uavcan.hpp>

#include <uavcan/node/publisher.hpp>
#include <drivers/drv_hrt.h>

namespace uavcannode
{

// Acquisition time of a sample taken at sample_hrt, in the bus shared time
// base. The bus time is 0 until the time-sync slave disciplines the clock, and
// the subtraction is unsigned, so publish UNKNOWN rather than underflow the
// uint56 field.
inline uint64_t bus_timestamp_usec(const uavcan::INode &node, hrt_abstime sample_hrt)
{
	const uint64_t bus_now_us = node.getUtcTime().toUSec();
	const uint64_t sample_age_us = hrt_absolute_time() - sample_hrt;
	return (bus_now_us > sample_age_us) ? (bus_now_us - sample_age_us) : 0;
}

class UavcanPublisherBase : public IntrusiveSortedListNode<UavcanPublisherBase *>
{
public:
	UavcanPublisherBase() = delete;
	explicit UavcanPublisherBase(uint16_t id) : _id(id) {}

	virtual ~UavcanPublisherBase() = default;

	/**
	 * Prints current status in a human readable format to stdout.
	 */
	virtual void PrintInfo() = 0;

	virtual void BroadcastAnyUpdates() = 0;

	// sorted numerically by ID
	bool operator<=(UavcanPublisherBase &rhs) { return id() <= rhs.id(); }

	uint16_t id() const { return _id; }

private:
	uint16_t _id{0};
};
} // namespace uavcannode
