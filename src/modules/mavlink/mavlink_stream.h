/****************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
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
 * @file mavlink_stream.h
 * Mavlink messages stream definition.
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#ifndef MAVLINK_STREAM_H_
#define MAVLINK_STREAM_H_

#include <drivers/drv_hrt.h>

class Mavlink;
class MavlinkStream;

class MavlinkStream
{

public:
	MavlinkStream *next;

	MavlinkStream(Mavlink *mavlink);
	virtual ~MavlinkStream();

	/**
	 * Get the interval
	 *
	 * @param interval the inveral in microseconds (us) between messages
	 */
	void set_interval(const unsigned int interval);

	/**
	 * Get the interval
	 *
	 * @return the inveral in microseconds (us) between messages
	 */
	unsigned get_interval() { return _interval; }

	/**
	 * @return 0 if updated / sent, -1 if unchanged
	 */
	int update(const hrt_abstime t);
	virtual const char *get_name() const = 0;
	virtual uint8_t get_id() = 0;

	/**
	 * @return true if steam rate shouldn't be adjusted
	 */
	virtual bool const_rate() { return false; }

	/**
	 * Get maximal total messages size on update
	 */
	virtual unsigned get_size() = 0;

protected:
	Mavlink     *_mavlink;
	unsigned int _interval;

	virtual void send(const hrt_abstime t) = 0;

private:
	hrt_abstime _last_sent;

	/* do not allow top copying this class */
	MavlinkStream(const MavlinkStream &);
	MavlinkStream &operator=(const MavlinkStream &);
};


#endif /* MAVLINK_STREAM_H_ */
