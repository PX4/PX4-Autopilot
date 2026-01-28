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

/**
 * @file uorb_subscriber.hpp
 *
* Defines generic, templatized uORB over UAVCANv1 publisher
 *
 * @author Peter van der Perk <peter.vanderperk@nxp.com>
 */

#pragma once

#include "../DynamicPortSubscriber.hpp"

#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/actuator_outputs.h>

#include <uORB/topics/sensor_gps.h>

template <class T>
class uORB_over_UAVCAN_Subscriber : public UavcanDynamicPortSubscriber
{
public:
	uORB_over_UAVCAN_Subscriber(CanardHandle &handle, UavcanParamManager &pmgr, const orb_metadata *meta,
				    uint8_t instance = 0) :
		UavcanDynamicPortSubscriber(handle, pmgr, "uorb.", meta->o_name, instance),
		_uorb_meta{meta},
		_uorb_pub(meta)
	{};

	~uORB_over_UAVCAN_Subscriber() override = default;

	void subscribe() override
	{
		T *data = NULL;

		// Subscribe to messages uORB sensor_gps payload over UAVCAN
		_canard_handle.RxSubscribe(CanardTransferKindMessage,
					   _subj_sub._canard_sub.port_id,
					   get_payload_size(data),
					   CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC * 10000,
					   &_subj_sub._canard_sub);
	};

	void callback(const CanardRxTransfer &receive) override
	{
		T *data = (T *)receive.payload;

		if (receive.payload_size == get_payload_size(data)) {

			/* Data type specific conversion if necceary  */
			convert(data);

			_uorb_pub.publish(*data);

		} else {
			PX4_ERR("uORB over UAVCAN %s payload size mismatch got %zu expected %zu",
				_subj_sub._subject_name, receive.payload_size, get_payload_size(data));
		}
	};

protected:
	// Default payload-size function -- can specialize in derived class
	size_t get_payload_size(const T *msg)
	{
		(void)msg;
		return sizeof(T);
	};

	void convert(T *data) {};

private:
	const orb_metadata *_uorb_meta;
	uORB::PublicationMulti<T> _uorb_pub;
};

/* ---- Specializations of get_payload_size() to reduce wasted bandwidth where possible ---- */


/* ---- Specializations of convert() to convert incompatbile data, instance no. timestamp ---- */

template<>
void uORB_over_UAVCAN_Subscriber<sensor_gps_s>::convert(sensor_gps_s *data);
