/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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
 * @file Service.hpp
 *
 * Implements a uORB Service that acts like ROS2 Service.
 */

#pragma once

#include <px4_platform_common/defines.h>
#include <systemlib/err.h>
#include <future>
#include <px4_platform_common/events.h>
#include <string.h>

#include <uORB/uORB.h>
#include "uORBManager.hpp"
#include "uORB/Subscription.hpp"
#include "uORB/Publication.hpp"

#include <uORB/topics/uORBTopics.hpp>

namespace uORB
{
/**
 * @brief Service class that provides the service
 *
 * @tparam req Request data structure defined by the Service
 * @tparam resp Response data structure defined by the Service
 */
template<typename req, typename resp>
class Service
{
public:
	/**
	 * @brief Construct a new Service object with the given request & response uORB topics
	 */
	Service(const orb_metadata *req_, const orb_metadata *resp_)
		: _request_sub(req_), _response_pub(resp_) {};

	// SYNCHRONOUS request handling implementation (Simple)
	/**
	 * @brief Get request if there's any
	 */
	bool get_request(req *request)
	{
		return _request_sub.update(request);
	}

	// SYNCHRONOUS response implementation
	/**
	 * @brief Send response to the last request
	 *
	 * @param request Request we are responding to
	 * @param response Response already filled in from the user
	 *
	 * Note: This 'assumes' that the client_id and sequence_id of the request
	 * will not be modified arbitrarily by the module. Otherwise, ther response
	 * won't be valid. The request & response part will eventually get handled in a
	 * single function and user won't be able to modify the internal ids in the end.
	 */
	bool send_response(const req &request, resp &response)
	{
		response.timestamp = hrt_absolute_time();

		// Response data is already filled in

		response.client_id = request.client_id;
		response.sequence_id = request.sequence_id;
		return _response_pub.publish(response);
	}

	// ASYNCHRONOUS CALLBACK IMPLEMENTATION (NOT COMPLETE YET)
	/**
	 * @brief Registers the callback function that will be called when a new request arrives
	 *
	 * @return true
	 * @return false
	 */
	bool registerCallback(void cb_func(const req &request, const resp &response))
	{
		//_request_sub.registerCallback(cb_func);
		_cb_func = cb_func; // Internally save the function pointer
		return false;
	}

private:
	void (*_cb_func)(const req &request, const resp &response); // Function pointer to the callback

	uORB::Subscription _request_sub;
	uORB::Publication<resp> _response_pub;
};


/**
 * @brief Client class that subscribes to the Service
 *
 * @tparam req Request data structure defined by the Service
 * @tparam resp Response data structure defined by the Service
 */
template<typename req, typename resp>
class Client
{
public:
	/**
	 * @brief Construct a new Client object with the given request & response uORB topics
	 *
	 * @param identifier Unique Identifier string (e.g. module's name) to calculate unique HASH id
	 */
	Client(const char *identifier, const orb_metadata *req_, const orb_metadata *resp_)
		: _request_pub(req_), _response_sub(resp_)
	{
		_client_id = (int32_t)events::util::hash_32_fnv1a_const(identifier);
	}

	// SIMPLE FUNCTIONAL IMPLEMENTATION (STEP 1)
	/**
	 * @brief Sends request, and returns true if the request uORB message was successfully published
	 */
	bool send_request(const req &request)
	{
		req *req_data = &_request_pub.get();

		// TODO: For the user, the 'client id', 'sequence id' etc should be invisible (decoupled).
		// So we need a user facing 'data structure' that can be translated from the uORB data
		// This would need to be generated during compile time like this:

		// void translate(Request *request_user_facing, const req *request_uorb_side)
		memcpy(req_data, &request, sizeof(req));

		req_data->timestamp = hrt_absolute_time();
		req_data->client_id = _client_id;
		req_data->sequence_id = _sequence_id;

		if (_request_pub.update()) {
			++_sequence_id;
			return true;
		}

		return false;
	}

	// TODO: Doesn't the user need to know from which 'request' this response is coming from?
	// How do we deal with this?

	/**
	 * @brief Checks if we have the response for the last request sent
	 */
	bool get_response(resp *response_ptr)
	{
		resp response_data;

		// Loop through available responses in the queue
		while (_response_sub.update(&response_data)) {
			// TODO: Need to ensure that our uORB topic for response (as well as request?) has a
			// ORB_QUEUE_LENGTH of 8 or so (arbitruary length to ensure no service requests will get lost)

			// TODO: Handle the case of identifying which 'request' corresponds to which 'response'
			// by utilizing sequence_id. E.g. when we send "Arm" and "Disarm", we need to distinguish
			// to which request the response was meant for.

			if (response_data.client_id == _client_id) {
				memcpy(response_ptr, &response_data, sizeof(resp));
				return true;
			}
		}

		return false;
	}

	// ASYNCHRONOUS STD::FUTURE IMPLMENTATION (NOT DONE YET)
	/**
	 * @brief Send the request asynchronously
	 *
	 * @return Future instance of the response that will asynchronously be updated
	 */
	std::future<resp *> async_send_request(const req &request)
	{
		_request_pub.update(request); // Publish new request
		return nullptr;
	}

private:
	uORB::PublicationData<req> _request_pub;
	uORB::Subscription _response_sub;

	// ROS2 does something similar (sequence id) to distinguish different requests from the same client, but here we don't copy the
	// exact same behavior since it is so complicated and I would like to add that support later on when I have a better understanding on ROS2 side.

	int32_t _client_id; // Unique identifier for the client (since multiple clients share the same Service)
	int32_t _sequence_id{0}; // Sequence ID that uniquely identifies the request message sent by this client in the past
};


}
