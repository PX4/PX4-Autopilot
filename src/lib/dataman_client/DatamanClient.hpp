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
#pragma once

#include <uORB/uORB.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/dataman_request.h>
#include <uORB/topics/dataman_response.h>
#include <dataman/dataman.h>
#include <lib/perf/perf_counter.h>

using namespace time_literals;

class DatamanClient
{
public:
	DatamanClient();
	~DatamanClient();

	DatamanClient(const DatamanClient &) = delete;
	DatamanClient &operator=(const DatamanClient &) = delete;

	bool readSync(dm_item_t item, uint32_t index, uint8_t *buffer, uint32_t length, hrt_abstime timeout = 1000_ms);
	bool writeSync(dm_item_t item, uint32_t index, uint8_t *buffer, uint32_t length, hrt_abstime timeout = 1000_ms);
	bool clearSync(dm_item_t item, hrt_abstime timeout = 1000_ms);
	bool lockSync(dm_item_t item, hrt_abstime timeout = 1000_ms);
	bool unlockSync(dm_item_t item, hrt_abstime timeout = 1000_ms);

	bool readAsync(dm_item_t item, uint32_t index, uint8_t *buffer, uint32_t length);
	bool writeAsync(dm_item_t item, uint32_t index, uint8_t *buffer, uint32_t length);
	bool clearAsync(dm_item_t item);
	bool lockAsync(dm_item_t item);
	bool unlockAsync(dm_item_t item);

	void update();

	bool lastOperationCompleted(bool &success);

	/**
	 * Abort any async operation currently in progress
	 */
	void abortCurrentOperation();

private:

	enum class State {
		Idle,
		RequestSent,
		ResponseReceived
	};

	struct Request {
		hrt_abstime timestamp;
		dm_function_t request_type;
		dm_item_t item;
		uint32_t index;
		uint8_t *buffer;
		uint32_t length;
	};

	/* Synchronous response/request handler */
	bool syncHandler(const dataman_request_s &request, dataman_response_s &response,
			 const hrt_abstime &start_time, hrt_abstime timeout);

	State _state{State::Idle};
	Request _active_request{};
	uint8_t _response_status{};

	int32_t _dataman_response_sub{};
	uORB::Publication<dataman_request_s> _dataman_request_pub{ORB_ID(dataman_request)};

	px4_pollfd_struct_t _fds;

	uint8_t _client_id{0};

	static constexpr uint8_t CLIENT_ID_NOT_SET{0};
};


class DatamanCache
{
public:
	DatamanCache(const char *cache_miss_perf_counter_name, uint32_t num_items);
	~DatamanCache();

	void resize(uint32_t num_items);

	void invalidate();

	bool load(dm_item_t item, uint32_t index);

	bool loadWait(dm_item_t item, uint32_t index, uint8_t *buffer, uint32_t length, hrt_abstime timeout = 0);

	void update();

	/**
	 * @brief Function providing info if there items to be loaded to dataman cache.
	 *
	 * @return true if there are items to be processed.
	 */
	bool isLoading() {return (_item_counter > 0);}

	DatamanClient &client() { return _client; }

	int size() const { return _num_items; }

private:

	enum class State {
		Idle,
		RequestPrepared,
		RequestSent,
		ResponseReceived,
		Error
	};

	struct Item {
		dataman_response_s response;
		State cache_state;
	};

	inline void changeUpdateIndex();

	Item *_items{nullptr};
	uint32_t _load_index{0};	///< index for tracking last index used by load function
	uint32_t _update_index{0};	///< index for tracking last index used by update function
	uint32_t _item_counter{0};	///< number of items to process with update function
	uint32_t _num_items{0};

	DatamanClient _client{};

	perf_counter_t	_cache_miss_perf;
};
