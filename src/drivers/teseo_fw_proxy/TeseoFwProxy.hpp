/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include <drivers/drv_hrt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/perf/perf_counter.h>

#include <uavcan/uavcan.hpp>
#include <uavcan/protocol/file/Read.hpp>

class TeseoFwProxy : public ModuleBase, public px4::ScheduledWorkItem
{
public:
	static ModuleBase::Descriptor desc;

	TeseoFwProxy();
	~TeseoFwProxy() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	int print_status() override;
	int Start();

	// Called from the shell thread via `teseo_fw_proxy update <node_id>`.
	// Posts a request that the WQ thread will pick up on the next tick.
	int requestUpdate(uint8_t server_node_id, const char *path);

private:
	void Run() override;

	using ReadService  = uavcan::protocol::file::Read;
	using ReadResult   = uavcan::ServiceCallResult<ReadService>;
	using ReadCallback = uavcan::MethodBinder<TeseoFwProxy *, void (TeseoFwProxy::*)(const ReadResult &)>;
	using ReadClient   = uavcan::ServiceClient<ReadService, ReadCallback>;

	// Fires from inside UavcanNode::Run()'s spinOnce(), on the same WQ
	// thread as our own Run() - they cannot overlap.  Keep this small:
	// stage the bytes and flip the state.
	void cb_read(const ReadResult &result);

	// Mock X-Loader UART/SPI write.  In real code this would clock the
	// chunk out to the Teseo and wait for the per-block ACK; for the
	// demo we just printf.
	void mock_xloader_write(const uint8_t *data, uint16_t len);

	bool ensure_client();

	enum class State : uint8_t {
		Idle,
		IssueRequest,
		AwaitResponse,
		WriteToTeseo,
		Done,
		Failed,
	};

	static constexpr uint16_t kChunkSize     = 256;
	static constexpr uint32_t kTickIntervalUs = 10'000;   // 100 Hz tick

	// --- request configuration latched from the shell command ---
	px4::atomic_bool _start_requested{false};
	uint8_t          _pending_node_id{0};
	char             _pending_path[40] {};

	// --- in-flight transfer state (only touched on the WQ thread) ---
	State    _state{State::Idle};
	uint8_t  _server_node_id{0};
	char     _path[40] {};
	uint64_t _offset{0};
	uint64_t _total_bytes{0};

	uint8_t  _staging[kChunkSize] {};
	uint16_t _staging_len{0};
	bool     _staging_eof{false};
	int      _staging_err{0};
	bool     _response_ready{false};

	ReadClient *_read_client{nullptr};

	perf_counter_t _cycle_perf{nullptr};
};
