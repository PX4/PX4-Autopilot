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

#include "TeseoFwProxy.hpp"

#include <cstdlib>
#include <cstring>

#include <px4_platform_common/getopt.h>

#include <uavcannode/UavcanNode.hpp>

using namespace time_literals;

ModuleBase::Descriptor TeseoFwProxy::desc{
	&TeseoFwProxy::task_spawn,
	&TeseoFwProxy::custom_command,
	&TeseoFwProxy::print_usage,
};

TeseoFwProxy::TeseoFwProxy() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::uavcan),  // share the WQ with UavcanNode
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
}

TeseoFwProxy::~TeseoFwProxy()
{
	ScheduleClear();
	delete _read_client;
	perf_free(_cycle_perf);
}

int TeseoFwProxy::Start()
{
	ScheduleOnInterval(kTickIntervalUs);
	return PX4_OK;
}

bool TeseoFwProxy::ensure_client()
{
	if (_read_client != nullptr) {
		return true;
	}

	uavcannode::UavcanNode *node = uavcannode::UavcanNode::instance();

	if (node == nullptr) {
		return false;
	}

	_read_client = new ReadClient(node->get_node());

	if (_read_client == nullptr) {
		return false;
	}

	const int res = _read_client->init();

	if (res < 0) {
		PX4_ERR("ReadClient init: %d", res);
		delete _read_client;
		_read_client = nullptr;
		return false;
	}

	_read_client->setCallback(ReadCallback(this, &TeseoFwProxy::cb_read));
	_read_client->setRequestTimeout(uavcan::MonotonicDuration::fromMSec(500));
	return true;
}

int TeseoFwProxy::requestUpdate(uint8_t server_node_id, const char *path)
{
	if (server_node_id == 0 || server_node_id > 127) {
		return -EINVAL;
	}

	if (_start_requested.load()) {
		return -EBUSY;
	}

	_pending_node_id = server_node_id;
	std::strncpy(_pending_path, path, sizeof(_pending_path) - 1);
	_pending_path[sizeof(_pending_path) - 1] = '\0';
	_start_requested.store(true);
	ScheduleNow();
	return PX4_OK;
}

void TeseoFwProxy::cb_read(const ReadResult &result)
{
	// Runs on the same WQ thread as Run() (UavcanNode::Run() dispatched
	// us via spinOnce()).  Stage the bytes and flip the flag - nothing else.
	if (!result.isSuccessful()) {
		_staging_len = 0;
		_staging_eof = false;
		_staging_err = -1;
		_response_ready = true;
		return;
	}

	const auto &resp = result.getResponse();
	_staging_err = resp.error.value;
	_staging_len = resp.data.size();
	_staging_eof = (_staging_len < kChunkSize);

	if (_staging_err == 0 && _staging_len > 0) {
		std::memcpy(_staging, resp.data.begin(), _staging_len);
	}

	_response_ready = true;
}

void TeseoFwProxy::mock_xloader_write(const uint8_t *data, uint16_t len)
{
	// Real driver: clock bytes out to the Teseo over UART/SPI and wait
	// for the X-Loader per-block ACK.  Demo just sums and prints.
	uint32_t sum = 0;

	for (uint16_t i = 0; i < len; ++i) {
		sum += data[i];
	}

	PX4_INFO("xloader: wrote %u B @off %llu (sum=0x%08lx)",
		 (unsigned)len, (unsigned long long)_offset, (unsigned long)sum);
}

void TeseoFwProxy::Run()
{
	perf_begin(_cycle_perf);

	// Pick up a fresh request, if any.
	if (_start_requested.load() && _state == State::Idle) {
		_server_node_id = _pending_node_id;
		std::strncpy(_path, _pending_path, sizeof(_path) - 1);
		_path[sizeof(_path) - 1] = '\0';
		_offset         = 0;
		_total_bytes    = 0;
		_response_ready = false;
		_state          = State::IssueRequest;
		_start_requested.store(false);
		PX4_INFO("starting update from node %u, path '%s'", _server_node_id, _path);
	}

	switch (_state) {
	case State::Idle:
	case State::Done:
	case State::Failed:
		break;

	case State::IssueRequest: {
			if (!ensure_client()) {
				PX4_ERR("UavcanNode not ready");
				_state = State::Failed;
				break;
			}

			ReadService::Request req;
			req.offset    = _offset;
			req.path.path = _path;

			const int call_res = _read_client->call(_server_node_id, req);

			if (call_res < 0) {
				PX4_ERR("file.Read call failed: %d", call_res);
				_state = State::Failed;
				break;
			}

			_response_ready = false;
			_state          = State::AwaitResponse;
			break;
		}

	case State::AwaitResponse:
		if (_response_ready) {
			_state = State::WriteToTeseo;
		}

		break;

	case State::WriteToTeseo:
		if (_staging_err != 0) {
			PX4_ERR("file.Read error %d at offset %llu",
				_staging_err, (unsigned long long)_offset);
			_state = State::Failed;
			break;
		}

		if (_staging_len > 0) {
			mock_xloader_write(_staging, _staging_len);
			_offset      += _staging_len;
			_total_bytes += _staging_len;
		}

		if (_staging_eof) {
			PX4_INFO("update complete: %llu B", (unsigned long long)_total_bytes);
			_state = State::Done;

		} else {
			_state = State::IssueRequest;
		}

		break;
	}

	perf_end(_cycle_perf);
}

int TeseoFwProxy::print_status()
{
	const char *state_name = "?";

	switch (_state) {
	case State::Idle:          state_name = "idle"; break;

	case State::IssueRequest:  state_name = "issue"; break;

	case State::AwaitResponse: state_name = "await"; break;

	case State::WriteToTeseo:  state_name = "write"; break;

	case State::Done:          state_name = "done"; break;

	case State::Failed:        state_name = "failed"; break;
	}

	PX4_INFO("state=%s offset=%llu total=%llu node=%u path='%s'",
		 state_name, (unsigned long long)_offset, (unsigned long long)_total_bytes,
		 (unsigned)_server_node_id, _path);
	perf_print_counter(_cycle_perf);
	return 0;
}

int TeseoFwProxy::task_spawn(int /*argc*/, char * /*argv*/[])
{
	TeseoFwProxy *instance = new TeseoFwProxy();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return -1;
	}

	const int ret = instance->Start();

	if (ret != PX4_OK) {
		delete instance;
		return ret;
	}

	desc.object.store(instance);
	desc.task_id = task_id_is_work_queue;
	return ret;
}

int TeseoFwProxy::custom_command(int argc, char *argv[])
{
	if (!is_running(desc)) {
		PX4_ERR("not running");
		return -1;
	}

	if (argc >= 1 && std::strcmp(argv[0], "update") == 0) {
		if (argc < 2) {
			return print_usage("missing <server_node_id>");
		}

		const int         node_id = std::atoi(argv[1]);
		const char *const path    = (argc >= 3) ? argv[2] : "teseo.bin";

		auto *instance = static_cast<TeseoFwProxy *>(desc.object.load());
		return instance->requestUpdate((uint8_t)node_id, path);
	}

	return print_usage("unknown command");
}

int TeseoFwProxy::print_usage(const char *reason)
{
	if (reason != nullptr) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Mock DroneCAN file.Read -> Teseo X-Loader firmware update proxy.  Streams
a firmware image chunk-by-chunk from the FC's UAVCAN v0 file server and
hands each chunk to a (mock) Teseo X-Loader writer.  No local storage is
used; chunks are ephemeral.

### Examples
$ teseo_fw_proxy start
$ teseo_fw_proxy update 1 teseo.bin
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("teseo_fw_proxy", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("update", "Start an update transfer");
	PRINT_MODULE_USAGE_ARG("<server_node_id> [<path>]", "FC node ID and (optional) firmware file path", false);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int teseo_fw_proxy_main(int argc, char *argv[]);

int teseo_fw_proxy_main(int argc, char *argv[])
{
	return ModuleBase::main(TeseoFwProxy::desc, argc, argv);
}
