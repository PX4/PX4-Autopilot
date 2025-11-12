/****************************************************************************
 *
 *   Copyright (c) 2025 ModalAI, inc. All rights reserved.
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
#include "mpa.hpp"
#include <sys/types.h>
#include <sys/stat.h>
#include <time.h>
#include <px4_log.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <uORB/uORB.h>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/crsf_raw.h>

class CrsfBridge : public ModuleBase<CrsfBridge>, public px4::WorkItem
{
public:

	CrsfBridge();
	~CrsfBridge() override = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	static void control_callback(int ch, char* data, int bytes, void* context);

	// Subscribe to RX topic (from serial) to forward to MPA
	uORB::SubscriptionCallbackWorkItem _crsf_raw_rx_sub{this, ORB_ID(crsf_raw_rx)};

	crsf_raw_s _crsf_raw_rx{};

	// Publish to TX topic (to serial) from MPA control pipe
	orb_advert_t _crsf_raw_tx_pub{nullptr};

	int crsf_pipe_ch{0};

};

CrsfBridge::CrsfBridge() :
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
}

bool CrsfBridge::init()
{
	if (MPA::Initialize() == -1) {
		PX4_ERR("MPA init failed");
		return false;
	}

	// Create pipe with control pipe enabled so clients can send data back
	char crsf_pipe_name[] = "crsf_raw";
	crsf_pipe_ch = MPA::PipeCreate(crsf_pipe_name, SERVER_FLAG_EN_CONTROL_PIPE);
	if (crsf_pipe_ch == -1) {
		PX4_ERR("Pipe create failed for %s", crsf_pipe_name);
		return false;
	}

	PX4_INFO("Created pipe '%s' on channel %d with control pipe enabled", crsf_pipe_name, crsf_pipe_ch);

	// Set control callback to receive CRSF data from external clients
	// MUST be set AFTER pipe creation and on the correct channel
	MPA::PipeServerSetControlCb(crsf_pipe_ch, &CrsfBridge::control_callback, this);

	if (!_crsf_raw_rx_sub.registerCallback()) {
		PX4_ERR("crsf_raw_rx callback registration failed");
		return false;
	}

	return true;
}

void CrsfBridge::Run()
{
	if (should_exit()) {
		_crsf_raw_rx_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	if (_crsf_raw_rx_sub.updated()) {
		if (_crsf_raw_rx_sub.update(&_crsf_raw_rx)) {
			crsf_raw_data_t crsf;
			memset(&crsf, 0, sizeof(crsf));

			crsf.magic_number = CRSF_RAW_MAGIC_NUMBER;
			crsf.timestamp_ns = _crsf_raw_rx.timestamp * 1000; // Convert µs to ns
			crsf.len = _crsf_raw_rx.len;
			memcpy(crsf.data, _crsf_raw_rx.data, sizeof(crsf.data));
			crsf.reserved_1 = 0;
			crsf.reserved_2 = 0;

			if (MPA::PipeWrite(crsf_pipe_ch, (void*)&crsf, sizeof(crsf_raw_data_t)) == -1) {
				PX4_ERR("Pipe %d write failed!", crsf_pipe_ch);
			}
		}
	}
}

void CrsfBridge::control_callback(int ch, char* data, int bytes, void* context)
{
	// PX4_INFO("CRSF control callback triggered: ch=%d, bytes=%d", ch, bytes);

	// context is the 'this' pointer we passed during callback registration
	CrsfBridge* bridge = static_cast<CrsfBridge*>(context);

	if (bytes != sizeof(crsf_raw_data_t)) {
		PX4_WARN("Invalid control data size: %d bytes (expected %zu)", bytes, sizeof(crsf_raw_data_t));
		return;
	}

	crsf_raw_data_t* crsf_data = reinterpret_cast<crsf_raw_data_t*>(data);

	// Verify magic number
	if (crsf_data->magic_number != CRSF_RAW_MAGIC_NUMBER) {
		PX4_WARN("Invalid magic number: 0x%08x (expected 0x%08x)",
		         crsf_data->magic_number, CRSF_RAW_MAGIC_NUMBER);
		return;
	}

	// PX4_INFO("Publishing to crsf_raw_tx: len=%u", crsf_data->len);

	// Convert from MPA format to uORB format and publish to TX topic (to serial)
	crsf_raw_s msg{};
	msg.timestamp = hrt_absolute_time(); // Use PX4's monotonic time
	msg.len = crsf_data->len;
	memcpy(msg.data, crsf_data->data, sizeof(msg.data));

	// Advertise and publish to crsf_raw_tx (CrsfRc will read and send to serial)
	if (bridge->_crsf_raw_tx_pub == nullptr) {
		bridge->_crsf_raw_tx_pub = orb_advertise(ORB_ID(crsf_raw_tx), &msg);
	} else {
		orb_publish(ORB_ID(crsf_raw_tx), bridge->_crsf_raw_tx_pub, &msg);
	}
}

int CrsfBridge::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int CrsfBridge::task_spawn(int argc, char *argv[])
{
	CrsfBridge *instance = new CrsfBridge();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int CrsfBridge::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
CRSF raw data bridge - forwards raw CRSF frames from uORB to MPA pipe

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("crsf_bridge", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int crsf_bridge_main(int argc, char *argv[])
{
	return CrsfBridge::main(argc, argv);
}
