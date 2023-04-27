#include "scout_sdk/Utilities.hpp"
#include <px4_platform_common/time.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>

#define MODULE_NAME "SCOUT_SDK"

namespace scoutsdk
{

ProtocolDetector::ProtocolDetector() : _can(nullptr), _msg_v1_detected(false), _msg_v2_detected(false) {}

ProtocolDetector::~ProtocolDetector()
{
	if (_can != nullptr) {
		_can->Close();
		delete _can;
		_can = nullptr;
	}
}

bool ProtocolDetector::Connect(const char *const _canname, const uint32_t bitrate)
{
	_can = new SocketCAN();

	if (_can->Init(_canname, bitrate) == PX4_OK) { return true; } else { return false; }
}

ProtocolVersion ProtocolDetector::DetectProtocolVersion(const uint64_t timeout_msec)
{
	_msg_v1_detected = false;
	_msg_v2_detected = false;

	const hrt_abstime start_time = hrt_absolute_time();

	while (hrt_elapsed_time(&start_time) < timeout_msec) {
		//ParseCANFrame();
		if (_msg_v1_detected || _msg_v2_detected) { break; }
	}

	// remove
	_msg_v2_detected = true;

	// make sure only one version is detected
	if (_msg_v1_detected && _msg_v2_detected) {
		return ProtocolVersion::UNKNOWN;

	} else if (_msg_v1_detected) {
		return ProtocolVersion::AGX_V1;

	} else if (_msg_v2_detected) {
		return ProtocolVersion::AGX_V2;

	} else {
		return ProtocolVersion::UNKNOWN;
	}
};

void ProtocolDetector::ParseCANFrame()
{
	uint8_t data[8] {0};
	RxFrame rxf{};
	rxf.frame.payload = &data;

	if (_can->ReceiveFrame(&rxf) <= 0) { return; }

	switch (rxf.frame.can_id) {
	// state feedback frame with id 0x151 is unique to V1 protocol
	case 0x151: {
			PX4_INFO("Protocol V1 detected");
			_msg_v1_detected = true;
			break;
		}

	// motion control feedback frame with id 0x221 is unique to V2 protocol
	case 0x221:

	// rc state feedback frame with id 0x241 is unique to V2 protocol
	case 0x241: {
			PX4_INFO("Protocol V2 detected");
			_msg_v2_detected = true;
			break;
		}
	}
}
}  // namespace scoutsdk
