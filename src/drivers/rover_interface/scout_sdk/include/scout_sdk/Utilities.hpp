#pragma once

#include <drivers/drv_hrt.h>

#include "CAN/SocketCAN.hpp"
#include "agilex_protocol/agilex_protocol_v2_parser.hpp"

namespace scoutsdk
{
class ProtocolDetector
{
public:
	ProtocolDetector();
	~ProtocolDetector();

	bool Connect(const char *const can_name, const uint32_t bitrate);

	ProtocolVersion DetectProtocolVersion(const uint64_t timeout_sec);

private:
	SocketCAN *_can;
	void ParseCANFrame();
	AgileXProtocolV2Parser _parser;

	bool _msg_v1_detected;
	bool _msg_v2_detected;
};
}  // namespace scoutsdk
