#ifndef AGILEX_PROTOCOL_V2_PARSER_HPP
#define AGILEX_PROTOCOL_V2_PARSER_HPP

#include "agilex_message.h"
#include "agilex_msg_parser_v2.h"
#include "../CAN/SocketCAN.hpp"

namespace scoutsdk
{
enum class ProtocolVersion { UNKNOWN, AGX_V1, AGX_V2 };

class AgileXProtocolV2Parser
{
public:
	bool DecodeMessage(const RxFrame *rxf, AgxMessage *msg);
	bool EncodeMessage(const AgxMessage *msg, TxFrame *txf);
	uint8_t CalculateChecksum(uint16_t id, uint8_t *data, uint8_t dlc);
};
}	// namespace scoutsdk
#endif /* AGILEX_PROTOCOL_V2_PARSER_HPP */
