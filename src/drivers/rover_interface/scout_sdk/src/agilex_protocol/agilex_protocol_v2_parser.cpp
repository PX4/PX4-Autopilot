#include "scout_sdk/agilex_protocol/agilex_protocol_v2_parser.hpp"

namespace scoutsdk
{
bool AgileXProtocolV2Parser::DecodeMessage(const RxFrame *rxf, AgxMessage *msg)
{
	return DecodeCanFrameV2(&rxf->frame, msg);
}

bool AgileXProtocolV2Parser::EncodeMessage(const AgxMessage *msg, TxFrame *txf)
{
	return EncodeCanFrameV2(msg, &txf->frame);
}

uint8_t AgileXProtocolV2Parser::CalculateChecksum(uint16_t id, uint8_t *data, uint8_t dlc)
{
	return CalcCanFrameChecksumV2(id, data, dlc);
}
} // namespace scoutsdk