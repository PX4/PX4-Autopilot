#ifndef AGILEX_MSG_PARSER_H
#define AGILEX_MSG_PARSER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "agilex_message.h"
#include "../CAN/can_frame.h"

bool DecodeCanFrameV2(const CANFrame *can_frame, AgxMessage *msg);
bool EncodeCanFrameV2(const AgxMessage *msg, CANFrame *can_frame);
uint8_t CalcCanFrameChecksumV2(uint16_t id, uint8_t *data, uint8_t dlc);

#ifdef __cplusplus
}
#endif

#endif /* AGILEX_MSG_PARSER_H */
