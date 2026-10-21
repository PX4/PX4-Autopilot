/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once
#include "mavlink_bridge_header.h"
class Mavlink;
void handleRidIdentityRequest(Mavlink &mavlink, const mavlink_message_t &message);
void handleRidUomStatus(const mavlink_message_t &message);

bool ridSystemSourceAllowed(uint8_t system, uint8_t component);
