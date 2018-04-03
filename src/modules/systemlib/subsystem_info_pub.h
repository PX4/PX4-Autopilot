/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
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

/**
 * @file subsystem_info_pub.h
 *
 * Contains helper functions to efficiently publish the subsystem_info topic from various locations inside the code.
 *
 * @author Philipp Oettershagen (philipp.oettershagen@mavt.ethz.ch)
 */

#pragma once

#include <px4_log.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_status.h>

void publish_subsystem_info_init(vehicle_status_s *commander_vehicle_status_ptr, bool *commander_status_changed_ptr);
void publish_subsystem_info(uint64_t subsystem_type, bool present, bool enabled, bool ok);

void publish_subsystem_info_present_healthy(uint64_t subsystem_type, bool present, bool healthy);
void publish_subsystem_info_present_enabled(uint64_t subsystem_type, bool present, bool enabled);
void publish_subsystem_info_enabled_healthy(uint64_t subsystem_type, bool enabled, bool ok);
void publish_subsystem_info_enabled(uint64_t subsystem_type, bool enabled);
void publish_subsystem_info_healthy(uint64_t subsystem_type, bool ok);

void publish_subsystem_info_print();

// Local helper functions
bool getPresent(uint64_t subsystem_type);
bool getEnabled(uint64_t subsystem_type);
bool getHealthy(uint64_t subsystem_type);
