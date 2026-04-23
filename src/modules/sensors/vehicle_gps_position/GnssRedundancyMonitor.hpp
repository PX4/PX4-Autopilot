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
#include <lib/matrix/matrix/math.hpp>
#include <px4_platform_common/module_params.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/gnss_redundancy_status.h>
#include <uORB/topics/sensor_gps.h>

#include "gps_blending.hpp"

namespace sensors
{

// Computes system-wide GNSS redundancy health from the last-known sample of
// each receiver and publishes it on the gnss_redundancy_status topic.
// Kept separate from VehicleGPSPosition so the detection logic is small and
// unit-testable; instantiated and driven by VehicleGPSPosition each cycle.
class GnssRedundancyMonitor : public ModuleParams
{
public:
	GnssRedundancyMonitor(ModuleParams *parent);

	// Evaluate the current state given the per-instance samples and resolved
	// antenna offsets (as maintained by the blender), and publish the status.
	void update(const sensor_gps_s *gps_states,
		    const matrix::Vector3f *antenna_offsets,
		    uint8_t num_receivers,
		    bool is_armed);

private:
	uORB::Publication<gnss_redundancy_status_s> _gnss_redundancy_status_pub{ORB_ID(gnss_redundancy_status)};

	uint8_t _peak_fixed_count{0};
	hrt_abstime _divergence_since{0};

	DEFINE_PARAMETERS_CUSTOM_PARENT(ModuleParams,
					(ParamInt<px4::params::SYS_HAS_NUM_GNSS>) _param_sys_has_num_gnss
				       )
};

} // namespace sensors
