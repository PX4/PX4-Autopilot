/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

#include "ekf.h"
#include <aid_sources/aux_global_position/aux_global_position.hpp>
#include <aid_sources/aux_global_position/aux_global_position_control.hpp>

#if defined(CONFIG_EKF2_AUX_GLOBAL_POSITION) && defined(MODULE_NAME)

AuxGlobalPosition::AuxGlobalPosition() : ModuleParams(nullptr)
{
	for (int slot = 0; slot < MAX_AGP_IDS; slot++) {
		_id_param_values[slot] = getAgpParamInt32("ID", slot);

		if (_id_param_values[slot] != 0) {
			_sources[slot] = new AgpSource(slot, this);
			_n_sources++;
		}
	}

	// Only subscribe to uORB instances if there are configured sources
	if (_n_sources > 0) {
		for (int i = 0; i < MAX_AGP_IDS; i++) {
			_agp_sub[i] = uORB::Subscription(ORB_ID(aux_global_position), i);
		}
	}
}

AuxGlobalPosition::~AuxGlobalPosition()
{
	for (int i = 0; i < MAX_AGP_IDS; i++) {
		delete _sources[i];
	}
}

void AuxGlobalPosition::update(Ekf &ekf, const estimator::imuSample &imu_delayed)
{
	// If there are no sources configured, we also don't subscribe to any uORB topic
	// and skip the update completely.
	if (_n_sources == 0) {
		return;
	}

	for (int instance = 0; instance < MAX_AGP_IDS; instance++) {
		if (_agp_sub[instance].updated()) {
			aux_global_position_s msg{};
			_agp_sub[instance].copy(&msg);

			int slot = _instance_slot_map[instance];

			if (slot < 0) {
				slot = mapSensorIdToSlot(msg.id);
				_instance_slot_map[instance] = static_cast<int8_t>(slot);
			}

			if (slot >= 0 && _sources[slot]) {
				_sources[slot]->bufferData(msg, imu_delayed);
			}
		}
	}

	for (int slot = 0; slot < MAX_AGP_IDS; slot++) {
		if (_sources[slot]) {
			if (_sources[slot]->update(ekf, imu_delayed)) {
				// Only update one source per update cycle
				break;
			}
		}
	}
}

void AuxGlobalPosition::paramsUpdated()
{
	for (int i = 0; i < MAX_AGP_IDS; i++) {
		if (_sources[i]) {
			_sources[i]->updateParams();
		}
	}
}

float AuxGlobalPosition::testRatioFiltered() const
{
	float max_ratio = 0.f;

	for (int i = 0; i < MAX_AGP_IDS; i++) {
		if (_sources[i] && _sources[i]->isFusing()) {
			max_ratio = math::max(max_ratio, _sources[i]->getTestRatioFiltered());
		}
	}

	return max_ratio;
}

bool AuxGlobalPosition::anySourceFusing() const
{
	for (int i = 0; i < MAX_AGP_IDS; i++) {
		if (_sources[i] && _sources[i]->isFusing()) {
			return true;
		}
	}

	return false;
}

int32_t AuxGlobalPosition::getAgpParamInt32(const char *param_suffix, int instance) const
{
	char param_name[20] {};
	snprintf(param_name, sizeof(param_name), "EKF2_AGP%d_%s", instance, param_suffix);

	int32_t value = 0;

	if (param_get(param_find(param_name), &value) != 0) {
		PX4_ERR("failed to get %s", param_name);
	}

	return value;
}

bool AuxGlobalPosition::setAgpParamInt32(const char *param_suffix, int instance, int32_t value)
{
	char param_name[20] {};
	snprintf(param_name, sizeof(param_name), "EKF2_AGP%d_%s", instance, param_suffix);

	return param_set_no_notification(param_find(param_name), &value) == PX4_OK;
}

int32_t AuxGlobalPosition::getIdParam(int instance)
{
	return _id_param_values[instance];
}

void AuxGlobalPosition::setIdParam(int instance, int32_t sensor_id)
{
	setAgpParamInt32("ID", instance, sensor_id);
	_id_param_values[instance] = sensor_id;
}

int AuxGlobalPosition::mapSensorIdToSlot(int32_t sensor_id)
{
	for (int slot = 0; slot < MAX_AGP_IDS; slot++) {
		if (getIdParam(slot) == sensor_id) {
			return slot;
		}
	}

	for (int slot = 0; slot < MAX_AGP_IDS; slot++) {
		if (getIdParam(slot) == 0) {
			setIdParam(slot, sensor_id);
			return slot;
		}
	}

	return -1;
}

#endif // CONFIG_EKF2_AUX_GLOBAL_POSITION && MODULE_NAME
