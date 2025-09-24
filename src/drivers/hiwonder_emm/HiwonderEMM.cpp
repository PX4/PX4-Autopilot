/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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

#include "HiwonderEMM.hpp"

HiwonderEMM::HiwonderEMM() :
	OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default), I2C(DRV_MOTOR_DEVTYPE_HIWONDER_EMM,
			MODULE_NAME, I2CBUS, I2C_ADDR, 400000)
{
}

int HiwonderEMM::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) {
		PX4_ERR("I2C init failed. Error: %d", ret);
		return ret;
	}

	const uint8_t set_motor_type[2] = {MOTOR_TYPE_ADDR, MOTOR_TYPE_JGB37_520_12V_110RPM};
	ret = transfer(set_motor_type, sizeof(set_motor_type), nullptr, 0);

	if (ret != PX4_OK) {
		PX4_ERR("Failed to set motor type. Error: %d", ret);
		return ret;
	}

	const uint8_t set_motor_polarity[2] = {MOTOR_ENCODER_POLARITY_ADDR, 0};
	ret = transfer(set_motor_polarity, sizeof(set_motor_polarity), nullptr, 0);

	if (ret != PX4_OK) {
		PX4_ERR("Failed to set encoder polarity. Error: %d", ret);
		return ret;
	}

	this->ChangeWorkQueue(px4::device_bus_to_wq(this->get_device_id()));

	PX4_INFO("Hiwonder EMM running on I2C bus %d address 0x%.2x", this->get_device_bus(), this->get_device_address());

	ScheduleNow();

	return PX4_OK;
}

bool HiwonderEMM::updateOutputs(uint16_t *outputs, unsigned num_outputs,
				unsigned num_control_groups_updated)
{
	uint8_t speed_values[CHANNEL_COUNT];

	for (unsigned i = 0; i < num_outputs && i < CHANNEL_COUNT; i++) {
		speed_values[i] = (uint8_t)(outputs[i] - 128);
	}

	set_motor_speed(speed_values, CHANNEL_COUNT);

	return true;
}

void HiwonderEMM::Run()
{
	if (should_exit()) {
		ScheduleClear();
		_mixing_output.unregister();
		exit_and_cleanup();
		return;
	}

	_mixing_output.update();
	_mixing_output.updateSubscriptions(false);
}

int HiwonderEMM::probe()
{
	int ret = I2C::probe();

	if (ret != PX4_OK) {
		PX4_ERR("I2C probe failed. Error: %d", ret);
		return ret;
	}

	int adc_value{0};
	ret = read_adc(adc_value);

	if (ret != PX4_OK) {
		PX4_ERR("Failed to probe Hiwonder EMM. Error: %d", ret);
		return ret;
	}

	PX4_INFO("Hiwonder EMM found");

	return PX4_OK;
}

int HiwonderEMM::read_adc(int &adc_value)
{
	const uint8_t cmd = ADC_BAT_ADDR;
	uint8_t buf[2] = {};
	const int ret = transfer(&cmd, sizeof(cmd), buf, sizeof(buf));

	if (ret != PX4_OK) {
		PX4_ERR("Failed to read ADC. Error: %d", ret);
		adc_value = 0;
		return ret;
	}

	adc_value = buf[0] | (buf[1] << 8);
	return ret;
}

int HiwonderEMM::read_encoder_counts(int32_t *encoder_counts, const uint8_t count)
{
	const uint8_t cmd = MOTOR_ENCODER_TOTAL_ADDR;
	uint8_t buf[CHANNEL_COUNT * 4]; // Each encoder count is a 32-bit integer (4 bytes)
	const int ret = transfer(&cmd, sizeof(cmd), buf, sizeof(buf));

	if (ret != PX4_OK) {
		PX4_ERR("Failed to read encoder counts. Error: %d", ret);
		return ret;
	}

	for (unsigned i = 0; i < count && i < CHANNEL_COUNT; i++) {
		encoder_counts[i] = buf[i * 4] | (buf[i * 4 + 1] << 8) | (buf[i * 4 + 2] << 16) | (buf[i * 4 + 3] << 24);
	}

	return ret;
}

int HiwonderEMM::set_motor_speed(const uint8_t *speed_values, const uint8_t count)
{
	uint8_t cmd[1 + CHANNEL_COUNT];
	cmd[0] = MOTOR_FIXED_SPEED_ADDR;

	for (unsigned int i = 0; i < count && i < CHANNEL_COUNT; i++) {
		cmd[i + 1] = speed_values[i];
	}

	const int ret = transfer(cmd, sizeof(cmd), nullptr, 0);

	if (ret != PX4_OK) {
		PX4_ERR("Failed to set motor speed. Error: %d", ret);
	}

	return ret;
}

int HiwonderEMM::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Hiwonder encoder motor module driver for PX4.
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("hiwonder_emm", "driver");
    PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the task");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

int HiwonderEMM::print_status() {
    int ret =  ModuleBase::print_status();
    PX4_INFO("HiwonderEMM @I2C Bus %d, address 0x%.2x",
            this->get_device_bus(),
            this->get_device_address());

    return ret;
}

int HiwonderEMM::custom_command(int argc, char **argv) {
    return PX4_OK;
}

int HiwonderEMM::task_spawn(int argc, char **argv) {
	auto *instance = new HiwonderEMM();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init() == PX4_OK) {
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

extern "C" __EXPORT int hiwonder_emm_main(int argc, char *argv[]){
	return HiwonderEMM::main(argc, argv);
}
