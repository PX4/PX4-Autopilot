/****************************************************************************
 *
 *   Copyright (C) 2025 PX4 Development Team. All rights reserved.
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
#include "MCP.hpp"
#include "MCP23009.hpp"
#include "MCP23017.hpp"


MCP230XX::MCP230XX(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": single-sample")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": comms errors"))
{
}

MCP230XX::~MCP230XX()
{
	ScheduleClear();
	perf_free(_cycle_perf);
	perf_free(_comms_errors);
}

int MCP230XX::init_uorb()
{
	if (!_gpio_config_sub.registerCallback() ||
	    !_gpio_request_sub.registerCallback() ||
	    !_gpio_out_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return -1;
	}

	return PX4_OK;
}

void MCP230XX::exit_and_cleanup()
{
	_gpio_config_sub.unregisterCallback();
	_gpio_request_sub.unregisterCallback();
	_gpio_out_sub.unregisterCallback();

	mcp230XX_unregister_gpios(mcp_config.first_minor, mcp_config.num_pins, mcp_config._gpio_handle);
}

int MCP230XX::read(uint16_t *mask){
	*mask = 0;
	int ret = PX4_OK;

	for(int i=0; i<mcp_config.num_banks; i++){
		uint8_t gpio_addr;
		uint8_t data;

		ret |= get_gpio(i, &gpio_addr);
		ret |= read_reg(gpio_addr, data);

		*mask |=  (((uint16_t)data & 0x00FF) << (8*i));
	}

	if(ret != PX4_OK){
		perf_count(_comms_errors);
	}

	return ret;
}

int MCP230XX::write(uint16_t mask_set, uint16_t mask_clear){

	int ret = PX4_OK;
	_olat = (_olat & ~mask_clear) | mask_set;

	for(int i=0; i<mcp_config.num_banks; i++){
		uint8_t reg_addr;
		uint8_t data;
		uint8_t curr_olat = (uint8_t)( (_olat >> (8*i)) & 0x00FF );

		ret = get_olat(i, &reg_addr);

		if(ret != PX4_OK){
			return ret;
		}

		ret |= write_reg(reg_addr, curr_olat);
		ret |= read_reg(reg_addr, data);

		if(ret != PX4_OK || data != curr_olat){
			perf_count(_comms_errors);
			return PX4_ERROR;
		}
	}

	return ret;
}

int MCP230XX::read_reg(uint8_t address, uint8_t &data)
{
	return transfer(&address, 1, &data, 1);
}


int MCP230XX::write_reg(uint8_t address, uint8_t value)
{
	uint8_t data[2] = {address, value};
	return transfer(data, 2, nullptr, 0);
}


int MCP230XX::configure(uint16_t mask, PinType type)
{
	switch (type) {
	case PinType::Input:
		_iodir |= mask;
		_gppu &= ~mask;
		break;

	case PinType::InputPullUp:
		_iodir |= mask;
		_gppu |= mask;
		break;

	case PinType::Output:
		_iodir &= ~mask;
		break;

	default:
		return -EINVAL;
	}

	int ret = PX4_OK;

	for(int i=0; i<mcp_config.num_banks; i++){
		uint8_t curr_iodir = (uint8_t)( (_iodir >> (8*i)) & 0x00FF );
		uint8_t curr_gppu = (uint8_t)( (_gppu >> (8*i)) & 0x00FF );
		uint8_t iodir_addr;
		uint8_t gppu_addr;

		ret = get_iodir(i, &iodir_addr);
		ret |= get_gppu(i, &gppu_addr);

		if(ret != PX4_OK){
			perf_count(_comms_errors);
			return ret;
		}

		ret = write_reg(iodir_addr, curr_iodir);
		ret |= write_reg(gppu_addr, curr_gppu);

		if(ret != PX4_OK){
			PX4_ERR("MCP configure failed \n");
			perf_count(_comms_errors);
			return ret;
		}
	}

	return PX4_OK;
}

int MCP230XX::init( ){

	int ret = I2C::init();

	if (ret != PX4_OK) {
		PX4_ERR("I2C init failed");
		return ret;
	}

	ScheduleNow();
	return PX4_OK;
}

int MCP230XX::init_communication(){
	int ret = mcp230XX_register_gpios(mcp_config.bus, mcp_config.i2c_addr, mcp_config.first_minor, _iodir, mcp_config.num_pins, mcp_config.device_type, mcp_config._gpio_handle);
	ret |= init_uorb();
	return ret;
}

int MCP230XX::set_up(){
	int ret = PX4_OK;
	uint8_t iodir_addr;
	uint8_t olat_addr;
	uint8_t gppu_addr;

	for(int i=0; i<mcp_config.num_banks; i++){
		uint8_t curr_iodir = (uint8_t)( (_iodir >> (8*i)) & 0x00FF );
		uint8_t curr_olat = (uint8_t)( (_olat >> (8*i)) & 0x00FF );
		uint8_t curr_gppu = (uint8_t)( (_gppu >> (8*i)) & 0x00FF );

		ret |= get_olat(i, &olat_addr);
		ret |= get_iodir(i, &iodir_addr);
		ret |= get_gppu(i, &gppu_addr);

		if(ret != PX4_OK){
			return ret;
		}

		ret |= write_reg(iodir_addr, curr_iodir);
		ret |= write_reg(olat_addr, curr_olat);
		ret |= write_reg(gppu_addr, curr_gppu);

		if(ret != PX4_OK){
			perf_count(_comms_errors);
			return ret;
		}
	}
	return ret;
}

int MCP230XX::sanity_check(){
	int ret = PX4_OK;

	for(int i=0; i<mcp_config.num_banks; i++){
		uint8_t curr_iodir = (uint8_t)( (_iodir >> (8*i)) & 0x00FF );
		uint8_t curr_olat = (uint8_t)( (_olat >> (8*i)) & 0x00FF );
		uint8_t curr_gppu = (uint8_t)( (_gppu >> (8*i)) & 0x00FF );

		uint8_t iodir_addr;
		uint8_t olat_addr;
		uint8_t gppu_addr;

		uint8_t read_iodir;
		uint8_t read_olat;
		uint8_t read_gppu;

		ret |= get_olat(i, &olat_addr);
		ret |= get_iodir(i, &iodir_addr);
		ret |= get_gppu(i, &gppu_addr);

		ret |= read_reg(iodir_addr, read_iodir);
		ret |= read_reg(olat_addr, read_olat);
		ret |= read_reg(gppu_addr, read_gppu);

		if(read_iodir != curr_iodir || read_olat != curr_olat || read_gppu != curr_gppu){
			PX4_ERR("Sanity check failed on bank %d.", i);
			perf_count(_comms_errors);
			return PX4_ERROR;
		}
	}
	return ret;
}

int MCP230XX::probe()
{
	// no whoami, try to read IOCONA
	uint8_t data;
	uint8_t addr;
	int ret = get_probe_reg(&addr);
	if(ret == PX4_OK){
		return read_reg(addr, data);
	}

	return PX4_ERROR;
}

void MCP230XX::RunImpl()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	static int count = 0;
	int ret = PX4_OK;

	switch (curr_state) {
	case STATE::INIT_COMMUNICATION:
		set_params();
		ret = init_communication();
		if(ret == PX4_OK){
			curr_state = STATE::CONFIGURE;
			ScheduleNow();
		}else{
			PX4_ERR("MCP230XX: INIT_COMMUNICATION (pin registration & uORB) failed, retrying...");
			ScheduleDelayed(100000); // 100ms
		}

		break;
	case STATE::CONFIGURE:
		ScheduleClear();
		ret = set_up();
		if(ret == PX4_OK){
			curr_state = STATE::CHECK;
		}
		ScheduleNow();
		break;

	case STATE::CHECK:
		ScheduleClear();
		ret = sanity_check();

		if(ret == PX4_OK){
			curr_state = STATE::RUNNING;
			ScheduleOnInterval(mcp_config.interval * 1000);
		}else{
			PX4_ERR("MCP230XX: Sanity check failed, reconfiguring...");
			curr_state = STATE::CONFIGURE;
			ScheduleNow();
		}
		break;

	case STATE::RUNNING:
		perf_begin(_cycle_perf);
		gpio_config_s config;

		if (_gpio_config_sub.update(&config) && config.device_id == get_device_id()) {

			PinType type = PinType::Input;

			switch (config.config) {
			case config.INPUT_PULLUP:	type = PinType::InputPullUp; break;

			case config.OUTPUT:		type = PinType::Output; break;
			}

			ret |= write(config.state, config.mask);
			ret |= configure(config.mask, type);
		}

		gpio_out_s output;

		if (_gpio_out_sub.update(&output) && output.device_id == get_device_id()) {
			ret |= write(output.state, output.mask);
		}

		{
			gpio_in_s _gpio_in;
			_gpio_in.timestamp = hrt_absolute_time();
			_gpio_in.device_id = get_device_id();
			uint16_t input = 0;
			if(read(&input) == PX4_OK){
				_gpio_in.state = input;
				_to_gpio_in.publish(_gpio_in);
			}
		}

		perf_end(_cycle_perf);

		if(count < checking_freq && ret == PX4_OK){
			count++;
		}else{
			curr_state = STATE::CHECK;
			count = 0;
			ScheduleNow();
		}
		break;
	}
}

void MCP230XX::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_cycle_perf);
	perf_print_counter(_comms_errors);
}

void MCP230XX::print_usage()
{
	PRINT_MODULE_USAGE_NAME("mcp230xx", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0x27);
	PRINT_MODULE_USAGE_PARAM_INT('D', 0, 0, 65535, "Direction (1=Input, 0=Output)", true);
	PRINT_MODULE_USAGE_PARAM_INT('O', 0, 0, 65535, "Output", true);
	PRINT_MODULE_USAGE_PARAM_INT('P', 0, 0, 65535, "Pullups", true);
	PRINT_MODULE_USAGE_PARAM_INT('U', 0, 0, 1000, "Update Interval [ms]", true);
	PRINT_MODULE_USAGE_PARAM_INT('M', 0, 0, 255, "First minor number", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

I2CSPIDriverBase *MCP230XX::instantiate(const I2CSPIDriverConfig &config, int runtime_instance)
{
	auto *init = (const init_config_t *)config.custom_data;
	MCP230XX *instance = nullptr;

	switch (config.devid_driver_index) {
	case DRV_GPIO_DEVTYPE_MCP23009:
		instance = new MCP23009(config);
		break;

	case DRV_GPIO_DEVTYPE_MCP23017:
		instance = new MCP23017(config);
		break;

	default:
		instance = nullptr;
		break;
	}

	if (!instance) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	if (OK != instance->init()) {
		delete instance;
		return nullptr;
	}

	instance->_iodir = init->direction;
	instance->_olat = init->state;
	instance->_gppu = init->pullup;

	instance->mcp_config.bus = init->i2c_bus;
	instance->mcp_config.i2c_addr = init->i2c_addr;
	instance->mcp_config.device_type = init->device_type;
	instance->mcp_config.first_minor = init->first_minor;
	instance->mcp_config.interval = init->interval;

	return instance;
}
