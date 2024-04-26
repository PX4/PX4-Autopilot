/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSf
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

#include "bmp581.h"
#include <cstdint>

BMP581::BMP581(const I2CSPIDriverConfig &config, IBMP581 *interface) :
	I2CSPIDriver(config),
	_interface(interface),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
	_measure_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": measure")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": comms_errors"))
{
}

BMP581::~BMP581()
{
	/* free perf counters */
	perf_free(_sample_perf);
	perf_free(_measure_perf);
	perf_free(_comms_errors);

	delete _interface;
}

int BMP581::init()
{
	if(!soft_reset()) {
		PX4_DEBUG("failed to reset baro during init");
		return -EIO;
	}

	_chip_id = _interface->get_reg(BMP5_REG_CHIP_ID_ADDR);

	if((_chip_id != BMP581_CHIP_ID_PRIM) && (_chip_id != BMP581_CHIP_ID_SEC)) {
		PX4_WARN("id of your baro is not: 0x%02x or 0x%02x", BMP581_CHIP_ID_PRIM, BMP581_CHIP_ID_SEC);
		return -EIO;
	}

	_chip_rev_id = _interface->get_reg(BMP5_REG_REV_ID_ADDR);

	if(!set_config()) {
		PX4_WARN("failed to set_config");
		return -EIO;
	}

	start();

	return OK;
}

void BMP581::print_status()
{
	I2CSPIDriverBase::print_status();
	printf("chip id: 0x%02x rev id: 0x%02x\n", _chip_id, _chip_rev_id);
	perf_print_counter(_sample_perf);
	perf_print_counter(_measure_perf);
	perf_print_counter(_comms_errors);
	printf("measurement interval: %u us \n", _measure_interval);
}

void BMP581::start()
{
	_collect_phase = false;

	ScheduleOnInterval(_measure_interval, _measure_interval * 3);
}

void BMP581::RunImpl()
{

	if(_collect_phase) {
		collect();
	}

	measure();
}

int BMP581::measure()
{
	_collect_phase = true;

	perf_begin(_measure_perf);

	/* start measurement */
	if(!set_power_mode(BMP5_POWERMODE_FORCED)) {
		PX4_DEBUG("failed to set power mode");
		perf_count(_comms_errors);
		perf_cancel(_measure_perf);
		return -EIO;
	}

	perf_end(_measure_perf);

	return OK;
}

int BMP581::collect()
{
	_collect_phase = false;

	bmp5_sensor_data data{};

	uint8_t int_status;
	int ret;

	perf_begin(_sample_perf);

	/* this should be fairly close to the end of the conversion, so the best approximation of the time */
	const hrt_abstime timestamp_sample = hrt_absolute_time();

	ret = get_interrupt_status(&int_status);

	if(ret) {
		if(int_status & BMP5_INT_ASSERTED_DRDY) {
			if(!get_sensor_data(&data)) {
				perf_count(_comms_errors);
				perf_cancel(_sample_perf);
				return -EIO;
			}
		}
	}

	//publish
	sensor_baro_s sensor_baro{};
	sensor_baro.timestamp_sample = timestamp_sample;
	sensor_baro.device_id = _interface->get_device_id();
	sensor_baro.pressure = data.pressure;
	sensor_baro.temperature = data.temperature;
	sensor_baro.error_count = perf_event_count(_comms_errors);
	sensor_baro.timestamp = hrt_absolute_time();
	_sensor_baro_pub.publish(sensor_baro);

	perf_end(_sample_perf);

	return OK;
}

/*!
 * @brief This API performs the soft reset of the sensor
 *
 * https://github.com/boschsensortec/BMP5-Sensor-API/blob/master/bmp5.c
 */
bool BMP581::soft_reset()
{
	bool result = false;
	 uint8_t status;
	int ret;

	if(intf == BMP5_SPI_INTF) {
		/* Performing a single read via SPI of registers,
		* e.g. registers CHIP_ID, before the actual
		* SPI communication with the device.
		*/
		status = _interface->get_reg(BMP5_REG_CHIP_ID_ADDR);
	}

	ret = _interface->set_reg(BMP5_SOFT_RESET_CMD, BMP5_REG_CMD_ADDR);

	if(ret == OK) {
		usleep(BMP5_DELAY_US_SOFT_RESET);

		if(intf == BMP5_SPI_INTF) {
			/* Performing a single read via SPI of registers,
            		* e.g. registers CHIP_ID, before the actual
            		* SPI communication with the device.
            		*/
			status = _interface->get_reg(BMP5_REG_CHIP_ID_ADDR);
		}

		status = _interface->get_reg(BMP5_REG_STATUS_ADDR);
		/* Check if nvm_rdy status = 1 and nvm_err status = 0 to proceed */
		if ((status & BMP5_INT_NVM_RDY) && (!(status & BMP5_INT_NVM_ERR))) {
			status = _interface->get_reg(BMP5_REG_INT_STATUS_ADDR);

			if(status & BMP5_INT_ASSERTED_POR_SOFTRESET_COMPLETE) {
				result = true;
			}
		}
	}

	return result;
}

bool BMP581::set_config()
{
	bool result = false;
	uint8_t rslt;
	if(set_power_mode(BMP5_POWERMODE_STANDBY)) {

		rslt = set_osr_odr_press_config();

		if(rslt) {
			rslt = set_iir_config();
		}

		if(rslt) {
			rslt = configure_interrupt();
		}

		if(rslt) {
			rslt = int_source_select();
		}

		if(rslt) {
			result = true;
		}
	}

	return result;
}

/*!
 *  @brief This API is used to get interrupt status.
 */
bool BMP581::get_interrupt_status(uint8_t *int_status)
{
	bool result = false;
	uint8_t status = 0xFF;

	status = _interface->get_reg(BMP5_REG_INT_STATUS_ADDR);

	result = true;
	*int_status = status;

	return result;
}

bool BMP581::check_deepstandby_mode(uint8_t *powermode)
{
	bool result = false;
	int rslt;
	uint8_t fifo_frame_sel;
	uint8_t reg_data[2];

	fifo_frame_sel = _interface->get_reg(BMP5_REG_FIFO_SEL_ADDR);
	fifo_frame_sel = BMP5_GET_BITS_POS_0(fifo_frame_sel, BMP5_FIFO_FRAME_SEL);

	rslt = _interface->get_reg_buf(BMP5_REG_OSR_CONFIG_ADDR, reg_data, 2);
	uint8_t odr_reg = 0xFF;
	if(rslt == OK) {
		odr_reg = BMP5_GET_BITSLICE(reg_data[1], BMP5_ODR);
	}

	rslt = _interface->get_reg_buf(BMP5_REG_DSP_CONFIG_ADDR, reg_data, 2);
	uint8_t set_iir_t_reg = 0xFF;
	uint8_t set_iir_p_reg = 0xFF;
	if(rslt == OK) {
		set_iir_t_reg = BMP5_GET_BITS_POS_0(reg_data[1], BMP5_SET_IIR_TEMP);
		set_iir_p_reg = BMP5_GET_BITSLICE(reg_data[1], BMP5_SET_IIR_PRESS);
		result = true;
	}

   	/* As per datasheet odr should be less than 5Hz. But register value for 5Hz is less than 4Hz and so,
     	* thus in this below condition odr is checked whether greater than 5Hz macro.
     	*/
	if((odr_reg > BMP5_ODR_05_HZ) && (fifo_frame_sel == BMP5_DISABLE) &&
	   (set_iir_t_reg == BMP5_IIR_FILTER_BYPASS) && (set_iir_p_reg == BMP5_IIR_FILTER_BYPASS)) {
		*powermode = BMP5_POWERMODE_DEEP_STANDBY;
	}

	return result;
}

/*!
 *  @brief This API is used to get powermode of the sensor.
 */
bool BMP581::get_power_mode(uint8_t *power_mode)
{
	bool result = false;
	uint8_t deep_dis;
	uint8_t reg_data;
	uint8_t powermode;

	if(power_mode != NULL) {
		reg_data = _interface->get_reg(BMP5_REG_ODR_CONFIG_ADDR);
		powermode = BMP5_GET_BITS_POS_0(reg_data, BMP5_POWERMODE);
		switch(powermode) {
			case BMP5_POWERMODE_STANDBY:
				deep_dis = BMP5_GET_BITSLICE(reg_data, BMP5_DEEP_DISABLE);

				if(deep_dis == BMP5_DEEP_ENABLED) {
					if(check_deepstandby_mode(power_mode)) {
						result = true;
					}
				}
				else {
					*power_mode = BMP5_POWERMODE_STANDBY;
					result = true;
				}
				break;
                	case BMP5_POWERMODE_NORMAL:
                    		*power_mode = BMP5_POWERMODE_NORMAL;
				result = true;
                    		break;
               		 case BMP5_POWERMODE_FORCED:
                    		*power_mode = BMP5_POWERMODE_FORCED;
				result = true;
                    		break;
                	case BMP5_POWERMODE_CONTINOUS:
                   		*power_mode = BMP5_POWERMODE_CONTINOUS;
				result = true;
                    		break;
		}
	}

	return result;
}
/*!
 *  @brief This API is used to set powermode of the sensor.
 */
bool BMP581::set_power_mode(uint8_t power_mode)
{
	bool result = false;
	uint8_t lst_pwrmode;
	uint8_t reg_data;
	int rslt;

	if(get_power_mode(&lst_pwrmode)) {
		if(lst_pwrmode != BMP5_POWERMODE_STANDBY) {
			reg_data = _interface->get_reg(BMP5_REG_ODR_CONFIG_ADDR);
			reg_data = BMP5_SET_BITSLICE(reg_data, BMP5_DEEP_DISABLE, BMP5_DEEP_DISABLED);
			reg_data = BMP5_SET_BITS_POS_0(reg_data, BMP5_POWERMODE, BMP5_POWERMODE_STANDBY);
			rslt = _interface->set_reg(reg_data, BMP5_REG_ODR_CONFIG_ADDR);
			if(rslt == OK) {
				usleep(BMP5_DELAY_US_STANDBY);
			}
		}

		switch (power_mode) {
			case BMP5_POWERMODE_DEEP_STANDBY:
				if(set_deep_standby_mode()) {
					result = true;
				}
				break;
			case BMP5_POWERMODE_STANDBY:
				result = true;
				break;
			case BMP5_POWERMODE_NORMAL:
			case BMP5_POWERMODE_FORCED:
			case BMP5_POWERMODE_CONTINOUS:
				reg_data = _interface->get_reg(BMP5_REG_ODR_CONFIG_ADDR);
				reg_data = BMP5_SET_BITSLICE(reg_data, BMP5_DEEP_DISABLE, BMP5_DEEP_DISABLED);
				reg_data = BMP5_SET_BITS_POS_0(reg_data, BMP5_POWERMODE, power_mode);
				rslt = _interface->set_reg(reg_data, BMP5_REG_ODR_CONFIG_ADDR);
				if(rslt == OK) {
					result = true;
				}
				break;
		}
	}

	return result;
}

/*!
 * @brief This internal API is used to set sensor in deep standby mode.
 */
bool BMP581::set_deep_standby_mode()
{
	bool resul = false;
	uint8_t reg_data;
	int rslt;

	reg_data = _interface->get_reg(BMP5_REG_ODR_CONFIG_ADDR);
	reg_data = BMP5_SET_BIT_VAL_0(reg_data, BMP5_DEEP_DISABLE);
	reg_data = BMP5_SET_BITSLICE(reg_data, BMP5_ODR, BMP5_ODR_01_HZ);
	rslt = _interface->set_reg(reg_data, BMP5_REG_ODR_CONFIG_ADDR);

	if(rslt == OK) {
		reg_data = _interface->get_reg(BMP5_REG_DSP_IIR_ADDR);
		reg_data = reg_data & BMP5_IIR_BYPASS;
		rslt = _interface->set_reg(reg_data, BMP5_REG_DSP_IIR_ADDR);
	}

	if(rslt == OK) {
		reg_data = _interface->get_reg(BMP5_REG_FIFO_SEL_ADDR);
		reg_data = BMP5_SET_BIT_VAL_0(reg_data, BMP5_FIFO_FRAME_SEL);
		rslt = _interface->set_reg(reg_data, BMP5_REG_FIFO_SEL_ADDR);
	}

	if(rslt == OK) {
		resul = true;
	}

	return resul;
}

/*!
 * @brief This internal API is used to set sensor in standby powermode when powermode is deepstandby mode.
 */
bool BMP581::set_standby_mode()
{
	bool result = false;
	uint8_t powermode;

	if(get_power_mode(&powermode)) {
		if(powermode == BMP5_POWERMODE_DEEP_STANDBY) {
			if(set_power_mode(BMP5_POWERMODE_STANDBY)) {
			}
		}
		result = true;
	}

	return result;
}

uint32_t BMP581::get_measurement_time()
{
	/*
	  From BST-BMP3581-DS004-04.pdf, page 19, table 9

	  Pressure      Temperature    Measurement
	  Oversample    Oversample     Time
	  1x            1x             2.7 ms
	  2x            1x             3.3 ms
	  4x            1x             4.6 ms
	  8x            1x             7.2 ms
	  16x           1x             12.5 ms
	  32x           2x             23.3 ms
	  64x           4x	       44.2 ms
	  128x          8x             88.0 ms
	*/

	uint32_t meas_time_us = 0; // unsupported value by default

	if (osr_t == BMP5_OVERSAMPLING_1X) {
		switch (osr_p) {
		case BMP5_OVERSAMPLING_1X:
			meas_time_us = 2700;
			break;

		case BMP5_OVERSAMPLING_2X:
			meas_time_us = 3300;
			break;

		case BMP5_OVERSAMPLING_4X:
			meas_time_us = 4600;
			break;

		case BMP5_OVERSAMPLING_8X:
			meas_time_us = 7200;
			break;

		case BMP5_OVERSAMPLING_16X:
			meas_time_us = 12500;
			break;
		}

	} else if (osr_t == BMP5_OVERSAMPLING_2X) {
		switch (osr_p) {
		case BMP5_OVERSAMPLING_32X:
			meas_time_us = 23300;
			break;

		}

	} else if(osr_t == BMP5_OVERSAMPLING_4X) {
		switch (osr_p) {
		case BMP5_OVERSAMPLING_64X:
			meas_time_us = 44200;
			break;

		}

	} else if(osr_t == BMP5_OVERSAMPLING_8X) {
		switch (osr_p) {
		case BMP5_OVERSAMPLING_128X:
			meas_time_us = 88000;
			break;

		}
	}

	return meas_time_us;
}

/*!
 *  @brief This API sets the configuration for oversampling temperature, oversampling of
 *  pressure and ODR configuration along with pressure enable.
 *
 *  @note If ODR is set to a value higher than 5Hz then powermode is set as standby mode, as ODR value greater than 5HZ
 *  without disabling deep-standby mode makes powermode invalid.
 */
bool BMP581::set_osr_odr_press_config()
{
	bool result = false;
	uint8_t reg_data[2] = {0};
	int rslt = 1;

	 _measure_interval = get_measurement_time();
	if (_measure_interval == 0) {
		PX4_WARN("unsupported oversampling selected");
		return false;
	}

	if(odr < BMP5_ODR_05_HZ) {
		rslt = set_standby_mode();
	}

	if(rslt) {
		rslt = _interface->get_reg_buf(BMP5_REG_OSR_CONFIG_ADDR, reg_data, 2);

		if(rslt == OK) {
			reg_data[0] = BMP5_SET_BITS_POS_0(reg_data[0], BMP5_TEMP_OS, osr_t);
			reg_data[0] = BMP5_SET_BITSLICE(reg_data[0], BMP5_PRESS_OS, osr_p);
			reg_data[0] = BMP5_SET_BITSLICE(reg_data[0], BMP5_PRESS_EN, press_en);
			reg_data[1] = BMP5_SET_BITSLICE(reg_data[1], BMP5_ODR, odr);

			rslt = _interface->set_reg(reg_data[0], BMP5_REG_OSR_CONFIG_ADDR);
			rslt = _interface->set_reg(reg_data[1], BMP5_REG_ODR_CONFIG_ADDR);
			if(rslt == OK) {
				result = true;
			}
 		}
	}

	return result;
}

/*!
 *  @brief This API sets the configuration for IIR of temperature and pressure.
 *
 *  @note If IIR value for both temperature and pressure is set a value other than bypass then powermode is set
 *  as standby mode, as IIR with value other than bypass without disabling deep-standby mode makes powermode invalid.
 */
bool BMP581::set_iir_config()
{
	bool result = false;
	uint8_t curr_powermdoe;
	uint8_t reg_data[2];
	int rslt = 1;

	if((iir_t != BMP5_IIR_FILTER_BYPASS) || (iir_p != BMP5_IIR_FILTER_BYPASS)) {
		rslt = set_standby_mode();
	}

	if(rslt) {
		rslt = get_power_mode(&curr_powermdoe);

		if(rslt) {
			if(curr_powermdoe != BMP5_POWERMODE_STANDBY) {
				rslt = set_power_mode(BMP5_POWERMODE_STANDBY);
			}

			if(rslt) {
				rslt = _interface->get_reg_buf(BMP5_REG_DSP_CONFIG_ADDR, reg_data, 2);
				if(rslt == OK) {
					reg_data[0] = BMP5_SET_BITSLICE(reg_data[0], BMP5_SHDW_SET_IIR_TEMP, BMP5_ENABLE);
					reg_data[0] = BMP5_SET_BITSLICE(reg_data[0], BMP5_SHDW_SET_IIR_PRESS, BMP5_ENABLE);
					reg_data[0] = BMP5_SET_BITSLICE(reg_data[0], BMP5_IIR_FLUSH_FORCED_EN, BMP5_ENABLE);

					reg_data[1] = iir_t;
					reg_data[1] = BMP5_SET_BITSLICE(reg_data[1], BMP5_SET_IIR_PRESS, iir_p);

					rslt = _interface->set_reg(reg_data[0], BMP5_REG_DSP_CONFIG_ADDR);
					rslt = _interface->set_reg(reg_data[1], BMP5_REG_DSP_IIR_ADDR);
				}
			}

			if(rslt == OK) {
                    		/* Since IIR works only in standby mode we are not re-writing to deepstandby mode
                    		* as deep standby mode resets the IIR settings to default
                    		*/
                    		if ((curr_powermdoe != BMP5_POWERMODE_STANDBY) && (curr_powermdoe != BMP5_POWERMODE_DEEP_STANDBY))
                    		{
                        		if(set_power_mode(curr_powermdoe)) {
						result = true;
					}

                    		}
			}

			if(rslt == OK) {
				result = true;
			}
		}
	}

	return result;
}

/*!
* @brief This API is used to configure the interrupt settings.
*/
bool BMP581::configure_interrupt()
{
	bool result = false;
	uint8_t reg_data = 0;
	uint8_t int_source = 0;
	int rslt;

	reg_data = _interface->get_reg(BMP5_REG_INT_CONFIG_ADDR);

	/* Any change between latched/pulsed mode has to be applied while interrupt is disabled */
        /* Step 1 : Turn off all INT sources (INT_SOURCE -> 0x00) */
	rslt = _interface->set_reg(int_source, BMP5_REG_INT_SOURCE_ADDR);

	if(rslt == OK) {
		/* Step 2 : Read the INT_STATUS register to clear the status */
		_interface->get_reg(BMP5_REG_INT_STATUS_ADDR);

		/* Step 3 : Set the desired mode in INT_CONFIG.int_mode */
		reg_data = BMP5_SET_BITS_POS_0(reg_data, BMP5_INT_MODE, intr_mode);
		reg_data = BMP5_SET_BITSLICE(reg_data, BMP5_INT_POL, intr_polarity);
		reg_data = BMP5_SET_BITSLICE(reg_data, BMP5_INT_OD, intr_drive);
		reg_data = BMP5_SET_BITSLICE(reg_data, BMP5_INT_EN, BMP5_ENABLE);

		rslt = _interface->set_reg(reg_data, BMP5_REG_INT_CONFIG_ADDR);
	}


	if(rslt == OK) {
		result = true;
	}

	return result;
}

bool BMP581::int_source_select()
{
	bool result = false;
	uint8_t reg_data;
	int rslt;

	reg_data = _interface->get_reg(BMP5_REG_INT_SOURCE_ADDR);

	reg_data = BMP5_SET_BITS_POS_0(reg_data, BMP5_INT_DRDY_EN, BMP5_ENABLE);
	reg_data = BMP5_SET_BITSLICE(reg_data, BMP5_INT_FIFO_FULL_EN, BMP5_DISABLE);
	reg_data = BMP5_SET_BITSLICE(reg_data, BMP5_INT_FIFO_THRES_EN, BMP5_DISABLE);
	reg_data = BMP5_SET_BITSLICE(reg_data, BMP5_INT_OOR_PRESS_EN, BMP5_DISABLE);

	rslt = _interface->set_reg(reg_data, BMP5_REG_INT_SOURCE_ADDR);

	if(rslt == OK) {
		result = true;
	}

	return result;
}

/*!
 *  @brief This API reads the temperature(deg C) or both pressure(Pa) and temperature(deg C) data from the
 * sensor and store it in the bmp5_sensor_data structure instance passed by the user.
 */
bool BMP581::get_sensor_data(bmp5_sensor_data *sensor_data)
{
	uint8_t reg_data[6] ={0};
	int32_t raw_data_t;
	uint32_t raw_data_p;
	int8_t rslt;

	rslt = _interface->get_reg_buf(BMP5_REG_TEMP_DATA_XLSB_ADDR, reg_data, 6);

	if(rslt == OK) {
		raw_data_t = (int32_t) ((int32_t) ((uint32_t)(((uint32_t)reg_data[2] << 16) | ((uint16_t)reg_data[1] << 8) | reg_data[0]) << 8) >> 8);
		/* Division by 2^16(whose equivalent value is 65536) is performed to get temperature data in deg C */
		sensor_data->temperature = (float)(raw_data_t / 65536.0);

		if(press_en == BMP5_ENABLE) {
            		raw_data_p = (uint32_t)((uint32_t)(reg_data[5] << 16) | (uint16_t)(reg_data[4] << 8) | reg_data[3]);
			/* Division by 2^6(whose equivalent value is 64) is performed to get pressure data in Pa */
			sensor_data->pressure = (float)(raw_data_p / 64.0);
		}
		else {
			sensor_data->pressure = 0.0;
		}

		rslt = true;
	}

	return rslt;
}
