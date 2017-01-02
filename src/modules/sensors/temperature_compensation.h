/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
 * @file temperature_compensation.h
 *
 * Sensor correction methods
 *
 * @author Paul Riseborough <gncsolns@gmail.com>
 */

#include <systemlib/param/param.h>
#include <mathlib/mathlib.h>

/* Struct containing parameters used by the single axis 5th order temperature compensation algorithm

Input:

measured_temp : temperature measured at the sensor (deg C)
raw_value : reading from the sensor before compensation
corrected_value : reading from the sensor after compensation for errors

Compute:

delta_temp = measured_temp - ref_temp
offset = x5 * delta_temp^5 + x4 * delta_temp^4 + x3 * delta_temp^3 + x2 * delta_temp^2 + x1 * delta_temp + x0
corrected_value = raw_value * scale + offset

*/
namespace sensors_temp_comp
{

struct SENSOR_CAL_DATA_1D {
	int ID;
	float x5;
	float x4;
	float x3;
	float x2;
	float x1;
	float x0;
	float scale;
	float ref_temp;
	float min_temp;
	float max_temp;
};

struct SENSOR_CAL_HANDLES_1D {
	param_t ID;
	param_t x5;
	param_t x4;
	param_t x3;
	param_t x2;
	param_t x1;
	param_t x0;
	param_t scale;
	param_t ref_temp;
	param_t min_temp;
	param_t max_temp;
};

/* Struct containing parameters used by the 3-axis 3rd order temperature compensation algorithm

Input:

measured_temp : temperature measured at the sensor (deg C)
raw_value[3] : XYZ readings from the sensor before compensation
corrected_value[3] : XYZ readings from the sensor after compensation for errors

Compute for each measurement index:

delta_temp = measured_temp - ref_temp
offset = x3 * delta_temp^3 + x2 * delta_temp^2 + x1 * delta_temp + x0
corrected_value = raw_value * scale + offset

*/
struct SENSOR_CAL_DATA_3D {
	int ID;			/**< sensor device ID*/
	float x3[3];		/**< x^3 term of polynomial */
	float x2[3];		/**< x^2 term of polynomial */
	float x1[3];		/**< x^1 term of polynomial */
	float x0[3];		/**< x^0 / offset term of polynomial */
	float scale[3];		/**< scale factor correction */
	float ref_temp;		/**< reference temperature used by the curve-fit */
	float min_temp;		/**< minimum temperature with valid compensation data */
	float max_temp;		/**< maximum temperature with valid compensation data */
};

struct SENSOR_CAL_HANDLES_3D {
	param_t ID;
	param_t x3[3];
	param_t x2[3];
	param_t x1[3];
	param_t x0[3];
	param_t scale[3];
	param_t ref_temp;
	param_t min_temp;
	param_t max_temp;
};

// create a struct containing all thermal calibration parameters
struct Parameters {
	int gyro_tc_enable;
	SENSOR_CAL_DATA_3D gyro_cal_data[3];
	int accel_tc_enable;
	SENSOR_CAL_DATA_3D accel_cal_data[3];
	int baro_tc_enable;
	SENSOR_CAL_DATA_1D baro_cal_data;
};

// create a struct containing the handles required to access all calibration parameters
struct ParameterHandles {
	param_t gyro_tc_enable;
	SENSOR_CAL_HANDLES_3D gyro_cal_handles[3];
	param_t accel_tc_enable;
	SENSOR_CAL_HANDLES_3D accel_cal_handles[3];
	param_t baro_tc_enable;
	SENSOR_CAL_HANDLES_1D baro_cal_handles;
};

/**
 * initialize ParameterHandles struct
 * @return 0 on succes, <0 on error
 */
int initialize_parameter_handles(ParameterHandles &parameter_handles);


/**
 * Read out the parameters using the handles into the parameters struct.
 * @return 0 on succes, <0 on error
 */
int update_parameters(const ParameterHandles &parameter_handles, Parameters &parameters);

/*

Calculate the offset required to compensate the sensor for temperature effects using a 5th order method
If the measured temperature is outside the calibration range, clip the temperature to remain within the range and return false.
If the measured temperature is within the calibration range, return true.

Arguments:

coef : reference to struct containing calibration coefficients
measured_temp : temperature measured at the sensor (deg C)
offset : reference to sensor offset

Returns:

Boolean true if the measured temperature is inside the valid range for the compensation

*/
bool calc_thermal_offsets_1D(SENSOR_CAL_DATA_1D &coef, const float &measured_temp, float &offset);

/*

Calculate the offsets required to compensate the sensor for temperature effects
If the measured temperature is outside the calibration range, clip the temperature to remain within the range and return false.
If the measured temperature is within the calibration range, return true.

Arguments:

coef : reference to struct containing calibration coefficients
measured_temp : temperature measured at the sensor (deg C)
offset : reference to sensor offset - array of 3

Returns:

Boolean true if the measured temperature is inside the valid range for the compensation

*/
bool calc_thermal_offsets_3D(SENSOR_CAL_DATA_3D &coef, const float &measured_temp, float offset[]);

}
