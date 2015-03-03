/****************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
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
 * @file mc_mixer.cpp
 * Dummy multicopter mixer for euroc simulator (gazebo)
 *
 * @author Roman Bapst <romanbapst@yahoo.de>
*/
#include <ros/ros.h>
#include <px4.h>
#include <lib/mathlib/mathlib.h>
#include <mav_msgs/MotorSpeed.h>
#include <string>

class MultirotorMixer
{
public:

	MultirotorMixer();

	struct Rotor {
		float roll_scale;
		float pitch_scale;
		float yaw_scale;
	};

	void actuatorControlsCallback(const px4::actuator_controls_0 &msg);
	void actuatorArmedCallback(const px4::actuator_armed &msg);

private:

	ros::NodeHandle _n;
	ros::Subscriber _sub;
	ros::Publisher _pub;

	const Rotor *_rotors;

	unsigned _rotor_count;

	struct {
		float control[8];
	} inputs;

	struct  {
		float control[8];
	} outputs;

	bool _armed;
	ros::Subscriber _sub_actuator_armed;

	void mix();
};

const MultirotorMixer::Rotor _config_x[] = {
	{ -0.707107,  0.707107,  1.00 },
	{  0.707107, -0.707107,  1.00 },
	{  0.707107,  0.707107, -1.00 },
	{ -0.707107, -0.707107, -1.00 },
};

const MultirotorMixer::Rotor _config_quad_plus[] = {
	{ -1.000000,  0.000000,  1.00 },
	{  1.000000,  0.000000,  1.00 },
	{  0.000000,  1.000000, -1.00 },
	{ -0.000000, -1.000000, -1.00 },
};

const MultirotorMixer::Rotor _config_quad_plus_euroc[] = {
	{  0.000000,  1.000000,  1.00 },
	{ -0.000000, -1.000000,  1.00 },
	{  1.000000,  0.000000, -1.00 },
	{ -1.000000,  0.000000, -1.00 },
};
const MultirotorMixer::Rotor _config_quad_wide[] = {
	{ -0.927184,  0.374607,  1.000000 },
	{  0.777146, -0.629320,  1.000000 },
	{  0.927184,  0.374607, -1.000000 },
	{ -0.777146, -0.629320, -1.000000 },
};
const MultirotorMixer::Rotor _config_quad_iris[] = {
	{ -0.876559,  0.481295,  1.000000 },
	{  0.826590, -0.562805,  1.000000 },
	{  0.876559,  0.481295, -1.000000 },
	{ -0.826590, -0.562805, -1.000000 },
};

const MultirotorMixer::Rotor *_config_index[5] = {
	&_config_x[0],
	&_config_quad_plus[0],
	&_config_quad_plus_euroc[0],
	&_config_quad_wide[0],
	&_config_quad_iris[0]
};

MultirotorMixer::MultirotorMixer():
	_n(),
	_rotor_count(4),
	_rotors(_config_index[0])
{
	_sub = _n.subscribe("actuator_controls_0", 1, &MultirotorMixer::actuatorControlsCallback, this);
	_pub = _n.advertise<mav_msgs::MotorSpeed>("/mixed_motor_commands", 10);

	if (!_n.hasParam("motor_scaling_radps")) {
		_n.setParam("motor_scaling_radps", 150.0);
	}

	if (!_n.hasParam("motor_offset_radps")) {
		_n.setParam("motor_offset_radps", 600.0);
	}

	if (!_n.hasParam("mixer")) {
		_n.setParam("mixer", "x");
	}

	_sub_actuator_armed = _n.subscribe("actuator_armed", 1, &MultirotorMixer::actuatorArmedCallback, this);
}

void MultirotorMixer::mix()
{
	float roll = math::constrain(inputs.control[0], -1.0f, 1.0f);
	float pitch = math::constrain(inputs.control[1], -1.0f, 1.0f);
	float yaw = math::constrain(inputs.control[2], -1.0f, 1.0f);
	float thrust = math::constrain(inputs.control[3], 0.0f, 1.0f);
	float min_out = 0.0f;
	float max_out = 0.0f;

	/* perform initial mix pass yielding unbounded outputs, ignore yaw */
	for (unsigned i = 0; i < _rotor_count; i++) {
		float out = roll * _rotors[i].roll_scale
			    + pitch * _rotors[i].pitch_scale + thrust;

		/* limit yaw if it causes outputs clipping */
		if (out >= 0.0f && out < -yaw * _rotors[i].yaw_scale) {
			yaw = -out / _rotors[i].yaw_scale;
		}

		/* calculate min and max output values */
		if (out < min_out) {
			min_out = out;
		}

		if (out > max_out) {
			max_out = out;
		}

		outputs.control[i] = out;
	}

	/* scale down roll/pitch controls if some outputs are negative, don't add yaw, keep total thrust */
	if (min_out < 0.0f) {
		float scale_in = thrust / (thrust - min_out);

		/* mix again with adjusted controls */
		for (unsigned i = 0; i < _rotor_count; i++) {
			outputs.control[i] = scale_in
					     * (roll * _rotors[i].roll_scale
						+ pitch * _rotors[i].pitch_scale) + thrust;
		}

	} else {
		/* roll/pitch mixed without limiting, add yaw control */
		for (unsigned i = 0; i < _rotor_count; i++) {
			outputs.control[i] += yaw * _rotors[i].yaw_scale;
		}
	}

	/* scale down all outputs if some outputs are too large, reduce total thrust */
	float scale_out;

	if (max_out > 1.0f) {
		scale_out = 1.0f / max_out;

	} else {
		scale_out = 1.0f;
	}

	/* scale outputs to range _idle_speed..1, and do final limiting */
	for (unsigned i = 0; i < _rotor_count; i++) {
		outputs.control[i] = math::constrain(outputs.control[i], 0.0f, 1.0f);
	}
}

void MultirotorMixer::actuatorControlsCallback(const px4::actuator_controls_0 &msg)
{
	// read message
	for (int i = 0; i < msg.NUM_ACTUATOR_CONTROLS; i++) {
		inputs.control[i] = msg.control[i];
	}

	// switch mixer if necessary
	std::string mixer_name;
	_n.getParamCached("mixer", mixer_name);
	if (mixer_name == "x") {
		_rotors = _config_index[0];
	} else if (mixer_name == "+") {
		_rotors = _config_index[1];
	} else if (mixer_name == "e") {
		_rotors = _config_index[2];
	} else if (mixer_name == "w") {
		_rotors = _config_index[3];
	} else if (mixer_name == "i") {
		_rotors = _config_index[4];
	}

	// mix
	mix();

	// publish message
	mav_msgs::MotorSpeed rotor_vel_msg;
	double scaling;
	double offset;
	_n.getParamCached("motor_scaling_radps", scaling);
	_n.getParamCached("motor_offset_radps", offset);

	if (_armed) {
		for (int i = 0; i < _rotor_count; i++) {
			rotor_vel_msg.motor_speed.push_back(outputs.control[i] * scaling + offset);
		}

	} else {
		for (int i = 0; i < _rotor_count; i++) {
			rotor_vel_msg.motor_speed.push_back(0.0);
		}
	}

	_pub.publish(rotor_vel_msg);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mc_mixer");
	MultirotorMixer mixer;
	ros::spin();

	return 0;
}

void MultirotorMixer::actuatorArmedCallback(const px4::actuator_armed &msg)
{
	_armed = msg.armed;
}
