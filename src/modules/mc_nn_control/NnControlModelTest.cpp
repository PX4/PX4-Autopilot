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

/**
 * @file NnControlModelTest.cpp
 * Loads the shipped network through the real interpreter and runs the layout
 * check the module runs at start on it, so a swapped model array that the
 * module cannot feed fails here before it fails on a vehicle.
 *
 * to run, on a config that enables the module:
 *   cmake -DCMAKE_TESTING=ON build/px4_sitl_neural
 *   ninja -C build/px4_sitl_neural unit-NnControlModel
 */

#include <gtest/gtest.h>

#include <tflite_micro/tensorflow/lite/micro/micro_interpreter.h>
#include <tflite_micro/tensorflow/lite/schema/schema_generated.h>

#include "control_net.hpp"
#include "nn_control_checks.hpp"
#include "nn_control_model.hpp"

static int element_count(const TfLiteTensor *tensor)
{
	int count = 1;

	for (int i = 0; i < tensor->dims->size; i++) {
		count *= tensor->dims->data[i];
	}

	return count;
}

TEST(NnControlModelTest, theShippedModelPassesTheLayoutCheck)
{
	const tflite::Model *model = ::tflite::GetModel(control_net_tflite);
	ASSERT_NE(model, nullptr);
	EXPECT_EQ(model->version(), TFLITE_SCHEMA_VERSION);

	static nn_control::OpResolver resolver;
	ASSERT_EQ(nn_control::register_ops(resolver), kTfLiteOk);

	constexpr int kArenaSize = 10 * 1024;
	static uint8_t arena[kArenaSize];
	tflite::MicroInterpreter interpreter(model, resolver, arena, kArenaSize);
	ASSERT_EQ(interpreter.AllocateTensors(), kTfLiteOk);

	const TfLiteTensor *input = interpreter.input(0);
	const TfLiteTensor *output = interpreter.output(0);
	ASSERT_NE(input, nullptr);
	ASSERT_NE(output, nullptr);

	EXPECT_EQ(interpreter.inputs_size(), 1u);
	EXPECT_EQ(interpreter.outputs_size(), 1u);
	EXPECT_EQ(element_count(input), nn_control::kInputSize);
	EXPECT_EQ(element_count(output), nn_control::kOutputSize);

	const char *problem = nn_control::model_layout_problem(model->version(), TFLITE_SCHEMA_VERSION,
			      (int)interpreter.inputs_size(), element_count(input), input->type == kTfLiteFloat32,
			      (int)interpreter.outputs_size(), element_count(output), output->type == kTfLiteFloat32);
	EXPECT_EQ(problem, nullptr) << problem;
}
