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

// Raw FIFO payload layouts (must match device FIFO packing)

#pragma pack(push, 1)

enum SensorType : uint8_t {
	SENSOR_TYPE_ACCELEROMETER        = 1,
	SENSOR_TYPE_MAGNETIC_FIELD       = 2,
	SENSOR_TYPE_ORIENTATION          = 3,
	SENSOR_TYPE_GYROSCOPE            = 4,
	SENSOR_TYPE_PRESSURE             = 6,
	SENSOR_TYPE_ROTATION_VECTOR      = 11,

	SENSOR_TYPE_ACCELEROMETER_WAKE   = 65,
	SENSOR_TYPE_MAGNETIC_FIELD_WAKE  = 66,
	SENSOR_TYPE_ORIENTATION_WAKE     = 67,
	SENSOR_TYPE_GYROSCOPE_WAKE       = 68,
	SENSOR_TYPE_ROTATION_VECTOR_WAKE = 75,
	SENSOR_TYPE_PRESSURE_WAKE        = 70,
};

struct PhysSensorStatus {
	uint16_t accel_sample_rate;
	uint16_t accel_dynamic_range;
	uint8_t  accel_flags;
	uint16_t gyro_sample_rate;
	uint16_t gyro_dynamic_range;
	uint8_t  gyro_flags;
	uint16_t mag_sample_rate;
	uint16_t mag_dynamic_range;
	uint8_t  mag_flags;
};

typedef uint64_t PhysSensorsPresent;

struct PhysSensorInfo {
	uint8_t  type;
	uint8_t  driver_id;
	uint8_t  driver_version;
	uint8_t  current;        // 0.1mA per lsb
	uint16_t current_range;  // dynamic range of sensor in SI units
	uint8_t  flags;
	uint8_t  reserved;
	uint16_t current_rate;   // sample rate in Hz
	uint8_t  num_axes;
	uint8_t  orient_matrix[5];
};

struct SensorInfo {
	uint8_t  type;
	uint8_t  driver_id;
	uint8_t  driver_version;
	uint8_t  power;          // 0.1mA per lsb
	uint16_t max_range;      // dynamic range of sensor in SI units
	uint16_t resolution;
	uint16_t max_rate;
	uint16_t fifo_reserved;  // sample rate in Hz
	uint16_t fifo_max;       // sample rate in Hz
	uint8_t  event_size;
	uint8_t  min_rate;
};

typedef float WarmStartCalScore;  // 0-100, 0 being most accurate calibration

struct SensorConfig {
	uint16_t sample_rate;
	uint16_t max_report_latency;
	uint16_t change_sensitivity;  // not implemented
	uint16_t dynamic_range;
};

struct SensorData3AxisRaw {
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t status;
};

struct RotationVectorRaw10 {
	int16_t x;
	int16_t y;
	int16_t z;
	int16_t w;
	uint16_t accuracy;
};

#pragma pack(pop)
