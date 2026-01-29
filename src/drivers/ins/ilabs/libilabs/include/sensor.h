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

#pragma once

#include <pthread.h>

#include <px4_platform_common/atomic.h>
#include <px4_platform_common/Serial.hpp>

#include "data.h"

namespace InertialLabs {

constexpr uint16_t BUFFER_SIZE{512};

class Sensor {
public:
	// Use C-style function pointer, because we can't use std::function on some platforms
	using DataHandler = void (*)(void *, SensorsData *);

	Sensor()                          = default;
	Sensor(const Sensor &)            = delete;
	Sensor &operator=(const Sensor &) = delete;
	~Sensor();

	bool init(const char *serialDeviceName, void *context, DataHandler dataHandler);
	void deinit();
	bool isInitialized() const;

	void updateData();

private:
	static void *updateDataThreadHelper(void *context) {
		if (!context) {
			return nullptr;
		}
		Sensor *sensor = reinterpret_cast<Sensor *>(context);
		sensor->updateData();
		return nullptr;
	}

	bool initSerialPort(const char *serialDeviceName);
	bool moveToBufferStart(const uint8_t *pos);
	bool skipPackageInBufferStart();
	bool movePackageHeaderToBufferStart();
	bool moveValidPackageToBufferStart();

	bool parseUDDPayload();

	device::Serial  *_serial{nullptr};
	pthread_t        _threadId{0};
	px4::atomic_bool _processInThread{false};

	// callback. C-style class method pointer
	void       *_context{nullptr};
	DataHandler _dataHandler{nullptr};

	px4::atomic_bool _isInitialized{false};
	px4::atomic_bool _isDeinitInProcess{false};

	uint8_t  _buf[BUFFER_SIZE]{};
	uint16_t _bufOffset{0};

	SensorsData _sensorData{};
};

}  // namespace InertialLabs
