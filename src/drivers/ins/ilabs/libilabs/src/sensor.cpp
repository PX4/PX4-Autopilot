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

#include "sensor.h"

#include <px4_platform_common/log.h>
#include <px4_platform_common/time.h>

#include "data.h"

#ifndef MODULE_NAME
#define MODULE_NAME "ilabs_ins_driver"  // NOLINT(cppcoreguidelines-macro-usage)
#endif

namespace {

constexpr int BAUDRATE = 921600;

// acceleration due to gravity in m/s^2 used in IL INS
constexpr float GRAVITY_MSS = 9.8106f;

bool isMessageHeaderCorrect(const InertialLabs::MessageHeader *header) {
	if (!header) {
		PX4_ERR("Invalid header pointer");
		return false;
	}

	return header->packageHeader == 0x55AA && header->msgType == 1 && header->msgId == 0x95 &&
		(header->msgLen + 2) <= InertialLabs::BUFFER_SIZE;  //< 2 is size of PackageHeader, that not included in msgLen
}

// sums the bytes in the supplied buffer, returns that sum mod 0xFFFF
uint16_t checksum(const uint8_t *data, uint16_t len) {
	uint16_t sum = 0;
	for (uint32_t i = 0; i < len; i++) {
		sum += data[i];
	}
	return sum;
}

uint16_t readPackageChecksum(const uint8_t *checksumByte) {
	return (checksumByte[1] << 8) | checksumByte[0];
}

// Convert from right-front-up to front-right-down or ENU to NED
matrix::Vector3f rfuToFrd(const matrix::Vector3f &vector3f) {
	return matrix::Vector3f{vector3f(1), vector3f(0), -vector3f(2)};
}

}  // namespace

namespace InertialLabs {

Sensor::~Sensor() {
	deinit();
}

bool Sensor::init(const char *serialDeviceName, void *context, DataHandler dataHandler) {
	if (_isInitialized.load()) {
		PX4_WARN("Serial device already initialized. Deinitialize first");
		return false;
	}
	if (_isDeinitInProcess.load()) {
		PX4_WARN("Deinitialization is in progress. Please wait until it completes before re-initializing");
		return false;
	}
	if (serialDeviceName[0] == '\0') {
		PX4_ERR("Empty serial device name");
		return false;
	}
	if (!context || !dataHandler) {
		PX4_ERR("Empty data handler callback");
		return false;
	}

	if (!initSerialPort(serialDeviceName)) {
		return false;
	}

	_context     = context;
	_dataHandler = dataHandler;

	_processInThread.store(true);
	const int result = pthread_create(&_threadId, nullptr, &Sensor::updateDataThreadHelper, this);
	if (result) {
		PX4_ERR("Cant create working thread");
		deinit();
		return false;
	}
	pthread_detach(_threadId);

	_isInitialized.store(true);
	return true;
}

void Sensor::deinit() {
	if (_isDeinitInProcess.load()) {
		PX4_WARN("Deinitialization already in process");
		return;
	}

	_isDeinitInProcess.store(true);
	_isInitialized.store(false);
	_processInThread.store(false);
	// Wait to be shure, that operations with UART are finished
	px4_sleep(1); // NOLINT(concurrency-mt-unsafe)
	if (_serial) {
		(void)_serial->close();
		delete _serial;
		_serial = nullptr;
	}

	_context     = nullptr;
	_dataHandler = nullptr;
	_bufOffset   = 0;
	_isDeinitInProcess.store(false);
}

bool Sensor::isInitialized() const {
	return _isInitialized.load();
}

void Sensor::updateData() {
	while (_processInThread.load()) {
		bool result = moveValidPackageToBufferStart();
		if (!result) {
			continue;
		}

		result = parseUDDPayload();
		if (!result) {
			continue;
		}

		if (_dataHandler) {
			_dataHandler(_context, &_sensorData);
		}

		skipPackageInBufferStart();
	}
}

bool Sensor::initSerialPort(const char *serialDeviceName) {
	if (_serial) {
		PX4_WARN("Serial port already initialized. Deinitialize first");
		return false;
	}

	_serial = new device::Serial(serialDeviceName, BAUDRATE);
	if (_serial == nullptr) {
		PX4_ERR("Error creating serial device: %s", serialDeviceName);
		px4_sleep(1); // NOLINT(concurrency-mt-unsafe)
		return false;
	}

	if (!_serial->isOpen()) {
		_serial->open();
		if (!_serial->isOpen()) {
			PX4_ERR("Error opening serial device: %s", serialDeviceName);
			deinit();
			return false;
		}
		PX4_INFO("Serial device opened sucessfully: %s", serialDeviceName);
	} else {
		PX4_INFO("Serial device already opened: %s", serialDeviceName);
	}
	_serial->flush();
	return true;
}

bool Sensor::moveToBufferStart(const uint8_t *pos) {
	if (!pos) {
		PX4_ERR("Invalid position pointer");
		return false;
	}

	const uint16_t bytesFromBufferStart = pos - _buf;
	if (bytesFromBufferStart > BUFFER_SIZE) {
		PX4_ERR("Invalid position pointer");
		return false;
	}

	if (_bufOffset < bytesFromBufferStart) {
		PX4_ERR("Buffer offset less than bytes diff need to move");
		return false;
	}

	memmove(_buf, pos, _bufOffset - bytesFromBufferStart);
	_bufOffset -= bytesFromBufferStart;
	return true;
}

bool Sensor::skipPackageInBufferStart() {
	const MessageHeader *packageHeader = reinterpret_cast<MessageHeader *>(_buf);
	if (!isMessageHeaderCorrect(packageHeader)) {
		PX4_ERR("Incorrect message header in buffer start. Can't skip the package correctly");
		return false;
	}

	const uint8_t *endPackagePos = &_buf[packageHeader->msgLen + 2];

	return moveToBufferStart(endPackagePos);
}

bool Sensor::movePackageHeaderToBufferStart() {
	if (_bufOffset < 3) {
		PX4_WARN("Buffer offset is too small for the package");
		_bufOffset = 0;
		return false;
	}

	const uint16_t packageHeader   = 0x55AA;
	const uint8_t *startPackagePos = (const uint8_t *)memmem(&_buf[1],
								 _bufOffset - sizeof(packageHeader),
			 					 &packageHeader,
								 sizeof(packageHeader));
	if (!startPackagePos) {
		PX4_WARN("Can't find the package start in the buffer");
		_bufOffset = 0;
		return false;
	}

	return moveToBufferStart(startPackagePos);
}

bool Sensor::moveValidPackageToBufferStart() {
	const uint16_t freeBufByteCount = BUFFER_SIZE - _bufOffset;
	if (freeBufByteCount < sizeof(MessageHeader)) {
		_bufOffset = 0;
		PX4_DEBUG("Buffer free space not enough for the message header. Buffer was cleaned");
		return false;
	}

	const ssize_t nRead = _serial->readAtLeast(&_buf[_bufOffset], freeBufByteCount, freeBufByteCount, 500);
	if (nRead != freeBufByteCount) {
		PX4_ERR("Can't read requested data from the serial device. Bytes requested: %d. Bytes read: %d",
				freeBufByteCount, static_cast<int>(nRead));
		_bufOffset = 0;
		return false;
	}

	_bufOffset += nRead;

	if (_bufOffset > BUFFER_SIZE) {
		PX4_ERR("Buffer read bytes: %d. Buffer size: %d. Offset will be reseted", _bufOffset, BUFFER_SIZE);
		_bufOffset = 0;
		return false;
	}

	const MessageHeader *packageHeader = reinterpret_cast<MessageHeader *>(_buf);
	if (!isMessageHeaderCorrect(packageHeader)) {
		PX4_DEBUG("Message header in the buffer start is incorrect");
		movePackageHeaderToBufferStart();
		return false;
	}

	const uint16_t fullMessageLength = packageHeader->msgLen + 2;
	if (fullMessageLength > _bufOffset) {
		_bufOffset = 0;
		PX4_ERR(
			"The message was not read in full. This situation is not healthy. Is the buffer size enough?\n"
			"Message was removed and buffer offset was reseted");
		return false;
	}

	// Check checksum
	const uint16_t calculatedChecksum = checksum(&_buf[2], packageHeader->msgLen - 2);
	const uint16_t readChecksum       = readPackageChecksum(&_buf[fullMessageLength - 2]);
	if (calculatedChecksum != readChecksum) {
		PX4_ERR("Invalid package checksum. Calculated checksum: %d. Read checksum: %d", calculatedChecksum,
				readChecksum);
		moveToBufferStart(&_buf[fullMessageLength]);
		return false;
	}

	return true;
}

bool Sensor::parseUDDPayload() {
	const MessageHeader *packageHeader = reinterpret_cast<MessageHeader *>(_buf);

	if (!isMessageHeaderCorrect(packageHeader)) {
		PX4_ERR("Package header in buffer start is incorrect");
		return false;
	}

	const uint16_t payloadSize = packageHeader->msgLen - 6;
	const uint8_t *payload     = &_buf[6];

	const uint8_t messageCount = payload[0];
	if (messageCount == 0 || messageCount > payloadSize - 1) {
		PX4_ERR("Invalid data package. Number of messages or messages data are incorrect");
		movePackageHeaderToBufferStart();
		return false;
	}
	// 1 byle for message count + byte list of messages types
	const uint8_t *messageDataOffset = &payload[1 + messageCount];

	for (uint8_t i = 0; i < messageCount; i++) {
		uint8_t         messageLength = 0;
		UDDMessageData &udd           = *(UDDMessageData *)messageDataOffset;
		uint8_t         messageType   = payload[1 + i];

		switch (messageType) {
			case DataType::GPS_INS_TIME_MS: {
				// this is the GPS tow timestamp in ms for when the IMU data was sampled
				_sensorData.ins.msTow = udd.gpsTimeMs;
				messageLength         = sizeof(udd.gpsTimeMs);
				break;
			}
			case DataType::GPS_WEEK: {
				_sensorData.gps.gpsWeek = udd.gpsWeek;
				messageLength           = sizeof(udd.gpsWeek);
				break;
			}
			case DataType::ACCEL_DATA_HR: {
				_sensorData.accel =
					rfuToFrd(udd.accelDataHr.toFloat()) * GRAVITY_MSS * 1.0e-6f;  // NED, in m/s^2
				messageLength = sizeof(udd.accelDataHr);
				break;
			}
			case DataType::GYRO_DATA_HR: {
				_sensorData.gyro =
					rfuToFrd(udd.gyroDataHr.toFloat()) * M_DEG_TO_RAD * 1.0e-5f;  // NED, in rad/s
				messageLength = sizeof(udd.gyroDataHr);
				break;
			}
			case DataType::BARO_DATA: {
				_sensorData.pressure    = static_cast<float>(udd.baroData.pressurePa2 * 2);  // Pa
				_sensorData.ins.baroAlt = static_cast<float>(udd.baroData.baroAlt) * 0.01f;  // m
				messageLength           = sizeof(udd.baroData);
				break;
			}
			case DataType::MAG_DATA: {
				_sensorData.mag =
					rfuToFrd(udd.magData.toFloat()) * 1.0e-6f;  // NED, in Gauss
				messageLength = sizeof(udd.magData);
				break;
			}
			case DataType::SENSOR_BIAS: {
				_sensorData.ins.sensorBias = udd.sensorBias;
				messageLength              = sizeof(udd.sensorBias);
				break;
			}
			case DataType::ORIENTATION_ANGLES: {
				_sensorData.ins.yaw   = static_cast<float>(udd.orientationAngles.yaw) * 0.01f;    // deg
				_sensorData.ins.pitch = static_cast<float>(udd.orientationAngles.pitch) * 0.01f;  // deg
				_sensorData.ins.roll  = static_cast<float>(udd.orientationAngles.roll) * 0.01f;   // deg
				messageLength         = sizeof(udd.orientationAngles);
				break;
			}
			case DataType::VELOCITIES: {
				_sensorData.ins.velocity = rfuToFrd(udd.velocity.toFloat()) * 0.01f;  // NED, in m/s
				messageLength            = sizeof(udd.velocity);
				break;
			}
			case DataType::POSITION: {
				_sensorData.ins.latitude  = static_cast<double>(udd.position.lat) * 1.0e-7;  // deg
				_sensorData.ins.longitude = static_cast<double>(udd.position.lon) * 1.0e-7;  // deg
				_sensorData.ins.altitude  = static_cast<float>(udd.position.alt) * 0.01f;    // m
				messageLength             = sizeof(udd.position);
				break;
			}
			case DataType::UNIT_STATUS: {
				_sensorData.ins.unitStatus = udd.unitStatus;
				messageLength              = sizeof(udd.unitStatus);
				break;
			}
			case DataType::GNSS_EXTENDED_INFO: {
				_sensorData.gps.fixType        = udd.gnssExtendedInfo.fixType;
				_sensorData.gps.spoofingStatus = udd.gnssExtendedInfo.spoofingStatus;
				messageLength                  = sizeof(udd.gnssExtendedInfo);
				break;
			}
			case DataType::GNSS_POSITION: {
				_sensorData.gps.latitude  = static_cast<double>(udd.gnssPosition.lat) * 1.0e-7;  // deg
				_sensorData.gps.longitude = static_cast<double>(udd.gnssPosition.lon) * 1.0e-7;  // deg
				_sensorData.gps.altitude  = static_cast<float>(udd.gnssPosition.alt) * 0.01f;    // m
				messageLength             = sizeof(udd.gnssPosition);
				break;
			}
			case DataType::GNSS_VEL_TRACK: {
				_sensorData.gps.horSpeed =
					static_cast<float>(udd.gnssVelTrack.horSpeed) * 0.01f;  // m/s
				_sensorData.gps.trackOverGround =
					static_cast<float>(udd.gnssVelTrack.trackOverGround) * 0.01f;  // deg
				_sensorData.gps.verSpeed =
					static_cast<float>(udd.gnssVelTrack.verSpeed) * 0.01f;  // m/s
				messageLength = sizeof(udd.gnssVelTrack);
				break;
			}
			case DataType::GNSS_POS_TIMESTAMP: {
				_sensorData.gps.msTow = udd.gnssPosTimestamp;
				messageLength         = sizeof(udd.gnssPosTimestamp);
				break;
			}
			case DataType::GNSS_NEW_DATA: {
				_sensorData.gps.newData = udd.gnssNewData;
				messageLength           = sizeof(udd.gnssNewData);
				break;
			}
			case DataType::GNSS_JAM_STATUS: {
				_sensorData.gps.jamStatus = udd.gnssJamStatus;
				messageLength             = sizeof(udd.gnssJamStatus);
				break;
			}
			case DataType::DIFFERENTIAL_PRESSURE: {
				_sensorData.differentialPressure =
					static_cast<float>(udd.differentialPressure) * 0.01f;  // Pa
				messageLength = sizeof(udd.differentialPressure);
				break;
			}
			case DataType::TRUE_AIRSPEED: {
				_sensorData.ins.trueAirspeed = static_cast<float>(udd.trueAirspeed) * 0.01f;  // m/s
				messageLength                = sizeof(udd.trueAirspeed);
				break;
			}
			case DataType::CALIBRATED_AIRSPEED: {
				_sensorData.ins.calibratedAirspeed =
					static_cast<float>(udd.calibratedAirspeed) * 0.01f;  // m/s
				messageLength = sizeof(udd.calibratedAirspeed);
				break;
			}
			case DataType::WIND_SPEED: {
				_sensorData.ins.airspeedSf = (udd.windSpeed.toFloat()(2)) * 1.0e-3f;
				_sensorData.ins.windSpeed  = rfuToFrd(udd.windSpeed.toFloat()) * 0.01f;  // NED, in m/s
				_sensorData.ins.windSpeed(2) = 0.0f;
				messageLength                = sizeof(udd.windSpeed);
				break;
			}
			case DataType::AIR_DATA_STATUS: {
				_sensorData.ins.airDataStatus = udd.airDataStatus;
				messageLength                 = sizeof(udd.airDataStatus);
				break;
			}
			case DataType::SUPPLY_VOLTAGE: {
				_sensorData.supplyVoltage = static_cast<float>(udd.supplyVoltage) * 0.01f;  // V
				messageLength             = sizeof(udd.supplyVoltage);
				break;
			}
			case DataType::TEMPERATURE: {
				_sensorData.temperature = static_cast<float>(udd.temperature) * 0.1f;  // degC
				messageLength           = sizeof(udd.temperature);
				break;
			}
			case DataType::UNIT_STATUS2: {
				_sensorData.ins.unitStatus2 = udd.unitStatus2;
				messageLength               = sizeof(udd.unitStatus2);
				break;
			}
			case DataType::GNSS_DOP: {
				_sensorData.gps.dop.gdop = udd.gnssDop.gdop;
				_sensorData.gps.dop.pdop = udd.gnssDop.pdop;
				_sensorData.gps.dop.hdop = udd.gnssDop.hdop;
				_sensorData.gps.dop.vdop = udd.gnssDop.vdop;
				_sensorData.gps.dop.tdop = udd.gnssDop.tdop;
				messageLength            = sizeof(udd.gnssDop);
				break;
			}
			case DataType::INS_SOLUTION_STATUS: {
				_sensorData.ins.solutionStatus = udd.insSolStatus;
				messageLength                  = sizeof(udd.insSolStatus);
				break;
			}
			case DataType::INS_POS_VEL_ACCURACY: {
				_sensorData.ins.accuracy.lat      = udd.insAccuracy.lat;
				_sensorData.ins.accuracy.lon      = udd.insAccuracy.lon;
				_sensorData.ins.accuracy.alt      = udd.insAccuracy.alt;
				_sensorData.ins.accuracy.eastVel  = udd.insAccuracy.eastVel;
				_sensorData.ins.accuracy.northVel = udd.insAccuracy.northVel;
				_sensorData.ins.accuracy.verVel   = udd.insAccuracy.verVel;
				messageLength                     = sizeof(udd.insAccuracy);
				break;
			}
			case USED_SAT_COUNT: {
				_sensorData.gps.usedSatCount = udd.usedSatCount;
				messageLength                 = sizeof(udd.usedSatCount);
				break;
			}
			case DataType::GNSS_VEL_LATENCY: {
				_sensorData.gps.velLatency = udd.gnssVelLatency;
				messageLength              = sizeof(udd.gnssVelLatency);
				break;
			}
			case DataType::GNSS_SOL_STATUS: {
				_sensorData.gps.gnssSolStatus = udd.gnssSolStatus;
				messageLength                 = sizeof(udd.gnssSolStatus);
				break;
			}
			case DataType::GNSS_POS_VEL_TYPE: {
				_sensorData.gps.gnssPosVelType = udd.gnssPosVelType;
				messageLength                  = sizeof(udd.gnssPosVelType);
				break;
			}
			case DataType::NEW_AIDING_DATA: {
				_sensorData.ext.newAidingData = udd.newAidingData;
				messageLength                 = sizeof(udd.newAidingData);
				break;
			}
			case DataType::NEW_AIDING_DATA2: {
				_sensorData.ext.newAidingData2 = udd.newAidingData2;
				messageLength                  = sizeof(udd.newAidingData2);
				break;
			}
			case DataType::EXT_SPEED: {
				_sensorData.ext.speed = udd.externalSpeed;
				messageLength         = sizeof(udd.externalSpeed);
				break;
			}
			case DataType::EXT_HOR_POS: {
				_sensorData.ext.horPos = udd.extHorPos;
				messageLength          = sizeof(udd.extHorPos);
				break;
			}
			case DataType::EXT_ALT: {
				_sensorData.ext.altitudeData = udd.extAlt;
				messageLength                = sizeof(udd.extAlt);
				break;
			}
			case DataType::EXT_HEADING: {
				_sensorData.ext.headingData = udd.extHeading;
				messageLength               = sizeof(udd.extHeading);
				break;
			}
			case DataType::EXT_AMBIENT_DATA: {
				_sensorData.ext.ambientAirData = udd.extAmbientAirData;
				messageLength                  = sizeof(udd.extAmbientAirData);
				break;
			}
			case DataType::EXT_WIND_DATA: {
				_sensorData.ext.windData = udd.extWindData;
				messageLength            = sizeof(udd.extWindData);
				break;
			}
			case DataType::MAG_CLB_ACCURACY: {
				_sensorData.ins.magClbAccuracy = static_cast<float>(udd.magClbAccuracy) * 0.1f;  // deg
				messageLength                  = sizeof(udd.magClbAccuracy);
				break;
			}
			default: {
				PX4_ERR("Unknown message type: %d. Further package parsing result will be incorrect",
				        messageType);
				messageLength = 0;
				break;
			}
		}

		messageDataOffset += messageLength;
	}

	return true;
}

}  // namespace InertialLabs
