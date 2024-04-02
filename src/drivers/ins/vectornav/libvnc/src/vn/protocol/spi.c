#include "vn/protocol/spi.h"
#include <string.h>
#include "vn/util.h"

VnError VnSpi_genGenericCommand(
	char cmdId,
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize)
{
	size_t i;

	if (*size < 1 || *size < desiredLength)
		return E_BUFFER_TOO_SMALL;

	buffer[0] = cmdId;

	for (i = 1; i < desiredLength; i++)
		buffer[i] = 0x00;

	*responseSize = 2;
	*size = desiredLength > 1 ? desiredLength : 1;

	return E_NONE;
}

VnError VnSpi_genWriteSettings(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize)
{
	return VnSpi_genGenericCommand(
		3,
		buffer,
		size,
		desiredLength,
		responseSize);
}

VnError VnSpi_genRestorFactorySettings(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize)
{
	return VnSpi_genGenericCommand(
		4,
		buffer,
		size,
		desiredLength,
		responseSize);
}

VnError VnSpi_genTare(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize)
{
	return VnSpi_genGenericCommand(
		5,
		buffer,
		size,
		desiredLength,
		responseSize);
}

VnError VnSpi_genReset(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize)
{
	return VnSpi_genGenericCommand(
		6,
		buffer,
		size,
		desiredLength,
		responseSize);
}

VnError VnSpi_genRead(
	char* buffer,
	size_t* size,
	uint8_t regId,
	size_t desiredLength)
{
	size_t i;

	if (*size < 4 || *size < desiredLength)
		return E_BUFFER_TOO_SMALL;

	buffer[0] = 0x01;
	buffer[1] = regId;
	buffer[2] = 0x00;
  buffer[3] = 0x00;

	for (i = 4; i < desiredLength; i++)
		buffer[i] = 0x00;

	*size = desiredLength > 3 ? desiredLength : 3;

	return E_NONE;
}

VnError VnSpi_parseUserTag(
	const char* response,
	char* tag,
	size_t tagLength)
{
	const char* pos = response + 3;

	if (*pos != 0)
		return (VnError)(*pos + E_SENSOR_HARD_FAULT - 1);

	pos++;

	if (tagLength < strlen(pos) + 1)
		return E_BUFFER_TOO_SMALL;

	#if defined(_MSC_VER)
		/* Disable warnings regarding using strcpy_s since this
		 * function's signature does not provide us with information
		 * about the length of 'out'. */
		#pragma warning(push)
		#pragma warning(disable:4996)
	#endif

	strcpy(tag, pos);

	#if defined(_MSC_VER)
		#pragma warning(pop)
	#endif

	return E_NONE;
}

VnError VnSpi_parseModelNumber(
	const char* response,
	char* productName,
	size_t productNameLength)
{
	const char* pos = response + 3;

	if (*pos != 0)
		return (VnError)(*pos + E_SENSOR_HARD_FAULT - 1);

	pos++;

	if (productNameLength < strlen(pos) + 1)
		return E_BUFFER_TOO_SMALL;

	#if defined(_MSC_VER)
		/* Disable warnings regarding using strcpy_s since this
		 * function's signature does not provide us with information
		 * about the length of 'out'. */
		#pragma warning(push)
		#pragma warning(disable:4996)
	#endif

	strcpy(productName, pos);

	#if defined(_MSC_VER)
		#pragma warning(pop)
	#endif

	return E_NONE;
}

VnError VnSpi_parseHardwareRevision(
	const char* response,
	uint32_t* revision)
{
	const char* pos = response + 3;

	if (*pos != 0)
		return (VnError)(*pos + E_SENSOR_HARD_FAULT - 1);

	pos++;

	*revision = VnUtil_extractUint32(pos);
	pos += sizeof(uint32_t);

	return E_NONE;
}

VnError VnSpi_parseSerialNumber(
	const char* response,
	uint32_t* serialNum)
{
	const char* pos = response + 3;

	if (*pos != 0)
		return (VnError)(*pos + E_SENSOR_HARD_FAULT - 1);

	pos++;

	*serialNum = VnUtil_extractUint32(pos);
	pos += sizeof(uint32_t);

	return E_NONE;
}

VnError VnSpi_parseFirmwareVersion(
	const char* response,
	char* firmwareVersion,
	size_t firmwareVersionLength)
{
	const char* pos = response + 3;

	if (*pos != 0)
		return (VnError)(*pos + E_SENSOR_HARD_FAULT - 1);

	pos++;

	if (firmwareVersionLength < strlen(pos) + 1)
		return E_BUFFER_TOO_SMALL;

	#if defined(_MSC_VER)
		/* Disable warnings regarding using strcpy_s since this
		 * function's signature does not provide us with information
		 * about the length of 'out'. */
		#pragma warning(push)
		#pragma warning(disable:4996)
	#endif

	strcpy(firmwareVersion, pos);

	#if defined(_MSC_VER)
		#pragma warning(pop)
	#endif

	return E_NONE;
}

VnError VnSpi_parseSerialBaudRate(
	const char* response,
	uint32_t* baudrate)
{
	const char* pos = response + 3;

	if (*pos != 0)
		return (VnError)(*pos + E_SENSOR_HARD_FAULT - 1);

	pos++;

	*baudrate = VnUtil_extractUint32(pos);
	pos += sizeof(uint32_t);

	return E_NONE;
}

VnError VnSpi_parseAsyncDataOutputType(
	const char* response,
	uint32_t* ador)
{
	const char* pos = response + 3;

	if (*pos != 0)
		return (VnError)(*pos + E_SENSOR_HARD_FAULT - 1);

	pos++;

	*ador = VnUtil_extractUint32(pos);
	pos += sizeof(uint32_t);

	return E_NONE;
}

VnError VnSpi_parseAsyncDataOutputFrequency(
	const char* response,
	uint32_t* adof)
{
	const char* pos = response + 3;

	if (*pos != 0)
		return (VnError)(*pos + E_SENSOR_HARD_FAULT - 1);

	pos++;

	*adof = VnUtil_extractUint32(pos);
	pos += sizeof(uint32_t);

	return E_NONE;
}

VnError VnSpi_parseYawPitchRoll(
	const char* response,
	vec3f* yawPitchRoll)
{
	const char* pos = response + 3;

	if (*pos != 0)
		return (VnError)(*pos + E_SENSOR_HARD_FAULT - 1);

	pos++;

	*yawPitchRoll = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);

	return E_NONE;
}

VnError VnSpi_parseAttitudeQuaternion(
	const char* response,
	vec4f* quat)
{
	const char* pos = response + 3;

	if (*pos != 0)
		return (VnError)(*pos + E_SENSOR_HARD_FAULT - 1);

	pos++;

	*quat = VnUtil_extractVec4f(pos);
	pos += 4 * sizeof(float);

	return E_NONE;
}

VnError VnSpi_parseQuaternionMagneticAccelerationAndAngularRates(
	const char* response,
	vec4f* quat,
	vec3f* mag,
	vec3f* accel,
	vec3f* gyro)
{
	const char* pos = response + 3;

	if (*pos != 0)
		return (VnError)(*pos + E_SENSOR_HARD_FAULT - 1);

	pos++;

	*quat = VnUtil_extractVec4f(pos);
	pos += 4 * sizeof(float);
	*mag = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);
	*accel = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);
	*gyro = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);

	return E_NONE;
}

VnError VnSpi_parseMagneticMeasurements(
	const char* response,
	vec3f* mag)
{
	const char* pos = response + 3;

	if (*pos != 0)
		return (VnError)(*pos + E_SENSOR_HARD_FAULT - 1);

	pos++;

	*mag = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);

	return E_NONE;
}

VnError VnSpi_parseAccelerationMeasurements(
	const char* response,
	vec3f* accel)
{
	const char* pos = response + 3;

	if (*pos != 0)
		return (VnError)(*pos + E_SENSOR_HARD_FAULT - 1);

	pos++;

	*accel = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);

	return E_NONE;
}

VnError VnSpi_parseAngularRateMeasurements(
	const char* response,
	vec3f* gyro)
{
	const char* pos = response + 3;

	if (*pos != 0)
		return (VnError)(*pos + E_SENSOR_HARD_FAULT - 1);

	pos++;

	*gyro = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);

	return E_NONE;
}

VnError VnSpi_parseMagneticAccelerationAndAngularRates(
	const char* response,
	vec3f* mag,
	vec3f* accel,
	vec3f* gyro)
{
	const char* pos = response + 3;

	if (*pos != 0)
		return (VnError)(*pos + E_SENSOR_HARD_FAULT - 1);

	pos++;

	*mag = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);
	*accel = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);
	*gyro = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);

	return E_NONE;
}

VnError VnSpi_parseMagneticAndGravityReferenceVectors(
	const char* response,
	vec3f* magRef,
	vec3f* accRef)
{
	const char* pos = response + 3;

	if (*pos != 0)
		return (VnError)(*pos + E_SENSOR_HARD_FAULT - 1);

	pos++;

	*magRef = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);
	*accRef = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);

	return E_NONE;
}

VnError VnSpi_parseFilterMeasurementsVarianceParameters(
	const char* response,
	float* angularWalkVariance,
	vec3f* angularRateVariance,
	vec3f* magneticVariance,
	vec3f* accelerationVariance)
{
	const char* pos = response + 3;

	if (*pos != 0)
		return (VnError)(*pos + E_SENSOR_HARD_FAULT - 1);

	pos++;

	*angularWalkVariance = VnUtil_extractFloat(pos);
	pos += sizeof(float);
	*angularRateVariance = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);
	*magneticVariance = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);
	*accelerationVariance = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);

	return E_NONE;
}

VnError VnSpi_parseMagnetometerCompensation(
	const char* response,
	mat3f* c,
	vec3f* b)
{
	const char* pos = response + 3;

	if (*pos != 0)
		return (VnError)(*pos + E_SENSOR_HARD_FAULT - 1);

	pos++;

	*c = VnUtil_extractMat3f(pos);
	pos += 9 * sizeof(float);
	*b = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);

	return E_NONE;
}

VnError VnSpi_parseFilterActiveTuningParameters(
	const char* response,
	float* magneticDisturbanceGain,
	float* accelerationDisturbanceGain,
	float* magneticDisturbanceMemory,
	float* accelerationDisturbanceMemory)
{
	const char* pos = response + 3;

	if (*pos != 0)
		return (VnError)(*pos + E_SENSOR_HARD_FAULT - 1);

	pos++;

	*magneticDisturbanceGain = VnUtil_extractFloat(pos);
	pos += sizeof(float);
	*accelerationDisturbanceGain = VnUtil_extractFloat(pos);
	pos += sizeof(float);
	*magneticDisturbanceMemory = VnUtil_extractFloat(pos);
	pos += sizeof(float);
	*accelerationDisturbanceMemory = VnUtil_extractFloat(pos);
	pos += sizeof(float);

	return E_NONE;
}

VnError VnSpi_parseAccelerationCompensation(
	const char* response,
	mat3f* c,
	vec3f* b)
{
	const char* pos = response + 3;

	if (*pos != 0)
		return (VnError)(*pos + E_SENSOR_HARD_FAULT - 1);

	pos++;

	*c = VnUtil_extractMat3f(pos);
	pos += 9 * sizeof(float);
	*b = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);

	return E_NONE;
}

VnError VnSpi_parseReferenceFrameRotation(
	const char* response,
	mat3f* c)
{
	const char* pos = response + 3;

	if (*pos != 0)
		return (VnError)(*pos + E_SENSOR_HARD_FAULT - 1);

	pos++;

	*c = VnUtil_extractMat3f(pos);
	pos += 9 * sizeof(float);

	return E_NONE;
}

VnError VnSpi_parseYawPitchRollMagneticAccelerationAndAngularRates(
	const char* response,
	vec3f* yawPitchRoll,
	vec3f* mag,
	vec3f* accel,
	vec3f* gyro)
{
	const char* pos = response + 3;

	if (*pos != 0)
		return (VnError)(*pos + E_SENSOR_HARD_FAULT - 1);

	pos++;

	*yawPitchRoll = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);
	*mag = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);
	*accel = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);
	*gyro = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);

	return E_NONE;
}

VnError VnSpi_parseCommunicationProtocolControl(
	const char* response,
	uint8_t* serialCount,
	uint8_t* serialStatus,
	uint8_t* spiCount,
	uint8_t* spiStatus,
	uint8_t* serialChecksum,
	uint8_t* spiChecksum,
	uint8_t* errorMode)
{
	const char* pos = response + 3;

	if (*pos != 0)
		return (VnError)(*pos + E_SENSOR_HARD_FAULT - 1);

	pos++;

	*serialCount = (uint8_t) *pos;
	pos += sizeof(uint8_t);
	*serialStatus = (uint8_t) *pos;
	pos += sizeof(uint8_t);
	*spiCount = (uint8_t) *pos;
	pos += sizeof(uint8_t);
	*spiStatus = (uint8_t) *pos;
	pos += sizeof(uint8_t);
	*serialChecksum = (uint8_t) *pos;
	pos += sizeof(uint8_t);
	*spiChecksum = (uint8_t) *pos;
	pos += sizeof(uint8_t);
	*errorMode = (uint8_t) *pos;
	pos += sizeof(uint8_t);

	return E_NONE;
}

VnError VnSpi_parseSynchronizationControl(
	const char* response,
	uint8_t* syncInMode,
	uint8_t* syncInEdge,
	uint16_t* syncInSkipFactor,
	uint32_t* reserved1,
	uint8_t* syncOutMode,
	uint8_t* syncOutPolarity,
	uint16_t* syncOutSkipFactor,
	uint32_t* syncOutPulseWidth,
	uint32_t* reserved2)
{
	const char* pos = response + 3;

	if (*pos != 0)
		return (VnError)(*pos + E_SENSOR_HARD_FAULT - 1);

	pos++;

	*syncInMode = (uint8_t) *pos;
	pos += sizeof(uint8_t);
	*syncInEdge = (uint8_t) *pos;
	pos += sizeof(uint8_t);
	*syncInSkipFactor = VnUtil_extractUint16(pos);
	pos += sizeof(uint16_t);
	*reserved1 = VnUtil_extractUint32(pos);
	pos += sizeof(uint32_t);
	*syncOutMode = (uint8_t) *pos;
	pos += sizeof(uint8_t);
	*syncOutPolarity = (uint8_t) *pos;
	pos += sizeof(uint8_t);
	*syncOutSkipFactor = VnUtil_extractUint16(pos);
	pos += sizeof(uint16_t);
	*syncOutPulseWidth = VnUtil_extractUint32(pos);
	pos += sizeof(uint32_t);
	*reserved2 = VnUtil_extractUint32(pos);
	pos += sizeof(uint32_t);

	return E_NONE;
}

VnError VnSpi_parseSynchronizationStatus(
	const char* response,
	uint32_t* syncInCount,
	uint32_t* syncInTime,
	uint32_t* syncOutCount)
{
	const char* pos = response + 3;

	if (*pos != 0)
		return (VnError)(*pos + E_SENSOR_HARD_FAULT - 1);

	pos++;

	*syncInCount = VnUtil_extractUint32(pos);
	pos += sizeof(uint32_t);
	*syncInTime = VnUtil_extractUint32(pos);
	pos += sizeof(uint32_t);
	*syncOutCount = VnUtil_extractUint32(pos);
	pos += sizeof(uint32_t);

	return E_NONE;
}

VnError VnSpi_parseFilterBasicControl(
	const char* response,
	uint8_t* magMode,
	uint8_t* extMagMode,
	uint8_t* extAccMode,
	uint8_t* extGyroMode,
	vec3f* gyroLimit)
{
	const char* pos = response + 3;

	if (*pos != 0)
		return (VnError)(*pos + E_SENSOR_HARD_FAULT - 1);

	pos++;

	*magMode = (uint8_t) *pos;
	pos += sizeof(uint8_t);
	*extMagMode = (uint8_t) *pos;
	pos += sizeof(uint8_t);
	*extAccMode = (uint8_t) *pos;
	pos += sizeof(uint8_t);
	*extGyroMode = (uint8_t) *pos;
	pos += sizeof(uint8_t);
	*gyroLimit = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);

	return E_NONE;
}

VnError VnSpi_parseVpeBasicControl(
	const char* response,
	uint8_t* enable,
	uint8_t* headingMode,
	uint8_t* filteringMode,
	uint8_t* tuningMode)
{
	const char* pos = response + 3;

	if (*pos != 0)
		return (VnError)(*pos + E_SENSOR_HARD_FAULT - 1);

	pos++;

	*enable = (uint8_t) *pos;
	pos += sizeof(uint8_t);
	*headingMode = (uint8_t) *pos;
	pos += sizeof(uint8_t);
	*filteringMode = (uint8_t) *pos;
	pos += sizeof(uint8_t);
	*tuningMode = (uint8_t) *pos;
	pos += sizeof(uint8_t);

	return E_NONE;
}

VnError VnSpi_parseVpeMagnetometerBasicTuning(
	const char* response,
	vec3f* baseTuning,
	vec3f* adaptiveTuning,
	vec3f* adaptiveFiltering)
{
	const char* pos = response + 3;

	if (*pos != 0)
		return (VnError)(*pos + E_SENSOR_HARD_FAULT - 1);

	pos++;

	*baseTuning = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);
	*adaptiveTuning = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);
	*adaptiveFiltering = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);

	return E_NONE;
}

VnError VnSpi_parseVpeMagnetometerAdvancedTuning(
	const char* response,
	vec3f* minFiltering,
	vec3f* maxFiltering,
	float* maxAdaptRate,
	float* disturbanceWindow,
	float* maxTuning)
{
	const char* pos = response + 3;

	if (*pos != 0)
		return (VnError)(*pos + E_SENSOR_HARD_FAULT - 1);

	pos++;

	*minFiltering = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);
	*maxFiltering = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);
	*maxAdaptRate = VnUtil_extractFloat(pos);
	pos += sizeof(float);
	*disturbanceWindow = VnUtil_extractFloat(pos);
	pos += sizeof(float);
	*maxTuning = VnUtil_extractFloat(pos);
	pos += sizeof(float);

	return E_NONE;
}

VnError VnSpi_parseVpeAccelerometerBasicTuning(
	const char* response,
	vec3f* baseTuning,
	vec3f* adaptiveTuning,
	vec3f* adaptiveFiltering)
{
	const char* pos = response + 3;

	if (*pos != 0)
		return (VnError)(*pos + E_SENSOR_HARD_FAULT - 1);

	pos++;

	*baseTuning = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);
	*adaptiveTuning = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);
	*adaptiveFiltering = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);

	return E_NONE;
}

VnError VnSpi_parseVpeAccelerometerAdvancedTuning(
	const char* response,
	vec3f* minFiltering,
	vec3f* maxFiltering,
	float* maxAdaptRate,
	float* disturbanceWindow,
	float* maxTuning)
{
	const char* pos = response + 3;

	if (*pos != 0)
		return (VnError)(*pos + E_SENSOR_HARD_FAULT - 1);

	pos++;

	*minFiltering = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);
	*maxFiltering = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);
	*maxAdaptRate = VnUtil_extractFloat(pos);
	pos += sizeof(float);
	*disturbanceWindow = VnUtil_extractFloat(pos);
	pos += sizeof(float);
	*maxTuning = VnUtil_extractFloat(pos);
	pos += sizeof(float);

	return E_NONE;
}

VnError VnSpi_parseVpeGyroBasicTuning(
	const char* response,
	vec3f* angularWalkVariance,
	vec3f* baseTuning,
	vec3f* adaptiveTuning)
{
	const char* pos = response + 3;

	if (*pos != 0)
		return (VnError)(*pos + E_SENSOR_HARD_FAULT - 1);

	pos++;

	*angularWalkVariance = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);
	*baseTuning = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);
	*adaptiveTuning = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);

	return E_NONE;
}

VnError VnSpi_parseFilterStartupGyroBias(
	const char* response,
	vec3f* bias)
{
	const char* pos = response + 3;

	if (*pos != 0)
		return (VnError)(*pos + E_SENSOR_HARD_FAULT - 1);

	pos++;

	*bias = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);

	return E_NONE;
}

VnError VnSpi_parseMagnetometerCalibrationControl(
	const char* response,
	uint8_t* hsiMode,
	uint8_t* hsiOutput,
	uint8_t* convergeRate)
{
	const char* pos = response + 3;

	if (*pos != 0)
		return (VnError)(*pos + E_SENSOR_HARD_FAULT - 1);

	pos++;

	*hsiMode = (uint8_t) *pos;
	pos += sizeof(uint8_t);
	*hsiOutput = (uint8_t) *pos;
	pos += sizeof(uint8_t);
	*convergeRate = (uint8_t) *pos;
	pos += sizeof(uint8_t);

	return E_NONE;
}

VnError VnSpi_parseCalculatedMagnetometerCalibration(
	const char* response,
	mat3f* c,
	vec3f* b)
{
	const char* pos = response + 3;

	if (*pos != 0)
		return (VnError)(*pos + E_SENSOR_HARD_FAULT - 1);

	pos++;

	*c = VnUtil_extractMat3f(pos);
	pos += 9 * sizeof(float);
	*b = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);

	return E_NONE;
}

VnError VnSpi_parseIndoorHeadingModeControl(
	const char* response,
	float* maxRateError,
	uint8_t* reserved1)
{
	const char* pos = response + 3;

	if (*pos != 0)
		return (VnError)(*pos + E_SENSOR_HARD_FAULT - 1);

	pos++;

	*maxRateError = VnUtil_extractFloat(pos);
	pos += sizeof(float);
	*reserved1 = (uint8_t) *pos;
	pos += sizeof(uint8_t);

	return E_NONE;
}

VnError VnSpi_parseVelocityCompensationMeasurement(
	const char* response,
	vec3f* velocity)
{
	const char* pos = response + 3;

	if (*pos != 0)
		return (VnError)(*pos + E_SENSOR_HARD_FAULT - 1);

	pos++;

	*velocity = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);

	return E_NONE;
}

VnError VnSpi_parseVelocityCompensationControl(
	const char* response,
	uint8_t* mode,
	float* velocityTuning,
	float* rateTuning)
{
	const char* pos = response + 3;

	if (*pos != 0)
		return (VnError)(*pos + E_SENSOR_HARD_FAULT - 1);

	pos++;

	*mode = (uint8_t) *pos;
	pos += sizeof(uint8_t);
	*velocityTuning = VnUtil_extractFloat(pos);
	pos += sizeof(float);
	*rateTuning = VnUtil_extractFloat(pos);
	pos += sizeof(float);

	return E_NONE;
}

VnError VnSpi_parseVelocityCompensationStatus(
	const char* response,
	float* x,
	float* xDot,
	vec3f* accelOffset,
	vec3f* omega)
{
	const char* pos = response + 3;

	if (*pos != 0)
		return (VnError)(*pos + E_SENSOR_HARD_FAULT - 1);

	pos++;

	*x = VnUtil_extractFloat(pos);
	pos += sizeof(float);
	*xDot = VnUtil_extractFloat(pos);
	pos += sizeof(float);
	*accelOffset = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);
	*omega = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);

	return E_NONE;
}

VnError VnSpi_parseImuMeasurements(
	const char* response,
	vec3f* mag,
	vec3f* accel,
	vec3f* gyro,
	float* temp,
	float* pressure)
{
	const char* pos = response + 3;

	if (*pos != 0)
		return (VnError)(*pos + E_SENSOR_HARD_FAULT - 1);

	pos++;

	*mag = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);
	*accel = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);
	*gyro = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);
	*temp = VnUtil_extractFloat(pos);
	pos += sizeof(float);
	*pressure = VnUtil_extractFloat(pos);
	pos += sizeof(float);

	return E_NONE;
}

VnError VnSpi_parseGpsConfiguration(
	const char* response,
	uint8_t* mode,
	uint8_t* ppsSource,
	uint8_t* reserved1,
	uint8_t* reserved2,
	uint8_t* reserved3)
{
	const char* pos = response + 3;

	if (*pos != 0)
		return (VnError)(*pos + E_SENSOR_HARD_FAULT - 1);

	pos++;

	*mode = (uint8_t) *pos;
	pos += sizeof(uint8_t);
	*ppsSource = (uint8_t) *pos;
	pos += sizeof(uint8_t);
	*reserved1 = (uint8_t) *pos;
	pos += sizeof(uint8_t);
	*reserved2 = (uint8_t) *pos;
	pos += sizeof(uint8_t);
	*reserved3 = (uint8_t) *pos;
	pos += sizeof(uint8_t);

	return E_NONE;
}

VnError VnSpi_parseGpsAntennaOffset(
	const char* response,
	vec3f* position)
{
	const char* pos = response + 3;

	if (*pos != 0)
		return (VnError)(*pos + E_SENSOR_HARD_FAULT - 1);

	pos++;

	*position = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);

	return E_NONE;
}

VnError VnSpi_parseGpsSolutionLla(
	const char* response,
	double* time,
	uint16_t* week,
	uint8_t* gpsFix,
	uint8_t* numSats,
	vec3d* lla,
	vec3f* nedVel,
	vec3f* nedAcc,
	float* speedAcc,
	float* timeAcc)
{
	const char* pos = response + 3;

	if (*pos != 0)
		return (VnError)(*pos + E_SENSOR_HARD_FAULT - 1);

	pos++;

	*time = VnUtil_extractDouble(pos);
	pos += sizeof(double);
	*week = VnUtil_extractUint16(pos);
	pos += sizeof(uint16_t);
	*gpsFix = (uint8_t) *pos;
	pos += sizeof(uint8_t);
	*numSats = (uint8_t) *pos;
	pos += sizeof(uint8_t);
	pos += 4;
	*lla = VnUtil_extractVec3d(pos);
	pos += 3 * sizeof(double);
	*nedVel = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);
	*nedAcc = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);
	*speedAcc = VnUtil_extractFloat(pos);
	pos += sizeof(float);
	*timeAcc = VnUtil_extractFloat(pos);
	pos += sizeof(float);

	return E_NONE;
}

VnError VnSpi_parseGpsSolutionEcef(
	const char* response,
	double* tow,
	uint16_t* week,
	uint8_t* gpsFix,
	uint8_t* numSats,
	vec3d* position,
	vec3f* velocity,
	vec3f* posAcc,
	float* speedAcc,
	float* timeAcc)
{
	const char* pos = response + 3;

	if (*pos != 0)
		return (VnError)(*pos + E_SENSOR_HARD_FAULT - 1);

	pos++;

	*tow = VnUtil_extractDouble(pos);
	pos += sizeof(double);
	*week = VnUtil_extractUint16(pos);
	pos += sizeof(uint16_t);
	*gpsFix = (uint8_t) *pos;
	pos += sizeof(uint8_t);
	*numSats = (uint8_t) *pos;
	pos += sizeof(uint8_t);
	pos += 4;
	*position = VnUtil_extractVec3d(pos);
	pos += 3 * sizeof(double);
	*velocity = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);
	*posAcc = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);
	*speedAcc = VnUtil_extractFloat(pos);
	pos += sizeof(float);
	*timeAcc = VnUtil_extractFloat(pos);
	pos += sizeof(float);

	return E_NONE;
}

VnError VnSpi_parseInsSolutionLla(
	const char* response,
	double* time,
	uint16_t* week,
	uint16_t* status,
	vec3f* yawPitchRoll,
	vec3d* position,
	vec3f* nedVel,
	float* attUncertainty,
	float* posUncertainty,
	float* velUncertainty)
{
	const char* pos = response + 3;

	if (*pos != 0)
		return (VnError)(*pos + E_SENSOR_HARD_FAULT - 1);

	pos++;

	*time = VnUtil_extractDouble(pos);
	pos += sizeof(double);
	*week = VnUtil_extractUint16(pos);
	pos += sizeof(uint16_t);
	/* Use this cast to avoid a compile warning. */
	UNUSED(status);
	*yawPitchRoll = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);
	*position = VnUtil_extractVec3d(pos);
	pos += 3 * sizeof(double);
	*nedVel = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);
	*attUncertainty = VnUtil_extractFloat(pos);
	pos += sizeof(float);
	*posUncertainty = VnUtil_extractFloat(pos);
	pos += sizeof(float);
	*velUncertainty = VnUtil_extractFloat(pos);
	pos += sizeof(float);

	return E_NONE;
}

VnError VnSpi_parseInsSolutionEcef(
	const char* response,
	double* time,
	uint16_t* week,
	uint16_t* status,
	vec3f* yawPitchRoll,
	vec3d* position,
	vec3f* velocity,
	float* attUncertainty,
	float* posUncertainty,
	float* velUncertainty)
{
	const char* pos = response + 3;

	if (*pos != 0)
		return (VnError)(*pos + E_SENSOR_HARD_FAULT - 1);

	pos++;

	*time = VnUtil_extractDouble(pos);
	pos += sizeof(double);
	*week = VnUtil_extractUint16(pos);
	pos += sizeof(uint16_t);
	/* Use this cast to avoid a compile warning. */
	UNUSED(status);
	*yawPitchRoll = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);
	*position = VnUtil_extractVec3d(pos);
	pos += 3 * sizeof(double);
	*velocity = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);
	*attUncertainty = VnUtil_extractFloat(pos);
	pos += sizeof(float);
	*posUncertainty = VnUtil_extractFloat(pos);
	pos += sizeof(float);
	*velUncertainty = VnUtil_extractFloat(pos);
	pos += sizeof(float);

	return E_NONE;
}

VnError VnSpi_parseInsBasicConfiguration(
	const char* response,
	uint8_t* scenario,
	uint8_t* ahrsAiding,
	uint8_t* estBaseline,
	uint8_t* resv2)
{
	const char* pos = response + 3;

	if (*pos != 0)
		return (VnError)(*pos + E_SENSOR_HARD_FAULT - 1);

	pos++;

	*scenario = (uint8_t) *pos;
	pos += sizeof(uint8_t);
	*ahrsAiding = (uint8_t) *pos;
	pos += sizeof(uint8_t);
	*estBaseline = (uint8_t) *pos;
	pos += sizeof(uint8_t);
	*resv2 = (uint8_t) *pos;
	pos += sizeof(uint8_t);

	return E_NONE;
}

VnError VnSpi_parseInsAdvancedConfiguration(
	const char* response,
	uint8_t* useMag,
	uint8_t* usePres,
	uint8_t* posAtt,
	uint8_t* velAtt,
	uint8_t* velBias,
	uint8_t* useFoam,
	uint8_t* gpsCovType,
	uint8_t* velCount,
	float* velInit,
	float* moveOrigin,
	float* gpsTimeout,
	float* deltaLimitPos,
	float* deltaLimitVel,
	float* minPosUncertainty,
	float* minVelUncertainty)
{
	const char* pos = response + 3;

	if (*pos != 0)
		return (VnError)(*pos + E_SENSOR_HARD_FAULT - 1);

	pos++;

	*useMag = (uint8_t) *pos;
	pos += sizeof(uint8_t);
	*usePres = (uint8_t) *pos;
	pos += sizeof(uint8_t);
	*posAtt = (uint8_t) *pos;
	pos += sizeof(uint8_t);
	*velAtt = (uint8_t) *pos;
	pos += sizeof(uint8_t);
	*velBias = (uint8_t) *pos;
	pos += sizeof(uint8_t);
	*useFoam = (uint8_t) *pos;
	pos += sizeof(uint8_t);
	*gpsCovType = (uint8_t) *pos;
	pos += sizeof(uint8_t);
	*velCount = (uint8_t) *pos;
	pos += sizeof(uint8_t);
	*velInit = VnUtil_extractFloat(pos);
	pos += sizeof(float);
	*moveOrigin = VnUtil_extractFloat(pos);
	pos += sizeof(float);
	*gpsTimeout = VnUtil_extractFloat(pos);
	pos += sizeof(float);
	*deltaLimitPos = VnUtil_extractFloat(pos);
	pos += sizeof(float);
	*deltaLimitVel = VnUtil_extractFloat(pos);
	pos += sizeof(float);
	*minPosUncertainty = VnUtil_extractFloat(pos);
	pos += sizeof(float);
	*minVelUncertainty = VnUtil_extractFloat(pos);
	pos += sizeof(float);

	return E_NONE;
}

VnError VnSpi_parseInsStateLla(
	const char* response,
	vec3f* yawPitchRoll,
	vec3d* position,
	vec3f* velocity,
	vec3f* accel,
	vec3f* angularRate)
{
	const char* pos = response + 3;

	if (*pos != 0)
		return (VnError)(*pos + E_SENSOR_HARD_FAULT - 1);

	pos++;

	*yawPitchRoll = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);
	*position = VnUtil_extractVec3d(pos);
	pos += 3 * sizeof(double);
	*velocity = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);
	*accel = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);
	*angularRate = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);

	return E_NONE;
}

VnError VnSpi_parseInsStateEcef(
	const char* response,
	vec3f* yawPitchRoll,
	vec3d* position,
	vec3f* velocity,
	vec3f* accel,
	vec3f* angularRate)
{
	const char* pos = response + 3;

	if (*pos != 0)
		return (VnError)(*pos + E_SENSOR_HARD_FAULT - 1);

	pos++;

	*yawPitchRoll = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);
	*position = VnUtil_extractVec3d(pos);
	pos += 3 * sizeof(double);
	*velocity = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);
	*accel = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);
	*angularRate = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);

	return E_NONE;
}

VnError VnSpi_parseStartupFilterBiasEstimate(
	const char* response,
	vec3f* gyroBias,
	vec3f* accelBias,
	float* pressureBias)
{
	const char* pos = response + 3;

	if (*pos != 0)
		return (VnError)(*pos + E_SENSOR_HARD_FAULT - 1);

	pos++;

	*gyroBias = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);
	*accelBias = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);
	*pressureBias = VnUtil_extractFloat(pos);
	pos += sizeof(float);

	return E_NONE;
}

VnError VnSpi_parseDeltaThetaAndDeltaVelocity(
	const char* response,
	float* deltaTime,
	vec3f* deltaTheta,
	vec3f* deltaVelocity)
{
	const char* pos = response + 3;

	if (*pos != 0)
		return (VnError)(*pos + E_SENSOR_HARD_FAULT - 1);

	pos++;

	*deltaTime = VnUtil_extractFloat(pos);
	pos += sizeof(float);
	*deltaTheta = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);
	*deltaVelocity = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);

	return E_NONE;
}

VnError VnSpi_parseDeltaThetaAndDeltaVelocityConfiguration(
	const char* response,
	uint8_t* integrationFrame,
	uint8_t* gyroCompensation,
	uint8_t* accelCompensation,
	uint8_t* reserved1,
	uint16_t* reserved2)
{
	const char* pos = response + 3;

	if (*pos != 0)
		return (VnError)(*pos + E_SENSOR_HARD_FAULT - 1);

	pos++;

	*integrationFrame = (uint8_t) *pos;
	pos += sizeof(uint8_t);
	*gyroCompensation = (uint8_t) *pos;
	pos += sizeof(uint8_t);
	*accelCompensation = (uint8_t) *pos;
	pos += sizeof(uint8_t);
	*reserved1 = (uint8_t) *pos;
	pos += sizeof(uint8_t);
	*reserved2 = VnUtil_extractUint16(pos);
	pos += sizeof(uint16_t);

	return E_NONE;
}

VnError VnSpi_parseReferenceVectorConfiguration(
	const char* response,
	uint8_t* useMagModel,
	uint8_t* useGravityModel,
	uint8_t* resv1,
	uint8_t* resv2,
	uint32_t* recalcThreshold,
	float* year,
	vec3d* position)
{
	const char* pos = response + 3;

	if (*pos != 0)
		return (VnError)(*pos + E_SENSOR_HARD_FAULT - 1);

	pos++;

	*useMagModel = (uint8_t) *pos;
	pos += sizeof(uint8_t);
	*useGravityModel = (uint8_t) *pos;
	pos += sizeof(uint8_t);
	*resv1 = (uint8_t) *pos;
	pos += sizeof(uint8_t);
	*resv2 = (uint8_t) *pos;
	pos += sizeof(uint8_t);
	*recalcThreshold = VnUtil_extractUint32(pos);
	pos += sizeof(uint32_t);
	*year = VnUtil_extractFloat(pos);
	pos += sizeof(float);
	pos += 4;
	*position = VnUtil_extractVec3d(pos);
	pos += 3 * sizeof(double);

	return E_NONE;
}

VnError VnSpi_parseGyroCompensation(
	const char* response,
	mat3f* c,
	vec3f* b)
{
	const char* pos = response + 3;

	if (*pos != 0)
		return (VnError)(*pos + E_SENSOR_HARD_FAULT - 1);

	pos++;

	*c = VnUtil_extractMat3f(pos);
	pos += 9 * sizeof(float);
	*b = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);

	return E_NONE;
}

VnError VnSpi_parseImuFilteringConfiguration(
	const char* response,
	uint16_t* magWindowSize,
	uint16_t* accelWindowSize,
	uint16_t* gyroWindowSize,
	uint16_t* tempWindowSize,
	uint16_t* presWindowSize,
	uint8_t* magFilterMode,
	uint8_t* accelFilterMode,
	uint8_t* gyroFilterMode,
	uint8_t* tempFilterMode,
	uint8_t* presFilterMode)
{
	const char* pos = response + 3;

	if (*pos != 0)
		return (VnError)(*pos + E_SENSOR_HARD_FAULT - 1);

	pos++;

	*magWindowSize = VnUtil_extractUint16(pos);
	pos += sizeof(uint16_t);
	*accelWindowSize = VnUtil_extractUint16(pos);
	pos += sizeof(uint16_t);
	*gyroWindowSize = VnUtil_extractUint16(pos);
	pos += sizeof(uint16_t);
	*tempWindowSize = VnUtil_extractUint16(pos);
	pos += sizeof(uint16_t);
	*presWindowSize = VnUtil_extractUint16(pos);
	pos += sizeof(uint16_t);
	*magFilterMode = (uint8_t) *pos;
	pos += sizeof(uint8_t);
	*accelFilterMode = (uint8_t) *pos;
	pos += sizeof(uint8_t);
	*gyroFilterMode = (uint8_t) *pos;
	pos += sizeof(uint8_t);
	*tempFilterMode = (uint8_t) *pos;
	pos += sizeof(uint8_t);
	*presFilterMode = (uint8_t) *pos;
	pos += sizeof(uint8_t);

	return E_NONE;
}

VnError VnSpi_parseGpsCompassBaseline(
	const char* response,
	vec3f* position,
	vec3f* uncertainty)
{
	const char* pos = response + 3;

	if (*pos != 0)
		return (VnError)(*pos + E_SENSOR_HARD_FAULT - 1);

	pos++;

	*position = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);
	*uncertainty = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);

	return E_NONE;
}

VnError VnSpi_parseGpsCompassEstimatedBaseline(
	const char* response,
	uint8_t* estBaselineUsed,
	uint8_t* resv,
	uint16_t* numMeas,
	vec3f* position,
	vec3f* uncertainty)
{
	const char* pos = response + 3;

	if (*pos != 0)
		return (VnError)(*pos + E_SENSOR_HARD_FAULT - 1);

	pos++;

	*estBaselineUsed = (uint8_t) *pos;
	pos += sizeof(uint8_t);
	*resv = (uint8_t) *pos;
	pos += sizeof(uint8_t);
	*numMeas = VnUtil_extractUint16(pos);
	pos += sizeof(uint16_t);
	*position = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);
	*uncertainty = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);

	return E_NONE;
}

VnError VnSpi_parseImuRateConfiguration(
	const char* response,
	uint16_t* imuRate,
	uint16_t* navDivisor,
	float* filterTargetRate,
	float* filterMinRate)
{
	const char* pos = response + 3;

	if (*pos != 0)
		return (VnError)(*pos + E_SENSOR_HARD_FAULT - 1);

	pos++;

	*imuRate = VnUtil_extractUint16(pos);
	pos += sizeof(uint16_t);
	*navDivisor = VnUtil_extractUint16(pos);
	pos += sizeof(uint16_t);
	*filterTargetRate = VnUtil_extractFloat(pos);
	pos += sizeof(float);
	*filterMinRate = VnUtil_extractFloat(pos);
	pos += sizeof(float);

	return E_NONE;
}

VnError VnSpi_parseYawPitchRollTrueBodyAccelerationAndAngularRates(
	const char* response,
	vec3f* yawPitchRoll,
	vec3f* bodyAccel,
	vec3f* gyro)
{
	const char* pos = response + 3;

	if (*pos != 0)
		return (VnError)(*pos + E_SENSOR_HARD_FAULT - 1);

	pos++;

	*yawPitchRoll = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);
	*bodyAccel = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);
	*gyro = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);

	return E_NONE;
}

VnError VnSpi_parseYawPitchRollTrueInertialAccelerationAndAngularRates(
	const char* response,
	vec3f* yawPitchRoll,
	vec3f* inertialAccel,
	vec3f* gyro)
{
	const char* pos = response + 3;

	if (*pos != 0)
		return (VnError)(*pos + E_SENSOR_HARD_FAULT - 1);

	pos++;

	*yawPitchRoll = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);
	*inertialAccel = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);
	*gyro = VnUtil_extractVec3f(pos);
	pos += 3 * sizeof(float);

	return E_NONE;
}

VnError VnSpi_genReadUserTag(char* buffer, size_t* size, size_t desiredLength, size_t* responseSize)
{
	*responseSize = 4;

	return VnSpi_genRead(buffer, size, 0, desiredLength);
}

VnError VnSpi_genReadModelNumber(char* buffer, size_t* size, size_t desiredLength, size_t* responseSize)
{
	*responseSize = 4;

	return VnSpi_genRead(buffer, size, 1, desiredLength);
}

VnError VnSpi_genReadHardwareRevision(char* buffer, size_t* size, size_t desiredLength, size_t* responseSize)
{
	*responseSize = 8;

	return VnSpi_genRead(buffer, size, 2, desiredLength);
}

VnError VnSpi_genReadSerialNumber(char* buffer, size_t* size, size_t desiredLength, size_t* responseSize)
{
	*responseSize = 8;

	return VnSpi_genRead(buffer, size, 3, desiredLength);
}

VnError VnSpi_genReadFirmwareVersion(char* buffer, size_t* size, size_t desiredLength, size_t* responseSize)
{
	*responseSize = 4;

	return VnSpi_genRead(buffer, size, 4, desiredLength);
}

VnError VnSpi_genReadSerialBaudRate(char* buffer, size_t* size, size_t desiredLength, size_t* responseSize)
{
	*responseSize = 9;

	return VnSpi_genRead(buffer, size, 5, desiredLength);
}

VnError VnSpi_genReadAsyncDataOutputType(char* buffer, size_t* size, size_t desiredLength, size_t* responseSize)
{
	*responseSize = 9;

	return VnSpi_genRead(buffer, size, 6, desiredLength);
}

VnError VnSpi_genReadAsyncDataOutputFrequency(char* buffer, size_t* size, size_t desiredLength, size_t* responseSize)
{
	*responseSize = 9;

	return VnSpi_genRead(buffer, size, 7, desiredLength);
}

VnError VnSpi_genReadYawPitchRoll(char* buffer, size_t* size, size_t desiredLength, size_t* responseSize)
{
	*responseSize = 16;

	return VnSpi_genRead(buffer, size, 8, desiredLength);
}

VnError VnSpi_genReadAttitudeQuaternion(char* buffer, size_t* size, size_t desiredLength, size_t* responseSize)
{
	*responseSize = 20;

	return VnSpi_genRead(buffer, size, 9, desiredLength);
}

VnError VnSpi_genReadQuaternionMagneticAccelerationAndAngularRates(char* buffer, size_t* size, size_t desiredLength, size_t* responseSize)
{
	*responseSize = 56;

	return VnSpi_genRead(buffer, size, 15, desiredLength);
}

VnError VnSpi_genReadMagneticMeasurements(char* buffer, size_t* size, size_t desiredLength, size_t* responseSize)
{
	*responseSize = 16;

	return VnSpi_genRead(buffer, size, 17, desiredLength);
}

VnError VnSpi_genReadAccelerationMeasurements(char* buffer, size_t* size, size_t desiredLength, size_t* responseSize)
{
	*responseSize = 16;

	return VnSpi_genRead(buffer, size, 18, desiredLength);
}

VnError VnSpi_genReadAngularRateMeasurements(char* buffer, size_t* size, size_t desiredLength, size_t* responseSize)
{
	*responseSize = 16;

	return VnSpi_genRead(buffer, size, 19, desiredLength);
}

VnError VnSpi_genReadMagneticAccelerationAndAngularRates(char* buffer, size_t* size, size_t desiredLength, size_t* responseSize)
{
	*responseSize = 40;

	return VnSpi_genRead(buffer, size, 20, desiredLength);
}

VnError VnSpi_genReadMagneticAndGravityReferenceVectors(char* buffer, size_t* size, size_t desiredLength, size_t* responseSize)
{
	*responseSize = 28;

	return VnSpi_genRead(buffer, size, 21, desiredLength);
}

VnError VnSpi_genReadMagnetometerCompensation(char* buffer, size_t* size, size_t desiredLength, size_t* responseSize)
{
	*responseSize = 52;

	return VnSpi_genRead(buffer, size, 23, desiredLength);
}

VnError VnSpi_genReadAccelerationCompensation(char* buffer, size_t* size, size_t desiredLength, size_t* responseSize)
{
	*responseSize = 52;

	return VnSpi_genRead(buffer, size, 25, desiredLength);
}

VnError VnSpi_genReadReferenceFrameRotation(char* buffer, size_t* size, size_t desiredLength, size_t* responseSize)
{
	*responseSize = 40;

	return VnSpi_genRead(buffer, size, 26, desiredLength);
}

VnError VnSpi_genReadYawPitchRollMagneticAccelerationAndAngularRates(char* buffer, size_t* size, size_t desiredLength, size_t* responseSize)
{
	*responseSize = 52;

	return VnSpi_genRead(buffer, size, 27, desiredLength);
}

VnError VnSpi_genReadCommunicationProtocolControl(char* buffer, size_t* size, size_t desiredLength, size_t* responseSize)
{
	*responseSize = 11;

	return VnSpi_genRead(buffer, size, 30, desiredLength);
}

VnError VnSpi_genReadSynchronizationControl(char* buffer, size_t* size, size_t desiredLength, size_t* responseSize)
{
	*responseSize = 24;

	return VnSpi_genRead(buffer, size, 32, desiredLength);
}

VnError VnSpi_genReadSynchronizationStatus(char* buffer, size_t* size, size_t desiredLength, size_t* responseSize)
{
	*responseSize = 16;

	return VnSpi_genRead(buffer, size, 33, desiredLength);
}

VnError VnSpi_genReadVpeBasicControl(char* buffer, size_t* size, size_t desiredLength, size_t* responseSize)
{
	*responseSize = 8;

	return VnSpi_genRead(buffer, size, 35, desiredLength);
}

VnError VnSpi_genReadVpeMagnetometerBasicTuning(char* buffer, size_t* size, size_t desiredLength, size_t* responseSize)
{
	*responseSize = 40;

	return VnSpi_genRead(buffer, size, 36, desiredLength);
}

VnError VnSpi_genReadVpeAccelerometerBasicTuning(char* buffer, size_t* size, size_t desiredLength, size_t* responseSize)
{
	*responseSize = 40;

	return VnSpi_genRead(buffer, size, 38, desiredLength);
}

VnError VnSpi_genReadMagnetometerCalibrationControl(char* buffer, size_t* size, size_t desiredLength, size_t* responseSize)
{
	*responseSize = 7;

	return VnSpi_genRead(buffer, size, 44, desiredLength);
}

VnError VnSpi_genReadCalculatedMagnetometerCalibration(char* buffer, size_t* size, size_t desiredLength, size_t* responseSize)
{
	*responseSize = 52;

	return VnSpi_genRead(buffer, size, 47, desiredLength);
}

VnError VnSpi_genReadVelocityCompensationMeasurement(char* buffer, size_t* size, size_t desiredLength, size_t* responseSize)
{
	*responseSize = 16;

	return VnSpi_genRead(buffer, size, 50, desiredLength);
}

VnError VnSpi_genReadVelocityCompensationControl(char* buffer, size_t* size, size_t desiredLength, size_t* responseSize)
{
	*responseSize = 13;

	return VnSpi_genRead(buffer, size, 51, desiredLength);
}

VnError VnSpi_genReadImuMeasurements(char* buffer, size_t* size, size_t desiredLength, size_t* responseSize)
{
	*responseSize = 48;

	return VnSpi_genRead(buffer, size, 54, desiredLength);
}

VnError VnSpi_genReadGpsConfiguration(char* buffer, size_t* size, size_t desiredLength, size_t* responseSize)
{
	*responseSize = 9;

	return VnSpi_genRead(buffer, size, 55, desiredLength);
}

VnError VnSpi_genReadGpsAntennaOffset(char* buffer, size_t* size, size_t desiredLength, size_t* responseSize)
{
	*responseSize = 16;

	return VnSpi_genRead(buffer, size, 57, desiredLength);
}

VnError VnSpi_genReadGpsSolutionLla(char* buffer, size_t* size, size_t desiredLength, size_t* responseSize)
{
	*responseSize = 76;

	return VnSpi_genRead(buffer, size, 58, desiredLength);
}

VnError VnSpi_genReadGpsSolutionEcef(char* buffer, size_t* size, size_t desiredLength, size_t* responseSize)
{
	*responseSize = 76;

	return VnSpi_genRead(buffer, size, 59, desiredLength);
}

VnError VnSpi_genReadInsSolutionLla(char* buffer, size_t* size, size_t desiredLength, size_t* responseSize)
{
	*responseSize = 76;

	return VnSpi_genRead(buffer, size, 63, desiredLength);
}

VnError VnSpi_genReadInsSolutionEcef(char* buffer, size_t* size, size_t desiredLength, size_t* responseSize)
{
	*responseSize = 76;

	return VnSpi_genRead(buffer, size, 64, desiredLength);
}

VnError VnSpi_genReadInsBasicConfiguration(char* buffer, size_t* size, size_t desiredLength, size_t* responseSize)
{
	*responseSize = 8;

	return VnSpi_genRead(buffer, size, 67, desiredLength);
}

VnError VnSpi_genReadInsStateLla(char* buffer, size_t* size, size_t desiredLength, size_t* responseSize)
{
	*responseSize = 76;

	return VnSpi_genRead(buffer, size, 72, desiredLength);
}

VnError VnSpi_genReadInsStateEcef(char* buffer, size_t* size, size_t desiredLength, size_t* responseSize)
{
	*responseSize = 76;

	return VnSpi_genRead(buffer, size, 73, desiredLength);
}

VnError VnSpi_genReadStartupFilterBiasEstimate(char* buffer, size_t* size, size_t desiredLength, size_t* responseSize)
{
	*responseSize = 32;

	return VnSpi_genRead(buffer, size, 74, desiredLength);
}

VnError VnSpi_genReadDeltaThetaAndDeltaVelocity(char* buffer, size_t* size, size_t desiredLength, size_t* responseSize)
{
	*responseSize = 32;

	return VnSpi_genRead(buffer, size, 80, desiredLength);
}

VnError VnSpi_genReadDeltaThetaAndDeltaVelocityConfiguration(char* buffer, size_t* size, size_t desiredLength, size_t* responseSize)
{
	*responseSize = 10;

	return VnSpi_genRead(buffer, size, 82, desiredLength);
}

VnError VnSpi_genReadReferenceVectorConfiguration(char* buffer, size_t* size, size_t desiredLength, size_t* responseSize)
{
	*responseSize = 44;

	return VnSpi_genRead(buffer, size, 83, desiredLength);
}

VnError VnSpi_genReadGyroCompensation(char* buffer, size_t* size, size_t desiredLength, size_t* responseSize)
{
	*responseSize = 52;

	return VnSpi_genRead(buffer, size, 84, desiredLength);
}

VnError VnSpi_genReadImuFilteringConfiguration(char* buffer, size_t* size, size_t desiredLength, size_t* responseSize)
{
	*responseSize = 19;

	return VnSpi_genRead(buffer, size, 85, desiredLength);
}

VnError VnSpi_genReadGpsCompassBaseline(char* buffer, size_t* size, size_t desiredLength, size_t* responseSize)
{
	*responseSize = 28;

	return VnSpi_genRead(buffer, size, 93, desiredLength);
}

VnError VnSpi_genReadGpsCompassEstimatedBaseline(char* buffer, size_t* size, size_t desiredLength, size_t* responseSize)
{
	*responseSize = 32;

	return VnSpi_genRead(buffer, size, 97, desiredLength);
}

VnError VnSpi_genReadYawPitchRollTrueBodyAccelerationAndAngularRates(char* buffer, size_t* size, size_t desiredLength, size_t* responseSize)
{
	*responseSize = 40;

	return VnSpi_genRead(buffer, size, 239, desiredLength);
}

VnError VnSpi_genReadYawPitchRollTrueInertialAccelerationAndAngularRates(char* buffer, size_t* size, size_t desiredLength, size_t* responseSize)
{
	*responseSize = 40;

	return VnSpi_genRead(buffer, size, 240, desiredLength);
}

VnError VnSpi_genWriteUserTag(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize,
	char* tag)
{
	char* pos = buffer;

	if (*size < 4 || *size < desiredLength)
		return E_BUFFER_TOO_SMALL;

	*responseSize = 4;

	*pos++ = 2;
	*pos++ = 0;
	*pos++ = 0;
	*pos++ = 0;
	memcpy(pos, &tag, strlen(tag));
	pos += strlen(tag);

	return E_NONE;
}

VnError VnSpi_genWriteSerialBaudRate(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize,
	uint32_t baudrate)
{
	char* pos = buffer;

	if (*size < 9 || *size < desiredLength)
		return E_BUFFER_TOO_SMALL;

	*responseSize = 9;

	*pos++ = 2;
	*pos++ = 5;
	*pos++ = 0;
	*pos++ = 0;
	baudrate = htos32(baudrate);
	memcpy(pos, &baudrate, sizeof(uint32_t));
	pos += sizeof(uint32_t);

	return E_NONE;
}

VnError VnSpi_genWriteAsyncDataOutputType(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize,
	uint32_t ador)
{
	char* pos = buffer;

	if (*size < 9 || *size < desiredLength)
		return E_BUFFER_TOO_SMALL;

	*responseSize = 9;

	*pos++ = 2;
	*pos++ = 6;
	*pos++ = 0;
	*pos++ = 0;
	ador = htos32(ador);
	memcpy(pos, &ador, sizeof(uint32_t));
	pos += sizeof(uint32_t);

	return E_NONE;
}

VnError VnSpi_genWriteAsyncDataOutputFrequency(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize,
	uint32_t adof)
{
	char* pos = buffer;

	if (*size < 9 || *size < desiredLength)
		return E_BUFFER_TOO_SMALL;

	*responseSize = 9;

	*pos++ = 2;
	*pos++ = 7;
	*pos++ = 0;
	*pos++ = 0;
	adof = htos32(adof);
	memcpy(pos, &adof, sizeof(uint32_t));
	pos += sizeof(uint32_t);

	return E_NONE;
}

VnError VnSpi_genWriteMagneticAndGravityReferenceVectors(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize,
	vec3f magRef,
	vec3f accRef)
{
	char* pos = buffer;

	if (*size < 28 || *size < desiredLength)
		return E_BUFFER_TOO_SMALL;

	*responseSize = 28;

	*pos++ = 2;
	*pos++ = 21;
	*pos++ = 0;
	*pos++ = 0;
	magRef.c[0] = htosf4(magRef.c[0]);
	magRef.c[1] = htosf4(magRef.c[1]);
	magRef.c[2] = htosf4(magRef.c[2]);
	memcpy(pos, &magRef, sizeof(vec3f));
	pos += sizeof(vec3f);
	accRef.c[0] = htosf4(accRef.c[0]);
	accRef.c[1] = htosf4(accRef.c[1]);
	accRef.c[2] = htosf4(accRef.c[2]);
	memcpy(pos, &accRef, sizeof(vec3f));
	pos += sizeof(vec3f);

	return E_NONE;
}

VnError VnSpi_genWriteMagnetometerCompensation(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize,
	mat3f c,
	vec3f b)
{
	char* pos = buffer;

	if (*size < 52 || *size < desiredLength)
		return E_BUFFER_TOO_SMALL;

	*responseSize = 52;

	*pos++ = 2;
	*pos++ = 23;
	*pos++ = 0;
	*pos++ = 0;
	c.e[0] = htosf4(c.e[0]);
	c.e[1] = htosf4(c.e[1]);
	c.e[2] = htosf4(c.e[2]);
	c.e[3] = htosf4(c.e[3]);
	c.e[4] = htosf4(c.e[4]);
	c.e[5] = htosf4(c.e[5]);
	c.e[6] = htosf4(c.e[6]);
	c.e[7] = htosf4(c.e[7]);
	c.e[8] = htosf4(c.e[8]);
	memcpy(pos, &c, sizeof(mat3f));
	pos += sizeof(mat3f);
	b.c[0] = htosf4(b.c[0]);
	b.c[1] = htosf4(b.c[1]);
	b.c[2] = htosf4(b.c[2]);
	memcpy(pos, &b, sizeof(vec3f));
	pos += sizeof(vec3f);

	return E_NONE;
}

VnError VnSpi_genWriteAccelerationCompensation(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize,
	mat3f c,
	vec3f b)
{
	char* pos = buffer;

	if (*size < 52 || *size < desiredLength)
		return E_BUFFER_TOO_SMALL;

	*responseSize = 52;

	*pos++ = 2;
	*pos++ = 25;
	*pos++ = 0;
	*pos++ = 0;
	c.e[0] = htosf4(c.e[0]);
	c.e[1] = htosf4(c.e[1]);
	c.e[2] = htosf4(c.e[2]);
	c.e[3] = htosf4(c.e[3]);
	c.e[4] = htosf4(c.e[4]);
	c.e[5] = htosf4(c.e[5]);
	c.e[6] = htosf4(c.e[6]);
	c.e[7] = htosf4(c.e[7]);
	c.e[8] = htosf4(c.e[8]);
	memcpy(pos, &c, sizeof(mat3f));
	pos += sizeof(mat3f);
	b.c[0] = htosf4(b.c[0]);
	b.c[1] = htosf4(b.c[1]);
	b.c[2] = htosf4(b.c[2]);
	memcpy(pos, &b, sizeof(vec3f));
	pos += sizeof(vec3f);

	return E_NONE;
}

VnError VnSpi_genWriteReferenceFrameRotation(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize,
	mat3f c)
{
	char* pos = buffer;

	if (*size < 40 || *size < desiredLength)
		return E_BUFFER_TOO_SMALL;

	*responseSize = 40;

	*pos++ = 2;
	*pos++ = 26;
	*pos++ = 0;
	*pos++ = 0;
	c.e[0] = htosf4(c.e[0]);
	c.e[1] = htosf4(c.e[1]);
	c.e[2] = htosf4(c.e[2]);
	c.e[3] = htosf4(c.e[3]);
	c.e[4] = htosf4(c.e[4]);
	c.e[5] = htosf4(c.e[5]);
	c.e[6] = htosf4(c.e[6]);
	c.e[7] = htosf4(c.e[7]);
	c.e[8] = htosf4(c.e[8]);
	memcpy(pos, &c, sizeof(mat3f));
	pos += sizeof(mat3f);

	return E_NONE;
}

VnError VnSpi_genWriteCommunicationProtocolControl(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize,
	uint8_t serialCount,
	uint8_t serialStatus,
	uint8_t spiCount,
	uint8_t spiStatus,
	uint8_t serialChecksum,
	uint8_t spiChecksum,
	uint8_t errorMode)
{
	char* pos = buffer;

	if (*size < 11 || *size < desiredLength)
		return E_BUFFER_TOO_SMALL;

	*responseSize = 11;

	*pos++ = 2;
	*pos++ = 30;
	*pos++ = 0;
	*pos++ = 0;
	*pos++ = serialCount;
	*pos++ = serialStatus;
	*pos++ = spiCount;
	*pos++ = spiStatus;
	*pos++ = serialChecksum;
	*pos++ = spiChecksum;
	*pos++ = errorMode;

	return E_NONE;
}

VnError VnSpi_genWriteSynchronizationControl(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize,
	uint8_t syncInMode,
	uint8_t syncInEdge,
	uint16_t syncInSkipFactor,
	uint32_t reserved1,
	uint8_t syncOutMode,
	uint8_t syncOutPolarity,
	uint16_t syncOutSkipFactor,
	uint32_t syncOutPulseWidth,
	uint32_t reserved2)
{
	char* pos = buffer;

	if (*size < 24 || *size < desiredLength)
		return E_BUFFER_TOO_SMALL;

	*responseSize = 24;

	*pos++ = 2;
	*pos++ = 32;
	*pos++ = 0;
	*pos++ = 0;
	*pos++ = syncInMode;
	*pos++ = syncInEdge;
	syncInSkipFactor = htos16(syncInSkipFactor);
	memcpy(pos, &syncInSkipFactor, sizeof(uint16_t));
	pos += sizeof(uint16_t);
	reserved1 = htos32(reserved1);
	memcpy(pos, &reserved1, sizeof(uint32_t));
	pos += sizeof(uint32_t);
	*pos++ = syncOutMode;
	*pos++ = syncOutPolarity;
	syncOutSkipFactor = htos16(syncOutSkipFactor);
	memcpy(pos, &syncOutSkipFactor, sizeof(uint16_t));
	pos += sizeof(uint16_t);
	syncOutPulseWidth = htos32(syncOutPulseWidth);
	memcpy(pos, &syncOutPulseWidth, sizeof(uint32_t));
	pos += sizeof(uint32_t);
	reserved2 = htos32(reserved2);
	memcpy(pos, &reserved2, sizeof(uint32_t));
	pos += sizeof(uint32_t);

	return E_NONE;
}

VnError VnSpi_genWriteSynchronizationStatus(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize,
	uint32_t syncInCount,
	uint32_t syncInTime,
	uint32_t syncOutCount)
{
	char* pos = buffer;

	if (*size < 16 || *size < desiredLength)
		return E_BUFFER_TOO_SMALL;

	*responseSize = 16;

	*pos++ = 2;
	*pos++ = 33;
	*pos++ = 0;
	*pos++ = 0;
	syncInCount = htos32(syncInCount);
	memcpy(pos, &syncInCount, sizeof(uint32_t));
	pos += sizeof(uint32_t);
	syncInTime = htos32(syncInTime);
	memcpy(pos, &syncInTime, sizeof(uint32_t));
	pos += sizeof(uint32_t);
	syncOutCount = htos32(syncOutCount);
	memcpy(pos, &syncOutCount, sizeof(uint32_t));
	pos += sizeof(uint32_t);

	return E_NONE;
}

VnError VnSpi_genWriteVpeBasicControl(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize,
	uint8_t enable,
	uint8_t headingMode,
	uint8_t filteringMode,
	uint8_t tuningMode)
{
	char* pos = buffer;

	if (*size < 8 || *size < desiredLength)
		return E_BUFFER_TOO_SMALL;

	*responseSize = 8;

	*pos++ = 2;
	*pos++ = 35;
	*pos++ = 0;
	*pos++ = 0;
	*pos++ = enable;
	*pos++ = headingMode;
	*pos++ = filteringMode;
	*pos++ = tuningMode;

	return E_NONE;
}

VnError VnSpi_genWriteVpeMagnetometerBasicTuning(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize,
	vec3f baseTuning,
	vec3f adaptiveTuning,
	vec3f adaptiveFiltering)
{
	char* pos = buffer;

	if (*size < 40 || *size < desiredLength)
		return E_BUFFER_TOO_SMALL;

	*responseSize = 40;

	*pos++ = 2;
	*pos++ = 36;
	*pos++ = 0;
	*pos++ = 0;
	baseTuning.c[0] = htosf4(baseTuning.c[0]);
	baseTuning.c[1] = htosf4(baseTuning.c[1]);
	baseTuning.c[2] = htosf4(baseTuning.c[2]);
	memcpy(pos, &baseTuning, sizeof(vec3f));
	pos += sizeof(vec3f);
	adaptiveTuning.c[0] = htosf4(adaptiveTuning.c[0]);
	adaptiveTuning.c[1] = htosf4(adaptiveTuning.c[1]);
	adaptiveTuning.c[2] = htosf4(adaptiveTuning.c[2]);
	memcpy(pos, &adaptiveTuning, sizeof(vec3f));
	pos += sizeof(vec3f);
	adaptiveFiltering.c[0] = htosf4(adaptiveFiltering.c[0]);
	adaptiveFiltering.c[1] = htosf4(adaptiveFiltering.c[1]);
	adaptiveFiltering.c[2] = htosf4(adaptiveFiltering.c[2]);
	memcpy(pos, &adaptiveFiltering, sizeof(vec3f));
	pos += sizeof(vec3f);

	return E_NONE;
}

VnError VnSpi_genWriteVpeAccelerometerBasicTuning(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize,
	vec3f baseTuning,
	vec3f adaptiveTuning,
	vec3f adaptiveFiltering)
{
	char* pos = buffer;

	if (*size < 40 || *size < desiredLength)
		return E_BUFFER_TOO_SMALL;

	*responseSize = 40;

	*pos++ = 2;
	*pos++ = 38;
	*pos++ = 0;
	*pos++ = 0;
	baseTuning.c[0] = htosf4(baseTuning.c[0]);
	baseTuning.c[1] = htosf4(baseTuning.c[1]);
	baseTuning.c[2] = htosf4(baseTuning.c[2]);
	memcpy(pos, &baseTuning, sizeof(vec3f));
	pos += sizeof(vec3f);
	adaptiveTuning.c[0] = htosf4(adaptiveTuning.c[0]);
	adaptiveTuning.c[1] = htosf4(adaptiveTuning.c[1]);
	adaptiveTuning.c[2] = htosf4(adaptiveTuning.c[2]);
	memcpy(pos, &adaptiveTuning, sizeof(vec3f));
	pos += sizeof(vec3f);
	adaptiveFiltering.c[0] = htosf4(adaptiveFiltering.c[0]);
	adaptiveFiltering.c[1] = htosf4(adaptiveFiltering.c[1]);
	adaptiveFiltering.c[2] = htosf4(adaptiveFiltering.c[2]);
	memcpy(pos, &adaptiveFiltering, sizeof(vec3f));
	pos += sizeof(vec3f);

	return E_NONE;
}

VnError VnSpi_genWriteMagnetometerCalibrationControl(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize,
	uint8_t hsiMode,
	uint8_t hsiOutput,
	uint8_t convergeRate)
{
	char* pos = buffer;

	if (*size < 7 || *size < desiredLength)
		return E_BUFFER_TOO_SMALL;

	*responseSize = 7;

	*pos++ = 2;
	*pos++ = 44;
	*pos++ = 0;
	*pos++ = 0;
	*pos++ = hsiMode;
	*pos++ = hsiOutput;
	*pos++ = convergeRate;

	return E_NONE;
}

VnError VnSpi_genWriteVelocityCompensationMeasurement(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize,
	vec3f velocity)
{
	char* pos = buffer;

	if (*size < 16 || *size < desiredLength)
		return E_BUFFER_TOO_SMALL;

	*responseSize = 16;

	*pos++ = 2;
	*pos++ = 50;
	*pos++ = 0;
	*pos++ = 0;
	velocity.c[0] = htosf4(velocity.c[0]);
	velocity.c[1] = htosf4(velocity.c[1]);
	velocity.c[2] = htosf4(velocity.c[2]);
	memcpy(pos, &velocity, sizeof(vec3f));
	pos += sizeof(vec3f);

	return E_NONE;
}

VnError VnSpi_genWriteVelocityCompensationControl(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize,
	uint8_t mode,
	float velocityTuning,
	float rateTuning)
{
	char* pos = buffer;

	if (*size < 13 || *size < desiredLength)
		return E_BUFFER_TOO_SMALL;

	*responseSize = 13;

	*pos++ = 2;
	*pos++ = 51;
	*pos++ = 0;
	*pos++ = 0;
	*pos++ = mode;
	velocityTuning = htosf4(velocityTuning);
	memcpy(pos, &velocityTuning, sizeof(float));
	pos += sizeof(float);
	rateTuning = htosf4(rateTuning);
	memcpy(pos, &rateTuning, sizeof(float));
	pos += sizeof(float);

	return E_NONE;
}

VnError VnSpi_genWriteGpsConfiguration(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize,
	uint8_t mode,
	uint8_t ppsSource,
	uint8_t reserved1,
	uint8_t reserved2,
	uint8_t reserved3)
{
	char* pos = buffer;

	if (*size < 9 || *size < desiredLength)
		return E_BUFFER_TOO_SMALL;

	*responseSize = 9;

	*pos++ = 2;
	*pos++ = 55;
	*pos++ = 0;
	*pos++ = 0;
	*pos++ = mode;
	*pos++ = ppsSource;
	*pos++ = reserved1;
	*pos++ = reserved2;
	*pos++ = reserved3;

	return E_NONE;
}

VnError VnSpi_genWriteGpsAntennaOffset(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize,
	vec3f position)
{
	char* pos = buffer;

	if (*size < 16 || *size < desiredLength)
		return E_BUFFER_TOO_SMALL;

	*responseSize = 16;

	*pos++ = 2;
	*pos++ = 57;
	*pos++ = 0;
	*pos++ = 0;
	position.c[0] = htosf4(position.c[0]);
	position.c[1] = htosf4(position.c[1]);
	position.c[2] = htosf4(position.c[2]);
	memcpy(pos, &position, sizeof(vec3f));
	pos += sizeof(vec3f);

	return E_NONE;
}

VnError VnSpi_genWriteInsBasicConfiguration(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize,
	uint8_t scenario,
	uint8_t ahrsAiding,
	uint8_t estBaseline,
	uint8_t resv2)
{
	char* pos = buffer;

	if (*size < 8 || *size < desiredLength)
		return E_BUFFER_TOO_SMALL;

	*responseSize = 8;

	*pos++ = 2;
	*pos++ = 67;
	*pos++ = 0;
	*pos++ = 0;
	*pos++ = scenario;
	*pos++ = ahrsAiding;
	*pos++ = estBaseline;
	*pos++ = resv2;

	return E_NONE;
}

VnError VnSpi_genWriteStartupFilterBiasEstimate(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize,
	vec3f gyroBias,
	vec3f accelBias,
	float pressureBias)
{
	char* pos = buffer;

	if (*size < 32 || *size < desiredLength)
		return E_BUFFER_TOO_SMALL;

	*responseSize = 32;

	*pos++ = 2;
	*pos++ = 74;
	*pos++ = 0;
	*pos++ = 0;
	gyroBias.c[0] = htosf4(gyroBias.c[0]);
	gyroBias.c[1] = htosf4(gyroBias.c[1]);
	gyroBias.c[2] = htosf4(gyroBias.c[2]);
	memcpy(pos, &gyroBias, sizeof(vec3f));
	pos += sizeof(vec3f);
	accelBias.c[0] = htosf4(accelBias.c[0]);
	accelBias.c[1] = htosf4(accelBias.c[1]);
	accelBias.c[2] = htosf4(accelBias.c[2]);
	memcpy(pos, &accelBias, sizeof(vec3f));
	pos += sizeof(vec3f);
	pressureBias = htosf4(pressureBias);
	memcpy(pos, &pressureBias, sizeof(float));
	pos += sizeof(float);

	return E_NONE;
}

VnError VnSpi_genWriteDeltaThetaAndDeltaVelocityConfiguration(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize,
	uint8_t integrationFrame,
	uint8_t gyroCompensation,
	uint8_t accelCompensation,
	uint8_t reserved1,
	uint16_t reserved2)
{
	char* pos = buffer;

	if (*size < 10 || *size < desiredLength)
		return E_BUFFER_TOO_SMALL;

	*responseSize = 10;

	*pos++ = 2;
	*pos++ = 82;
	*pos++ = 0;
	*pos++ = 0;
	*pos++ = integrationFrame;
	*pos++ = gyroCompensation;
	*pos++ = accelCompensation;
	*pos++ = reserved1;
	reserved2 = htos16(reserved2);
	memcpy(pos, &reserved2, sizeof(uint16_t));
	pos += sizeof(uint16_t);

	return E_NONE;
}

VnError VnSpi_genWriteReferenceVectorConfiguration(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize,
	uint8_t useMagModel,
	uint8_t useGravityModel,
	uint8_t resv1,
	uint8_t resv2,
	uint32_t recalcThreshold,
	float year,
	vec3d position)
{
	char* pos = buffer;

	if (*size < 44 || *size < desiredLength)
		return E_BUFFER_TOO_SMALL;

	*responseSize = 44;

	*pos++ = 2;
	*pos++ = 83;
	*pos++ = 0;
	*pos++ = 0;
	*pos++ = useMagModel;
	*pos++ = useGravityModel;
	*pos++ = resv1;
	*pos++ = resv2;
	recalcThreshold = htos32(recalcThreshold);
	memcpy(pos, &recalcThreshold, sizeof(uint32_t));
	pos += sizeof(uint32_t);
	year = htosf4(year);
	memcpy(pos, &year, sizeof(float));
	pos += sizeof(float);
	pos += 4;
	position.c[0] = htosf8(position.c[0]);
	position.c[1] = htosf8(position.c[1]);
	position.c[2] = htosf8(position.c[2]);
	memcpy(pos, &position, sizeof(vec3d));
	pos += sizeof(vec3d);

	return E_NONE;
}

VnError VnSpi_genWriteGyroCompensation(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize,
	mat3f c,
	vec3f b)
{
	char* pos = buffer;

	if (*size < 52 || *size < desiredLength)
		return E_BUFFER_TOO_SMALL;

	*responseSize = 52;

	*pos++ = 2;
	*pos++ = 84;
	*pos++ = 0;
	*pos++ = 0;
	c.e[0] = htosf4(c.e[0]);
	c.e[1] = htosf4(c.e[1]);
	c.e[2] = htosf4(c.e[2]);
	c.e[3] = htosf4(c.e[3]);
	c.e[4] = htosf4(c.e[4]);
	c.e[5] = htosf4(c.e[5]);
	c.e[6] = htosf4(c.e[6]);
	c.e[7] = htosf4(c.e[7]);
	c.e[8] = htosf4(c.e[8]);
	memcpy(pos, &c, sizeof(mat3f));
	pos += sizeof(mat3f);
	b.c[0] = htosf4(b.c[0]);
	b.c[1] = htosf4(b.c[1]);
	b.c[2] = htosf4(b.c[2]);
	memcpy(pos, &b, sizeof(vec3f));
	pos += sizeof(vec3f);

	return E_NONE;
}

VnError VnSpi_genWriteImuFilteringConfiguration(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize,
	uint16_t magWindowSize,
	uint16_t accelWindowSize,
	uint16_t gyroWindowSize,
	uint16_t tempWindowSize,
	uint16_t presWindowSize,
	uint8_t magFilterMode,
	uint8_t accelFilterMode,
	uint8_t gyroFilterMode,
	uint8_t tempFilterMode,
	uint8_t presFilterMode)
{
	char* pos = buffer;

	if (*size < 19 || *size < desiredLength)
		return E_BUFFER_TOO_SMALL;

	*responseSize = 19;

	*pos++ = 2;
	*pos++ = 85;
	*pos++ = 0;
	*pos++ = 0;
	magWindowSize = htos16(magWindowSize);
	memcpy(pos, &magWindowSize, sizeof(uint16_t));
	pos += sizeof(uint16_t);
	accelWindowSize = htos16(accelWindowSize);
	memcpy(pos, &accelWindowSize, sizeof(uint16_t));
	pos += sizeof(uint16_t);
	gyroWindowSize = htos16(gyroWindowSize);
	memcpy(pos, &gyroWindowSize, sizeof(uint16_t));
	pos += sizeof(uint16_t);
	tempWindowSize = htos16(tempWindowSize);
	memcpy(pos, &tempWindowSize, sizeof(uint16_t));
	pos += sizeof(uint16_t);
	presWindowSize = htos16(presWindowSize);
	memcpy(pos, &presWindowSize, sizeof(uint16_t));
	pos += sizeof(uint16_t);
	*pos++ = magFilterMode;
	*pos++ = accelFilterMode;
	*pos++ = gyroFilterMode;
	*pos++ = tempFilterMode;
	*pos++ = presFilterMode;

	return E_NONE;
}

VnError VnSpi_genWriteGpsCompassBaseline(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize,
	vec3f position,
	vec3f uncertainty)
{
	char* pos = buffer;

	if (*size < 28 || *size < desiredLength)
		return E_BUFFER_TOO_SMALL;

	*responseSize = 28;

	*pos++ = 2;
	*pos++ = 93;
	*pos++ = 0;
	*pos++ = 0;
	position.c[0] = htosf4(position.c[0]);
	position.c[1] = htosf4(position.c[1]);
	position.c[2] = htosf4(position.c[2]);
	memcpy(pos, &position, sizeof(vec3f));
	pos += sizeof(vec3f);
	uncertainty.c[0] = htosf4(uncertainty.c[0]);
	uncertainty.c[1] = htosf4(uncertainty.c[1]);
	uncertainty.c[2] = htosf4(uncertainty.c[2]);
	memcpy(pos, &uncertainty, sizeof(vec3f));
	pos += sizeof(vec3f);

	return E_NONE;
}
