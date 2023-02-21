#ifndef _VNSPI_H_
#define _VNSPI_H_

#include <stddef.h>

#include "vn/int.h"
#include "vn/error.h"
#include "vn/math/vector.h"
#include "vn/math/matrix.h"

#ifdef __cplusplus
extern "C" {
#endif

/** \brief Generates a command to write settings.
*
* \param[in] buffer Caller provided buffer to place the generated command.
* \param[in/out] size Number of bytes available in the buffer.Will contain on output the number of bytes in the command to send.
* \param[in] desiredLength The total number of bytes to pad with 0x00 should
*     the total constructed length of the command be less than the
*     desiredLength.This is useful back - to - back command where the
*     desiredLength will be the responseSize of the previous command sent.
* \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
* \return Indicates any errors encountered. */
VnError VnSpi_genWriteSettings(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize);

/** \brief Generates a command to restore factory settings.
*
* \param[in] buffer Caller provided buffer to place the generated command.
* \param[in/out] size Number of bytes available in the buffer.Will contain on output the number of bytes in the command to send.
* \param[in] desiredLength The total number of bytes to pad with 0x00 should
*     the total constructed length of the command be less than the
*     desiredLength.This is useful back - to - back command where the
*     desiredLength will be the responseSize of the previous command sent.
* \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
* \return Indicates any errors encountered. */
VnError VnSpi_genRestorFactorySettings(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize);

/** \brief Generates a command to tare the sensor.
*
* \param[in] buffer Caller provided buffer to place the generated command.
* \param[in/out] size Number of bytes available in the buffer.Will contain on output the number of bytes in the command to send.
* \param[in] desiredLength The total number of bytes to pad with 0x00 should
*     the total constructed length of the command be less than the
*     desiredLength.This is useful back - to - back command where the
*     desiredLength will be the responseSize of the previous command sent.
* \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
* \return Indicates any errors encountered. */
VnError VnSpi_genTare(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize);

/** \brief Generates a command to reset the sensor.
*
* \param[in] buffer Caller provided buffer to place the generated command.
* \param[in/out] size Number of bytes available in the buffer.Will contain on output the number of bytes in the command to send.
* \param[in] desiredLength The total number of bytes to pad with 0x00 should
*     the total constructed length of the command be less than the
*     desiredLength.This is useful back - to - back command where the
*     desiredLength will be the responseSize of the previous command sent.
* \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
* \return Indicates any errors encountered. */
VnError VnSpi_genReset(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize);

/** \brief Generic function for making register read commands.
*
* \param[out] buffer Caller provided buffer to place the generated command.
* \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
* \param[in] regId The register ID to generate the read command for.
* \param[in] desiredLength The total number of bytes to pad with 0x00 should
*     the total constructed length of the command be less than the
*     desiredLength. This is useful back-to-back command where the
*     desiredLength will be the responseSize of the previous command sent.
* \return Indicates any error encountered. */
VnError VnSpi_genRead(
	char* buffer,
	size_t* size,
	uint8_t regId,
	size_t desiredLength);

/** \brief Parses a response from reading the User Tag register using the SPI protocol.
*
* \param[in] response Pointer to the buffer containing the response from the VectorNav sensor, including the leading 0x00 byte.
* \param[out] tag The register's Tag field.
* \param[in] tagLength The number of bytes available in the buffer tag.
* \return Indicates any error encountered. */
VnError VnSpi_parseUserTag(
	const char* response,
	char* tag,
	size_t tagLength);

/** \brief Parses a response from reading the Model Number register using the SPI protocol.
*
* \param[in] response Pointer to the buffer containing the response from the VectorNav sensor, including the leading 0x00 byte.
* \param[out] productName The register's Product Name field.
* \param[in] productNameLength The number of bytes available in the buffer productName.
* \return Indicates any error encountered. */
VnError VnSpi_parseModelNumber(
	const char* response,
	char* productName,
	size_t productNameLength);

/** \brief Parses a response from reading the Hardware Revision register using the SPI protocol.
*
* \param[in] response Pointer to the buffer containing the response from the VectorNav sensor, including the leading 0x00 byte.
* \param[out] revision The register's Revision field.
* \return Indicates any error encountered. */
VnError VnSpi_parseHardwareRevision(
	const char* response,
	uint32_t* revision);

/** \brief Parses a response from reading the Serial Number register using the SPI protocol.
*
* \param[in] response Pointer to the buffer containing the response from the VectorNav sensor, including the leading 0x00 byte.
* \param[out] serialNum The register's SerialNum field.
* \return Indicates any error encountered. */
VnError VnSpi_parseSerialNumber(
	const char* response,
	uint32_t* serialNum);

/** \brief Parses a response from reading the Firmware Version register using the SPI protocol.
*
* \param[in] response Pointer to the buffer containing the response from the VectorNav sensor, including the leading 0x00 byte.
* \param[out] firmwareVersion The register's Firmware Version field.
* \param[in] firmwareVersionLength The number of bytes available in the buffer firmwareVersion.
* \return Indicates any error encountered. */
VnError VnSpi_parseFirmwareVersion(
	const char* response,
	char* firmwareVersion,
	size_t firmwareVersionLength);

/** \brief Parses a response from reading the Serial Baud Rate register using the SPI protocol.
*
* \param[in] response Pointer to the buffer containing the response from the VectorNav sensor, including the leading 0x00 byte.
* \param[out] baudrate The register's Baud Rate field.
* \return Indicates any error encountered. */
VnError VnSpi_parseSerialBaudRate(
	const char* response,
	uint32_t* baudrate);

/** \brief Parses a response from reading the Async Data Output Type register using the SPI protocol.
*
* \param[in] response Pointer to the buffer containing the response from the VectorNav sensor, including the leading 0x00 byte.
* \param[out] ador The register's ADOR field.
* \return Indicates any error encountered. */
VnError VnSpi_parseAsyncDataOutputType(
	const char* response,
	uint32_t* ador);

/** \brief Parses a response from reading the Async Data Output Frequency register using the SPI protocol.
*
* \param[in] response Pointer to the buffer containing the response from the VectorNav sensor, including the leading 0x00 byte.
* \param[out] adof The register's ADOF field.
* \return Indicates any error encountered. */
VnError VnSpi_parseAsyncDataOutputFrequency(
	const char* response,
	uint32_t* adof);

/** \brief Parses a response from reading the Yaw Pitch Roll register using the SPI protocol.
*
* \param[in] response Pointer to the buffer containing the response from the VectorNav sensor, including the leading 0x00 byte.
* \param[out] yawPitchRoll The register's YawPitchRoll field.
* \return Indicates any error encountered. */
VnError VnSpi_parseYawPitchRoll(
	const char* response,
	vec3f* yawPitchRoll);

/** \brief Parses a response from reading the Attitude Quaternion register using the SPI protocol.
*
* \param[in] response Pointer to the buffer containing the response from the VectorNav sensor, including the leading 0x00 byte.
* \param[out] quat The register's Quat field.
* \return Indicates any error encountered. */
VnError VnSpi_parseAttitudeQuaternion(
	const char* response,
	vec4f* quat);

/** \brief Parses a response from reading the Quaternion, Magnetic, Acceleration and Angular Rates register using the SPI protocol.
*
* \param[in] response Pointer to the buffer containing the response from the VectorNav sensor, including the leading 0x00 byte.
* \param[out] quat The register's Quat field.
* \param[out] mag The register's Mag field.
* \param[out] accel The register's Accel field.
* \param[out] gyro The register's Gyro field.
* \return Indicates any error encountered. */
VnError VnSpi_parseQuaternionMagneticAccelerationAndAngularRates(
	const char* response,
	vec4f* quat, 
	vec3f* mag, 
	vec3f* accel, 
	vec3f* gyro);

/** \brief Parses a response from reading the Magnetic Measurements register using the SPI protocol.
*
* \param[in] response Pointer to the buffer containing the response from the VectorNav sensor, including the leading 0x00 byte.
* \param[out] mag The register's Mag field.
* \return Indicates any error encountered. */
VnError VnSpi_parseMagneticMeasurements(
	const char* response,
	vec3f* mag);

/** \brief Parses a response from reading the Acceleration Measurements register using the SPI protocol.
*
* \param[in] response Pointer to the buffer containing the response from the VectorNav sensor, including the leading 0x00 byte.
* \param[out] accel The register's Accel field.
* \return Indicates any error encountered. */
VnError VnSpi_parseAccelerationMeasurements(
	const char* response,
	vec3f* accel);

/** \brief Parses a response from reading the Angular Rate Measurements register using the SPI protocol.
*
* \param[in] response Pointer to the buffer containing the response from the VectorNav sensor, including the leading 0x00 byte.
* \param[out] gyro The register's Gyro field.
* \return Indicates any error encountered. */
VnError VnSpi_parseAngularRateMeasurements(
	const char* response,
	vec3f* gyro);

/** \brief Parses a response from reading the Magnetic, Acceleration and Angular Rates register using the SPI protocol.
*
* \param[in] response Pointer to the buffer containing the response from the VectorNav sensor, including the leading 0x00 byte.
* \param[out] mag The register's Mag field.
* \param[out] accel The register's Accel field.
* \param[out] gyro The register's Gyro field.
* \return Indicates any error encountered. */
VnError VnSpi_parseMagneticAccelerationAndAngularRates(
	const char* response,
	vec3f* mag, 
	vec3f* accel, 
	vec3f* gyro);

/** \brief Parses a response from reading the Magnetic and Gravity Reference Vectors register using the SPI protocol.
*
* \param[in] response Pointer to the buffer containing the response from the VectorNav sensor, including the leading 0x00 byte.
* \param[out] magRef The register's MagRef field.
* \param[out] accRef The register's AccRef field.
* \return Indicates any error encountered. */
VnError VnSpi_parseMagneticAndGravityReferenceVectors(
	const char* response,
	vec3f* magRef, 
	vec3f* accRef);

/** \brief Parses a response from reading the Filter Measurements Variance Parameters register using the SPI protocol.
*
* \param[in] response Pointer to the buffer containing the response from the VectorNav sensor, including the leading 0x00 byte.
* \param[out] angularWalkVariance The register's Angular Walk Variance field.
* \param[out] angularRateVariance The register's Angular Rate Variance field.
* \param[out] magneticVariance The register's Magnetic Variance field.
* \param[out] accelerationVariance The register's Acceleration Variance field.
* \return Indicates any error encountered. */
VnError VnSpi_parseFilterMeasurementsVarianceParameters(
	const char* response,
	float* angularWalkVariance, 
	vec3f* angularRateVariance, 
	vec3f* magneticVariance, 
	vec3f* accelerationVariance);

/** \brief Parses a response from reading the Magnetometer Compensation register using the SPI protocol.
*
* \param[in] response Pointer to the buffer containing the response from the VectorNav sensor, including the leading 0x00 byte.
* \param[out] c The register's C field.
* \param[out] b The register's B field.
* \return Indicates any error encountered. */
VnError VnSpi_parseMagnetometerCompensation(
	const char* response,
	mat3f* c, 
	vec3f* b);

/** \brief Parses a response from reading the Filter Active Tuning Parameters register using the SPI protocol.
*
* \param[in] response Pointer to the buffer containing the response from the VectorNav sensor, including the leading 0x00 byte.
* \param[out] magneticDisturbanceGain The register's Magnetic Disturbance Gain field.
* \param[out] accelerationDisturbanceGain The register's Acceleration Disturbance Gain field.
* \param[out] magneticDisturbanceMemory The register's Magnetic Disturbance Memory field.
* \param[out] accelerationDisturbanceMemory The register's Acceleration Disturbance Memory field.
* \return Indicates any error encountered. */
VnError VnSpi_parseFilterActiveTuningParameters(
	const char* response,
	float* magneticDisturbanceGain, 
	float* accelerationDisturbanceGain, 
	float* magneticDisturbanceMemory, 
	float* accelerationDisturbanceMemory);

/** \brief Parses a response from reading the Acceleration Compensation register using the SPI protocol.
*
* \param[in] response Pointer to the buffer containing the response from the VectorNav sensor, including the leading 0x00 byte.
* \param[out] c The register's C field.
* \param[out] b The register's B field.
* \return Indicates any error encountered. */
VnError VnSpi_parseAccelerationCompensation(
	const char* response,
	mat3f* c, 
	vec3f* b);

/** \brief Parses a response from reading the Reference Frame Rotation register using the SPI protocol.
*
* \param[in] response Pointer to the buffer containing the response from the VectorNav sensor, including the leading 0x00 byte.
* \param[out] c The register's C field.
* \return Indicates any error encountered. */
VnError VnSpi_parseReferenceFrameRotation(
	const char* response,
	mat3f* c);

/** \brief Parses a response from reading the Yaw, Pitch, Roll, Magnetic, Acceleration and Angular Rates register using the SPI protocol.
*
* \param[in] response Pointer to the buffer containing the response from the VectorNav sensor, including the leading 0x00 byte.
* \param[out] yawPitchRoll The register's YawPitchRoll field.
* \param[out] mag The register's Mag field.
* \param[out] accel The register's Accel field.
* \param[out] gyro The register's Gyro field.
* \return Indicates any error encountered. */
VnError VnSpi_parseYawPitchRollMagneticAccelerationAndAngularRates(
	const char* response,
	vec3f* yawPitchRoll, 
	vec3f* mag, 
	vec3f* accel, 
	vec3f* gyro);

/** \brief Parses a response from reading the Communication Protocol Control register using the SPI protocol.
*
* \param[in] response Pointer to the buffer containing the response from the VectorNav sensor, including the leading 0x00 byte.
* \param[out] serialCount The register's SerialCount field.
* \param[out] serialStatus The register's SerialStatus field.
* \param[out] spiCount The register's SPICount field.
* \param[out] spiStatus The register's SPIStatus field.
* \param[out] serialChecksum The register's SerialChecksum field.
* \param[out] spiChecksum The register's SPIChecksum field.
* \param[out] errorMode The register's ErrorMode field.
* \return Indicates any error encountered. */
VnError VnSpi_parseCommunicationProtocolControl(
	const char* response,
	uint8_t* serialCount, 
	uint8_t* serialStatus, 
	uint8_t* spiCount, 
	uint8_t* spiStatus, 
	uint8_t* serialChecksum, 
	uint8_t* spiChecksum, 
	uint8_t* errorMode);

/** \brief Parses a response from reading the Synchronization Control register using the SPI protocol.
*
* \param[in] response Pointer to the buffer containing the response from the VectorNav sensor, including the leading 0x00 byte.
* \param[out] syncInMode The register's SyncInMode field.
* \param[out] syncInEdge The register's SyncInEdge field.
* \param[out] syncInSkipFactor The register's SyncInSkipFactor field.
* \param[out] reserved1 The register's RESERVED1 field.
* \param[out] syncOutMode The register's SyncOutMode field.
* \param[out] syncOutPolarity The register's SyncOutPolarity field.
* \param[out] syncOutSkipFactor The register's SyncOutSkipFactor field.
* \param[out] syncOutPulseWidth The register's SyncOutPulseWidth field.
* \param[out] reserved2 The register's RESERVED2 field.
* \return Indicates any error encountered. */
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
	uint32_t* reserved2);

/** \brief Parses a response from reading the Synchronization Status register using the SPI protocol.
*
* \param[in] response Pointer to the buffer containing the response from the VectorNav sensor, including the leading 0x00 byte.
* \param[out] syncInCount The register's SyncInCount field.
* \param[out] syncInTime The register's SyncInTime field.
* \param[out] syncOutCount The register's SyncOutCount field.
* \return Indicates any error encountered. */
VnError VnSpi_parseSynchronizationStatus(
	const char* response,
	uint32_t* syncInCount, 
	uint32_t* syncInTime, 
	uint32_t* syncOutCount);

/** \brief Parses a response from reading the Filter Basic Control register using the SPI protocol.
*
* \param[in] response Pointer to the buffer containing the response from the VectorNav sensor, including the leading 0x00 byte.
* \param[out] magMode The register's MagMode field.
* \param[out] extMagMode The register's ExtMagMode field.
* \param[out] extAccMode The register's ExtAccMode field.
* \param[out] extGyroMode The register's ExtGyroMode field.
* \param[out] gyroLimit The register's GyroLimit field.
* \return Indicates any error encountered. */
VnError VnSpi_parseFilterBasicControl(
	const char* response,
	uint8_t* magMode, 
	uint8_t* extMagMode, 
	uint8_t* extAccMode, 
	uint8_t* extGyroMode, 
	vec3f* gyroLimit);

/** \brief Parses a response from reading the VPE Basic Control register using the SPI protocol.
*
* \param[in] response Pointer to the buffer containing the response from the VectorNav sensor, including the leading 0x00 byte.
* \param[out] enable The register's Enable field.
* \param[out] headingMode The register's HeadingMode field.
* \param[out] filteringMode The register's FilteringMode field.
* \param[out] tuningMode The register's TuningMode field.
* \return Indicates any error encountered. */
VnError VnSpi_parseVpeBasicControl(
	const char* response,
	uint8_t* enable, 
	uint8_t* headingMode, 
	uint8_t* filteringMode, 
	uint8_t* tuningMode);

/** \brief Parses a response from reading the VPE Magnetometer Basic Tuning register using the SPI protocol.
*
* \param[in] response Pointer to the buffer containing the response from the VectorNav sensor, including the leading 0x00 byte.
* \param[out] baseTuning The register's BaseTuning field.
* \param[out] adaptiveTuning The register's AdaptiveTuning field.
* \param[out] adaptiveFiltering The register's AdaptiveFiltering field.
* \return Indicates any error encountered. */
VnError VnSpi_parseVpeMagnetometerBasicTuning(
	const char* response,
	vec3f* baseTuning, 
	vec3f* adaptiveTuning, 
	vec3f* adaptiveFiltering);

/** \brief Parses a response from reading the VPE Magnetometer Advanced Tuning register using the SPI protocol.
*
* \param[in] response Pointer to the buffer containing the response from the VectorNav sensor, including the leading 0x00 byte.
* \param[out] minFiltering The register's MinFiltering field.
* \param[out] maxFiltering The register's MaxFiltering field.
* \param[out] maxAdaptRate The register's MaxAdaptRate field.
* \param[out] disturbanceWindow The register's DisturbanceWindow field.
* \param[out] maxTuning The register's MaxTuning field.
* \return Indicates any error encountered. */
VnError VnSpi_parseVpeMagnetometerAdvancedTuning(
	const char* response,
	vec3f* minFiltering, 
	vec3f* maxFiltering, 
	float* maxAdaptRate, 
	float* disturbanceWindow, 
	float* maxTuning);

/** \brief Parses a response from reading the VPE Accelerometer Basic Tuning register using the SPI protocol.
*
* \param[in] response Pointer to the buffer containing the response from the VectorNav sensor, including the leading 0x00 byte.
* \param[out] baseTuning The register's BaseTuning field.
* \param[out] adaptiveTuning The register's AdaptiveTuning field.
* \param[out] adaptiveFiltering The register's AdaptiveFiltering field.
* \return Indicates any error encountered. */
VnError VnSpi_parseVpeAccelerometerBasicTuning(
	const char* response,
	vec3f* baseTuning, 
	vec3f* adaptiveTuning, 
	vec3f* adaptiveFiltering);

/** \brief Parses a response from reading the VPE Accelerometer Advanced Tuning register using the SPI protocol.
*
* \param[in] response Pointer to the buffer containing the response from the VectorNav sensor, including the leading 0x00 byte.
* \param[out] minFiltering The register's MinFiltering field.
* \param[out] maxFiltering The register's MaxFiltering field.
* \param[out] maxAdaptRate The register's MaxAdaptRate field.
* \param[out] disturbanceWindow The register's DisturbanceWindow field.
* \param[out] maxTuning The register's MaxTuning field.
* \return Indicates any error encountered. */
VnError VnSpi_parseVpeAccelerometerAdvancedTuning(
	const char* response,
	vec3f* minFiltering, 
	vec3f* maxFiltering, 
	float* maxAdaptRate, 
	float* disturbanceWindow, 
	float* maxTuning);

/** \brief Parses a response from reading the VPE Gyro Basic Tuning register using the SPI protocol.
*
* \param[in] response Pointer to the buffer containing the response from the VectorNav sensor, including the leading 0x00 byte.
* \param[out] angularWalkVariance The register's AngularWalkVariance field.
* \param[out] baseTuning The register's BaseTuning field.
* \param[out] adaptiveTuning The register's AdaptiveTuning field.
* \return Indicates any error encountered. */
VnError VnSpi_parseVpeGyroBasicTuning(
	const char* response,
	vec3f* angularWalkVariance, 
	vec3f* baseTuning, 
	vec3f* adaptiveTuning);

/** \brief Parses a response from reading the Filter Startup Gyro Bias register using the SPI protocol.
*
* \param[in] response Pointer to the buffer containing the response from the VectorNav sensor, including the leading 0x00 byte.
* \param[out] bias The register's Bias field.
* \return Indicates any error encountered. */
VnError VnSpi_parseFilterStartupGyroBias(
	const char* response,
	vec3f* bias);

/** \brief Parses a response from reading the Magnetometer Calibration Control register using the SPI protocol.
*
* \param[in] response Pointer to the buffer containing the response from the VectorNav sensor, including the leading 0x00 byte.
* \param[out] hsiMode The register's HSIMode field.
* \param[out] hsiOutput The register's HSIOutput field.
* \param[out] convergeRate The register's ConvergeRate field.
* \return Indicates any error encountered. */
VnError VnSpi_parseMagnetometerCalibrationControl(
	const char* response,
	uint8_t* hsiMode, 
	uint8_t* hsiOutput, 
	uint8_t* convergeRate);

/** \brief Parses a response from reading the Calculated Magnetometer Calibration register using the SPI protocol.
*
* \param[in] response Pointer to the buffer containing the response from the VectorNav sensor, including the leading 0x00 byte.
* \param[out] c The register's C field.
* \param[out] b The register's B field.
* \return Indicates any error encountered. */
VnError VnSpi_parseCalculatedMagnetometerCalibration(
	const char* response,
	mat3f* c, 
	vec3f* b);

/** \brief Parses a response from reading the Indoor Heading Mode Control register using the SPI protocol.
*
* \param[in] response Pointer to the buffer containing the response from the VectorNav sensor, including the leading 0x00 byte.
* \param[out] maxRateError The register's Max Rate Error field.
* \param[out] reserved1 The register's Reserved1 field.
* \return Indicates any error encountered. */
VnError VnSpi_parseIndoorHeadingModeControl(
	const char* response,
	float* maxRateError, 
	uint8_t* reserved1);

/** \brief Parses a response from reading the Velocity Compensation Measurement register using the SPI protocol.
*
* \param[in] response Pointer to the buffer containing the response from the VectorNav sensor, including the leading 0x00 byte.
* \param[out] velocity The register's Velocity field.
* \return Indicates any error encountered. */
VnError VnSpi_parseVelocityCompensationMeasurement(
	const char* response,
	vec3f* velocity);

/** \brief Parses a response from reading the Velocity Compensation Control register using the SPI protocol.
*
* \param[in] response Pointer to the buffer containing the response from the VectorNav sensor, including the leading 0x00 byte.
* \param[out] mode The register's Mode field.
* \param[out] velocityTuning The register's VelocityTuning field.
* \param[out] rateTuning The register's RateTuning field.
* \return Indicates any error encountered. */
VnError VnSpi_parseVelocityCompensationControl(
	const char* response,
	uint8_t* mode, 
	float* velocityTuning, 
	float* rateTuning);

/** \brief Parses a response from reading the Velocity Compensation Status register using the SPI protocol.
*
* \param[in] response Pointer to the buffer containing the response from the VectorNav sensor, including the leading 0x00 byte.
* \param[out] x The register's x field.
* \param[out] xDot The register's xDot field.
* \param[out] accelOffset The register's accelOffset field.
* \param[out] omega The register's omega field.
* \return Indicates any error encountered. */
VnError VnSpi_parseVelocityCompensationStatus(
	const char* response,
	float* x, 
	float* xDot, 
	vec3f* accelOffset, 
	vec3f* omega);

/** \brief Parses a response from reading the IMU Measurements register using the SPI protocol.
*
* \param[in] response Pointer to the buffer containing the response from the VectorNav sensor, including the leading 0x00 byte.
* \param[out] mag The register's Mag field.
* \param[out] accel The register's Accel field.
* \param[out] gyro The register's Gyro field.
* \param[out] temp The register's Temp field.
* \param[out] pressure The register's Pressure field.
* \return Indicates any error encountered. */
VnError VnSpi_parseImuMeasurements(
	const char* response,
	vec3f* mag, 
	vec3f* accel, 
	vec3f* gyro, 
	float* temp, 
	float* pressure);

/** \brief Parses a response from reading the GPS Configuration register using the SPI protocol.
*
* \param[in] response Pointer to the buffer containing the response from the VectorNav sensor, including the leading 0x00 byte.
* \param[out] mode The register's Mode field.
* \param[out] ppsSource The register's PpsSource field.
* \param[out] reserved1 The register's Reserved1 field.
* \param[out] reserved2 The register's Reserved2 field.
* \param[out] reserved3 The register's Reserved3 field.
* \return Indicates any error encountered. */
VnError VnSpi_parseGpsConfiguration(
	const char* response,
	uint8_t* mode, 
	uint8_t* ppsSource, 
	uint8_t* reserved1, 
	uint8_t* reserved2, 
	uint8_t* reserved3);

/** \brief Parses a response from reading the GPS Antenna Offset register using the SPI protocol.
*
* \param[in] response Pointer to the buffer containing the response from the VectorNav sensor, including the leading 0x00 byte.
* \param[out] position The register's Position field.
* \return Indicates any error encountered. */
VnError VnSpi_parseGpsAntennaOffset(
	const char* response,
	vec3f* position);

/** \brief Parses a response from reading the GPS Solution - LLA register using the SPI protocol.
*
* \param[in] response Pointer to the buffer containing the response from the VectorNav sensor, including the leading 0x00 byte.
* \param[out] time The register's Time field.
* \param[out] week The register's Week field.
* \param[out] gpsFix The register's GpsFix field.
* \param[out] numSats The register's NumSats field.
* \param[out] lla The register's Lla field.
* \param[out] nedVel The register's NedVel field.
* \param[out] nedAcc The register's NedAcc field.
* \param[out] speedAcc The register's SpeedAcc field.
* \param[out] timeAcc The register's TimeAcc field.
* \return Indicates any error encountered. */
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
	float* timeAcc);

/** \brief Parses a response from reading the GPS Solution - ECEF register using the SPI protocol.
*
* \param[in] response Pointer to the buffer containing the response from the VectorNav sensor, including the leading 0x00 byte.
* \param[out] tow The register's Tow field.
* \param[out] week The register's Week field.
* \param[out] gpsFix The register's GpsFix field.
* \param[out] numSats The register's NumSats field.
* \param[out] position The register's Position field.
* \param[out] velocity The register's Velocity field.
* \param[out] posAcc The register's PosAcc field.
* \param[out] speedAcc The register's SpeedAcc field.
* \param[out] timeAcc The register's TimeAcc field.
* \return Indicates any error encountered. */
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
	float* timeAcc);

/** \brief Parses a response from reading the INS Solution - LLA register using the SPI protocol.
*
* \param[in] response Pointer to the buffer containing the response from the VectorNav sensor, including the leading 0x00 byte.
* \param[out] time The register's Time field.
* \param[out] week The register's Week field.
* \param[out] status The register's Status field.
* \param[out] yawPitchRoll The register's YawPitchRoll field.
* \param[out] position The register's Position field.
* \param[out] nedVel The register's NedVel field.
* \param[out] attUncertainty The register's AttUncertainty field.
* \param[out] posUncertainty The register's PosUncertainty field.
* \param[out] velUncertainty The register's VelUncertainty field.
* \return Indicates any error encountered. */
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
	float* velUncertainty);

/** \brief Parses a response from reading the INS Solution - ECEF register using the SPI protocol.
*
* \param[in] response Pointer to the buffer containing the response from the VectorNav sensor, including the leading 0x00 byte.
* \param[out] time The register's Time field.
* \param[out] week The register's Week field.
* \param[out] status The register's Status field.
* \param[out] yawPitchRoll The register's YawPitchRoll field.
* \param[out] position The register's Position field.
* \param[out] velocity The register's Velocity field.
* \param[out] attUncertainty The register's AttUncertainty field.
* \param[out] posUncertainty The register's PosUncertainty field.
* \param[out] velUncertainty The register's VelUncertainty field.
* \return Indicates any error encountered. */
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
	float* velUncertainty);

/** \brief Parses a response from reading the INS Basic Configuration register using the SPI protocol.
*
* \param[in] response Pointer to the buffer containing the response from the VectorNav sensor, including the leading 0x00 byte.
* \param[out] scenario The register's Scenario field.
* \param[out] ahrsAiding The register's AhrsAiding field.
* \param[out] estBaseline The register's EstBaseline field.
* \param[out] resv2 The register's Resv2 field.
* \return Indicates any error encountered. */
VnError VnSpi_parseInsBasicConfiguration(
	const char* response,
	uint8_t* scenario, 
	uint8_t* ahrsAiding, 
	uint8_t* estBaseline, 
	uint8_t* resv2);

/** \brief Parses a response from reading the INS Advanced Configuration register using the SPI protocol.
*
* \param[in] response Pointer to the buffer containing the response from the VectorNav sensor, including the leading 0x00 byte.
* \param[out] useMag The register's UseMag field.
* \param[out] usePres The register's UsePres field.
* \param[out] posAtt The register's PosAtt field.
* \param[out] velAtt The register's VelAtt field.
* \param[out] velBias The register's VelBias field.
* \param[out] useFoam The register's UseFoam field.
* \param[out] gpsCovType The register's GPSCovType field.
* \param[out] velCount The register's VelCount field.
* \param[out] velInit The register's VelInit field.
* \param[out] moveOrigin The register's MoveOrigin field.
* \param[out] gpsTimeout The register's GPSTimeout field.
* \param[out] deltaLimitPos The register's DeltaLimitPos field.
* \param[out] deltaLimitVel The register's DeltaLimitVel field.
* \param[out] minPosUncertainty The register's MinPosUncertainty field.
* \param[out] minVelUncertainty The register's MinVelUncertainty field.
* \return Indicates any error encountered. */
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
	float* minVelUncertainty);

/** \brief Parses a response from reading the INS State - LLA register using the SPI protocol.
*
* \param[in] response Pointer to the buffer containing the response from the VectorNav sensor, including the leading 0x00 byte.
* \param[out] yawPitchRoll The register's YawPitchRoll field.
* \param[out] position The register's Position field.
* \param[out] velocity The register's Velocity field.
* \param[out] accel The register's Accel field.
* \param[out] angularRate The register's AngularRate field.
* \return Indicates any error encountered. */
VnError VnSpi_parseInsStateLla(
	const char* response,
	vec3f* yawPitchRoll, 
	vec3d* position, 
	vec3f* velocity, 
	vec3f* accel, 
	vec3f* angularRate);

/** \brief Parses a response from reading the INS State - ECEF register using the SPI protocol.
*
* \param[in] response Pointer to the buffer containing the response from the VectorNav sensor, including the leading 0x00 byte.
* \param[out] yawPitchRoll The register's YawPitchRoll field.
* \param[out] position The register's Position field.
* \param[out] velocity The register's Velocity field.
* \param[out] accel The register's Accel field.
* \param[out] angularRate The register's AngularRate field.
* \return Indicates any error encountered. */
VnError VnSpi_parseInsStateEcef(
	const char* response,
	vec3f* yawPitchRoll, 
	vec3d* position, 
	vec3f* velocity, 
	vec3f* accel, 
	vec3f* angularRate);

/** \brief Parses a response from reading the Startup Filter Bias Estimate register using the SPI protocol.
*
* \param[in] response Pointer to the buffer containing the response from the VectorNav sensor, including the leading 0x00 byte.
* \param[out] gyroBias The register's GyroBias field.
* \param[out] accelBias The register's AccelBias field.
* \param[out] pressureBias The register's PressureBias field.
* \return Indicates any error encountered. */
VnError VnSpi_parseStartupFilterBiasEstimate(
	const char* response,
	vec3f* gyroBias, 
	vec3f* accelBias, 
	float* pressureBias);

/** \brief Parses a response from reading the Delta Theta and Delta Velocity register using the SPI protocol.
*
* \param[in] response Pointer to the buffer containing the response from the VectorNav sensor, including the leading 0x00 byte.
* \param[out] deltaTime The register's DeltaTime field.
* \param[out] deltaTheta The register's DeltaTheta field.
* \param[out] deltaVelocity The register's DeltaVelocity field.
* \return Indicates any error encountered. */
VnError VnSpi_parseDeltaThetaAndDeltaVelocity(
	const char* response,
	float* deltaTime, 
	vec3f* deltaTheta, 
	vec3f* deltaVelocity);

/** \brief Parses a response from reading the Delta Theta and Delta Velocity Configuration register using the SPI protocol.
*
* \param[in] response Pointer to the buffer containing the response from the VectorNav sensor, including the leading 0x00 byte.
* \param[out] integrationFrame The register's IntegrationFrame field.
* \param[out] gyroCompensation The register's GyroCompensation field.
* \param[out] accelCompensation The register's AccelCompensation field.
* \param[out] reserved1 The register's Reserved1 field.
* \param[out] reserved2 The register's Reserved2 field.
* \return Indicates any error encountered. */
VnError VnSpi_parseDeltaThetaAndDeltaVelocityConfiguration(
	const char* response,
	uint8_t* integrationFrame, 
	uint8_t* gyroCompensation, 
	uint8_t* accelCompensation, 
	uint8_t* reserved1, 
	uint16_t* reserved2);

/** \brief Parses a response from reading the Reference Vector Configuration register using the SPI protocol.
*
* \param[in] response Pointer to the buffer containing the response from the VectorNav sensor, including the leading 0x00 byte.
* \param[out] useMagModel The register's UseMagModel field.
* \param[out] useGravityModel The register's UseGravityModel field.
* \param[out] resv1 The register's Resv1 field.
* \param[out] resv2 The register's Resv2 field.
* \param[out] recalcThreshold The register's RecalcThreshold field.
* \param[out] year The register's Year field.
* \param[out] position The register's Position field.
* \return Indicates any error encountered. */
VnError VnSpi_parseReferenceVectorConfiguration(
	const char* response,
	uint8_t* useMagModel, 
	uint8_t* useGravityModel, 
	uint8_t* resv1, 
	uint8_t* resv2, 
	uint32_t* recalcThreshold, 
	float* year, 
	vec3d* position);

/** \brief Parses a response from reading the Gyro Compensation register using the SPI protocol.
*
* \param[in] response Pointer to the buffer containing the response from the VectorNav sensor, including the leading 0x00 byte.
* \param[out] c The register's C field.
* \param[out] b The register's B field.
* \return Indicates any error encountered. */
VnError VnSpi_parseGyroCompensation(
	const char* response,
	mat3f* c, 
	vec3f* b);

/** \brief Parses a response from reading the IMU Filtering Configuration register using the SPI protocol.
*
* \param[in] response Pointer to the buffer containing the response from the VectorNav sensor, including the leading 0x00 byte.
* \param[out] magWindowSize The register's MagWindowSize field.
* \param[out] accelWindowSize The register's AccelWindowSize field.
* \param[out] gyroWindowSize The register's GyroWindowSize field.
* \param[out] tempWindowSize The register's TempWindowSize field.
* \param[out] presWindowSize The register's PresWindowSize field.
* \param[out] magFilterMode The register's MagFilterMode field.
* \param[out] accelFilterMode The register's AccelFilterMode field.
* \param[out] gyroFilterMode The register's GyroFilterMode field.
* \param[out] tempFilterMode The register's TempFilterMode field.
* \param[out] presFilterMode The register's PresFilterMode field.
* \return Indicates any error encountered. */
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
	uint8_t* presFilterMode);

/** \brief Parses a response from reading the GPS Compass Baseline register using the SPI protocol.
*
* \param[in] response Pointer to the buffer containing the response from the VectorNav sensor, including the leading 0x00 byte.
* \param[out] position The register's Position field.
* \param[out] uncertainty The register's Uncertainty field.
* \return Indicates any error encountered. */
VnError VnSpi_parseGpsCompassBaseline(
	const char* response,
	vec3f* position, 
	vec3f* uncertainty);

/** \brief Parses a response from reading the GPS Compass Estimated Baseline register using the SPI protocol.
*
* \param[in] response Pointer to the buffer containing the response from the VectorNav sensor, including the leading 0x00 byte.
* \param[out] estBaselineUsed The register's EstBaselineUsed field.
* \param[out] resv The register's Resv field.
* \param[out] numMeas The register's NumMeas field.
* \param[out] position The register's Position field.
* \param[out] uncertainty The register's Uncertainty field.
* \return Indicates any error encountered. */
VnError VnSpi_parseGpsCompassEstimatedBaseline(
	const char* response,
	uint8_t* estBaselineUsed, 
	uint8_t* resv, 
	uint16_t* numMeas, 
	vec3f* position, 
	vec3f* uncertainty);

/** \brief Parses a response from reading the IMU Rate Configuration register using the SPI protocol.
*
* \param[in] response Pointer to the buffer containing the response from the VectorNav sensor, including the leading 0x00 byte.
* \param[out] imuRate The register's imuRate field.
* \param[out] navDivisor The register's NavDivisor field.
* \param[out] filterTargetRate The register's filterTargetRate field.
* \param[out] filterMinRate The register's filterMinRate field.
* \return Indicates any error encountered. */
VnError VnSpi_parseImuRateConfiguration(
	const char* response,
	uint16_t* imuRate, 
	uint16_t* navDivisor, 
	float* filterTargetRate, 
	float* filterMinRate);

/** \brief Parses a response from reading the Yaw, Pitch, Roll, True Body Acceleration and Angular Rates register using the SPI protocol.
*
* \param[in] response Pointer to the buffer containing the response from the VectorNav sensor, including the leading 0x00 byte.
* \param[out] yawPitchRoll The register's YawPitchRoll field.
* \param[out] bodyAccel The register's BodyAccel field.
* \param[out] gyro The register's Gyro field.
* \return Indicates any error encountered. */
VnError VnSpi_parseYawPitchRollTrueBodyAccelerationAndAngularRates(
	const char* response,
	vec3f* yawPitchRoll, 
	vec3f* bodyAccel, 
	vec3f* gyro);

/** \brief Parses a response from reading the Yaw, Pitch, Roll, True Inertial Acceleration and Angular Rates register using the SPI protocol.
*
* \param[in] response Pointer to the buffer containing the response from the VectorNav sensor, including the leading 0x00 byte.
* \param[out] yawPitchRoll The register's YawPitchRoll field.
* \param[out] inertialAccel The register's InertialAccel field.
* \param[out] gyro The register's Gyro field.
* \return Indicates any error encountered. */
VnError VnSpi_parseYawPitchRollTrueInertialAccelerationAndAngularRates(
	const char* response,
	vec3f* yawPitchRoll, 
	vec3f* inertialAccel, 
	vec3f* gyro);

/** \defgroup spi_genread_functions SPI Generate Read Functions
* \brief This set of functions will generate command strings for reading
*     registers on VectorNav sensors using the SPI protocol.
*
* These functions take the form of
* <c>VnError VnSpi_genReadXXX(char *buffer, size_t* size, size_t desiredLength, size_t* responseSize)</c>
* where XXX is replaced with the name of the register, <c>buffer</c> is provided by
* the user to be filled with the generated command, <c>size</c> is the number of
* bytes available in the provided buffer and will contain the number of bytes of the command
* to send, <c>desiredLength</c> is the total number of bytes to pad with 0x00 should
* the total constructed length of the command be less than the desiredLength,
* <c>responseSize</c> is the number of bytes to read during the SPI transaction to get this
* command's response.
*
* \{ */

/** \brief Generates a command to read the User Tag register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \return Indicates any errors encountered. */
VnError VnSpi_genReadUserTag(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize);

/** \brief Generates a command to read the Model Number register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \return Indicates any errors encountered. */
VnError VnSpi_genReadModelNumber(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize);

/** \brief Generates a command to read the Hardware Revision register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \return Indicates any errors encountered. */
VnError VnSpi_genReadHardwareRevision(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize);

/** \brief Generates a command to read the Serial Number register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \return Indicates any errors encountered. */
VnError VnSpi_genReadSerialNumber(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize);

/** \brief Generates a command to read the Firmware Version register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \return Indicates any errors encountered. */
VnError VnSpi_genReadFirmwareVersion(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize);

/** \brief Generates a command to read the Serial Baud Rate register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \return Indicates any errors encountered. */
VnError VnSpi_genReadSerialBaudRate(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize);

/** \brief Generates a command to read the Async Data Output Type register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \return Indicates any errors encountered. */
VnError VnSpi_genReadAsyncDataOutputType(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize);

/** \brief Generates a command to read the Async Data Output Frequency register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \return Indicates any errors encountered. */
VnError VnSpi_genReadAsyncDataOutputFrequency(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize);

/** \brief Generates a command to read the Yaw Pitch Roll register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \return Indicates any errors encountered. */
VnError VnSpi_genReadYawPitchRoll(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize);

/** \brief Generates a command to read the Attitude Quaternion register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \return Indicates any errors encountered. */
VnError VnSpi_genReadAttitudeQuaternion(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize);

/** \brief Generates a command to read the Quaternion, Magnetic, Acceleration and Angular Rates register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \return Indicates any errors encountered. */
VnError VnSpi_genReadQuaternionMagneticAccelerationAndAngularRates(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize);

/** \brief Generates a command to read the Magnetic Measurements register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \return Indicates any errors encountered. */
VnError VnSpi_genReadMagneticMeasurements(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize);

/** \brief Generates a command to read the Acceleration Measurements register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \return Indicates any errors encountered. */
VnError VnSpi_genReadAccelerationMeasurements(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize);

/** \brief Generates a command to read the Angular Rate Measurements register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \return Indicates any errors encountered. */
VnError VnSpi_genReadAngularRateMeasurements(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize);

/** \brief Generates a command to read the Magnetic, Acceleration and Angular Rates register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \return Indicates any errors encountered. */
VnError VnSpi_genReadMagneticAccelerationAndAngularRates(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize);

/** \brief Generates a command to read the Magnetic and Gravity Reference Vectors register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \return Indicates any errors encountered. */
VnError VnSpi_genReadMagneticAndGravityReferenceVectors(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize);

/** \brief Generates a command to read the Magnetometer Compensation register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \return Indicates any errors encountered. */
VnError VnSpi_genReadMagnetometerCompensation(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize);

/** \brief Generates a command to read the Acceleration Compensation register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \return Indicates any errors encountered. */
VnError VnSpi_genReadAccelerationCompensation(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize);

/** \brief Generates a command to read the Reference Frame Rotation register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \return Indicates any errors encountered. */
VnError VnSpi_genReadReferenceFrameRotation(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize);

/** \brief Generates a command to read the Yaw, Pitch, Roll, Magnetic, Acceleration and Angular Rates register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \return Indicates any errors encountered. */
VnError VnSpi_genReadYawPitchRollMagneticAccelerationAndAngularRates(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize);

/** \brief Generates a command to read the Communication Protocol Control register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \return Indicates any errors encountered. */
VnError VnSpi_genReadCommunicationProtocolControl(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize);

/** \brief Generates a command to read the Synchronization Control register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \return Indicates any errors encountered. */
VnError VnSpi_genReadSynchronizationControl(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize);

/** \brief Generates a command to read the Synchronization Status register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \return Indicates any errors encountered. */
VnError VnSpi_genReadSynchronizationStatus(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize);

/** \brief Generates a command to read the VPE Basic Control register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \return Indicates any errors encountered. */
VnError VnSpi_genReadVpeBasicControl(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize);

/** \brief Generates a command to read the VPE Magnetometer Basic Tuning register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \return Indicates any errors encountered. */
VnError VnSpi_genReadVpeMagnetometerBasicTuning(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize);

/** \brief Generates a command to read the VPE Accelerometer Basic Tuning register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \return Indicates any errors encountered. */
VnError VnSpi_genReadVpeAccelerometerBasicTuning(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize);

/** \brief Generates a command to read the Magnetometer Calibration Control register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \return Indicates any errors encountered. */
VnError VnSpi_genReadMagnetometerCalibrationControl(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize);

/** \brief Generates a command to read the Calculated Magnetometer Calibration register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \return Indicates any errors encountered. */
VnError VnSpi_genReadCalculatedMagnetometerCalibration(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize);

/** \brief Generates a command to read the Velocity Compensation Measurement register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \return Indicates any errors encountered. */
VnError VnSpi_genReadVelocityCompensationMeasurement(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize);

/** \brief Generates a command to read the Velocity Compensation Control register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \return Indicates any errors encountered. */
VnError VnSpi_genReadVelocityCompensationControl(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize);

/** \brief Generates a command to read the IMU Measurements register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \return Indicates any errors encountered. */
VnError VnSpi_genReadImuMeasurements(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize);

/** \brief Generates a command to read the GPS Configuration register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \return Indicates any errors encountered. */
VnError VnSpi_genReadGpsConfiguration(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize);

/** \brief Generates a command to read the GPS Antenna Offset register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \return Indicates any errors encountered. */
VnError VnSpi_genReadGpsAntennaOffset(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize);

/** \brief Generates a command to read the GPS Solution - LLA register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \return Indicates any errors encountered. */
VnError VnSpi_genReadGpsSolutionLla(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize);

/** \brief Generates a command to read the GPS Solution - ECEF register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \return Indicates any errors encountered. */
VnError VnSpi_genReadGpsSolutionEcef(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize);

/** \brief Generates a command to read the INS Solution - LLA register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \return Indicates any errors encountered. */
VnError VnSpi_genReadInsSolutionLla(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize);

/** \brief Generates a command to read the INS Solution - ECEF register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \return Indicates any errors encountered. */
VnError VnSpi_genReadInsSolutionEcef(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize);

/** \brief Generates a command to read the INS Basic Configuration register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \return Indicates any errors encountered. */
VnError VnSpi_genReadInsBasicConfiguration(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize);

/** \brief Generates a command to read the INS Basic Configuration register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \return Indicates any errors encountered. */
VnError VnSpi_genReadInsBasicConfiguration(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize);

/** \brief Generates a command to read the INS State - LLA register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \return Indicates any errors encountered. */
VnError VnSpi_genReadInsStateLla(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize);

/** \brief Generates a command to read the INS State - ECEF register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \return Indicates any errors encountered. */
VnError VnSpi_genReadInsStateEcef(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize);

/** \brief Generates a command to read the Startup Filter Bias Estimate register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \return Indicates any errors encountered. */
VnError VnSpi_genReadStartupFilterBiasEstimate(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize);

/** \brief Generates a command to read the Delta Theta and Delta Velocity register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \return Indicates any errors encountered. */
VnError VnSpi_genReadDeltaThetaAndDeltaVelocity(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize);

/** \brief Generates a command to read the Delta Theta and Delta Velocity Configuration register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \return Indicates any errors encountered. */
VnError VnSpi_genReadDeltaThetaAndDeltaVelocityConfiguration(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize);

/** \brief Generates a command to read the Reference Vector Configuration register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \return Indicates any errors encountered. */
VnError VnSpi_genReadReferenceVectorConfiguration(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize);

/** \brief Generates a command to read the Gyro Compensation register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \return Indicates any errors encountered. */
VnError VnSpi_genReadGyroCompensation(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize);

/** \brief Generates a command to read the IMU Filtering Configuration register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \return Indicates any errors encountered. */
VnError VnSpi_genReadImuFilteringConfiguration(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize);

/** \brief Generates a command to read the GPS Compass Baseline register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \return Indicates any errors encountered. */
VnError VnSpi_genReadGpsCompassBaseline(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize);

/** \brief Generates a command to read the GPS Compass Estimated Baseline register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \return Indicates any errors encountered. */
VnError VnSpi_genReadGpsCompassEstimatedBaseline(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize);

/** \brief Generates a command to read the Yaw, Pitch, Roll, True Body Acceleration and Angular Rates register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \return Indicates any errors encountered. */
VnError VnSpi_genReadYawPitchRollTrueBodyAccelerationAndAngularRates(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize);

/** \brief Generates a command to read the Yaw, Pitch, Roll, True Inertial Acceleration and Angular Rates register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \return Indicates any errors encountered. */
VnError VnSpi_genReadYawPitchRollTrueInertialAccelerationAndAngularRates(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize);

/* \}*/

/** \defgroup spi_genwrite_functions SPI Generate Write Functions
* \brief This set of functions will generate command strings for writing to
*     registers on VectorNav sensors using the SPI protocol.
*
* These functions take the form shown below. <c>XXX</c> is replaced by the name
* of the register, <c>buffer</c> is provided by the user to be filled with the
* generated command, <c>size</c> is the number of bytes available in the provided
* buffer and will contain the number of bytes of the command to send, <c>desiredLength</c>
* is the total number of bytes to pad with 0x00 should the total constructed length
* of the command be less than the desired length, and <c>[Variable argument list]</c>
* varies with the specified register being written to.
*
* \code
 * VnError VnSpi_genWriteXXX(
 *     char *buffer,
 *     size_t* size,
 *     size_t desiredLength,
 *     size_t* responseSize,
 *     [Variable argument list]);
 * \endcode
 *
 * \{ */

/** \brief Generates a command to write the User Tag register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \param[in] tag The register's Tag field. This should be a null-terminated string.
 * \return Indicates any errors encountered. */
VnError VnSpi_genWriteUserTag(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize,
	char* tag);

/** \brief Generates a command to write the Serial Baud Rate register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \param[in] baudrate The register's Baud Rate field.
 * \return Indicates any errors encountered. */
VnError VnSpi_genWriteSerialBaudRate(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize,
	uint32_t baudrate);

/** \brief Generates a command to write the Serial Baud Rate register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \param[in] baudrate The register's Baud Rate field.
 * \return Indicates any errors encountered. */
VnError VnSpi_genWriteSerialBaudRateWithOptions(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize,
	uint32_t baudrate);

/** \brief Generates a command to write the Async Data Output Type register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \param[in] ador The register's ADOR field.
 * \return Indicates any errors encountered. */
VnError VnSpi_genWriteAsyncDataOutputType(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize,
	uint32_t ador);

/** \brief Generates a command to write the Async Data Output Type register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \param[in] ador The register's ADOR field.
 * \return Indicates any errors encountered. */
VnError VnSpi_genWriteAsyncDataOutputTypeWithOptions(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize,
	uint32_t ador);

/** \brief Generates a command to write the Async Data Output Frequency register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \param[in] adof The register's ADOF field.
 * \return Indicates any errors encountered. */
VnError VnSpi_genWriteAsyncDataOutputFrequency(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize,
	uint32_t adof);

/** \brief Generates a command to write the Async Data Output Frequency register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \param[in] adof The register's ADOF field.
 * \return Indicates any errors encountered. */
VnError VnSpi_genWriteAsyncDataOutputFrequencyWithOptions(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize,
	uint32_t adof);

/** \brief Generates a command to write the Magnetic and Gravity Reference Vectors register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \param[in] magRef The register's MagRef field.
 * \param[in] accRef The register's AccRef field.
 * \return Indicates any errors encountered. */
VnError VnSpi_genWriteMagneticAndGravityReferenceVectors(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize,
	vec3f magRef,
	vec3f accRef);

/** \brief Generates a command to write the Magnetometer Compensation register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \param[in] c The register's C field.
 * \param[in] b The register's B field.
 * \return Indicates any errors encountered. */
VnError VnSpi_genWriteMagnetometerCompensation(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize,
	mat3f c,
	vec3f b);

/** \brief Generates a command to write the Acceleration Compensation register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \param[in] c The register's C field.
 * \param[in] b The register's B field.
 * \return Indicates any errors encountered. */
VnError VnSpi_genWriteAccelerationCompensation(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize,
	mat3f c,
	vec3f b);

/** \brief Generates a command to write the Reference Frame Rotation register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \param[in] c The register's C field.
 * \return Indicates any errors encountered. */
VnError VnSpi_genWriteReferenceFrameRotation(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize,
	mat3f c);

/** \brief Generates a command to write the Communication Protocol Control register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \param[in] serialCount The register's SerialCount field.
 * \param[in] serialStatus The register's SerialStatus field.
 * \param[in] spiCount The register's SPICount field.
 * \param[in] spiStatus The register's SPIStatus field.
 * \param[in] serialChecksum The register's SerialChecksum field.
 * \param[in] spiChecksum The register's SPIChecksum field.
 * \param[in] errorMode The register's ErrorMode field.
 * \return Indicates any errors encountered. */
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
	uint8_t errorMode);

/** \brief Generates a command to write the Synchronization Control register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \param[in] syncInMode The register's SyncInMode field.
 * \param[in] syncInEdge The register's SyncInEdge field.
 * \param[in] syncInSkipFactor The register's SyncInSkipFactor field.
 * \param[in] reserved1 The register's RESERVED1 field.
 * \param[in] syncOutMode The register's SyncOutMode field.
 * \param[in] syncOutPolarity The register's SyncOutPolarity field.
 * \param[in] syncOutSkipFactor The register's SyncOutSkipFactor field.
 * \param[in] syncOutPulseWidth The register's SyncOutPulseWidth field.
 * \param[in] reserved2 The register's RESERVED2 field.
 * \return Indicates any errors encountered. */
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
	uint32_t reserved2);

/** \brief Generates a command to write the Synchronization Status register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \param[in] syncInCount The register's SyncInCount field.
 * \param[in] syncInTime The register's SyncInTime field.
 * \param[in] syncOutCount The register's SyncOutCount field.
 * \return Indicates any errors encountered. */
VnError VnSpi_genWriteSynchronizationStatus(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize,
	uint32_t syncInCount,
	uint32_t syncInTime,
	uint32_t syncOutCount);

/** \brief Generates a command to write the VPE Basic Control register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \param[in] enable The register's Enable field.
 * \param[in] headingMode The register's HeadingMode field.
 * \param[in] filteringMode The register's FilteringMode field.
 * \param[in] tuningMode The register's TuningMode field.
 * \return Indicates any errors encountered. */
VnError VnSpi_genWriteVpeBasicControl(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize,
	uint8_t enable,
	uint8_t headingMode,
	uint8_t filteringMode,
	uint8_t tuningMode);

/** \brief Generates a command to write the VPE Magnetometer Basic Tuning register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \param[in] baseTuning The register's BaseTuning field.
 * \param[in] adaptiveTuning The register's AdaptiveTuning field.
 * \param[in] adaptiveFiltering The register's AdaptiveFiltering field.
 * \return Indicates any errors encountered. */
VnError VnSpi_genWriteVpeMagnetometerBasicTuning(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize,
	vec3f baseTuning,
	vec3f adaptiveTuning,
	vec3f adaptiveFiltering);

/** \brief Generates a command to write the VPE Accelerometer Basic Tuning register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \param[in] baseTuning The register's BaseTuning field.
 * \param[in] adaptiveTuning The register's AdaptiveTuning field.
 * \param[in] adaptiveFiltering The register's AdaptiveFiltering field.
 * \return Indicates any errors encountered. */
VnError VnSpi_genWriteVpeAccelerometerBasicTuning(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize,
	vec3f baseTuning,
	vec3f adaptiveTuning,
	vec3f adaptiveFiltering);

/** \brief Generates a command to write the Magnetometer Calibration Control register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \param[in] hsiMode The register's HSIMode field.
 * \param[in] hsiOutput The register's HSIOutput field.
 * \param[in] convergeRate The register's ConvergeRate field.
 * \return Indicates any errors encountered. */
VnError VnSpi_genWriteMagnetometerCalibrationControl(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize,
	uint8_t hsiMode,
	uint8_t hsiOutput,
	uint8_t convergeRate);

/** \brief Generates a command to write the Velocity Compensation Measurement register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \param[in] velocity The register's Velocity field.
 * \return Indicates any errors encountered. */
VnError VnSpi_genWriteVelocityCompensationMeasurement(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize,
	vec3f velocity);

/** \brief Generates a command to write the Velocity Compensation Control register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \param[in] mode The register's Mode field.
 * \param[in] velocityTuning The register's VelocityTuning field.
 * \param[in] rateTuning The register's RateTuning field.
 * \return Indicates any errors encountered. */
VnError VnSpi_genWriteVelocityCompensationControl(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize,
	uint8_t mode,
	float velocityTuning,
	float rateTuning);

/** \brief Generates a command to write the GPS Configuration register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \param[in] mode The register's Mode field.
 * \param[in] ppsSource The register's PpsSource field.
 * \param[in] reserved1 The register's Reserved1 field.
 * \param[in] reserved2 The register's Reserved2 field.
 * \param[in] reserved3 The register's Reserved3 field.
 * \return Indicates any errors encountered. */
VnError VnSpi_genWriteGpsConfiguration(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize,
	uint8_t mode,
	uint8_t ppsSource,
	uint8_t reserved1,
	uint8_t reserved2,
	uint8_t reserved3);

/** \brief Generates a command to write the GPS Antenna Offset register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \param[in] position The register's Position field.
 * \return Indicates any errors encountered. */
VnError VnSpi_genWriteGpsAntennaOffset(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize,
	vec3f position);

/** \brief Generates a command to write the INS Basic Configuration register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \param[in] scenario The register's Scenario field.
 * \param[in] ahrsAiding The register's AhrsAiding field.
 * \param[in] estBaseline The register's EstBaseline field.
 * \param[in] resv2 The register's Resv2 field.
 * \return Indicates any errors encountered. */
VnError VnSpi_genWriteInsBasicConfiguration(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize,
	uint8_t scenario,
	uint8_t ahrsAiding,
	uint8_t estBaseline,
	uint8_t resv2);

/** \brief Generates a command to write the Startup Filter Bias Estimate register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \param[in] gyroBias The register's GyroBias field.
 * \param[in] accelBias The register's AccelBias field.
 * \param[in] pressureBias The register's PressureBias field.
 * \return Indicates any errors encountered. */
VnError VnSpi_genWriteStartupFilterBiasEstimate(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize,
	vec3f gyroBias,
	vec3f accelBias,
	float pressureBias);

/** \brief Generates a command to write the Delta Theta and Delta Velocity Configuration register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \param[in] integrationFrame The register's IntegrationFrame field.
 * \param[in] gyroCompensation The register's GyroCompensation field.
 * \param[in] accelCompensation The register's AccelCompensation field.
 * \param[in] reserved1 The register's Reserved1 field.
 * \param[in] reserved2 The register's Reserved2 field.
 * \return Indicates any errors encountered. */
VnError VnSpi_genWriteDeltaThetaAndDeltaVelocityConfiguration(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize,
	uint8_t integrationFrame,
	uint8_t gyroCompensation,
	uint8_t accelCompensation,
	uint8_t reserved1,
	uint16_t reserved2);

/** \brief Generates a command to write the Reference Vector Configuration register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \param[in] useMagModel The register's UseMagModel field.
 * \param[in] useGravityModel The register's UseGravityModel field.
 * \param[in] resv1 The register's Resv1 field.
 * \param[in] resv2 The register's Resv2 field.
 * \param[in] recalcThreshold The register's RecalcThreshold field.
 * \param[in] year The register's Year field.
 * \param[in] position The register's Position field.
 * \return Indicates any errors encountered. */
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
	vec3d position);

/** \brief Generates a command to write the Gyro Compensation register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \param[in] c The register's C field.
 * \param[in] b The register's B field.
 * \return Indicates any errors encountered. */
VnError VnSpi_genWriteGyroCompensation(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize,
	mat3f c,
	vec3f b);

/** \brief Generates a command to write the IMU Filtering Configuration register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \param[in] magWindowSize The register's MagWindowSize field.
 * \param[in] accelWindowSize The register's AccelWindowSize field.
 * \param[in] gyroWindowSize The register's GyroWindowSize field.
 * \param[in] tempWindowSize The register's TempWindowSize field.
 * \param[in] presWindowSize The register's PresWindowSize field.
 * \param[in] magFilterMode The register's MagFilterMode field.
 * \param[in] accelFilterMode The register's AccelFilterMode field.
 * \param[in] gyroFilterMode The register's GyroFilterMode field.
 * \param[in] tempFilterMode The register's TempFilterMode field.
 * \param[in] presFilterMode The register's PresFilterMode field.
 * \return Indicates any errors encountered. */
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
	uint8_t presFilterMode);

/** \brief Generates a command to write the GPS Compass Baseline register on a VectorNav sensor.
 *
 * \param[in] buffer Caller provided buffer to place the generated command.
 * \param[in/out] size Number of bytes available in the buffer. Will contain on output the number of bytes in the command to send.
 * \param[in] desiredLength The total number of bytes to pad with 0x00 should
 *     the total constructed length of the command be less than the
 *     desiredLength. This is useful back-to-back command where the
 *     desiredLength will be the responseSize of the previous command sent.
 * \param[out] responseSize The number of bytes to read during the SPI transaction to get this commands response.
 * \param[in] position The register's Position field.
 * \param[in] uncertainty The register's Uncertainty field.
 * \return Indicates any errors encountered. */
VnError VnSpi_genWriteGpsCompassBaseline(
	char* buffer,
	size_t* size,
	size_t desiredLength,
	size_t* responseSize,
	vec3f position,
	vec3f uncertainty);

#ifdef __cplusplus
}
#endif

#endif
