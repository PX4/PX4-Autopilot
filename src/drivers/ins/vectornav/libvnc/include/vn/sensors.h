/** \file
* {COMMON_HEADER}
*
* \section Description
* This header file contains declarations for using VectorNav sensors.
*/
#ifndef _VNSENSORS_H_
#define _VNSENSORS_H_

#include <stddef.h>
#include <stdio.h>

#include "vn/int.h"
#include "vn/error.h"
#include "vn/enum.h"
#include "vn/bool.h"
#include "vn/protocol/upack.h"
#include "vn/protocol/upackf.h"
#include "vn/xplat/serialport.h"
#include "vn/xplat/event.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef _WIN32
#pragma warning(push)
#pragma warning(disable : 4820)
#endif

// Define the max record size of a firmware update file
#define MAXFIRMWAREUPDATERECORDSIZE 300

/** \defgroup registerStructures Register Structures
 * \brief These structures represent the various registers on a VecotorNav
 * sensor.
 *
 * \{ */


/** \brief Structure representing a Binary Output register.
 *
 * The field outputGroup available on the sensor's register is not necessary
 * in this structure since all read/writes operations will automatically
 * determine this from the settings for the individual groups within this
 * structure. */
typedef struct
{
	AsyncMode asyncMode;			/**< The asyncMode field. */
	uint16_t rateDivisor;			/**< The rateDivisor field. */
	CommonGroup commonField;		/**< Group 1 (Common) */
	TimeGroup timeField;			/**< Group 2 (Time) */
	ImuGroup imuField;				/**< Group 3 (IMU) */
	GpsGroup gpsField;				/**< Group 4 (GPS) */
	AttitudeGroup attitudeField;	/**< Group 5 (Attitude) */
	InsGroup insField;				/**< Group 6 (INS) */
  GpsGroup gps2Field;				/**< Group 7 (GPS2) */
} BinaryOutputRegister;

/** \brief Initializes a BinaryOutputRegister structure.
 *
 * \param[in] reg The BinaryOutputRegister structure to initialize.
 * \param[in] asyncMode Value to initialize the asyncMode field with.
 * \param[in] rateDivisor Value to initialize the rateDivisor field with.
 * \param[in] commonField Value to initialize the commonField with.
 * \param[in] timeField Value to initialize the timeField with.
 * \param[in] imuField Value to initialize the imuField with.
 * \param[in] gpsField Value to initialize the gpsField with.
 * \param[in] attitudeField Value to initialize the attitudeField with.
 * \param[in] insField Value to initialize the insField with. */
void BinaryOutputRegister_initialize(
	BinaryOutputRegister *reg,
	AsyncMode asyncMode,
	uint32_t rateDivisor,
	CommonGroup commonField,
	TimeGroup timeField,
	ImuGroup imuField,
	GpsGroup gpsField,
	AttitudeGroup attitudeField,
	InsGroup insField,
  GpsGroup gps2Field);

/** \brief Structure representing the Quaternion, Magnetic, Acceleration and Angular Rates register. */
typedef struct
{
	/** \brief The Quat field. */
	vec4f quat;

	/** \brief The Mag field. */
	vec3f mag;

	/** \brief The Accel field. */
	vec3f accel;

	/** \brief The Gyro field. */
	vec3f gyro;

} QuaternionMagneticAccelerationAndAngularRatesRegister;

/** \brief Structure representing the Magnetic, Acceleration and Angular Rates register. */
typedef struct
{
	/** \brief The Mag field. */
	vec3f mag;

	/** \brief The Accel field. */
	vec3f accel;

	/** \brief The Gyro field. */
	vec3f gyro;

} MagneticAccelerationAndAngularRatesRegister;

/** \brief Structure representing the Magnetic and Gravity Reference Vectors register. */
typedef struct
{
	/** \brief The MagRef field. */
	vec3f magRef;

	/** \brief The AccRef field. */
	vec3f accRef;

} MagneticAndGravityReferenceVectorsRegister;

/** \brief Structure representing the Filter Measurements Variance Parameters register. */
typedef struct
{
	/** \brief The Angular Walk Variance field. */
	float angularWalkVariance;

	/** \brief The Angular Rate Variance field. */
	vec3f angularRateVariance;

	/** \brief The Magnetic Variance field. */
	vec3f magneticVariance;

	/** \brief The Acceleration Variance field. */
	vec3f accelerationVariance;

} FilterMeasurementsVarianceParametersRegister;

/** \brief Structure representing the Magnetometer Compensation register. */
typedef struct
{
	/** \brief The C field. */
	mat3f c;

	/** \brief The B field. */
	vec3f b;

} MagnetometerCompensationRegister;

/** \brief Structure representing the Filter Active Tuning Parameters register. */
typedef struct
{
	/** \brief The Magnetic Disturbance Gain field. */
	float magneticDisturbanceGain;

	/** \brief The Acceleration Disturbance Gain field. */
	float accelerationDisturbanceGain;

	/** \brief The Magnetic Disturbance Memory field. */
	float magneticDisturbanceMemory;

	/** \brief The Acceleration Disturbance Memory field. */
	float accelerationDisturbanceMemory;

} FilterActiveTuningParametersRegister;

/** \brief Structure representing the Acceleration Compensation register. */
typedef struct
{
	/** \brief The C field. */
	mat3f c;

	/** \brief The B field. */
	vec3f b;

} AccelerationCompensationRegister;

/** \brief Structure representing the Yaw, Pitch, Roll, Magnetic, Acceleration and Angular Rates register. */
typedef struct
{
	/** \brief The YawPitchRoll field. */
	vec3f yawPitchRoll;

	/** \brief The Mag field. */
	vec3f mag;

	/** \brief The Accel field. */
	vec3f accel;

	/** \brief The Gyro field. */
	vec3f gyro;

} YawPitchRollMagneticAccelerationAndAngularRatesRegister;

/** \brief Structure representing the Communication Protocol Control register. */
typedef struct
{
	/** \brief The SerialCount field. */
	uint8_t serialCount;

	/** \brief The SerialStatus field. */
	uint8_t serialStatus;

	/** \brief The SPICount field. */
	uint8_t spiCount;

	/** \brief The SPIStatus field. */
	uint8_t spiStatus;

	/** \brief The SerialChecksum field. */
	uint8_t serialChecksum;

	/** \brief The SPIChecksum field. */
	uint8_t spiChecksum;

	/** \brief The ErrorMode field. */
	uint8_t errorMode;

} CommunicationProtocolControlRegister;

/** \brief Structure representing the Synchronization Control register. */
typedef struct
{
	/** \brief The SyncInMode field. */
	uint8_t syncInMode;

	/** \brief The SyncInEdge field. */
	uint8_t syncInEdge;

	/** \brief The SyncInSkipFactor field. */
	uint16_t syncInSkipFactor;

	/** \brief The SyncOutMode field. */
	uint8_t syncOutMode;

	/** \brief The SyncOutPolarity field. */
	uint8_t syncOutPolarity;

	/** \brief The SyncOutSkipFactor field. */
	uint16_t syncOutSkipFactor;

	/** \brief The SyncOutPulseWidth field. */
	uint32_t syncOutPulseWidth;

} SynchronizationControlRegister;

/** \brief Structure representing the Synchronization Status register. */
typedef struct
{
	/** \brief The SyncInCount field. */
	uint32_t syncInCount;

	/** \brief The SyncInTime field. */
	uint32_t syncInTime;

	/** \brief The SyncOutCount field. */
	uint32_t syncOutCount;

} SynchronizationStatusRegister;

/** \brief Structure representing the Filter Basic Control register. */
typedef struct
{
	/** \brief The MagMode field. */
	uint8_t magMode;

	/** \brief The ExtMagMode field. */
	uint8_t extMagMode;

	/** \brief The ExtAccMode field. */
	uint8_t extAccMode;

	/** \brief The ExtGyroMode field. */
	uint8_t extGyroMode;

	/** \brief The GyroLimit field. */
	vec3f gyroLimit;

} FilterBasicControlRegister;

/** \brief Structure representing the VPE Basic Control register. */
typedef struct
{
	/** \brief The Enable field. */
	uint8_t enable;

	/** \brief The HeadingMode field. */
	uint8_t headingMode;

	/** \brief The FilteringMode field. */
	uint8_t filteringMode;

	/** \brief The TuningMode field. */
	uint8_t tuningMode;

} VpeBasicControlRegister;

/** \brief Structure representing the VPE Magnetometer Basic Tuning register. */
typedef struct
{
	/** \brief The BaseTuning field. */
	vec3f baseTuning;

	/** \brief The AdaptiveTuning field. */
	vec3f adaptiveTuning;

	/** \brief The AdaptiveFiltering field. */
	vec3f adaptiveFiltering;

} VpeMagnetometerBasicTuningRegister;

/** \brief Structure representing the VPE Magnetometer Advanced Tuning register. */
typedef struct
{
	/** \brief The MinFiltering field. */
	vec3f minFiltering;

	/** \brief The MaxFiltering field. */
	vec3f maxFiltering;

	/** \brief The MaxAdaptRate field. */
	float maxAdaptRate;

	/** \brief The DisturbanceWindow field. */
	float disturbanceWindow;

	/** \brief The MaxTuning field. */
	float maxTuning;

} VpeMagnetometerAdvancedTuningRegister;

/** \brief Structure representing the VPE Accelerometer Basic Tuning register. */
typedef struct
{
	/** \brief The BaseTuning field. */
	vec3f baseTuning;

	/** \brief The AdaptiveTuning field. */
	vec3f adaptiveTuning;

	/** \brief The AdaptiveFiltering field. */
	vec3f adaptiveFiltering;

} VpeAccelerometerBasicTuningRegister;

/** \brief Structure representing the VPE Accelerometer Advanced Tuning register. */
typedef struct
{
	/** \brief The MinFiltering field. */
	vec3f minFiltering;

	/** \brief The MaxFiltering field. */
	vec3f maxFiltering;

	/** \brief The MaxAdaptRate field. */
	float maxAdaptRate;

	/** \brief The DisturbanceWindow field. */
	float disturbanceWindow;

	/** \brief The MaxTuning field. */
	float maxTuning;

} VpeAccelerometerAdvancedTuningRegister;

/** \brief Structure representing the VPE Gyro Basic Tuning register. */
typedef struct
{
	/** \brief The AngularWalkVariance field. */
	vec3f angularWalkVariance;

	/** \brief The BaseTuning field. */
	vec3f baseTuning;

	/** \brief The AdaptiveTuning field. */
	vec3f adaptiveTuning;

} VpeGyroBasicTuningRegister;

/** \brief Structure representing the Magnetometer Calibration Control register. */
typedef struct
{
	/** \brief The HSIMode field. */
	uint8_t hsiMode;

	/** \brief The HSIOutput field. */
	uint8_t hsiOutput;

	/** \brief The ConvergeRate field. */
	uint8_t convergeRate;

} MagnetometerCalibrationControlRegister;

/** \brief Structure representing the Calculated Magnetometer Calibration register. */
typedef struct
{
	/** \brief The C field. */
	mat3f c;

	/** \brief The B field. */
	vec3f b;

} CalculatedMagnetometerCalibrationRegister;

/** \brief Structure representing the Velocity Compensation Control register. */
typedef struct
{
	/** \brief The Mode field. */
	uint8_t mode;

	/** \brief The VelocityTuning field. */
	float velocityTuning;

	/** \brief The RateTuning field. */
	float rateTuning;

} VelocityCompensationControlRegister;

/** \brief Structure representing the Velocity Compensation Status register. */
typedef struct
{
	/** \brief The x field. */
	float x;

	/** \brief The xDot field. */
	float xDot;

	/** \brief The accelOffset field. */
	vec3f accelOffset;

	/** \brief The omega field. */
	vec3f omega;

} VelocityCompensationStatusRegister;

/** \brief Structure representing the IMU Measurements register. */
typedef struct
{
	/** \brief The Mag field. */
	vec3f mag;

	/** \brief The Accel field. */
	vec3f accel;

	/** \brief The Gyro field. */
	vec3f gyro;

	/** \brief The Temp field. */
	float temp;

	/** \brief The Pressure field. */
	float pressure;

} ImuMeasurementsRegister;

/** \brief Structure representing the GPS Configuration register. */
typedef struct
{
	/** \brief The Mode field. */
	uint8_t mode;

	/** \brief The PpsSource field. */
	uint8_t ppsSource;

} GpsConfigurationRegister;

/** \brief Structure representing the GPS Solution - LLA register. */
typedef struct
{
	/** \brief The Time field. */
	double time;

	/** \brief The Week field. */
	uint16_t week;

	/** \brief The GpsFix field. */
	uint8_t gpsFix;

	/** \brief The NumSats field. */
	uint8_t numSats;

	/** \brief The Lla field. */
	vec3d lla;

	/** \brief The NedVel field. */
	vec3f nedVel;

	/** \brief The NedAcc field. */
	vec3f nedAcc;

	/** \brief The SpeedAcc field. */
	float speedAcc;

	/** \brief The TimeAcc field. */
	float timeAcc;

} GpsSolutionLlaRegister;

/** \brief Structure representing the GPS Solution - ECEF register. */
typedef struct
{
	/** \brief The Tow field. */
	double tow;

	/** \brief The Week field. */
	uint16_t week;

	/** \brief The GpsFix field. */
	uint8_t gpsFix;

	/** \brief The NumSats field. */
	uint8_t numSats;

	/** \brief The Position field. */
	vec3d position;

	/** \brief The Velocity field. */
	vec3f velocity;

	/** \brief The PosAcc field. */
	vec3f posAcc;

	/** \brief The SpeedAcc field. */
	float speedAcc;

	/** \brief The TimeAcc field. */
	float timeAcc;

} GpsSolutionEcefRegister;

/** \brief Structure representing the INS Solution - LLA register. */
typedef struct
{
	/** \brief The Time field. */
	double time;

	/** \brief The Week field. */
	uint16_t week;

	/** \brief The Status field. */
	uint16_t status;

	/** \brief The YawPitchRoll field. */
	vec3f yawPitchRoll;

	/** \brief The Position field. */
	vec3d position;

	/** \brief The NedVel field. */
	vec3f nedVel;

	/** \brief The AttUncertainty field. */
	float attUncertainty;

	/** \brief The PosUncertainty field. */
	float posUncertainty;

	/** \brief The VelUncertainty field. */
	float velUncertainty;

} InsSolutionLlaRegister;

/** \brief Structure representing the INS Solution - ECEF register. */
typedef struct
{
	/** \brief The Time field. */
	double time;

	/** \brief The Week field. */
	uint16_t week;

	/** \brief The Status field. */
	uint16_t status;

	/** \brief The YawPitchRoll field. */
	vec3f yawPitchRoll;

	/** \brief The Position field. */
	vec3d position;

	/** \brief The Velocity field. */
	vec3f velocity;

	/** \brief The AttUncertainty field. */
	float attUncertainty;

	/** \brief The PosUncertainty field. */
	float posUncertainty;

	/** \brief The VelUncertainty field. */
	float velUncertainty;

} InsSolutionEcefRegister;

/** \brief Structure representing the INS Basic Configuration register for a VN-200 sensor. */
typedef struct
{
	/** \brief The Scenario field. */
	uint8_t scenario;

	/** \brief The AhrsAiding field. */
	uint8_t ahrsAiding;

} InsBasicConfigurationRegisterVn200;

/** \brief Structure representing the INS Basic Configuration register for a VN-300 sensor. */
typedef struct
{
	/** \brief The Scenario field. */
	uint8_t scenario;

	/** \brief The AhrsAiding field. */
	uint8_t ahrsAiding;

	/** \brief The EstBaseline field. */
	uint8_t estBaseline;

} InsBasicConfigurationRegisterVn300;

/** \brief Structure representing the INS Advanced Configuration register. */
typedef struct
{
	/** \brief The UseMag field. */
	uint8_t useMag;

	/** \brief The UsePres field. */
	uint8_t usePres;

	/** \brief The PosAtt field. */
	uint8_t posAtt;

	/** \brief The VelAtt field. */
	uint8_t velAtt;

	/** \brief The VelBias field. */
	uint8_t velBias;

	/** \brief The UseFoam field. */
	uint8_t useFoam;

	/** \brief The GPSCovType field. */
	uint8_t gpsCovType;

	/** \brief The VelCount field. */
	uint8_t velCount;

	/** \brief The VelInit field. */
	float velInit;

	/** \brief The MoveOrigin field. */
	float moveOrigin;

	/** \brief The GPSTimeout field. */
	float gpsTimeout;

	/** \brief The DeltaLimitPos field. */
	float deltaLimitPos;

	/** \brief The DeltaLimitVel field. */
	float deltaLimitVel;

	/** \brief The MinPosUncertainty field. */
	float minPosUncertainty;

	/** \brief The MinVelUncertainty field. */
	float minVelUncertainty;

} InsAdvancedConfigurationRegister;

/** \brief Structure representing the INS State - LLA register. */
typedef struct
{
	/** \brief The YawPitchRoll field. */
	vec3f yawPitchRoll;

	/** \brief The Position field. */
	vec3d position;

	/** \brief The Velocity field. */
	vec3f velocity;

	/** \brief The Accel field. */
	vec3f accel;

	/** \brief The AngularRate field. */
	vec3f angularRate;

} InsStateLlaRegister;

/** \brief Structure representing the INS State - ECEF register. */
typedef struct
{
	/** \brief The YawPitchRoll field. */
	vec3f yawPitchRoll;

	/** \brief The Position field. */
	vec3d position;

	/** \brief The Velocity field. */
	vec3f velocity;

	/** \brief The Accel field. */
	vec3f accel;

	/** \brief The AngularRate field. */
	vec3f angularRate;

} InsStateEcefRegister;

/** \brief Structure representing the Startup Filter Bias Estimate register. */
typedef struct
{
	/** \brief The GyroBias field. */
	vec3f gyroBias;

	/** \brief The AccelBias field. */
	vec3f accelBias;

	/** \brief The PressureBias field. */
	float pressureBias;

} StartupFilterBiasEstimateRegister;

/** \brief Structure representing the Delta Theta and Delta Velocity register. */
typedef struct
{
	/** \brief The DeltaTime field. */
	float deltaTime;

	/** \brief The DeltaTheta field. */
	vec3f deltaTheta;

	/** \brief The DeltaVelocity field. */
	vec3f deltaVelocity;

} DeltaThetaAndDeltaVelocityRegister;

/** \brief Structure representing the Delta Theta and Delta Velocity Configuration register. */
typedef struct
{
	/** \brief The IntegrationFrame field. */
	uint8_t integrationFrame;

	/** \brief The GyroCompensation field. */
	uint8_t gyroCompensation;

	/** \brief The AccelCompensation field. */
	uint8_t accelCompensation;

} DeltaThetaAndDeltaVelocityConfigurationRegister;

/** \brief Structure representing the Reference Vector Configuration register. */
typedef struct
{
	/** \brief The UseMagModel field. */
	uint8_t useMagModel;

	/** \brief The UseGravityModel field. */
	uint8_t useGravityModel;

	/** \brief The RecalcThreshold field. */
	uint32_t recalcThreshold;

	/** \brief The Year field. */
	float year;

	/** \brief The Position field. */
	vec3d position;

} ReferenceVectorConfigurationRegister;

/** \brief Structure representing the Gyro Compensation register. */
typedef struct
{
	/** \brief The C field. */
	mat3f c;

	/** \brief The B field. */
	vec3f b;

} GyroCompensationRegister;

/** \brief Structure representing the IMU Filtering Configuration register. */
typedef struct
{
	/** \brief The MagWindowSize field. */
	uint16_t magWindowSize;

	/** \brief The AccelWindowSize field. */
	uint16_t accelWindowSize;

	/** \brief The GyroWindowSize field. */
	uint16_t gyroWindowSize;

	/** \brief The TempWindowSize field. */
	uint16_t tempWindowSize;

	/** \brief The PresWindowSize field. */
	uint16_t presWindowSize;

	/** \brief The MagFilterMode field. */
	uint8_t magFilterMode;

	/** \brief The AccelFilterMode field. */
	uint8_t accelFilterMode;

	/** \brief The GyroFilterMode field. */
	uint8_t gyroFilterMode;

	/** \brief The TempFilterMode field. */
	uint8_t tempFilterMode;

	/** \brief The PresFilterMode field. */
	uint8_t presFilterMode;

} ImuFilteringConfigurationRegister;

/** \brief Structure representing the GPS Compass Baseline register. */
typedef struct
{
	/** \brief The Position field. */
	vec3f position;

	/** \brief The Uncertainty field. */
	vec3f uncertainty;

} GpsCompassBaselineRegister;

/** \brief Structure representing the GPS Compass Estimated Baseline register. */
typedef struct
{
	/** \brief The EstBaselineUsed field. */
	uint8_t estBaselineUsed;

	/** \brief The NumMeas field. */
	uint16_t numMeas;

	/** \brief The Position field. */
	vec3f position;

	/** \brief The Uncertainty field. */
	vec3f uncertainty;

} GpsCompassEstimatedBaselineRegister;

/** \brief Structure representing the IMU Rate Configuration register. */
typedef struct
{
	/** \brief The imuRate field. */
	uint16_t imuRate;

	/** \brief The NavDivisor field. */
	uint16_t navDivisor;

	/** \brief The filterTargetRate field. */
	float filterTargetRate;

	/** \brief The filterMinRate field. */
	float filterMinRate;

} ImuRateConfigurationRegister;

/** \brief Structure representing the Yaw, Pitch, Roll, True Body Acceleration and Angular Rates register. */
typedef struct
{
	/** \brief The YawPitchRoll field. */
	vec3f yawPitchRoll;

	/** \brief The BodyAccel field. */
	vec3f bodyAccel;

	/** \brief The Gyro field. */
	vec3f gyro;

} YawPitchRollTrueBodyAccelerationAndAngularRatesRegister;

/** \brief Structure representing the Yaw, Pitch, Roll, True Inertial Acceleration and Angular Rates register. */
typedef struct
{
	/** \brief The YawPitchRoll field. */
	vec3f yawPitchRoll;

	/** \brief The InertialAccel field. */
	vec3f inertialAccel;

	/** \brief The Gyro field. */
	vec3f gyro;

} YawPitchRollTrueInertialAccelerationAndAngularRatesRegister;

/* \} */

typedef void (*VnSensor_PacketFoundHandler)(void *userData, VnUartPacket *packet, size_t runningIndex);

/** \brief Helpful structure for working with VectorNav sensors. */
typedef struct
{
	VnSerialPort serialPort;

	/* Error detection mode to use for outgoing packets. */
	VnErrorDetectionMode sendErrorDetectionMode;

	/* Timeout duration for waiting for a response from the sensor. */
	uint16_t responseTimeoutMs;

	/* Delay between retransmitting commands. */
	uint16_t retransmitDelayMs;

	VnCriticalSection transactionCS;

	VnEvent newResponsesEvent;

	/* Indicates if the transaction function is waiting for a response. */
	bool waitingForResponse;

	/* Indicates if bootloader responses are expected */
	bool bootloaderFilter;

	/* Indicates if a response is waiting for processing by the transaction functions. */
	bool responseWaitingForProcessing;

	size_t runningDataIndex;

	VnUartPacketFinder packetFinder;

	VnSensor_PacketFoundHandler asyncPacketFoundHandler;
	void *asyncPacketFoundHandlerUserData;

	VnSensor_PacketFoundHandler errorMessageReceivedHandler;
	void *errorMessageReceivedHandlerUserData;

	size_t responseLength;

	/* Holds any received response from the sensor for processing in our transaction functions. */
	char response[0x100];

} VnSensor;

/** \brief Initializes a VnSensor structure.
*
* \param[in] sensor The structure to initialize.
* \return Any errors encountered. */
VnError VnSensor_initialize(VnSensor* sensor);

/** \brief Connects to a VectorNav sensor.
 *
 * \param[in] sensor The VnSensor structure.
 * \param[in] portName The name of the serial port to connect to.
 * \param[in] baudrate The baudrate to connect at.
 * \return Any errors encountered. */
VnError VnSensor_connect(VnSensor* sensor, const char* portName, uint32_t baudrate);

/** \brief Disconnects from a VectorNav sensor.
*
* \param[in] sensor The associated sensor.
* \return Any errors encountered. */
VnError VnSensor_disconnect(VnSensor* sensor);

/** \brief Issues a change baudrate to the VectorNav sensor and then reconnectes
*   the attached serial port at the new baudrate.
*
* \param[in] sensor The VnSensor structure.
* \param[in] baudrate The new sensor baudrate.
* \return Any errors encountered. */
VnError VnSensor_changeBaudrate(VnSensor* sensor, uint32_t baudrate);

/** \brief Sends the provided command and returns the response from the sensor.
*
* If the command does not have an asterisk '*', the a checksum will be performed
* and appended based on the current error detection mode. Also, if the line-ending
* \\r\\n is not present, these will be added also.
*
* \param[in] sensor The associated VnSensor.
* \param[in] toSend The command to send to the sensor.
* \param[in] toSendLength The number of bytes provided in the toSend buffer.
* \param[out] response The response received from the sensor.
* \param[in,out] responseLength The size of the provided response buffer and will be
*     set with the returned response length.
* \return Any errors encountered. */
VnError VnSensor_transaction(VnSensor* sensor, char* toSend, size_t toSendLength, char* response, size_t* responseLength);

/** \brief Indicates if the VnSensor is connected.
*
* \param[in] sensor The associated VnSensor.
* \return <c>true</c> if the VnSensor is connected; otherwise <c>false</c>. */
bool VnSensor_isConnected(VnSensor* sensor);

/** \brief Issues a Write Settings command to the VectorNav Sensor.
*
* \param[in] sensor The associated VnSensor.
* \param[in] waitForReply  Indicates if the method should wait for a response
*                          from the sensor.
* \return Any errors encountered*/
VnError VnSensor_writeSettings(VnSensor* sensor, bool waitForReply);

/** \brief Issues a Restore Factory Settings command to the VectorNav sensor.
*
* \param[in] sensor The associated VnSensor.
* \param[in] waitForReply  Indicates if the method should wait for a response
*                          from the sensor.
* \return Any errors encountered*/
VnError VnSensor_restoreFactorySettings(VnSensor* sensor, bool waitForReply);

/** \brief Issues a Reset command to the VectorNav sensor.
*
* \param[in] sensor The associated VnSensor.
* \param[in] waitForReply  Indicates if the method should wait for a response
*                          from the sensor.
* \return Any errors encountered*/
VnError VnSensor_reset(VnSensor* sensor, bool waitForReply);

/** \brief Issues a firmware update command to the VectorNav sensor.
*
* \param[in] sensor The associated VnSensor.
* \param[in] waitForReply  Indicates if the method should wait for a response
*                          from the sensor.
* \return Any errors encountered*/
VnError VnSensor_firmwareUpdateMode(VnSensor* sensor, bool waitForReply);

/** \brief Issues a tare command to the VectorNav Sensor.
*
* \param[in] sensor The associated VnSensor.
* \param[in] waitForReply  Indicates if the method should wait for a response
*                          from the sensor.
* \return Any errors encountered*/
VnError VnSensor_tare(VnSensor* sensor, bool waitForReply);

/** \brief Issues a command to the VectorNav Sensor to set the Gyro's bias.
*
* \param[in] sensor The associated VnSensor.
* \param[in] waitForReply  Indicates if the method should wait for a response
*                          from the sensor.
* \return Any errors encountered*/
VnError VnSensor_setGyroBias(VnSensor* sensor, bool waitForReply);

/** \brief Command to inform the VectorNav Sensor if there is a magnetic disturbance present.
*
* \param[in] sensor The associated VnSensor.
* \param[in] disturbancePresent Indicates the presense of a disturbance.
* \param[in] waitForReply  Indicates if the method should wait for a response
*                          from the sensor.
* \return Any errors encountered*/
VnError VnSensor_magneticDisturbancePresent(VnSensor* sensor, bool disturbancePresent, bool waitForReply);

/** \brief Command to inform the VectorNav Sensor if there is an acceleration disturbance present.
*
* \param[in] sensor The associated VnSensor.
* \param[in] disturbancePresent Indicates the presense of a disturbance.
* \param[in] waitForReply  Indicates if the method should wait for a response
*                          from the sensor.
* \return Any errors encountered*/
VnError VnSensor_accelerationDisturbancePresent(VnSensor* sensor, bool disturbancePresent, bool waitForReply);

/** \brief Checks if we are able to send and receive communication with a sensor.
 *
 * \param[in] sensor The associated sensor.
 * \return <c>true</c> if we can communicate with a sensor; otherwise <c>false</c>. */
bool VnSensor_verifySensorConnectivity(VnSensor* sensor);

/** \brief Returns the current response timeout value in milliseconds used for
 *  communication with a sensor.
 *
 * The response timeout is used on commands that require a response to be
 * received from the sensor. If a response has not been received from the sensor
 * in the amount of time specified by this value, the called function will
 * return an E_TIMEOUT error.
 *
 * \param[in] sensor The associated VnSensor.
 * \return The current response timeout value in milliseconds. */
uint16_t VnSensor_getResponseTimeoutMs(VnSensor* sensor);

/** \brief Sets the current response timeout value in milliseconds used for
 *  communication with a sensor.
 *
 * The response timeout is used on commands that require a response to be
 * received from the sensor. If a response has not been received from the sensor
 * in the amount of time specified by this value, the called function will
 * return an E_TIMEOUT error.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[in] responseTimeoutMs The new value for the response timeout in milliseconds.
 * \return Any errors encountered. */
VnError VnSensor_setResponseTimeoutMs(VnSensor* sensor, uint16_t reponseTimeoutMs);

/** \brief Gets the current retransmit delay used for communication with a
 *  sensor.
 *
 * During the time that the VnSensor is awaiting a response from a sensor, the
 * command will be retransmitted to the sensor at the interval specified by this
 * value.
 *
 * \param[in] sensor The associated VnSensor.
 * \return The current retransmit delay value in milliseconds. */
uint16_t VnSensor_getRetransmitDelayMs(VnSensor* sensor);

/** \brief Sets the current retransmit delay used for communication with a
 *  sensor.
 *
 * During the time that the VnSensor is awaiting a response from a sensor, the
 * command will be retransmitted to the sensor at the interval specified by this
 * value.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[in] retransmitDelayMs The new value for the retransmit delay in milliseconds.
 * \return Any errors encountered. */
VnError VnSensor_setRetransmitDelayMs(VnSensor* sensor, uint16_t retransmitDelayMs);

/** \brief Allows registering a callback for notification of when an asynchronous data packet is received.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[in] handler The callback handler.
 * \param[in] userData Data which will be provided with all callbacks.
 * \return Any errors encountered. */
VnError VnSensor_registerAsyncPacketReceivedHandler(VnSensor *sensor, VnSensor_PacketFoundHandler handler, void *userData);

/** \brief Allows unregistering from callback notifications when asynchronous data packets are received.
 *
 * \param[in] sensor The associated sensor. */
VnError VnSensor_unregisterAsyncPacketReceivedHandler(VnSensor *sensor);

/** \brief Allows registering a callback for notification of when a sensor error message is received.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[in] handler The callback handler.
 * \param[in] userData Data which will be provided with all callbacks.
 * \return Any errors encountered. */
VnError VnSensor_registerErrorPacketReceivedHandler(VnSensor *sensor, VnSensor_PacketFoundHandler handler, void *userData);

/** \brief Allows unregistering callbacks for notifications of when a sensor error message is recieved.
 *
 * \param[in] sensor The associated VnSensor.
 * \return Any errors encountered. */
VnError VnSensor_unregisterErrorPacketReceivedHandler(VnSensor *sensor);

/** \defgroup registerAccessMethods Register Access Methods
 * \brief This group of methods provide access to read and write to the
 * sensor's registers.
 *
 * \{ */

/** \brief Reads the Binary Output 1 register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[in] fields The register's values.
 * \return Any errors encountered. */
VnError VnSensor_readBinaryOutput1(VnSensor *sensor, BinaryOutputRegister *fields);

/** \brief Writes to the Binary Output 1 register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[in] fields The register's fields.
 * \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
 * \return Any errors encountered. */
VnError VnSensor_writeBinaryOutput1(VnSensor *sensor, BinaryOutputRegister *fields, bool waitForReply);

/** \brief Reads the Binary Output 2 register.
*
* \param[in] sensor The associated VnSensor.
* \param[in] fields The register's values.
* \return Any errors encountered. */
VnError VnSensor_readBinaryOutput2(VnSensor *sensor, BinaryOutputRegister *fields);

/** \brief Writes to the Binary Output 2 register.
*
* \param[in] sensor The associated VnSensor.
* \param[in] fields The register's fields.
* \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
* \return Any errors encountered. */
VnError VnSensor_writeBinaryOutput2(VnSensor *sensor, BinaryOutputRegister *fields, bool waitForReply);

/** \brief Reads the Binary Output 3 register.
*
* \param[in] sensor The associated VnSensor.
* \param[in] fields The register's values.
* \return Any errors encountered. */
VnError VnSensor_readBinaryOutput3(VnSensor *sensor, BinaryOutputRegister *fields);

/** \brief Writes to the Binary Output 3 register.
*
* \param[in] sensor The associated VnSensor.
* \param[in] fields The register's fields.
* \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
* \return Any errors encountered. */
VnError VnSensor_writeBinaryOutput3(VnSensor *sensor, BinaryOutputRegister *fields, bool waitForReply);

#ifdef EXTRA

/** \brief Reads the Binary Output 4 register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[in] fields The register's values.
 * \return Any errors encountered. */
VnError VnSensor_readBinaryOutput4(VnSensor *sensor, BinaryOutputRegister *fields);

/** \brief Writes to the Binary Output 4 register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[in] fields The register's fields.
 * \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
 * \return Any errors encountered. */
VnError VnSensor_writeBinaryOutput4(VnSensor *sensor, BinaryOutputRegister *fields, bool waitForReply);

/** \brief Reads the Binary Output 5 register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[in] fields The register's values.
 * \return Any errors encountered. */
VnError VnSensor_readBinaryOutput5(VnSensor *sensor, BinaryOutputRegister *fields);

/** \brief Writes to the Binary Output 5 register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[in] fields The register's fields.
 * \param[in] waitForReply Indicates if the method should wait for a response from the sensor.
 * \return Any errors encountered. */
VnError VnSensor_writeBinaryOutput5(VnSensor *sensor, BinaryOutputRegister *fields, bool waitForReply);

#endif

/** \brief Reads the User Tag register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[out] tagBuffer Buffer to place the read register value.
 * \param[in] tagBufferLength Length of the provided buffer.
 * \return Any errors encountered. */
VnError VnSensor_readUserTag(VnSensor *sensor, char *tagBuffer, size_t tagBufferLength);

/** \brief Writes to the User Tag register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[in] tag The value to write to the register.
 * \param[in] waitForReply Set to <c>true</c> to wait for a response from the sensor; otherwise set to <c>false</c> to just immediately send the command and return.
 * \return Any errors encountered. */
VnError VnSensor_writeUserTag(VnSensor *sensor, char* tag, bool waitForReply);

/** \brief Reads the Model Number register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[out] productNameBuffer Buffer to place the read register value.
 * \param[in] productNameBufferLength Length of the provided buffer.
 * \return Any errors encountered. */
VnError VnSensor_readModelNumber(VnSensor *sensor, char *productNameBuffer, size_t productNameBufferLength);

/** \brief Reads the Hardware Revision register.
 *
 * \param[in] sensor The associated VnSensor.
 * \return Any errors encountered. */
VnError VnSensor_readHardwareRevision(VnSensor *sensor, uint32_t *revision);

/** \brief Reads the Serial Number register.
 *
 * \param[in] sensor The associated VnSensor.
 * \return Any errors encountered. */
VnError VnSensor_readSerialNumber(VnSensor *sensor, uint32_t *serialNum);

/** \brief Reads the Firmware Version register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[out] firmwareVersionBuffer Buffer to place the read register value.
 * \param[in] firmwareVersionBufferLength Length of the provided buffer.
 * \return Any errors encountered. */
VnError VnSensor_readFirmwareVersion(VnSensor *sensor, char *firmwareVersionBuffer, size_t firmwareVersionBufferLength);

/** \brief Reads the Serial Baud Rate register.
 *
 * \param[in] sensor The associated VnSensor.
 * \return Any errors encountered. */
VnError VnSensor_readSerialBaudRate(VnSensor *sensor, uint32_t *baudrate);

/** \brief Writes to the Serial Baud Rate register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[in] baudrate The value to write to the register.
 * \param[in] waitForReply Set to <c>true</c> to wait for a response from the sensor; otherwise set to <c>false</c> to just immediately send the command and return.
 * \return Any errors encountered. */
VnError VnSensor_writeSerialBaudRate(VnSensor *sensor, uint32_t baudrate, bool waitForReply);

/** \brief Reads the Async Data Output Type register.
 *
 * \param[in] sensor The associated VnSensor.
 * \return Any errors encountered. */
VnError VnSensor_readAsyncDataOutputType(VnSensor *sensor, VnAsciiAsync *ador);

/** \brief Writes to the Async Data Output Type register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[in] ador The value to write to the register.
 * \param[in] waitForReply Set to <c>true</c> to wait for a response from the sensor; otherwise set to <c>false</c> to just immediately send the command and return.
 * \return Any errors encountered. */
VnError VnSensor_writeAsyncDataOutputType(VnSensor *sensor, VnAsciiAsync ador, bool waitForReply);

/** \brief Reads the Async Data Output Frequency register.
 *
 * \param[in] sensor The associated VnSensor.
 * \return Any errors encountered. */
VnError VnSensor_readAsyncDataOutputFrequency(VnSensor *sensor, uint32_t *adof);

/** \brief Writes to the Async Data Output Frequency register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[in] adof The value to write to the register.
 * \param[in] waitForReply Set to <c>true</c> to wait for a response from the sensor; otherwise set to <c>false</c> to just immediately send the command and return.
 * \return Any errors encountered. */
VnError VnSensor_writeAsyncDataOutputFrequency(VnSensor *sensor, uint32_t adof, bool waitForReply);

/** \brief Reads the Yaw Pitch Roll register.
 *
 * \param[in] sensor The associated VnSensor.
 * \return Any errors encountered. */
VnError VnSensor_readYawPitchRoll(VnSensor *sensor, vec3f *yawPitchRoll);

/** \brief Reads the Attitude Quaternion register.
 *
 * \param[in] sensor The associated VnSensor.
 * \return Any errors encountered. */
VnError VnSensor_readAttitudeQuaternion(VnSensor *sensor, vec4f *quat);

/** \brief Reads the Quaternion, Magnetic, Acceleration and Angular Rates register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
VnError VnSensor_readQuaternionMagneticAccelerationAndAngularRates(VnSensor *sensor, QuaternionMagneticAccelerationAndAngularRatesRegister *fields);

/** \brief Reads the Magnetic Measurements register.
 *
 * \param[in] sensor The associated VnSensor.
 * \return Any errors encountered. */
VnError VnSensor_readMagneticMeasurements(VnSensor *sensor, vec3f *mag);

/** \brief Reads the Acceleration Measurements register.
 *
 * \param[in] sensor The associated VnSensor.
 * \return Any errors encountered. */
VnError VnSensor_readAccelerationMeasurements(VnSensor *sensor, vec3f *accel);

/** \brief Reads the Angular Rate Measurements register.
 *
 * \param[in] sensor The associated VnSensor.
 * \return Any errors encountered. */
VnError VnSensor_readAngularRateMeasurements(VnSensor *sensor, vec3f *gyro);

/** \brief Reads the Magnetic, Acceleration and Angular Rates register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
VnError VnSensor_readMagneticAccelerationAndAngularRates(VnSensor *sensor, MagneticAccelerationAndAngularRatesRegister *fields);

/** \brief Reads the Magnetic and Gravity Reference Vectors register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
VnError VnSensor_readMagneticAndGravityReferenceVectors(VnSensor *sensor, MagneticAndGravityReferenceVectorsRegister *fields);

/** \brief Writes to the Magnetic and Gravity Reference Vectors register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[in] fields The values to write to the register.
 * \param[in] waitForReply Set to <c>true</c> to wait for a response from the sensor; otherwise set to <c>false</c> to just immediately send the command and return.
 * \return Any errors encountered. */
VnError VnSensor_writeMagneticAndGravityReferenceVectors(VnSensor *sensor, MagneticAndGravityReferenceVectorsRegister fields, bool waitForReply);

/** \brief Reads the Magnetometer Compensation register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
VnError VnSensor_readMagnetometerCompensation(VnSensor *sensor, MagnetometerCompensationRegister *fields);

/** \brief Writes to the Magnetometer Compensation register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[in] fields The values to write to the register.
 * \param[in] waitForReply Set to <c>true</c> to wait for a response from the sensor; otherwise set to <c>false</c> to just immediately send the command and return.
 * \return Any errors encountered. */
VnError VnSensor_writeMagnetometerCompensation(VnSensor *sensor, MagnetometerCompensationRegister fields, bool waitForReply);

/** \brief Reads the Acceleration Compensation register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
VnError VnSensor_readAccelerationCompensation(VnSensor *sensor, AccelerationCompensationRegister *fields);

/** \brief Writes to the Acceleration Compensation register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[in] fields The values to write to the register.
 * \param[in] waitForReply Set to <c>true</c> to wait for a response from the sensor; otherwise set to <c>false</c> to just immediately send the command and return.
 * \return Any errors encountered. */
VnError VnSensor_writeAccelerationCompensation(VnSensor *sensor, AccelerationCompensationRegister fields, bool waitForReply);

/** \brief Reads the Reference Frame Rotation register.
 *
 * \param[in] sensor The associated VnSensor.
 * \return Any errors encountered. */
VnError VnSensor_readReferenceFrameRotation(VnSensor *sensor, mat3f *c);

/** \brief Writes to the Reference Frame Rotation register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[in] c The value to write to the register.
 * \param[in] waitForReply Set to <c>true</c> to wait for a response from the sensor; otherwise set to <c>false</c> to just immediately send the command and return.
 * \return Any errors encountered. */
VnError VnSensor_writeReferenceFrameRotation(VnSensor *sensor, mat3f c, bool waitForReply);

/** \brief Reads the Yaw, Pitch, Roll, Magnetic, Acceleration and Angular Rates register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
VnError VnSensor_readYawPitchRollMagneticAccelerationAndAngularRates(VnSensor *sensor, YawPitchRollMagneticAccelerationAndAngularRatesRegister *fields);

/** \brief Reads the Communication Protocol Control register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
VnError VnSensor_readCommunicationProtocolControl(VnSensor *sensor, CommunicationProtocolControlRegister *fields);

/** \brief Writes to the Communication Protocol Control register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[in] fields The values to write to the register.
 * \param[in] waitForReply Set to <c>true</c> to wait for a response from the sensor; otherwise set to <c>false</c> to just immediately send the command and return.
 * \return Any errors encountered. */
VnError VnSensor_writeCommunicationProtocolControl(VnSensor *sensor, CommunicationProtocolControlRegister fields, bool waitForReply);

/** \brief Reads the Synchronization Control register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
VnError VnSensor_readSynchronizationControl(VnSensor *sensor, SynchronizationControlRegister *fields);

/** \brief Writes to the Synchronization Control register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[in] fields The values to write to the register.
 * \param[in] waitForReply Set to <c>true</c> to wait for a response from the sensor; otherwise set to <c>false</c> to just immediately send the command and return.
 * \return Any errors encountered. */
VnError VnSensor_writeSynchronizationControl(VnSensor *sensor, SynchronizationControlRegister fields, bool waitForReply);

/** \brief Reads the Synchronization Status register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
VnError VnSensor_readSynchronizationStatus(VnSensor *sensor, SynchronizationStatusRegister *fields);

/** \brief Writes to the Synchronization Status register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[in] fields The values to write to the register.
 * \param[in] waitForReply Set to <c>true</c> to wait for a response from the sensor; otherwise set to <c>false</c> to just immediately send the command and return.
 * \return Any errors encountered. */
VnError VnSensor_writeSynchronizationStatus(VnSensor *sensor, SynchronizationStatusRegister fields, bool waitForReply);

/** \brief Reads the VPE Basic Control register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
VnError VnSensor_readVpeBasicControl(VnSensor *sensor, VpeBasicControlRegister *fields);

/** \brief Writes to the VPE Basic Control register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[in] fields The values to write to the register.
 * \param[in] waitForReply Set to <c>true</c> to wait for a response from the sensor; otherwise set to <c>false</c> to just immediately send the command and return.
 * \return Any errors encountered. */
VnError VnSensor_writeVpeBasicControl(VnSensor *sensor, VpeBasicControlRegister fields, bool waitForReply);

/** \brief Reads the VPE Magnetometer Basic Tuning register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
VnError VnSensor_readVpeMagnetometerBasicTuning(VnSensor *sensor, VpeMagnetometerBasicTuningRegister *fields);

/** \brief Writes to the VPE Magnetometer Basic Tuning register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[in] fields The values to write to the register.
 * \param[in] waitForReply Set to <c>true</c> to wait for a response from the sensor; otherwise set to <c>false</c> to just immediately send the command and return.
 * \return Any errors encountered. */
VnError VnSensor_writeVpeMagnetometerBasicTuning(VnSensor *sensor, VpeMagnetometerBasicTuningRegister fields, bool waitForReply);

/** \brief Reads the VPE Accelerometer Basic Tuning register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
VnError VnSensor_readVpeAccelerometerBasicTuning(VnSensor *sensor, VpeAccelerometerBasicTuningRegister *fields);

/** \brief Writes to the VPE Accelerometer Basic Tuning register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[in] fields The values to write to the register.
 * \param[in] waitForReply Set to <c>true</c> to wait for a response from the sensor; otherwise set to <c>false</c> to just immediately send the command and return.
 * \return Any errors encountered. */
VnError VnSensor_writeVpeAccelerometerBasicTuning(VnSensor *sensor, VpeAccelerometerBasicTuningRegister fields, bool waitForReply);

/** \brief Reads the Magnetometer Calibration Control register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
VnError VnSensor_readMagnetometerCalibrationControl(VnSensor *sensor, MagnetometerCalibrationControlRegister *fields);

/** \brief Writes to the Magnetometer Calibration Control register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[in] fields The values to write to the register.
 * \param[in] waitForReply Set to <c>true</c> to wait for a response from the sensor; otherwise set to <c>false</c> to just immediately send the command and return.
 * \return Any errors encountered. */
VnError VnSensor_writeMagnetometerCalibrationControl(VnSensor *sensor, MagnetometerCalibrationControlRegister fields, bool waitForReply);

/** \brief Reads the Calculated Magnetometer Calibration register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
VnError VnSensor_readCalculatedMagnetometerCalibration(VnSensor *sensor, CalculatedMagnetometerCalibrationRegister *fields);

/** \brief Reads the Velocity Compensation Measurement register.
 *
 * \param[in] sensor The associated VnSensor.
 * \return Any errors encountered. */
VnError VnSensor_readVelocityCompensationMeasurement(VnSensor *sensor, vec3f *velocity);

/** \brief Writes to the Velocity Compensation Measurement register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[in] velocity The value to write to the register.
 * \param[in] waitForReply Set to <c>true</c> to wait for a response from the sensor; otherwise set to <c>false</c> to just immediately send the command and return.
 * \return Any errors encountered. */
VnError VnSensor_writeVelocityCompensationMeasurement(VnSensor *sensor, vec3f velocity, bool waitForReply);

/** \brief Reads the Velocity Compensation Control register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
VnError VnSensor_readVelocityCompensationControl(VnSensor *sensor, VelocityCompensationControlRegister *fields);

/** \brief Writes to the Velocity Compensation Control register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[in] fields The values to write to the register.
 * \param[in] waitForReply Set to <c>true</c> to wait for a response from the sensor; otherwise set to <c>false</c> to just immediately send the command and return.
 * \return Any errors encountered. */
VnError VnSensor_writeVelocityCompensationControl(VnSensor *sensor, VelocityCompensationControlRegister fields, bool waitForReply);

/** \brief Reads the IMU Measurements register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
VnError VnSensor_readImuMeasurements(VnSensor *sensor, ImuMeasurementsRegister *fields);

/** \brief Reads the GPS Configuration register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
VnError VnSensor_readGpsConfiguration(VnSensor *sensor, GpsConfigurationRegister *fields);

/** \brief Writes to the GPS Configuration register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[in] fields The values to write to the register.
 * \param[in] waitForReply Set to <c>true</c> to wait for a response from the sensor; otherwise set to <c>false</c> to just immediately send the command and return.
 * \return Any errors encountered. */
VnError VnSensor_writeGpsConfiguration(VnSensor *sensor, GpsConfigurationRegister fields, bool waitForReply);

/** \brief Reads the GPS Antenna Offset register.
 *
 * \param[in] sensor The associated VnSensor.
 * \return Any errors encountered. */
VnError VnSensor_readGpsAntennaOffset(VnSensor *sensor, vec3f *position);

/** \brief Writes to the GPS Antenna Offset register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[in] position The value to write to the register.
 * \param[in] waitForReply Set to <c>true</c> to wait for a response from the sensor; otherwise set to <c>false</c> to just immediately send the command and return.
 * \return Any errors encountered. */
VnError VnSensor_writeGpsAntennaOffset(VnSensor *sensor, vec3f position, bool waitForReply);

/** \brief Reads the GPS Solution - LLA register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
VnError VnSensor_readGpsSolutionLla(VnSensor *sensor, GpsSolutionLlaRegister *fields);

/** \brief Reads the GPS Solution - ECEF register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
VnError VnSensor_readGpsSolutionEcef(VnSensor *sensor, GpsSolutionEcefRegister *fields);

/** \brief Reads the INS Solution - LLA register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
VnError VnSensor_readInsSolutionLla(VnSensor *sensor, InsSolutionLlaRegister *fields);

/** \brief Reads the INS Solution - ECEF register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
VnError VnSensor_readInsSolutionEcef(VnSensor *sensor, InsSolutionEcefRegister *fields);

/** \brief Reads the INS Basic Configuration register for a VN-200 sensor.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
VnError VnSensor_readInsBasicConfigurationVn200(VnSensor *sensor, InsBasicConfigurationRegisterVn200 *fields);

/** \brief Writes to the INS Basic Configuration register for a VN-200 sensor.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[in] fields The values to write to the register.
 * \param[in] waitForReply Set to <c>true</c> to wait for a response from the sensor; otherwise set to <c>false</c> to just immediately send the command and return.
 * \return Any errors encountered. */
VnError VnSensor_writeInsBasicConfigurationVn200(VnSensor *sensor, InsBasicConfigurationRegisterVn200 fields, bool waitForReply);

/** \brief Reads the INS Basic Configuration register for a VN-300 sensor.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
VnError VnSensor_readInsBasicConfigurationVn300(VnSensor *sensor, InsBasicConfigurationRegisterVn300 *fields);

/** \brief Writes to the INS Basic Configuration register for a VN-300 sensor.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[in] fields The values to write to the register.
 * \param[in] waitForReply Set to <c>true</c> to wait for a response from the sensor; otherwise set to <c>false</c> to just immediately send the command and return.
 * \return Any errors encountered. */
VnError VnSensor_writeInsBasicConfigurationVn300(VnSensor *sensor, InsBasicConfigurationRegisterVn300 fields, bool waitForReply);

/** \brief Reads the INS State - LLA register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
VnError VnSensor_readInsStateLla(VnSensor *sensor, InsStateLlaRegister *fields);

/** \brief Reads the INS State - ECEF register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
VnError VnSensor_readInsStateEcef(VnSensor *sensor, InsStateEcefRegister *fields);

/** \brief Reads the Startup Filter Bias Estimate register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
VnError VnSensor_readStartupFilterBiasEstimate(VnSensor *sensor, StartupFilterBiasEstimateRegister *fields);

/** \brief Writes to the Startup Filter Bias Estimate register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[in] fields The values to write to the register.
 * \param[in] waitForReply Set to <c>true</c> to wait for a response from the sensor; otherwise set to <c>false</c> to just immediately send the command and return.
 * \return Any errors encountered. */
VnError VnSensor_writeStartupFilterBiasEstimate(VnSensor *sensor, StartupFilterBiasEstimateRegister fields, bool waitForReply);

/** \brief Reads the Delta Theta and Delta Velocity register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
VnError VnSensor_readDeltaThetaAndDeltaVelocity(VnSensor *sensor, DeltaThetaAndDeltaVelocityRegister *fields);

/** \brief Reads the Delta Theta and Delta Velocity Configuration register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
VnError VnSensor_readDeltaThetaAndDeltaVelocityConfiguration(VnSensor *sensor, DeltaThetaAndDeltaVelocityConfigurationRegister *fields);

/** \brief Writes to the Delta Theta and Delta Velocity Configuration register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[in] fields The values to write to the register.
 * \param[in] waitForReply Set to <c>true</c> to wait for a response from the sensor; otherwise set to <c>false</c> to just immediately send the command and return.
 * \return Any errors encountered. */
VnError VnSensor_writeDeltaThetaAndDeltaVelocityConfiguration(VnSensor *sensor, DeltaThetaAndDeltaVelocityConfigurationRegister fields, bool waitForReply);

/** \brief Reads the Reference Vector Configuration register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
VnError VnSensor_readReferenceVectorConfiguration(VnSensor *sensor, ReferenceVectorConfigurationRegister *fields);

/** \brief Writes to the Reference Vector Configuration register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[in] fields The values to write to the register.
 * \param[in] waitForReply Set to <c>true</c> to wait for a response from the sensor; otherwise set to <c>false</c> to just immediately send the command and return.
 * \return Any errors encountered. */
VnError VnSensor_writeReferenceVectorConfiguration(VnSensor *sensor, ReferenceVectorConfigurationRegister fields, bool waitForReply);

/** \brief Reads the Gyro Compensation register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
VnError VnSensor_readGyroCompensation(VnSensor *sensor, GyroCompensationRegister *fields);

/** \brief Writes to the Gyro Compensation register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[in] fields The values to write to the register.
 * \param[in] waitForReply Set to <c>true</c> to wait for a response from the sensor; otherwise set to <c>false</c> to just immediately send the command and return.
 * \return Any errors encountered. */
VnError VnSensor_writeGyroCompensation(VnSensor *sensor, GyroCompensationRegister fields, bool waitForReply);

/** \brief Reads the IMU Filtering Configuration register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
VnError VnSensor_readImuFilteringConfiguration(VnSensor *sensor, ImuFilteringConfigurationRegister *fields);

/** \brief Writes to the IMU Filtering Configuration register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[in] fields The values to write to the register.
 * \param[in] waitForReply Set to <c>true</c> to wait for a response from the sensor; otherwise set to <c>false</c> to just immediately send the command and return.
 * \return Any errors encountered. */
VnError VnSensor_writeImuFilteringConfiguration(VnSensor *sensor, ImuFilteringConfigurationRegister fields, bool waitForReply);

/** \brief Reads the GPS Compass Baseline register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
VnError VnSensor_readGpsCompassBaseline(VnSensor *sensor, GpsCompassBaselineRegister *fields);

/** \brief Writes to the GPS Compass Baseline register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[in] fields The values to write to the register.
 * \param[in] waitForReply Set to <c>true</c> to wait for a response from the sensor; otherwise set to <c>false</c> to just immediately send the command and return.
 * \return Any errors encountered. */
VnError VnSensor_writeGpsCompassBaseline(VnSensor *sensor, GpsCompassBaselineRegister fields, bool waitForReply);

/** \brief Reads the GPS Compass Estimated Baseline register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
VnError VnSensor_readGpsCompassEstimatedBaseline(VnSensor *sensor, GpsCompassEstimatedBaselineRegister *fields);

/** \brief Reads the Yaw, Pitch, Roll, True Body Acceleration and Angular Rates register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
VnError VnSensor_readYawPitchRollTrueBodyAccelerationAndAngularRates(VnSensor *sensor, YawPitchRollTrueBodyAccelerationAndAngularRatesRegister *fields);

/** \brief Reads the Yaw, Pitch, Roll, True Inertial Acceleration and Angular Rates register.
 *
 * \param[in] sensor The associated VnSensor.
 * \param[out] fields The register's values.
 * \return Any errors encountered. */
VnError VnSensor_readYawPitchRollTrueInertialAccelerationAndAngularRates(VnSensor *sensor, YawPitchRollTrueInertialAccelerationAndAngularRatesRegister *fields);

/** \brief Upgrade the connected sensor with the supplied firmware file
 *
 * \param[in] sensor	The associated VnSensor.
 * \param[in] processor The target processor to switch to.
 * \param[in] model		The model of sensor.
 * \param[in] firmware	The current firmware version of sensor.
 * \return	Any errors encountered. */
VnError VnSensor_switchProcessors(VnSensor* s, VnProcessorType processor, char* model, char* firmware);

/** \brief Update the connected sensor with the supplied firmware file
 *
 * \param[in] sensor The associated VnSensor.
 * \param]in] portName The baud rate used to update the firmware. (Can be different than the current baud rate)
 * \param[in] filename The path to the VNX firmware file for the sensor.
 * \return Any errors encountered. */
VnError VnSensor_firmwareUpdate(VnSensor* s, int baudRate, char* fileName);

/** \brief Open a firmware update file to parse.
 *
 * \param[in] filename The path to the VNX firmware file for the sensor.
 * \return A file handle. */
FILE* VnSensor_openFirmwareUpdateFile(char* filename);

/** \brief Parse out and copy the next record in the firmware update file.
 *
 * \param[in] file The file handle returned by VnSensor_openFirmwareUpdateFile.
 * \param]in] record A buffer to hold the next record from the firmware update file.
 * \param[in] MaxRecordSize Pass in the current size of the record buffer. 
 * \return True if another record was found, otherwise false. */
bool VnSensor_getNextFirmwareUpdateRecord(FILE* file, char* record, int MaxRecordSize);

/** \brief Close the firmware update file
 *
 * \param[in] file The file handle returned by VnSensor_openFirmwareUpdateFile.*/
void VnSensor_closeFirmwareUpdateFile(FILE* file);

/** \brief Calibrate the bootloader baudrate once in bootloader mode
 *
 * \param[in] sensor The associated VnSensor.*/
void VnSensor_calibrateBootloader(VnSensor* s);

/** \brief Write the firmware update record to the bootloader
 *
 * \param[in] sensor The associated VnSensor.
 * \param]in] record A line from the firmware update file.
 * \return Any errors encountered. */
VnError VnSensor_writeFirmwareUpdateRecord(VnSensor* s, char* record);

/** \} */

/** \brief Converts a sensor error into a string.
*
* \param[out] out The buffer to place the string in.
* \param[in] val The SensorError value to convert to string.
* \return The converted value.
*/
void strFromSensorError(char *out, SensorError val);

/** \brief Converts a SyncInMode into a string.
 *
 * \param[out] out The buffer to place the string in.
 * \param[in] val The SyncInMode value to convert to string.
 */
void strFromSyncInMode(char *out, VnSyncInMode val);

/** \brief Converts a SyncInEdge into a string.
 *
 * \param[out] out The buffer to place the string in.
 * \param[in] val The SyncInEdge value to convert to string.
 */
void strFromSyncInEdge(char *out, VnSyncInEdge val);

/** \brief Converts a SyncOutMode into a string.
 *
 * \param[out] out The buffer to place the string in.
 * \param[in] val The SyncOutMode value to convert to string.
 */
void strFromSyncOutMode(char *out, VnSyncOutMode val);

/** \brief Converts a SyncOutPolarity into a string.
 *
 * \param[out] out The buffer to place the string in.
 * \param[in] val The SyncOutPolarity value to convert to string.
 */
void strFromSyncOutPolarity(char *out, VnSyncOutPolarity val);

/** \brief Converts a CountMode into a string.
 *
 * \param[out] out The buffer to place the string in.
 * \param[in] val The CountMode value to convert to string.
 */
void strFromCountMode(char *out, VnCountMode val);

/** \brief Converts a StatusMode into a string.
 *
 * \param[out] out The buffer to place the string in.
 * \param[in] val The StatusMode value to convert to string.
 */
void strFromStatusMode(char *out, VnStatusMode val);

/** \brief Converts a ChecksumMode into a string.
 *
 * \param[out] out The buffer to place the string in.
 * \param[in] val The ChecksumMode value to convert to string.
 */
void strFromChecksumMode(char *out, VnChecksumMode val);

/** \brief Converts a ErrorMode into a string.
 *
 * \param[out] out The buffer to place the string in.
 * \param[in] val The ErrorMode value to convert to string.
 */
void strFromErrorMode(char *out, VnErrorMode val);

/** \brief Converts a FilterMode into a string.
 *
 * \param[out] out The buffer to place the string in.
 * \param[in] val The FilterMode value to convert to string.
 */
void strFromFilterMode(char *out, VnFilterMode val);

/** \brief Converts a IntegrationFrame into a string.
 *
 * \param[out] out The buffer to place the string in.
 * \param[in] val The IntegrationFrame value to convert to string.
 */
void strFromIntegrationFrame(char *out, VnIntegrationFrame val);

/** \brief Converts a CompensationMode into a string.
 *
 * \param[out] out The buffer to place the string in.
 * \param[in] val The CompensationMode value to convert to string.
 */
void strFromCompensationMode(char *out, VnCompensationMode val);

/** \brief Converts a GpsFix into a string.
 *
 * \param[out] out The buffer to place the string in.
 * \param[in] val The GpsFix value to convert to string.
 */
void strFromGpsFix(char *out, VnGpsFix val);

/** \brief Converts a GpsMode into a string.
 *
 * \param[out] out The buffer to place the string in.
 * \param[in] val The GpsMode value to convert to string.
 */
void strFromGpsMode(char *out, VnGpsMode val);

/** \brief Converts a PpsSource into a string.
 *
 * \param[out] out The buffer to place the string in.
 * \param[in] val The PpsSource value to convert to string.
 */
void strFromPpsSource(char *out, VnPpsSource val);

/** \brief Converts a VpeEnable into a string.
 *
 * \param[out] out The buffer to place the string in.
 * \param[in] val The VpeEnable value to convert to string.
 */
void strFromVpeEnable(char *out, VnVpeEnable val);

/** \brief Converts a HeadingMode into a string.
 *
 * \param[out] out The buffer to place the string in.
 * \param[in] val The HeadingMode value to convert to string.
 */
void strFromHeadingMode(char *out, VnHeadingMode val);

/** \brief Converts a VpeMode into a string.
 *
 * \param[out] out The buffer to place the string in.
 * \param[in] val The VpeMode value to convert to string.
 */
void strFromVpeMode(char *out, VnVpeMode val);

/** \brief Converts a Scenario into a string.
 *
 * \param[out] out The buffer to place the string in.
 * \param[in] val The Scenario value to convert to string.
 */
void strFromScenario(char *out, VnScenario val);

/** \brief Converts a HsiMode into a string.
 *
 * \param[out] out The buffer to place the string in.
 * \param[in] val The HsiMode value to convert to string.
 */
void strFromHsiMode(char *out, VnHsiMode val);

/** \brief Converts a HsiOutput into a string.
 *
 * \param[out] out The buffer to place the string in.
 * \param[in] val The HsiOutput value to convert to string.
 */
void strFromHsiOutput(char *out, VnHsiOutput val);

/** \brief Converts a VelocityCompensationMode into a string.
 *
 * \param[out] out The buffer to place the string in.
 * \param[in] val The VelocityCompensationMode value to convert to string.
 */
void strFromVelocityCompensationMode(char *out, VnVelocityCompensationMode val);

/** \brief Converts a MagneticMode into a string.
 *
 * \param[out] out The buffer to place the string in.
 * \param[in] val The MagneticMode value to convert to string.
 */
void strFromMagneticMode(char *out, VnMagneticMode val);

/** \brief Converts a ExternalSensorMode into a string.
 *
 * \param[out] out The buffer to place the string in.
 * \param[in] val The ExternalSensorMode value to convert to string.
 */
void strFromExternalSensorMode(char *out, VnExternalSensorMode val);

/** \brief Converts a FoamInit into a string.
 *
 * \param[out] out The buffer to place the string in.
 * \param[in] val The FoamInit value to convert to string.
 */
void strFromFoamInit(char *out, VnFoamInit val);

#ifdef __cplusplus
}
#endif

#ifdef _WIN32
#pragma warning(pop)
#endif

#endif
