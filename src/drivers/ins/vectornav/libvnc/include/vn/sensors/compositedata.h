#ifndef _VNCOMPOSITEDATA_H_
#define _VNCOMPOSITEDATA_H_

#include "vn/bool.h"
#include "vn/xplat/criticalsection.h"
#include "vn/enum.h"
#include "vn/int.h"
#include "vn/math/vector.h"
#include "vn/math/matrix.h"
#include "vn/protocol/upack.h"
#include "vn/math/vector.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef _WIN32
#pragma warning(push)
#pragma warning(disable : 4820)
#endif

/** \brief Composite structure of all available data types from VectorNav sensors. */
typedef struct
{
	vec3f yawPitchRoll;					/**< Yaw, pitch, roll data. */
	vec4f quaternion;					/**< Quaternion data. */
	mat3f directionCosineMatrix;		/**< Direction cosine matrix data. */
	vec3d positionGpsLla;				/**< GPS latitude, longitude, altitude data. */
	vec3d positionGpsEcef;				/**< GPS earth-centered, earth-fixed data. */
	vec3d positionEstimatedLla;			/**< Estimated latitude, longitude, altitude data. */
	vec3d positionEstimatedEcef;		/**< Estimated earth-centered, earth-fixed position data. */
	VelocityType velocityType;     /**< Type of velocity in the struct. */
	vec3f velocityGpsNed;				/**< GPS velocity NED data. */
	vec3f velocityGpsEcef;				/**< GPS velocity ECEF data. */
	vec3f velocityEstimatedBody;		/**< Estimated velocity body data. */
	vec3f velocityEstimatedNed;			/**< Estimated velocity NED data. */
	vec3f velocityEstimatedEcef;		/**< Estimated velocity ECEF data. */
	vec3f magnetic;						/**< Magnetic data. */
	vec3f magneticUncompensated;		/**< Magnetic uncompensated data. */
	vec3f magneticNed;					/**< Magnetic NED data. */
	vec3f magneticEcef;					/**< Magnetic ECEF data. */
	#ifdef EXTRA
	vec3f magneticRaw;					/**< Magnetic raw data. */
	#endif
	vec3f acceleration;					/**< Acceleration data. */
	vec3f accelerationUncompensated;	/**< Acceleration uncompensated data. */
	vec3f accelerationNed;				/**< Acceleration NED data. */
	vec3f accelerationEcef;				/**< Acceleration ECEF data. */
	vec3f accelerationLinearBody;		/**< Acceleration linear body data. */
	vec3f accelerationLinearNed;		/**< Acceleration linear NED data. */
	vec3f accelerationLinearEcef;		/**< Acceleration linear ECEF data. */
	#ifdef EXTRA
	vec3f accelerationRaw;				/**< Acceleration raw data. */
	#endif
	vec3f angularRate;					/**< Angular rate data. */
	vec3f angularRateUncompensated;		/**< Angular rate uncompensated data. */
	#ifdef EXTRA
	vec3f angularRateRaw;				/**< Angular rate raw data. */
	#endif
	float temperature;					/**< Temperature data. */
	#ifdef EXTRA
	float temperatureRaw;				/**< Temperature raw data. */
	#endif
	float pressure;						/**< Pressure data. */
	uint64_t timeStartup;				/**< Time startup data. */
	float deltaTime;					/**< Delta time data. */
	vec3f deltaTheta;					/**< Delta theta data. */
	vec3f deltaVelocity;				/**< Delta velocity data. */
	double tow;							/**< GPS time of week data. */
	uint16_t week;						/**< Week data. */
	uint8_t gpsFix;						/**< GPS fix data. */
	uint8_t numSats;					/**< NumSats data. */
	uint64_t timeGps;					/**< TimeGps data. */
	uint64_t timeGpsPps;				/**< TimeGpsPps data. */
	TimeUtc timeUtc;					/**< TimeUtc data. */
	uint64_t gpsTow;					/**< GpsTow data. */
	vec3f attitudeUncertainty;			/**< Attitude uncertainty data. */
	vec3f positionUncertaintyGpsNed;	/**< GPS position uncertainty NED data. */
	vec3f positionUncertaintyGpsEcef;	/**< GPS position uncertainty ECEF data. */
	float positionUncertaintyEstimated;	/**< Estimated position uncertainty data. */
	float velocityUncertaintyGps;		/**< GPS velocity uncertainty data. */
	float velocityUncertaintyEstimated;	/**< Estimated velocity uncertainty data. */
	uint32_t timeUncertainty;			/**< Time uncertainty data. */
	uint16_t vpeStatus;					/**< VpeStatus data. */
	uint16_t insStatus;					/**< InsStatus data. */
	uint64_t timeSyncIn;				/**< TimeSyncIn data. */
	uint32_t syncInCnt;					/**< SyncInCnt data. */
  uint32_t syncOutCnt;					/**< SyncInCnt data. */
  uint16_t sensSat;					/**< SensSat data. */
	#ifdef EXTRA
	vec3f yprRates;						/**< YprRates data. */
	#endif
  vec3d positionGps2Lla;				/**< GPS2 latitude, longitude, altitude data. */
  vec3d positionGps2Ecef;				/**< GPS2 earth-centered, earth-fixed data. */
  vec3f velocityGps2Ned;				/**< GPS2 velocity NED data. */
  vec3f velocityGps2Ecef;				/**< GPS2 velocity ECEF data. */
  uint16_t weekGps2;						/**< GPS2 Week data. */
  uint8_t fixGps2;						/**< GPS2 fix data. */
  uint8_t numSatsGps2;					/**< GPS2 NumSats data. */
  uint64_t timeGps2;					/**< GPS2 TimeGps data. */
  uint64_t timeGps2Pps;				/**< GPS2 TimeGpsPps data. */
  uint64_t gps2Tow;					/**< GPS2 GpsTow data. */
  float velocityUncertaintyGps2;		/**< GPS2velocity uncertainty data. */
  vec3f positionUncertaintyGps2Ned;	/**< GPS2 position uncertainty NED data. */
  vec3f positionUncertaintyGps2Ecef;	/**< GPS2 position uncertainty ECEF data. */
  uint32_t timeUncertaintyGps2;			/**< GPS2 Time uncertainty data. */
  TimeInfo timeInfo;
  GpsDop dop;
} VnCompositeData;
#ifdef _WIN32
#pragma warning(pop)
#endif

/** \brief Indicates if course over ground has valid data
*
* \param[in] compositeData The associated VnCompositeData structure.
* \return Flag indicating if the course over ground data is available. */
bool VnCompositeData_hasCourseOverGround(VnCompositeData* compositeData);

/** \brief Computers the course over ground from any velocity data available
*
* \param[in] compositeData The associated VnCompositeData structure.
* \param[out] courseOverGroundOut The computered course over ground.
* \return Flag indicating if the calculation was successful. */
bool VnCompositeData_courseOverGround(VnCompositeData* compositeData, float* courseOverGroundOut);

/** \brief Indicates if speed over ground has valid data..
*
* \param[in] compositeData The associated VnCompositeData structure.
* \return Flag indicating if the speed over ground data is available. */
bool VnCompositeData_hasSpeedOverGround(VnCompositeData* compositeData);

/** \brief Computers the speed over ground from any velocity data available
*
* \param[in] compositeData The associated VnCompositeData structure.
* \param[out] speedOverGroundOut The computered course over ground.
* \return Flag indicating if the calculation was successful. */
bool VnCompositeData_speedOverGround(VnCompositeData* compositeData, float* speedOverGroundOut);

/** \brief
*
* \param[in]
* \param[out]
* \return
*/
void VnCompositeData_initialize(VnCompositeData* compositeData);

/** \brief
*
* \param[in]
* \param[out]
* \return
*/
void VnCompositeData_processBinaryPacket(VnCompositeData* compositeData, VnUartPacket* packet, VnCriticalSection* criticalSection);

/** \brief
*
* \param[in]
* \param[out]
* \return
*/
void VnCompositeData_processAsciiAsyncPacket(VnCompositeData* compositeData, VnUartPacket* packet, VnCriticalSection* criticalSection);

/** \brief
*
* \param[in]
* \param[out]
* \return
*/
void VnCompositeData_processBinaryPacketCommonGroup(
    VnCompositeData* compositeData,
	VnUartPacket* packet,
	CommonGroup groupFlags);

/** \brief
*
* \param[in]
* \param[out]
* \return
*/
void VnCompositeData_processBinaryPacketTimeGroup(
	VnCompositeData* compositeData,
	VnUartPacket* packet,
	TimeGroup groupFlags);

/** \brief
*
* \param[in]
* \param[out]
* \return
*/
void VnCompositeData_processBinaryPacketImuGroup(
	VnCompositeData* compositeData,
	VnUartPacket* packet,
	ImuGroup groupFlags);

/** \brief
*
* \param[in]
* \param[out]
* \return
*/
void VnCompositeData_processBinaryPacketGpsGroup(
	VnCompositeData* compositeData,
	VnUartPacket* packet,
	GpsGroup groupFlags);

/** \brief
*
* \param[in]
* \param[out]
* \return
*/
void VnCompositeData_processBinaryPacketAttitudeGroup(
	VnCompositeData* compositeData,
	VnUartPacket* packet,
	AttitudeGroup groupFlags);

/** \brief
*
* \param[in]
* \param[out]
* \return
*/
void VnCompositeData_processBinaryPacketInsGroup(
	VnCompositeData* compositeData,
	VnUartPacket* packet,
	InsGroup groupFlags);

/** \brief
*
* \param[in]
* \param[out]
* \return
*/
void VnCompositeData_processBinaryPacketGps2Group(
  VnCompositeData* compositeData,
  VnUartPacket* packet,
  GpsGroup groupFlags);

#ifdef __cplusplus
}
#endif

#endif
