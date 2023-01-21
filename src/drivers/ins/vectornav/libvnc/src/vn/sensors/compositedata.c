#include <math.h>
#include <string.h>

#include "vn/sensors/compositedata.h"
#include "vn/xplat/criticalsection.h"
#include "vn/protocol/upack.h"
#include "vn/math/vector.h"
#include "vn/math/matrix.h"

float VnCompositeData_calculateCourseOverGround(float velNedX, float velNedY);
float VnCompositeData_calculateSpeedOverGround(float velNedX, float velNedY);

void VnCompositeData_initialize(VnCompositeData* compositeData)
{
	memset(compositeData, 0, sizeof(VnCompositeData));
}

bool VnCompositeData_hasCourseOverGround(VnCompositeData* compositeData)
{
	return (CDVEL_None != compositeData->velocityType &&
		CDVEL_EstimatedBody != compositeData->velocityType &&
		CDVEL_EstimatedEcef != compositeData->velocityType &&
		CDVEL_GpsEcef != compositeData->velocityType);
}

bool VnCompositeData_courseOverGround(VnCompositeData* compositeData, float* courseOverGroundOut)
{
	bool success = false;

	if (VnCompositeData_hasCourseOverGround(compositeData))
	{
		switch (compositeData->velocityType)
		{
		case CDVEL_GpsNed:
			*courseOverGroundOut = VnCompositeData_calculateCourseOverGround(compositeData->velocityGpsNed.c[0],
				compositeData->velocityGpsNed.c[1]);
			success = true;
			break;
		case CDVEL_EstimatedNed:
			*courseOverGroundOut = VnCompositeData_calculateCourseOverGround(compositeData->velocityGpsNed.c[0],
				compositeData->velocityGpsNed.c[1]);
			success = true;
			break;
		default:
			break;
		}
	}

	return success;
}

float VnCompositeData_calculateCourseOverGround(float velNedX, float velNedY)
{
	/* This is handled by calculating the atan2 of the input. */
	/* Since the input for this is a velocity then we only need */
	/* XY coordinates to calculate. */
	return (float)atan2(velNedY, velNedX);
}

bool VnCompositeData_hasSpeedOverGround(VnCompositeData* compositeData)
{
	return (CDVEL_None != compositeData->velocityType &&
		CDVEL_EstimatedBody != compositeData->velocityType &&
		CDVEL_EstimatedEcef != compositeData->velocityType &&
		CDVEL_GpsEcef != compositeData->velocityType);
}

bool VnCompositeData_speedOverGround(VnCompositeData* compositeData, float* speedOverGroundOut)
{
	bool success = false;

	if (VnCompositeData_hasSpeedOverGround(compositeData))
	{
		switch (compositeData->velocityType)
		{
		case CDVEL_GpsNed:
			*speedOverGroundOut = VnCompositeData_calculateSpeedOverGround(compositeData->velocityGpsNed.c[0],
				compositeData->velocityGpsNed.c[1]);
			success = true;
			break;
		case CDVEL_EstimatedNed:
			*speedOverGroundOut = VnCompositeData_calculateSpeedOverGround(compositeData->velocityGpsNed.c[0],
				compositeData->velocityGpsNed.c[1]);
			success = true;
			break;
		default:
			break;
		}
	}

	return success;
}

float VnCompositeData_calculateSpeedOverGround(float velNedX, float velNedY)
{
	/* This is handled by calculating the magnitude of the input. */
	/* Since the input for this is a velocity then we only need */
	/* XY coordinates to calculate. */
	return (float)sqrt((velNedX * velNedX) + (velNedY * velNedY));
}

void VnCompositeData_processBinaryPacket(VnCompositeData* compositeData, VnUartPacket* packet, VnCriticalSection* criticalSection)
{
	BinaryGroupType groups = (BinaryGroupType)VnUartPacket_groups(packet);
	size_t curGroupFieldIndex = 0;

	VnCriticalSection_enter(criticalSection);

	if ((groups & BINARYGROUPTYPE_COMMON) != 0)
		VnCompositeData_processBinaryPacketCommonGroup(compositeData, packet, (CommonGroup) VnUartPacket_groupField(packet, curGroupFieldIndex++));
	if ((groups & BINARYGROUPTYPE_TIME) != 0)
		VnCompositeData_processBinaryPacketTimeGroup(compositeData, packet, (TimeGroup) VnUartPacket_groupField(packet, curGroupFieldIndex++));
	if ((groups & BINARYGROUPTYPE_IMU) != 0)
		VnCompositeData_processBinaryPacketImuGroup(compositeData, packet, (ImuGroup)VnUartPacket_groupField(packet, curGroupFieldIndex++));
	if ((groups & BINARYGROUPTYPE_GPS) != 0)
		VnCompositeData_processBinaryPacketGpsGroup(compositeData, packet, (GpsGroup)VnUartPacket_groupField(packet, curGroupFieldIndex++));
	if ((groups & BINARYGROUPTYPE_ATTITUDE) != 0)
		VnCompositeData_processBinaryPacketAttitudeGroup(compositeData, packet, (AttitudeGroup)VnUartPacket_groupField(packet, curGroupFieldIndex++));
	if ((groups & BINARYGROUPTYPE_INS) != 0)
		VnCompositeData_processBinaryPacketInsGroup(compositeData, packet, (InsGroup)VnUartPacket_groupField(packet, curGroupFieldIndex++));
  if ((groups & BINARYGROUPTYPE_GPS2) != 0)
    VnCompositeData_processBinaryPacketGps2Group(compositeData, packet, (GpsGroup)VnUartPacket_groupField(packet, curGroupFieldIndex));

	VnCriticalSection_leave(criticalSection);
}

void VnCompositeData_processBinaryPacketCommonGroup(VnCompositeData* compositeData, VnUartPacket* packet, CommonGroup commonGroup)
{
	if (commonGroup & COMMONGROUP_TIMESTARTUP)
		compositeData->timeStartup = VnUartPacket_extractUint64(packet);

	if (commonGroup & COMMONGROUP_TIMEGPS)
		compositeData->timeGps = VnUartPacket_extractUint64(packet);

	if (commonGroup & COMMONGROUP_TIMESYNCIN)
		compositeData->timeSyncIn = VnUartPacket_extractUint64(packet);

	if (commonGroup & COMMONGROUP_YAWPITCHROLL)
		compositeData->yawPitchRoll = VnUartPacket_extractVec3f(packet);

	if (commonGroup & COMMONGROUP_QUATERNION)
		compositeData->quaternion = VnUartPacket_extractVec4f(packet);

	if (commonGroup & COMMONGROUP_ANGULARRATE)
		compositeData->angularRate = VnUartPacket_extractVec3f(packet);

	if (commonGroup & COMMONGROUP_POSITION)
		compositeData->positionEstimatedLla = VnUartPacket_extractVec3d(packet);

	if (commonGroup & COMMONGROUP_VELOCITY)
		compositeData->velocityEstimatedNed = VnUartPacket_extractVec3f(packet);

	if (commonGroup & COMMONGROUP_ACCEL)
		compositeData->acceleration = VnUartPacket_extractVec3f(packet);

	if (commonGroup & COMMONGROUP_IMU)
	{
		compositeData->accelerationUncompensated = VnUartPacket_extractVec3f(packet);
		compositeData->angularRateUncompensated = VnUartPacket_extractVec3f(packet);
	}

	if (commonGroup & COMMONGROUP_MAGPRES)
	{
		compositeData->magnetic = VnUartPacket_extractVec3f(packet);
		compositeData->temperature = VnUartPacket_extractFloat(packet);
		compositeData->pressure = VnUartPacket_extractFloat(packet);
	}

	if (commonGroup & COMMONGROUP_DELTATHETA)
	{
		compositeData->deltaTime = VnUartPacket_extractFloat(packet);
		compositeData->deltaTheta = VnUartPacket_extractVec3f(packet);
		compositeData->deltaVelocity = VnUartPacket_extractVec3f(packet);
	}

	if (commonGroup & COMMONGROUP_INSSTATUS)
	{
		/* Don't know if this is a VN-100, VN-200 or VN-300 so we can't know for sure if
		this is VpeStatus or InsStatus. */
		compositeData->vpeStatus = compositeData->insStatus = VnUartPacket_extractUint16(packet);
	}

	if (commonGroup & COMMONGROUP_SYNCINCNT)
		compositeData->syncInCnt = VnUartPacket_extractUint32(packet);

	if (commonGroup & COMMONGROUP_TIMEGPSPPS)
		compositeData->timeGpsPps = VnUartPacket_extractUint64(packet);
}

void VnCompositeData_processBinaryPacketTimeGroup(VnCompositeData* compositeData, VnUartPacket* packet, TimeGroup timeGroup)
{
	if (timeGroup & TIMEGROUP_TIMESTARTUP)
		compositeData->timeStartup = VnUartPacket_extractUint64(packet);

	if (timeGroup & TIMEGROUP_TIMEGPS)
		compositeData->timeGps = VnUartPacket_extractUint64(packet);

	if (timeGroup & TIMEGROUP_GPSTOW)
		compositeData->gpsTow = VnUartPacket_extractUint64(packet);

	if (timeGroup & TIMEGROUP_GPSWEEK)
		compositeData->week = VnUartPacket_extractUint16(packet);

	if (timeGroup & TIMEGROUP_TIMESYNCIN)
		compositeData->timeSyncIn = VnUartPacket_extractUint64(packet);

	if (timeGroup & TIMEGROUP_TIMEGPSPPS)
		compositeData->timeGpsPps = VnUartPacket_extractUint64(packet);

	if (timeGroup & TIMEGROUP_TIMEUTC)
	{
		compositeData->timeUtc.year = VnUartPacket_extractInt8(packet);
		compositeData->timeUtc.month = VnUartPacket_extractUint8(packet);
		compositeData->timeUtc.day = VnUartPacket_extractUint8(packet);
		compositeData->timeUtc.hour = VnUartPacket_extractUint8(packet);
		compositeData->timeUtc.min = VnUartPacket_extractUint8(packet);
		compositeData->timeUtc.sec = VnUartPacket_extractUint8(packet);
		compositeData->timeUtc.ms = VnUartPacket_extractUint16(packet);
	}

	if (timeGroup & TIMEGROUP_SYNCINCNT)
		compositeData->syncInCnt = VnUartPacket_extractUint32(packet);

	if (timeGroup & TIMEGROUP_SYNCOUTCNT)
		compositeData->syncOutCnt = VnUartPacket_extractUint32(packet);
}

void VnCompositeData_processBinaryPacketImuGroup(VnCompositeData* compositeData, VnUartPacket* packet, ImuGroup imuGroup)
{
	if (imuGroup & IMUGROUP_IMUSTATUS)
		/* This field is currently reserved. */
		VnUartPacket_extractUint16(packet);

	if (imuGroup & IMUGROUP_UNCOMPMAG)
		compositeData->magneticUncompensated = VnUartPacket_extractVec3f(packet);

	if (imuGroup & IMUGROUP_UNCOMPACCEL)
		compositeData->accelerationUncompensated = VnUartPacket_extractVec3f(packet);

	if (imuGroup & IMUGROUP_UNCOMPGYRO)
		compositeData->angularRateUncompensated = VnUartPacket_extractVec3f(packet);

	if (imuGroup & IMUGROUP_TEMP)
		compositeData->temperature = VnUartPacket_extractFloat(packet);

	if (imuGroup & IMUGROUP_PRES)
		compositeData->pressure = VnUartPacket_extractFloat(packet);

	if (imuGroup & IMUGROUP_DELTATHETA)
	{
		compositeData->deltaTime = VnUartPacket_extractFloat(packet);
		compositeData->deltaTheta = VnUartPacket_extractVec3f(packet);
	}

	if (imuGroup & IMUGROUP_DELTAVEL)
		compositeData->deltaVelocity = VnUartPacket_extractVec3f(packet);

	if (imuGroup & IMUGROUP_MAG)
		compositeData->magnetic = VnUartPacket_extractVec3f(packet);

	if (imuGroup & IMUGROUP_ACCEL)
		compositeData->acceleration = VnUartPacket_extractVec3f(packet);

	if (imuGroup & IMUGROUP_ANGULARRATE)
		compositeData->angularRate = VnUartPacket_extractVec3f(packet);

	if (imuGroup & IMUGROUP_SENSSAT)
		compositeData->sensSat = VnUartPacket_extractUint16(packet);

#ifdef EXTRA

	if (imuGroup & IMUGROUP_RAW)
	{
		compositeData->magneticRaw = VnUartPacket_extractVec3f(packet);
		compositeData->accelerationRaw = VnUartPacket_extractVec3f(packet);
		compositeData->angularRateRaw = VnUartPacket_extractVec3f(packet);
		compositeData->temperatureRaw = VnUartPacket_extractFloat(packet);
	}

#endif
}

void VnCompositeData_processBinaryPacketGpsGroup(VnCompositeData* compositeData, VnUartPacket* packet, GpsGroup gpsGroup)
{
	if (gpsGroup & GPSGROUP_UTC)
	{
		compositeData->timeUtc.year = VnUartPacket_extractInt8(packet);
		compositeData->timeUtc.month = VnUartPacket_extractUint8(packet);
		compositeData->timeUtc.day = VnUartPacket_extractUint8(packet);
		compositeData->timeUtc.hour = VnUartPacket_extractUint8(packet);
		compositeData->timeUtc.min = VnUartPacket_extractUint8(packet);
		compositeData->timeUtc.sec = VnUartPacket_extractUint8(packet);
		compositeData->timeUtc.ms = VnUartPacket_extractUint16(packet);
	}

	if (gpsGroup & GPSGROUP_TOW)
		compositeData->gpsTow = VnUartPacket_extractUint64(packet);

	if (gpsGroup & GPSGROUP_WEEK)
		compositeData->week = VnUartPacket_extractUint16(packet);

	if (gpsGroup & GPSGROUP_NUMSATS)
		compositeData->numSats = VnUartPacket_extractUint8(packet);

	if (gpsGroup & GPSGROUP_FIX)
		compositeData->gpsFix = VnUartPacket_extractUint8(packet);

	if (gpsGroup & GPSGROUP_POSLLA)
		compositeData->positionGpsLla = VnUartPacket_extractVec3d(packet);

	if (gpsGroup & GPSGROUP_POSECEF)
		compositeData->positionGpsEcef = VnUartPacket_extractVec3d(packet);

	if (gpsGroup & GPSGROUP_VELNED)
		compositeData->velocityGpsNed = VnUartPacket_extractVec3f(packet);

	if (gpsGroup & GPSGROUP_VELECEF)
		compositeData->velocityGpsEcef = VnUartPacket_extractVec3f(packet);

	if (gpsGroup & GPSGROUP_POSU)
		compositeData->positionUncertaintyGpsNed = VnUartPacket_extractVec3f(packet);

	if (gpsGroup & GPSGROUP_VELU)
		compositeData->velocityUncertaintyGps = VnUartPacket_extractFloat(packet);

	if (gpsGroup & GPSGROUP_TIMEU)
		compositeData->timeUncertainty = VnUartPacket_extractUint32(packet);

  if (gpsGroup & GPSGROUP_TIMEINFO)
    compositeData->timeInfo = VnUartPacket_extractTimeInfo(packet);

  if (gpsGroup & GPSGROUP_DOP)
    compositeData->dop = VnUartPacket_extractGpsDop(packet);
}

void VnCompositeData_processBinaryPacketAttitudeGroup(VnCompositeData* compositeData, VnUartPacket* packet, AttitudeGroup attitudeGroup)
{
	if (attitudeGroup & ATTITUDEGROUP_VPESTATUS)
		compositeData->vpeStatus = VnUartPacket_extractUint16(packet);

	if (attitudeGroup & ATTITUDEGROUP_YAWPITCHROLL)
		compositeData->yawPitchRoll = VnUartPacket_extractVec3f(packet);

	if (attitudeGroup & ATTITUDEGROUP_QUATERNION)
		compositeData->quaternion = VnUartPacket_extractVec4f(packet);

	if (attitudeGroup & ATTITUDEGROUP_DCM)
		compositeData->directionCosineMatrix = VnUartPacket_extractMat3f(packet);

	if (attitudeGroup & ATTITUDEGROUP_MAGNED)
		compositeData->magneticNed = VnUartPacket_extractVec3f(packet);

	if (attitudeGroup & ATTITUDEGROUP_ACCELNED)
		compositeData->accelerationNed = VnUartPacket_extractVec3f(packet);

	if (attitudeGroup & ATTITUDEGROUP_LINEARACCELBODY)
	{
		compositeData->accelerationLinearBody = VnUartPacket_extractVec3f(packet);
		compositeData->velocityType = CDVEL_EstimatedBody;
	}

	if (attitudeGroup & ATTITUDEGROUP_LINEARACCELNED)
		compositeData->accelerationLinearNed = VnUartPacket_extractVec3f(packet);

	if (attitudeGroup & ATTITUDEGROUP_YPRU)
		compositeData->attitudeUncertainty = VnUartPacket_extractVec3f(packet);

#ifdef EXTRA

	if (attitudeGroup & ATTITUDEGROUP_YPRRATE)
		compositeData->yprRates = VnUartPacket_extractVec3f(packet);

	if (attitudeGroup & ATTITUDEGROUP_STATEAHRS)
	{
		/* Currently not doing anything with these values. */
		size_t i;
		for (i = 0; i < 7; i++)
			VnUartPacket_extractFloat(packet);
	}

	if (attitudeGroup & ATTITUDEGROUP_COVAHRS)
	{
		/* Currently not doing anything with these values. */
		size_t i;
		for (i = 0; i < 6; i++)
			VnUartPacket_extractFloat(packet);
	}

#endif
}

void VnCompositeData_processBinaryPacketInsGroup(VnCompositeData* compositeData, VnUartPacket* packet, InsGroup insGroup)
{
	if (insGroup & INSGROUP_INSSTATUS)
		compositeData->insStatus = VnUartPacket_extractUint16(packet);

	if (insGroup & INSGROUP_POSLLA)
		compositeData->positionEstimatedLla = VnUartPacket_extractVec3d(packet);

	if (insGroup & INSGROUP_POSECEF)
		compositeData->positionEstimatedEcef = VnUartPacket_extractVec3d(packet);

	if (insGroup & INSGROUP_VELBODY)
	{
		compositeData->velocityEstimatedBody = VnUartPacket_extractVec3f(packet);
		compositeData->velocityType = CDVEL_EstimatedNed;
	}

	if (insGroup & INSGROUP_VELNED)
		compositeData->velocityEstimatedNed = VnUartPacket_extractVec3f(packet);

	if (insGroup & INSGROUP_VELECEF)
		compositeData->velocityEstimatedEcef = VnUartPacket_extractVec3f(packet);

	if (insGroup & INSGROUP_MAGECEF)
		compositeData->magneticEcef = VnUartPacket_extractVec3f(packet);

	if (insGroup & INSGROUP_ACCELECEF)
		compositeData->accelerationEcef = VnUartPacket_extractVec3f(packet);

	if (insGroup & INSGROUP_LINEARACCELECEF)
		compositeData->accelerationLinearEcef = VnUartPacket_extractVec3f(packet);

	if (insGroup & INSGROUP_POSU)
		compositeData->positionUncertaintyEstimated = VnUartPacket_extractFloat(packet);

	if (insGroup & INSGROUP_VELU)
		compositeData->velocityUncertaintyEstimated = VnUartPacket_extractFloat(packet);

#ifdef EXTRA

	if (insGroup & INSGROUP_STATEINS)
	{
		/* Currently not doing anything with these values. */
		size_t i;
		for (i = 0; i < 17; i++)
			VnUartPacket_extractFloat(packet);
	}

	if (insGroup & INSGROUP_COVINS)
	{
		/* Currently not doing anything with these values. */
		size_t i;
		for (i = 0; i < 16; i++)
			VnUartPacket_extractFloat(packet);
	}

#endif
}

void VnCompositeData_processBinaryPacketGps2Group(VnCompositeData* compositeData, VnUartPacket* packet, GpsGroup gps2Group)
{
  if (gps2Group & GPSGROUP_UTC)
  {
    /* TODO: Need to store this in the current data. */
    VnUartPacket_extractInt8(packet);
    VnUartPacket_extractUint8(packet);
    VnUartPacket_extractUint8(packet);
    VnUartPacket_extractUint8(packet);
    VnUartPacket_extractUint8(packet);
    VnUartPacket_extractUint8(packet);
    VnUartPacket_extractUint16(packet);
  }

  if (gps2Group & GPSGROUP_TOW)
    compositeData->gps2Tow = VnUartPacket_extractUint64(packet);

  if (gps2Group & GPSGROUP_WEEK)
    compositeData->weekGps2 = VnUartPacket_extractUint16(packet);

  if (gps2Group & GPSGROUP_NUMSATS)
    compositeData->numSatsGps2 = VnUartPacket_extractUint8(packet);

  if (gps2Group & GPSGROUP_FIX)
    compositeData->fixGps2 = VnUartPacket_extractUint8(packet);

  if (gps2Group & GPSGROUP_POSLLA)
    compositeData->positionGps2Lla = VnUartPacket_extractVec3d(packet);

  if (gps2Group & GPSGROUP_POSECEF)
    compositeData->positionGps2Ecef = VnUartPacket_extractVec3d(packet);

  if (gps2Group & GPSGROUP_VELNED)
    compositeData->velocityGps2Ned = VnUartPacket_extractVec3f(packet);

  if (gps2Group & GPSGROUP_VELECEF)
    compositeData->velocityGps2Ecef = VnUartPacket_extractVec3f(packet);

  if (gps2Group & GPSGROUP_POSU)
    compositeData->positionUncertaintyGps2Ned = VnUartPacket_extractVec3f(packet);

  if (gps2Group & GPSGROUP_VELU)
    compositeData->velocityUncertaintyGps2 = VnUartPacket_extractFloat(packet);

  if (gps2Group & GPSGROUP_TIMEU)
    compositeData->timeUncertaintyGps2 = VnUartPacket_extractUint32(packet);

#ifdef EXTRA

  if (gps2Group & GPSGROUP_SVSTAT)
  {
    /* Current not doing anything with these values. */
    size_t i;
    for (i = 0; i < 8; i++)
      VnUartPacket_extractFloat(packet);
  }

#endif
}

void VnCompositeData_processAsciiAsyncPacket(VnCompositeData* compositeData, VnUartPacket* packet, VnCriticalSection* criticalSection)
{
	VnCriticalSection_enter(criticalSection);

	switch (VnUartPacket_determineAsciiAsyncType(packet))
	{
	case VNYPR:
		VnUartPacket_parseVNYPR(packet, &compositeData->yawPitchRoll);
		break;

	case VNQTN:
		VnUartPacket_parseVNQTN(packet, &compositeData->quaternion);
		break;

#ifdef EXTRA

	case VNQTM:
		VnUartPacket_parseVNQTM(packet, &compositeData->quaternion, &compositeData->magnetic);
		break;

	case VNQTA:
		VnUartPacket_parseVNQTA(packet, &compositeData->quaternion, &compositeData->acceleration);
		break;

	case VNQTR:
		VnUartPacket_parseVNQTR(packet, &compositeData->quaternion, &compositeData->angularRate);
		break;

	case VNQMA:
		VnUartPacket_parseVNQMA(packet, &compositeData->quaternion, &compositeData->magnetic, &compositeData->acceleration);
		break;

	case VNQAR:
		VnUartPacket_parseVNQAR(packet, &compositeData->quaternion, &compositeData->acceleration, &compositeData->angularRate);
		break;

#endif

	case VNQMR:
		VnUartPacket_parseVNQMR(packet, &compositeData->quaternion, &compositeData->magnetic, &compositeData->acceleration, &compositeData->angularRate);
		break;

#ifdef EXTRA

	case VNDCM:
		VnUartPacket_parseVNDCM(packet, &compositeData->directionCosineMatrix);
		break;

#endif

	case VNMAG:
		VnUartPacket_parseVNMAG(packet, &compositeData->magnetic);
		break;

	case VNACC:
		VnUartPacket_parseVNACC(packet, &compositeData->acceleration);
		break;

	case VNGYR:
		VnUartPacket_parseVNGYR(packet, &compositeData->angularRate);
		break;

	case VNMAR:
		VnUartPacket_parseVNMAR(packet, &compositeData->magnetic, &compositeData->acceleration, &compositeData->angularRate);
		break;

	case VNYMR:
		VnUartPacket_parseVNYMR(packet, &compositeData->yawPitchRoll, &compositeData->magnetic, &compositeData->acceleration, &compositeData->angularRate);
		break;

#ifdef EXTRA

	case VNYCM:
		VnUartPacket_parseVNYCM(
			packet,
			&compositeData->yawPitchRoll,
			&compositeData->magnetic,
			&compositeData->acceleration,
			&compositeData->angularRateUncompensated,
			&compositeData->temperature);
		break;

#endif

	case VNYBA:
		VnUartPacket_parseVNYBA(packet, &compositeData->yawPitchRoll, &compositeData->accelerationLinearBody, &compositeData->angularRate);
		compositeData->velocityType = CDVEL_EstimatedBody;
		break;

	case VNYIA:
		VnUartPacket_parseVNYIA(packet, &compositeData->yawPitchRoll, &compositeData->accelerationLinearNed, &compositeData->angularRate);
		break;

#ifdef EXTRA

	case VNICM:
		// TODO: Implement once figure out how to store/process the inertial mag/accel.
		break;

#endif

	case VNIMU:
		VnUartPacket_parseVNIMU(
			packet,
			&compositeData->magneticUncompensated,
			&compositeData->accelerationUncompensated,
			&compositeData->angularRateUncompensated,
			&compositeData->temperature,
			&compositeData->pressure);
		break;

	case VNGPS:
	{
		float timeUncertainty;
		VnUartPacket_parseVNGPS(
			packet,
			&compositeData->tow,
			&compositeData->week,
			&compositeData->gpsFix,
			&compositeData->numSats,
			&compositeData->positionGpsLla,
			&compositeData->velocityGpsNed,
			&compositeData->positionUncertaintyGpsNed,
			&compositeData->velocityUncertaintyGps,
			&timeUncertainty);
		/* Convert to UInt32 since this is the binary representation in nanoseconds. */
		compositeData->timeUncertainty = (uint32_t)(timeUncertainty * 1e9);
		compositeData->velocityType = CDVEL_GpsNed;
		break;
	}

	case VNGPE:
	{
		float timeUncertainty;
		VnUartPacket_parseVNGPE(
			packet,
			&compositeData->tow,
			&compositeData->week,
			&compositeData->gpsFix,
			&compositeData->numSats,
			&compositeData->positionGpsEcef,
			&compositeData->velocityGpsEcef,
			&compositeData->positionUncertaintyGpsEcef,
			&compositeData->velocityUncertaintyGps,
			&timeUncertainty);
		/* Convert to UInt32 since this is the binary representation in nanoseconds. */
		compositeData->timeUncertainty = (uint32_t)(timeUncertainty * 1e9);
		compositeData->velocityType = CDVEL_GpsEcef;
		break;
	}

	case VNINS:
	{
		float attUncertainty;
		VnUartPacket_parseVNINS(
			packet,
			&compositeData->tow,
			&compositeData->week,
			&compositeData->insStatus,
			&compositeData->yawPitchRoll,
			&compositeData->positionEstimatedLla,
			&compositeData->velocityEstimatedNed,
			&attUncertainty,
			&compositeData->positionUncertaintyEstimated,
			&compositeData->velocityUncertaintyEstimated);
		/* Binary data provides 3 components to yaw, pitch, roll uncertainty. */
		compositeData->attitudeUncertainty.c[0] = compositeData->attitudeUncertainty.c[1] = compositeData->attitudeUncertainty.c[2] = attUncertainty;
		compositeData->velocityType = CDVEL_EstimatedNed;
		break;
	}

	case VNINE:
	{
		float attUncertainty;
		VnUartPacket_parseVNINE(
			packet,
			&compositeData->tow,
			&compositeData->week,
			&compositeData->insStatus,
			&compositeData->yawPitchRoll,
			&compositeData->positionEstimatedEcef,
			&compositeData->velocityEstimatedEcef,
			&attUncertainty,
			&compositeData->positionUncertaintyEstimated,
			&compositeData->velocityUncertaintyEstimated);
		/* Binary data provides 3 components to yaw, pitch, roll uncertainty. */
		compositeData->attitudeUncertainty.c[0] = compositeData->attitudeUncertainty.c[1] = compositeData->attitudeUncertainty.c[2] = attUncertainty;
		compositeData->velocityType = CDVEL_EstimatedEcef;
		break;
	}

	case VNISL:
		VnUartPacket_parseVNISL(
			packet,
			&compositeData->yawPitchRoll,
			&compositeData->positionEstimatedLla,
			&compositeData->velocityEstimatedNed,
			&compositeData->acceleration,
			&compositeData->angularRate);
		compositeData->velocityType = CDVEL_EstimatedNed;
		break;

	case VNISE:
		VnUartPacket_parseVNISE(
			packet,
			&compositeData->yawPitchRoll,
			&compositeData->positionEstimatedEcef,
			&compositeData->velocityEstimatedEcef,
			&compositeData->acceleration,
			&compositeData->angularRate);
		compositeData->velocityType = CDVEL_EstimatedEcef;
		break;

	case VNDTV:
		VnUartPacket_parseVNDTV(
			packet,
			&compositeData->deltaTime,
			&compositeData->deltaTheta,
			&compositeData->deltaVelocity);
		break;

#ifdef EXTRA

	case VNRAW:
		VnUartPacket_parseVNRAW(
			packet,
			&compositeData->magneticRaw,
			&compositeData->accelerationRaw,
			&compositeData->angularRateRaw,
			&compositeData->temperatureRaw);
		break;

	case VNCMV:
		/* TODO: Need to implement. */
		break;

	case VNSTV:
		/* TODO: Need to implement. */
		break;

	case VNCOV:
		/* TODO: Need to implement. */
		break;

#endif

	default:
		/* Not doing anything for unknown ASCII asynchronous packets. */
		;
	}

	VnCriticalSection_leave(criticalSection);
}
