#include "vn/protocol/upack.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "vn/util.h"

/*#define MAXIMUM_REGISTER_ID		255
#define ASCII_START_CHAR		'$'
#define ASCII_END_CHAR1			'\r'
#define ASCII_END_CHAR2			'\n'
#define BINARY_START_CHAR		0xFA
#define MAX_BINARY_PACKET_SIZE	256*/

const uint8_t BinaryPacketGroupLengths[7][15] = {
	{ 8, 8, 8, 12, 16, 12, 24, 12, 12, 24, 20, 28, 2, 4, 8},
	{ 8, 8, 8, 2, 8, 8, 8, 4, 4, 1, 0, 0, 0, 0, 0},
	{ 2, 12, 12, 12, 4, 4, 16, 12, 12, 12, 12, 2, 40, 0, 0 },
	{ 8, 8, 2, 1, 1, 24, 24, 12, 12, 12, 4, 4, 2, 28, 0 },
	{ 2, 12, 16, 36, 12, 12, 12, 12, 12, 12, 28, 24, 0, 0, 0 },
	{ 2, 24, 24, 12, 12, 12, 12, 12, 12, 4, 4, 68, 64, 0, 0 },
  { 8, 8, 2, 1, 1, 24, 24, 12, 12, 12, 4, 4, 2, 28, 0 },
};

#define NEXT result = VnUartPacket_getNextData((uint8_t*)packet->data, &packetIndex); if (result == NULL) return;

#define NEXTR result = VnUartPacket_getNextData((uint8_t*)packet->data, &packetIndex); if (result == NULL) return E_UNKNOWN;

#define NEXTRAW result = (char*)VnUartPacket_getNextData((uint8_t*)packet, &packetIndex); if (result == NULL) return;

#define ATOFF ((float)atof((char*) result))
#define ATOFD atof((char*) result)
#define ATOU32 ((uint32_t) atoi((char*) result))
#define ATOU16X ((uint16_t) strtol((char*) result, NULL, 16))
#define ATOU16 ((uint16_t) atoi((char*) result))
#define ATOU8 ((uint8_t) atoi((char*) result))

/* Function declarations */
uint8_t* VnUartPacket_startAsciiPacketParse(uint8_t* packetStart, size_t* index);
uint8_t* VnUartPacket_startAsciiResponsePacketParse(uint8_t* packetStart, size_t* index);
uint8_t* VnUartPacket_getNextData(uint8_t* str, size_t* startIndex);
uint8_t* vnstrtok(uint8_t* str, size_t* startIndex);
VnError VnUartPacket_genWriteBinaryOutput(
	uint8_t *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	uint8_t binaryOutputNumber,
	uint16_t asyncMode,
	uint16_t rateDivisor,
	uint16_t commonField,
	uint16_t timeField,
	uint16_t imuField,
	uint16_t gpsField,
	uint16_t attitudeField,
	uint16_t insField,
  uint16_t gps2Field);
void VnUartPacket_startExtractionIfNeeded(VnUartPacket *packet);

void VnUartPacket_initialize(VnUartPacket *packet, uint8_t *data, size_t length)
{
	packet->curExtractLoc = 0;
	packet->length = length;
	packet->data = data;
}

void VnUartPacket_initializeFromStr(VnUartPacket* packet, char* data)
{
	VnUartPacket_initialize(packet, (uint8_t*) data, strlen(data));
}

bool VnUartPacket_isValid(VnUartPacket *packet)
{
	PacketType t;

	if (packet->length < 7) /* minumum binary packet is 7 bytes, minimum ASCII is 8 bytes */
		return false;

	t = VnUartPacket_type(packet);

	if (t == PACKETTYPE_ASCII)
	{
		/* First determine if this packet does not have a checksum or CRC. */
		if (packet->data[packet->length - 3] == 'X' && packet->data[packet->length - 4] == 'X')
			return true;

		/* First determine if this packet has an 8-bit checksum or a 16-bit CRC. */
		if (packet->data[packet->length - 5] == '*')
		{
			/* Appears we have an 8-bit checksum packet. */
			uint8_t expectedChecksum = VnUtil_toUint8FromHexStr((char*) packet->data + packet->length - 4);

			uint8_t computedChecksum = VnChecksum8_compute((char*) packet->data + 1, packet->length - 6);

			return (bool) (expectedChecksum == computedChecksum);
		}
		else if (packet->data[packet->length - 7] == '*')
		{
			/* Appears we have a 16-bit CRC packet. */
			uint16_t packetCrc = VnUtil_toUint16FromHexStr((char*) packet->data + packet->length - 6);

			uint16_t computedCrc = VnCrc16_compute((char*) packet->data + 1, packet->length - 8);

			return (bool) (packetCrc == computedCrc);
		}
		else if (packet->data[packet->length - 6] == '*') /* Special Bootloader Update Response */
		{
			/* Appears we have a 16-bit CRC packet. */
			uint16_t packetCrc = VnUtil_toUint16FromHexStr((char*)packet->data + packet->length - 5);

			uint16_t computedCrc = VnCrc16_compute((char*)packet->data + 1, packet->length - 7);

			return (bool)(packetCrc == computedCrc);
		}
		else
		{
			/* Don't know what we have. */
			return false;
		}
	}
	else if (t == PACKETTYPE_BINARY)
	{
		uint16_t computedCrc = VnCrc16_compute((char*) packet->data + 1, packet->length - 1);

		return computedCrc == 0;
	}
	else 
	{
		char bootloadSignature[] = "VectorNav Bootloader";
		if (strncmp(packet->data, bootloadSignature, sizeof(bootloadSignature) - 1) == 0)
		{
			return true;
		}
		else
		{
			/* Unknown packet type. */
			return false;
		}
	}
}

bool VnUartPacket_isAsciiAsync(VnUartPacket *packet)
{
	/* Pointer to the unique asynchronous data type identifier. */
	char* pAT = (char*) packet->data + 3;

	if (strncmp(pAT, "YPR", 3) == 0)
		return true;
	if (strncmp(pAT, "QTN", 3) == 0)
		return true;
	#ifdef EXTRA
	if (strncmp(pAT, "QTM", 3) == 0)
		return true;
	if (strncmp(pAT, "QTA", 3) == 0)
		return true;
	if (strncmp(pAT, "QTR", 3) == 0)
		return true;
	if (strncmp(pAT, "QMA", 3) == 0)
		return true;
	if (strncmp(pAT, "QAR", 3) == 0)
		return true;
	#endif
	if (strncmp(pAT, "QMR", 3) == 0)
		return true;
	#ifdef EXTRA
	if (strncmp(pAT, "DCM", 3) == 0)
		return true;
	#endif
	if (strncmp(pAT, "MAG", 3) == 0)
		return true;
	if (strncmp(pAT, "ACC", 3) == 0)
		return true;
	if (strncmp(pAT, "GYR", 3) == 0)
		return true;
	if (strncmp(pAT, "MAR", 3) == 0)
		return true;
	if (strncmp(pAT, "YMR", 3) == 0)
		return true;
	#ifdef EXTRA
	if (strncmp(pAT, "YCM", 3) == 0)
		return true;
	#endif
	if (strncmp(pAT, "YBA", 3) == 0)
		return true;
	if (strncmp(pAT, "YIA", 3) == 0)
		return true;
	#ifdef EXTRA
	if (strncmp(pAT, "ICM", 3) == 0)
		return true;
	#endif
	if (strncmp(pAT, "IMU", 3) == 0)
		return true;
	if (strncmp(pAT, "GPS", 3) == 0)
		return true;
	if (strncmp(pAT, "GPE", 3) == 0)
		return true;
	if (strncmp(pAT, "INS", 3) == 0)
		return true;
	if (strncmp(pAT, "INE", 3) == 0)
		return true;
	if (strncmp(pAT, "ISL", 3) == 0)
		return true;
	if (strncmp(pAT, "ISE", 3) == 0)
		return true;
	if (strncmp(pAT, "DTV", 3) == 0)
		return true;
	#ifdef EXTRA
	if (strncmp(pAT, "RAW", 3) == 0)
		return true;
	if (strncmp(pAT, "CMV", 3) == 0)
		return true;
	if (strncmp(pAT, "STV", 3) == 0)
		return true;
	if (strncmp(pAT, "COV", 3) == 0)
		return true;
	#endif
	else
		return false;
}

bool VnUartPacket_isResponse(VnUartPacket *packet)
{
	char* p = (char*) packet->data + 3;
	if (strncmp(p, "WRG", 3) == 0)
		return true;
	if (strncmp(p, "RRG", 3) == 0)
		return true;
	if (strncmp(p, "WNV", 3) == 0)
		return true;
	if (strncmp(p, "RFS", 3) == 0)
		return true;
	if (strncmp(p, "RST", 3) == 0)
		return true;
	if (strncmp(p, "FWU", 3) == 0)
		return true;
	if (strncmp(p, "CMD", 3) == 0)
		return true;
	if (strncmp(p, "ASY", 3) == 0)
		return true;
	if (strncmp(p, "TAR", 3) == 0)
		return true;
	if (strncmp(p, "KMD", 3) == 0)
		return true;
	if (strncmp(p, "KAD", 3) == 0)
		return true;
	if (strncmp(p, "SGB", 3) == 0)
		return true;
	if (strncmp(p, "DBS", 3) == 0)
		return true;
	if (strncmp(p, "MCU", 3) == 0)
		return true;
	if (strncmp(p, "SBL", 3) == 0)
		return true;
	if (strncmp(p, "SPS", 3) == 0)
		return true;
	if (strncmp(p, "BLD", 3) == 0)
		return true;
	if (strncmp(packet->data, "VectorNav Bootloader", 20) == 0)
		return true;

	return false;
}

bool VnUartPacket_isBootloader(VnUartPacket* packet)
{
	char* p = (char*)packet->data + 3;
	if (strncmp(p, "BLD", 3) == 0)
		return true;
	if (strncmp(packet->data, "VectorNav Bootloader", 20) == 0)
		return true;

	return false;
}


bool VnUartPacket_isError(VnUartPacket *packet)
{
	return VnUartPacket_isErrorRaw(packet->data);
}

bool VnUartPacket_isErrorRaw(uint8_t *packet)
{
	return strncmp((char*) (packet + 3), "ERR", 3) == 0;
}

PacketType VnUartPacket_type(VnUartPacket *packet)
{
	if (packet->length < 1)
		/* Is really an invalid packet. */
		return PACKETTYPE_UNKNOWN;

	if (packet->data[0] == VN_ASCII_START_CHAR)
		return PACKETTYPE_ASCII;
	if ((uint8_t) packet->data[0] == VN_BINARY_START_CHAR)
		return PACKETTYPE_BINARY;

	return PACKETTYPE_UNKNOWN;
}

uint8_t VnUartPacket_groups(VnUartPacket* packet)
{
	return packet->data[1];
}

uint16_t VnUartPacket_groupField(VnUartPacket* packet, size_t groupIndex)
{
	return stoh16(*((uint16_t*) (packet->data + groupIndex * sizeof(uint16_t) + 2)));
}

VnAsciiAsync VnUartPacket_determineAsciiAsyncType(VnUartPacket *packet)
{
	/* Pointer to the unique asynchronous data type identifier. */
	char* pAT = (char*) (packet->data + 3);

	if (strncmp(pAT, "YPR", 3) == 0)
		return VNYPR;
	if (strncmp(pAT, "QTN", 3) == 0)
		return VNQTN;
	#ifdef EXTRA
	if (strncmp(pAT, "QTM", 3) == 0)
		return VNQTM;
	if (strncmp(pAT, "QTA", 3) == 0)
		return VNQTA;
	if (strncmp(pAT, "QTR", 3) == 0)
		return VNQTR;
	if (strncmp(pAT, "QMA", 3) == 0)
		return VNQMA;
	if (strncmp(pAT, "QAR", 3) == 0)
		return VNQAR;
	#endif
	if (strncmp(pAT, "QMR", 3) == 0)
		return VNQMR;
	#ifdef EXTRA
	if (strncmp(pAT, "DCM", 3) == 0)
		return VNDCM;
	#endif
	if (strncmp(pAT, "MAG", 3) == 0)
		return VNMAG;
	if (strncmp(pAT, "ACC", 3) == 0)
		return VNACC;
	if (strncmp(pAT, "GYR", 3) == 0)
		return VNGYR;
	if (strncmp(pAT, "MAR", 3) == 0)
		return VNMAR;
	if (strncmp(pAT, "YMR", 3) == 0)
		return VNYMR;
	#ifdef EXTRA
	if (strncmp(pAT, "YCM", 3) == 0)
		return VNYCM;
	#endif
	if (strncmp(pAT, "YBA", 3) == 0)
		return VNYBA;
	if (strncmp(pAT, "YIA", 3) == 0)
		return VNYIA;
	#ifdef EXTRA
	if (strncmp(pAT, "ICM", 3) == 0)
		return VNICM;
	#endif
	if (strncmp(pAT, "IMU", 3) == 0)
		return VNIMU;
	if (strncmp(pAT, "GPS", 3) == 0)
		return VNGPS;
	if (strncmp(pAT, "GPE", 3) == 0)
		return VNGPE;
	if (strncmp(pAT, "INS", 3) == 0)
		return VNINS;
	if (strncmp(pAT, "INE", 3) == 0)
		return VNINE;
	if (strncmp(pAT, "ISL", 3) == 0)
		return VNISL;
	if (strncmp(pAT, "ISE", 3) == 0)
		return VNISE;
	if (strncmp(pAT, "DTV", 3) == 0)
		return VNDTV;
	if (strncmp(pAT, "RTK", 3) == 0)
		return VNRTK;
	#ifdef EXTRA
	if (strncmp(pAT, "RAW", 3) == 0)
		return VNRAW;
	if (strncmp(pAT, "CMV", 3) == 0)
		return VNCMV;
	if (strncmp(pAT, "STV", 3) == 0)
		return VNSTV;
	if (strncmp(pAT, "COV", 3) == 0)
		return VNCOV;
	#endif
	else
		/* Can't determine the packet type. */
		return VNOFF;
}

size_t VnUartPacket_computeNumOfBytesForBinaryGroupPayload(BinaryGroupType groupType, uint16_t groupField)
{
	size_t runningLength = 0;
	size_t i;

	/* Determine which group is present. */
	size_t groupIndex = 0;
	for (i = 0; i < 7; i++, groupIndex++)
	{
		if (((size_t) groupType >> i) & 0x01)
			break;
	}

	for (i = 0; i < sizeof(uint16_t) * 8; i++)
	{
		if ((groupField >> i) & 1)
		{
			runningLength += BinaryPacketGroupLengths[groupIndex][i];
		}
	}

	return runningLength;
}

bool VnUartPacket_isCompatible(
	VnUartPacket *packet,
	CommonGroup commonGroup,
	TimeGroup timeGroup,
	ImuGroup imuGroup,
	GpsGroup gpsGroup,
	AttitudeGroup attitudeGroup,
	InsGroup insGroup,
  GpsGroup gps2Group)
{
	/* First make sure the appropriate groups are specified. */
	uint8_t groups = packet->data[1];
	uint8_t *curField = packet->data + 2;

	if (commonGroup)
	{
		if (stoh16(*(uint16_t*) curField) != commonGroup)
			/* Not the expected collection of field data types. */
			return false;

		curField += 2;
	}
	else if (groups & 0x01)
	{
		/* There is unexpected Common Group data. */
		return false;
	}

	if (timeGroup)
	{
		if (stoh16(*(uint16_t*) curField) != timeGroup)
			/* Not the expected collection of field data types. */
			return false;

		curField += 2;
	}
	else if (groups & 0x02)
	{
		/* There is unexpected Time Group data. */
		return false;
	}

	if (imuGroup)
	{
		if (stoh16(*(uint16_t*) curField) != imuGroup)
			/* Not the expected collection of field data types. */
			return false;

		curField += 2;
	}
	else if (groups & 0x04)
	{
		/* There is unexpected IMU Group data. */
		return false;
	}

	if (gpsGroup)
	{
		if (stoh16(*(uint16_t*) curField) != gpsGroup)
			/* Not the expected collection of field data types. */
			return false;

		curField += 2;
	}
	else if (groups & 0x08)
	{
		/* There is unexpected GPS Group data. */
		return false;
	}

	if (attitudeGroup)
	{
		if (stoh16(*(uint16_t*) curField) != attitudeGroup)
			/* Not the expected collection of field data types. */
			return false;

		curField += 2;
	}
	else if (groups & 0x10)
	{
		/* There is unexpected Attitude Group data. */
		return false;
	}

	if (insGroup)
	{
		if (stoh16(*(uint16_t*) curField) != insGroup)
			/* Not the expected collection of field data types. */
			return false;

		curField += 2;
	}
	else if (groups & 0x20)
	{
		/* There is unexpected INS Group data. */
		return false;
	}

  if (gps2Group)
  {
    if (stoh16(*(uint16_t*)curField) != gps2Group)
      /* Not the expected collection of field data types. */
      return false;

    curField += 2;
  }
  else if (groups & 0x40)
  {
    /* There is unexpected GPS2 Group data. */
    return false;
  }

	/* Everything checks out. */
	return true;
}

void VnUartPacket_startExtractionIfNeeded(VnUartPacket *packet)
{
	if (packet->curExtractLoc == 0)
		/* Determine the location to start extracting. */
		packet->curExtractLoc = VnUtil_countSetBitsUint8(packet->data[1]) * 2 + 2;
}

uint8_t VnUartPacket_extractUint8(VnUartPacket *packet)
{
	uint8_t d;

	VnUartPacket_startExtractionIfNeeded(packet);

	d =  *(uint8_t*) (packet->data + packet->curExtractLoc);

	packet->curExtractLoc += sizeof(uint8_t);

	return d;
}

int8_t VnUartPacket_extractInt8(VnUartPacket *packet)
{
	int8_t d;

	VnUartPacket_startExtractionIfNeeded(packet);

	d =  *(int8_t*) (packet->data + packet->curExtractLoc);

	packet->curExtractLoc += sizeof(int8_t);

	return d;
}

uint16_t VnUartPacket_extractUint16(VnUartPacket *packet)
{
	uint8_t* extractLocation;

	VnUartPacket_startExtractionIfNeeded(packet);

	extractLocation = packet->data + packet->curExtractLoc;

	packet->curExtractLoc += sizeof(uint16_t);

	return VnUtil_extractUint16((char*) extractLocation);
}

uint32_t VnUartPacket_extractUint32(VnUartPacket *packet)
{
	uint8_t* extractLocation;

	VnUartPacket_startExtractionIfNeeded(packet);
	
	extractLocation = packet->data + packet->curExtractLoc;
	
	packet->curExtractLoc += sizeof(uint32_t);

	return VnUtil_extractUint32((char*) extractLocation);
}

uint64_t VnUartPacket_extractUint64(VnUartPacket *packet)
{
	uint64_t d;

	VnUartPacket_startExtractionIfNeeded(packet);

	memcpy(&d, packet->data + packet->curExtractLoc, sizeof(uint64_t));

	packet->curExtractLoc += sizeof(uint64_t);

	return stoh64(d);
}

float VnUartPacket_extractFloat(VnUartPacket* packet)
{
	uint8_t* extractLocation;

	VnUartPacket_startExtractionIfNeeded(packet);

	extractLocation = packet->data + packet->curExtractLoc;

	packet->curExtractLoc += sizeof(float);

	return VnUtil_extractFloat((char*) extractLocation);
}

vec3f VnUartPacket_extractVec3f(VnUartPacket *packet)
{
	uint8_t* extractLocation;

	VnUartPacket_startExtractionIfNeeded(packet);

	extractLocation = packet->data + packet->curExtractLoc;

	packet->curExtractLoc += 3 * sizeof(float);

	return VnUtil_extractVec3f((char*) extractLocation);
}

vec3d VnUartPacket_extractVec3d(VnUartPacket *packet)
{
	uint8_t* extractLocation;

	VnUartPacket_startExtractionIfNeeded(packet);

	extractLocation = packet->data + packet->curExtractLoc;

	packet->curExtractLoc += 3 * sizeof(double);

	return VnUtil_extractVec3d((char*) extractLocation);
}

vec4f VnUartPacket_extractVec4f(VnUartPacket *packet)
{
	uint8_t* extractLocation;

	VnUartPacket_startExtractionIfNeeded(packet);

	extractLocation = packet->data + packet->curExtractLoc;

	packet->curExtractLoc += 4 * sizeof(float);

	return VnUtil_extractVec4f((char*) extractLocation);
}

mat3f VnUartPacket_extractMat3f(VnUartPacket *packet)
{
	uint8_t* extractLocation;

	VnUartPacket_startExtractionIfNeeded(packet);

	extractLocation = packet->data + packet->curExtractLoc;

	packet->curExtractLoc += 9 * sizeof(float);

	return VnUtil_extractMat3f((char*) extractLocation);
}

GpsDop VnUartPacket_extractGpsDop(VnUartPacket *packet)
{
  uint8_t* extractLocation;

  VnUartPacket_startExtractionIfNeeded(packet);

  extractLocation = packet->data + packet->curExtractLoc;

  packet->curExtractLoc += sizeof(GpsDop);

  return VnUtil_extractGpsDop((char*)extractLocation);
}

TimeUtc VnUartPacket_extractTimeUtc(VnUartPacket *packet)
{
  uint8_t* extractLocation;

  VnUartPacket_startExtractionIfNeeded(packet);

  extractLocation = packet->data + packet->curExtractLoc;

  packet->curExtractLoc += sizeof(TimeUtc);

  return VnUtil_extractTimeUtc((char*)extractLocation);
}

TimeInfo VnUartPacket_extractTimeInfo(VnUartPacket *packet)
{
  uint8_t* extractLocation;

  VnUartPacket_startExtractionIfNeeded(packet);

  extractLocation = packet->data + packet->curExtractLoc;

  packet->curExtractLoc += sizeof(TimeInfo);

  return VnUtil_extractTimeInfo((char*)extractLocation);
}

size_t VnUartPacket_computeBinaryPacketLength(uint8_t const* startOfPossibleBinaryPacket)
{
	uint8_t groupsPresent = startOfPossibleBinaryPacket[1];
	size_t runningPayloadLength = 2;	/* Start of packet character plus groups present field. */
	uint8_t const* pCurrentGroupField = startOfPossibleBinaryPacket + 2;

	if (groupsPresent & 0x01)
	{
		runningPayloadLength += 2 + VnUartPacket_computeNumOfBytesForBinaryGroupPayload(BINARYGROUPTYPE_COMMON, stoh16(*(uint16_t*)pCurrentGroupField));
		pCurrentGroupField += 2;
	}

	if (groupsPresent & 0x02)
	{
		runningPayloadLength += 2 + VnUartPacket_computeNumOfBytesForBinaryGroupPayload(BINARYGROUPTYPE_TIME, stoh16(*(uint16_t*)pCurrentGroupField));
		pCurrentGroupField += 2;
	}

	if (groupsPresent & 0x04)
	{
		runningPayloadLength += 2 + VnUartPacket_computeNumOfBytesForBinaryGroupPayload(BINARYGROUPTYPE_IMU, stoh16(*(uint16_t*)pCurrentGroupField));
		pCurrentGroupField += 2;
	}

	if (groupsPresent & 0x08)
	{
		runningPayloadLength += 2 + VnUartPacket_computeNumOfBytesForBinaryGroupPayload(BINARYGROUPTYPE_GPS, stoh16(*(uint16_t*)pCurrentGroupField));
		pCurrentGroupField += 2;
	}

	if (groupsPresent & 0x10)
	{
		runningPayloadLength += 2 + VnUartPacket_computeNumOfBytesForBinaryGroupPayload(BINARYGROUPTYPE_ATTITUDE, stoh16(*(uint16_t*)pCurrentGroupField));
		pCurrentGroupField += 2;
	}

	if (groupsPresent & 0x20)
	{
		runningPayloadLength += 2 + VnUartPacket_computeNumOfBytesForBinaryGroupPayload(BINARYGROUPTYPE_INS, stoh16(*(uint16_t*)pCurrentGroupField));
		pCurrentGroupField += 2;
	}

  if (groupsPresent & 0x40)
  {
    runningPayloadLength += 2 + VnUartPacket_computeNumOfBytesForBinaryGroupPayload(BINARYGROUPTYPE_GPS2, stoh16(*(uint16_t*)pCurrentGroupField));
    pCurrentGroupField += 2;
  }

	return runningPayloadLength + 2;	/* Add 2 bytes for CRC. */
}

uint8_t* VnUartPacket_startAsciiPacketParse(uint8_t* packetStart, size_t* index)
{
	*index = 7;

	return vnstrtok(packetStart, index);
}

uint8_t* VnUartPacket_startAsciiResponsePacketParse(uint8_t* packetStart, size_t* index)
{
	VnUartPacket_startAsciiPacketParse(packetStart, index);

	return vnstrtok(packetStart, index);
}

uint8_t* VnUartPacket_getNextData(uint8_t* str, size_t* startIndex)
{
	return vnstrtok(str, startIndex);
}

uint8_t* vnstrtok(uint8_t* str, size_t* startIndex)
{
	size_t origIndex = *startIndex;

	while (str[*startIndex] != ',' && str[*startIndex] != '*')
		(*startIndex)++;

	str[(*startIndex)++] = '\0';

	return str + origIndex;
}

void VnUartPacket_parseVNYPR(VnUartPacket* packet, vec3f* yawPitchRoll)
{
	size_t packetIndex;

	uint8_t* result = VnUartPacket_startAsciiPacketParse(packet->data, &packetIndex);

	yawPitchRoll->c[0] = ATOFF; NEXT
	yawPitchRoll->c[1] = ATOFF; NEXT
	yawPitchRoll->c[2] = ATOFF;
}

void VnUartPacket_parseVNQTN(VnUartPacket* packet, vec4f* quaternion)
{
	size_t packetIndex;

	uint8_t* result = VnUartPacket_startAsciiPacketParse(packet->data, &packetIndex);

	quaternion->c[0] = ATOFF; NEXT
	quaternion->c[1] = ATOFF; NEXT
	quaternion->c[2] = ATOFF; NEXT
	quaternion->c[3] = ATOFF;
}

#ifdef EXTRA 

void VnUartPacket_parseVNQTM(VnUartPacket* packet, vec4f *quaternion, vec3f *magnetic)
{
	size_t packetIndex;

	uint8_t* result = VnUartPacket_startAsciiPacketParse(packet->data, &packetIndex);

	quaternion->c[0] = ATOFF; NEXT
	quaternion->c[1] = ATOFF; NEXT
	quaternion->c[2] = ATOFF; NEXT
	quaternion->c[3] = ATOFF; NEXT
	magnetic->c[0] = ATOFF; NEXT
	magnetic->c[1] = ATOFF; NEXT
	magnetic->c[2] = ATOFF;
}

void VnUartPacket_parseVNQTA(VnUartPacket* packet, vec4f* quaternion, vec3f* acceleration)
{
	size_t packetIndex;

	uint8_t* result = VnUartPacket_startAsciiPacketParse(packet->data, &packetIndex);

	quaternion->c[0] = ATOFF; NEXT
	quaternion->c[1] = ATOFF; NEXT
	quaternion->c[2] = ATOFF; NEXT
	quaternion->c[3] = ATOFF; NEXT
	acceleration->c[0] = ATOFF; NEXT
	acceleration->c[1] = ATOFF; NEXT
	acceleration->c[2] = ATOFF;
}

void VnUartPacket_parseVNQTR(VnUartPacket* packet, vec4f* quaternion, vec3f* angularRate)
{
	size_t packetIndex;

	uint8_t* result = VnUartPacket_startAsciiPacketParse(packet->data, &packetIndex);

	quaternion->c[0] = ATOFF; NEXT
	quaternion->c[1] = ATOFF; NEXT
	quaternion->c[2] = ATOFF; NEXT
	quaternion->c[3] = ATOFF; NEXT
	angularRate->c[0] = ATOFF; NEXT
	angularRate->c[1] = ATOFF; NEXT
	angularRate->c[2] = ATOFF;
}

void VnUartPacket_parseVNQMA(VnUartPacket* packet, vec4f* quaternion, vec3f* magnetic, vec3f* acceleration)
{
	size_t packetIndex;

	uint8_t* result = VnUartPacket_startAsciiPacketParse(packet->data, &packetIndex);

	quaternion->c[0] = ATOFF; NEXT
	quaternion->c[1] = ATOFF; NEXT
	quaternion->c[2] = ATOFF; NEXT
	quaternion->c[3] = ATOFF; NEXT
	magnetic->c[0] = ATOFF; NEXT
	magnetic->c[1] = ATOFF; NEXT
	magnetic->c[2] = ATOFF; NEXT
	acceleration->c[0] = ATOFF; NEXT
	acceleration->c[1] = ATOFF; NEXT
	acceleration->c[2] = ATOFF;
}

void VnUartPacket_parseVNQAR(VnUartPacket* packet, vec4f* quaternion, vec3f* acceleration, vec3f* angularRate)
{
	size_t packetIndex;

	uint8_t* result = VnUartPacket_startAsciiPacketParse(packet->data, &packetIndex);

	quaternion->c[0] = ATOFF; NEXT
	quaternion->c[1] = ATOFF; NEXT
	quaternion->c[2] = ATOFF; NEXT
	quaternion->c[3] = ATOFF; NEXT
	acceleration->c[0] = ATOFF; NEXT
	acceleration->c[1] = ATOFF; NEXT
	acceleration->c[2] = ATOFF; NEXT
	angularRate->c[0] = ATOFF; NEXT
	angularRate->c[1] = ATOFF; NEXT
	angularRate->c[2] = ATOFF;
}

#endif

void VnUartPacket_parseVNQMR(VnUartPacket* packet, vec4f* quaternion, vec3f* magnetic, vec3f* acceleration, vec3f* angularRate)
{
	size_t packetIndex;

	uint8_t* result = VnUartPacket_startAsciiPacketParse(packet->data, &packetIndex);

	quaternion->c[0] = ATOFF; NEXT
	quaternion->c[1] = ATOFF; NEXT
	quaternion->c[2] = ATOFF; NEXT
	quaternion->c[3] = ATOFF; NEXT
	magnetic->c[0] = ATOFF; NEXT
	magnetic->c[1] = ATOFF; NEXT
	magnetic->c[2] = ATOFF; NEXT
	acceleration->c[0] = ATOFF; NEXT
	acceleration->c[1] = ATOFF; NEXT
	acceleration->c[2] = ATOFF; NEXT
	angularRate->c[0] = ATOFF; NEXT
	angularRate->c[1] = ATOFF; NEXT
	angularRate->c[2] = ATOFF;
}

#ifdef EXTRA

void VnUartPacket_parseVNDCM(VnUartPacket* packet, mat3f* dcm)
{
	size_t packetIndex;

	uint8_t* result = VnUartPacket_startAsciiPacketParse(packet->data, &packetIndex);

	dcm->c[0] = ATOFF; NEXT
	dcm->c[3] = ATOFF; NEXT
	dcm->c[6] = ATOFF; NEXT
	dcm->c[1] = ATOFF; NEXT
	dcm->c[4] = ATOFF; NEXT
	dcm->c[7] = ATOFF; NEXT
	dcm->c[2] = ATOFF; NEXT
	dcm->c[5] = ATOFF; NEXT
	dcm->c[8] = ATOFF;
}

#endif

void VnUartPacket_parseVNMAG(VnUartPacket* packet, vec3f* magnetic)
{
	size_t packetIndex;

	uint8_t* result = VnUartPacket_startAsciiPacketParse(packet->data, &packetIndex);

	magnetic->c[0] = ATOFF; NEXT
	magnetic->c[1] = ATOFF; NEXT
	magnetic->c[2] = ATOFF;
}

void VnUartPacket_parseVNACC(VnUartPacket* packet, vec3f* acceleration)
{
	size_t packetIndex;

	uint8_t* result = VnUartPacket_startAsciiPacketParse(packet->data, &packetIndex);

	acceleration->c[0] = ATOFF; NEXT
	acceleration->c[1] = ATOFF; NEXT
	acceleration->c[2] = ATOFF;
}

void VnUartPacket_parseVNGYR(VnUartPacket* packet, vec3f* angularRate)
{
	size_t packetIndex;

	uint8_t* result = VnUartPacket_startAsciiPacketParse(packet->data, &packetIndex);

	angularRate->c[0] = ATOFF; NEXT
	angularRate->c[1] = ATOFF; NEXT
	angularRate->c[2] = ATOFF;
}

void VnUartPacket_parseVNMAR(VnUartPacket* packet, vec3f* magnetic, vec3f* acceleration, vec3f* angularRate)
{
	size_t packetIndex;

	uint8_t* result = VnUartPacket_startAsciiPacketParse(packet->data, &packetIndex);

	magnetic->c[0] = ATOFF; NEXT
	magnetic->c[1] = ATOFF; NEXT
	magnetic->c[2] = ATOFF; NEXT
	acceleration->c[0] = ATOFF; NEXT
	acceleration->c[1] = ATOFF; NEXT
	acceleration->c[2] = ATOFF; NEXT
	angularRate->c[0] = ATOFF; NEXT
	angularRate->c[1] = ATOFF; NEXT
	angularRate->c[2] = ATOFF;
}

VnError VnUartPacket_parseVNYMR(VnUartPacket* packet, vec3f* yawPitchRoll, vec3f* magnetic, vec3f* acceleration, vec3f* angularRate)
{
	size_t packetIndex;

	uint8_t* result = VnUartPacket_startAsciiPacketParse(packet->data, &packetIndex);

	yawPitchRoll->c[0] = ATOFF; NEXTR
	yawPitchRoll->c[1] = ATOFF; NEXTR
	yawPitchRoll->c[2] = ATOFF; NEXTR
	magnetic->c[0] = ATOFF; NEXTR
	magnetic->c[1] = ATOFF; NEXTR
	magnetic->c[2] = ATOFF; NEXTR
	acceleration->c[0] = ATOFF; NEXTR
	acceleration->c[1] = ATOFF; NEXTR
	acceleration->c[2] = ATOFF; NEXTR
	angularRate->c[0] = ATOFF; NEXTR
	angularRate->c[1] = ATOFF; NEXTR
	angularRate->c[2] = ATOFF;

	return E_NONE;
}

/*VnError vn_uart_packet_parse_vnymr_buffer(
	uint8_t* packetBuf,
	size_t packetLen,
	vec3f *yawPitchRoll,
	vec3f *magnetic,
	vec3f *acceleration,
	vec3f *angularRate)
{
	VnUartPacket p;

	vn_uart_packet_init(&p, packetBuf, packetLen);

	return vn_uart_packet_parse_vnymr(&p, yawPitchRoll, magnetic, acceleration, angularRate);
}*/

#ifdef EXTRA

void VnUartPacket_parseVNYCM(VnUartPacket* packet, vec3f* yawPitchRoll, vec3f* magnetic, vec3f* acceleration, vec3f* angularRate, float* temperature)
{
	size_t packetIndex;

	uint8_t* result = VnUartPacket_startAsciiPacketParse(packet->data, &packetIndex);

	yawPitchRoll->c[0] = ATOFF; NEXT
	yawPitchRoll->c[1] = ATOFF; NEXT
	yawPitchRoll->c[2] = ATOFF; NEXT
	magnetic->c[0] = ATOFF; NEXT
	magnetic->c[1] = ATOFF; NEXT
	magnetic->c[2] = ATOFF; NEXT
	acceleration->c[0] = ATOFF; NEXT
	acceleration->c[1] = ATOFF; NEXT
	acceleration->c[2] = ATOFF; NEXT
	angularRate->c[0] = ATOFF; NEXT
	angularRate->c[1] = ATOFF; NEXT
	angularRate->c[2] = ATOFF; NEXT
	*temperature = ATOFF;
}

#endif

void VnUartPacket_parseVNYBA(VnUartPacket* packet, vec3f* yawPitchRoll, vec3f* accelerationBody, vec3f* angularRate)
{
	size_t packetIndex;

	uint8_t* result = VnUartPacket_startAsciiPacketParse(packet->data, &packetIndex);

	yawPitchRoll->c[0] = ATOFF; NEXT
	yawPitchRoll->c[1] = ATOFF; NEXT
	yawPitchRoll->c[2] = ATOFF; NEXT
	accelerationBody->c[0] = ATOFF; NEXT
	accelerationBody->c[1] = ATOFF; NEXT
	accelerationBody->c[2] = ATOFF; NEXT
	angularRate->c[0] = ATOFF; NEXT
	angularRate->c[1] = ATOFF; NEXT
	angularRate->c[2] = ATOFF;
}

void VnUartPacket_parseVNYIA(VnUartPacket* packet, vec3f* yawPitchRoll, vec3f* accelerationInertial, vec3f* angularRate)
{
	size_t packetIndex;

	uint8_t* result = VnUartPacket_startAsciiPacketParse(packet->data, &packetIndex);

	yawPitchRoll->c[0] = ATOFF; NEXT
	yawPitchRoll->c[1] = ATOFF; NEXT
	yawPitchRoll->c[2] = ATOFF; NEXT
	accelerationInertial->c[0] = ATOFF; NEXT
	accelerationInertial->c[1] = ATOFF; NEXT
	accelerationInertial->c[2] = ATOFF; NEXT
	angularRate->c[0] = ATOFF; NEXT
	angularRate->c[1] = ATOFF; NEXT
	angularRate->c[2] = ATOFF;
}

#ifdef EXTRA

void VnUartPacket_parseVNICM(VnUartPacket* packet, vec3f* yawPitchRoll, vec3f* magnetic, vec3f* accelerationInertial, vec3f* angularRate)
{
	size_t packetIndex;

	uint8_t* result = VnUartPacket_startAsciiPacketParse(packet->data, &packetIndex);

	yawPitchRoll->c[0] = ATOFF; NEXT
	yawPitchRoll->c[1] = ATOFF; NEXT
	yawPitchRoll->c[2] = ATOFF; NEXT
	magnetic->c[0] = ATOFF; NEXT
	magnetic->c[1] = ATOFF; NEXT
	magnetic->c[2] = ATOFF; NEXT
	accelerationInertial->c[0] = ATOFF; NEXT
	accelerationInertial->c[1] = ATOFF; NEXT
	accelerationInertial->c[2] = ATOFF; NEXT
	angularRate->c[0] = ATOFF; NEXT
	angularRate->c[1] = ATOFF; NEXT
	angularRate->c[2] = ATOFF;
}

#endif

void VnUartPacket_parseVNIMU(VnUartPacket* packet, vec3f* magneticUncompensated, vec3f* accelerationUncompensated, vec3f* angularRateUncompensated, float* temperature, float* pressure)
{
	size_t packetIndex;

	uint8_t* result = VnUartPacket_startAsciiPacketParse(packet->data, &packetIndex);

	magneticUncompensated->c[0] = ATOFF; NEXT
	magneticUncompensated->c[1] = ATOFF; NEXT
	magneticUncompensated->c[2] = ATOFF; NEXT
	accelerationUncompensated->c[0] = ATOFF; NEXT
	accelerationUncompensated->c[1] = ATOFF; NEXT
	accelerationUncompensated->c[2] = ATOFF; NEXT
	angularRateUncompensated->c[0] = ATOFF; NEXT
	angularRateUncompensated->c[1] = ATOFF; NEXT
	angularRateUncompensated->c[2] = ATOFF; NEXT
	*temperature = ATOFF; NEXT
	*pressure = ATOFF;
}

void VnUartPacket_parseVNGPS(VnUartPacket* packet, double* time, uint16_t* week, uint8_t* gpsFix, uint8_t* numSats, vec3d* lla, vec3f* nedVel, vec3f* nedAcc, float* speedAcc, float* timeAcc)
{
	size_t packetIndex;

	uint8_t* result = VnUartPacket_startAsciiPacketParse(packet->data, &packetIndex);

	*time = ATOFD; NEXT
	*week = ATOU16; NEXT
	*gpsFix = ATOU8; NEXT
	*numSats = ATOU8; NEXT
	lla->c[0] = ATOFD; NEXT
	lla->c[1] = ATOFD; NEXT
	lla->c[2] = ATOFD; NEXT
	nedVel->c[0] = ATOFF; NEXT
	nedVel->c[1] = ATOFF; NEXT
	nedVel->c[2] = ATOFF; NEXT
	nedAcc->c[0] = ATOFF; NEXT
	nedAcc->c[1] = ATOFF; NEXT
	nedAcc->c[2] = ATOFF; NEXT
	*speedAcc = ATOFF; NEXT
	*timeAcc = ATOFF;
}

void VnUartPacket_parseVNINS(VnUartPacket* packet, double* time, uint16_t* week, uint16_t* status, vec3f* yawPitchRoll, vec3d* lla, vec3f* nedVel, float* attUncertainty, float* posUncertainty, float* velUncertainty)
{
	size_t packetIndex;

	uint8_t* result = VnUartPacket_startAsciiPacketParse(packet->data, &packetIndex);

	*time = ATOFD; NEXT
	*week = ATOU16; NEXT
	*status = ATOU16X; NEXT
	yawPitchRoll->c[0] = ATOFF; NEXT
	yawPitchRoll->c[1] = ATOFF; NEXT
	yawPitchRoll->c[2] = ATOFF; NEXT
	lla->c[0] = ATOFD; NEXT
	lla->c[1] = ATOFD; NEXT
	lla->c[2] = ATOFD; NEXT
	nedVel->c[0] = ATOFF; NEXT
	nedVel->c[1] = ATOFF; NEXT
	nedVel->c[2] = ATOFF; NEXT
	*attUncertainty = ATOFF; NEXT
	*posUncertainty = ATOFF; NEXT
	*velUncertainty = ATOFF;
}

void VnUartPacket_parseVNINE(VnUartPacket* packet, double* time, uint16_t* week, uint16_t* status, vec3f* yawPitchRoll, vec3d* position, vec3f* velocity, float* attUncertainty, float* posUncertainty, float* velUncertainty)
{
	size_t packetIndex;

	uint8_t* result = VnUartPacket_startAsciiPacketParse(packet->data, &packetIndex);

	*time = ATOFD; NEXT
	*week = ATOU16; NEXT
	*status = ATOU16X; NEXT
	yawPitchRoll->c[0] = ATOFF; NEXT
	yawPitchRoll->c[1] = ATOFF; NEXT
	yawPitchRoll->c[2] = ATOFF; NEXT
	position->c[0] = ATOFD; NEXT
	position->c[1] = ATOFD; NEXT
	position->c[2] = ATOFD; NEXT
	velocity->c[0] = ATOFF; NEXT
	velocity->c[1] = ATOFF; NEXT
	velocity->c[2] = ATOFF; NEXT
	*attUncertainty = ATOFF; NEXT
	*posUncertainty = ATOFF; NEXT
	*velUncertainty = ATOFF;
}

void VnUartPacket_parseVNISL(VnUartPacket* packet, vec3f* ypr, vec3d* lla, vec3f* velocity, vec3f* acceleration, vec3f* angularRate)
{
	size_t packetIndex;

	uint8_t* result = VnUartPacket_startAsciiPacketParse(packet->data, &packetIndex);

	ypr->c[0] = ATOFF; NEXT
	ypr->c[1] = ATOFF; NEXT
	ypr->c[2] = ATOFF; NEXT
	lla->c[0] = ATOFD; NEXT
	lla->c[1] = ATOFD; NEXT
	lla->c[2] = ATOFD; NEXT
	velocity->c[0] = ATOFF; NEXT
	velocity->c[1] = ATOFF; NEXT
	velocity->c[2] = ATOFF; NEXT
	acceleration->c[0] = ATOFF; NEXT
	acceleration->c[1] = ATOFF; NEXT
	acceleration->c[2] = ATOFF; NEXT
	angularRate->c[0] = ATOFF; NEXT
	angularRate->c[1] = ATOFF; NEXT
	angularRate->c[2] = ATOFF;
}

void VnUartPacket_parseVNISE(VnUartPacket* packet, vec3f* ypr, vec3d* position, vec3f* velocity, vec3f* acceleration, vec3f* angularRate)
{
	size_t packetIndex;

	uint8_t* result = VnUartPacket_startAsciiPacketParse(packet->data, &packetIndex);

	ypr->c[0] = ATOFF; NEXT
	ypr->c[1] = ATOFF; NEXT
	ypr->c[2] = ATOFF; NEXT
	position->c[0] = ATOFD; NEXT
	position->c[1] = ATOFD; NEXT
	position->c[2] = ATOFD; NEXT
	velocity->c[0] = ATOFF; NEXT
	velocity->c[1] = ATOFF; NEXT
	velocity->c[2] = ATOFF; NEXT
	acceleration->c[0] = ATOFF; NEXT
	acceleration->c[1] = ATOFF; NEXT
	acceleration->c[2] = ATOFF; NEXT
	angularRate->c[0] = ATOFF; NEXT
	angularRate->c[1] = ATOFF; NEXT
	angularRate->c[2] = ATOFF;
}

#ifdef EXTRA

void VnUartPacket_parseVNRAW(VnUartPacket* packet, vec3f *magneticVoltage, vec3f *accelerationVoltage, vec3f *angularRateVoltage, float* temperatureVoltage)
{
	size_t packetIndex;

	uint8_t* result = VnUartPacket_startAsciiPacketParse(packet->data, &packetIndex);

	magneticVoltage->c[0] = ATOFF; NEXT
	magneticVoltage->c[1] = ATOFF; NEXT
	magneticVoltage->c[2] = ATOFF; NEXT
	accelerationVoltage->c[0] = ATOFF; NEXT
	accelerationVoltage->c[1] = ATOFF; NEXT
	accelerationVoltage->c[2] = ATOFF; NEXT
	angularRateVoltage->c[0] = ATOFF; NEXT
	angularRateVoltage->c[1] = ATOFF; NEXT
	angularRateVoltage->c[2] = ATOFF; NEXT
	*temperatureVoltage = ATOFF;
}

void VnUartPacket_parseVNCMV(VnUartPacket* packet, vec3f* magneticUncompensated, vec3f* accelerationUncompensated, vec3f* angularRateUncompensated, float* temperature)
{
	size_t packetIndex;

	uint8_t* result = VnUartPacket_startAsciiPacketParse(packet->data, &packetIndex);

	magneticUncompensated->c[0] = ATOFF; NEXT
	magneticUncompensated->c[1] = ATOFF; NEXT
	magneticUncompensated->c[2] = ATOFF; NEXT
	accelerationUncompensated->c[0] = ATOFF; NEXT
	accelerationUncompensated->c[1] = ATOFF; NEXT
	accelerationUncompensated->c[2] = ATOFF; NEXT
	angularRateUncompensated->c[0] = ATOFF; NEXT
	angularRateUncompensated->c[1] = ATOFF; NEXT
	angularRateUncompensated->c[2] = ATOFF; NEXT
	*temperature = ATOFF;
}

void VnUartPacket_parseVNSTV(VnUartPacket* packet, vec4f* quaternion, vec3f* angularRateBias)
{
	size_t packetIndex;

	uint8_t* result = VnUartPacket_startAsciiPacketParse(packet->data, &packetIndex);

	quaternion->c[0] = ATOFF; NEXT
	quaternion->c[1] = ATOFF; NEXT
	quaternion->c[2] = ATOFF; NEXT
	quaternion->c[3] = ATOFF; NEXT
	angularRateBias->c[0] = ATOFF; NEXT
	angularRateBias->c[1] = ATOFF; NEXT
	angularRateBias->c[2] = ATOFF;
}

void VnUartPacket_parseVNCOV(VnUartPacket* packet, vec3f* attitudeVariance, vec3f* angularRateBiasVariance)
{
	size_t packetIndex;

	uint8_t* result = VnUartPacket_startAsciiPacketParse(packet->data, &packetIndex);

	attitudeVariance->c[0] = ATOFF; NEXT
	attitudeVariance->c[1] = ATOFF; NEXT
	attitudeVariance->c[2] = ATOFF; NEXT
	angularRateBiasVariance->c[0] = ATOFF; NEXT
	angularRateBiasVariance->c[1] = ATOFF; NEXT
	angularRateBiasVariance->c[2] = ATOFF;
}

#endif

void VnUartPacket_parseVNGPE(VnUartPacket* packet, double* tow, uint16_t* week, uint8_t* gpsFix, uint8_t* numSats, vec3d* position, vec3f* velocity, vec3f* posAcc, float* speedAcc, float* timeAcc)
{
	size_t packetIndex;

	uint8_t* result = VnUartPacket_startAsciiPacketParse(packet->data, &packetIndex);

	*tow = ATOFD; NEXT
	*week = ATOU16; NEXT
	*gpsFix = ATOU8; NEXT
	*numSats = ATOU8; NEXT
	position->c[0] = ATOFD; NEXT
	position->c[1] = ATOFD; NEXT
	position->c[2] = ATOFD; NEXT
	velocity->c[0] = ATOFF; NEXT
	velocity->c[1] = ATOFF; NEXT
	velocity->c[2] = ATOFF; NEXT
	posAcc->c[0] = ATOFF; NEXT
	posAcc->c[1] = ATOFF; NEXT
	posAcc->c[2] = ATOFF; NEXT
	*speedAcc = ATOFF; NEXT
	*timeAcc = ATOFF;
}

void VnUartPacket_parseVNDTV(VnUartPacket* packet, float* deltaTime, vec3f* deltaTheta, vec3f* deltaVelocity)
{
	size_t packetIndex;

	uint8_t* result = VnUartPacket_startAsciiPacketParse(packet->data, &packetIndex);

	*deltaTime = ATOFF; NEXT
	deltaTheta->c[0] = ATOFF; NEXT
	deltaTheta->c[1] = ATOFF; NEXT
	deltaTheta->c[2] = ATOFF; NEXT
	deltaVelocity->c[0] = ATOFF; NEXT
	deltaVelocity->c[1] = ATOFF; NEXT
	deltaVelocity->c[2] = ATOFF;
}

VnError VnUartPacket_genWrite(
	uint8_t *buffer,
    size_t bufferSize,
    VnErrorDetectionMode errorDetectionMode,
    uint16_t registerId,
    size_t *cmdSize,
	char const *format,
    ...)
{
    va_list ap;
	uint8_t *curOutputLoc = buffer;
	char const *curFormatPos = format;

	/* Avoid a compiler warning by doing this */
	(void)bufferSize;

	#if defined(_MSC_VER)
		/* Disable warnings regarding using strcpy_s since this
		 * function's signature does not provide us with information
		 * about the length of 'out'. */
		#pragma warning(push)
		#pragma warning(disable:4996)
	#endif

	/* Add the message header and register number. */
	curOutputLoc += sprintf((char*) curOutputLoc, "$VNWRG,%d", registerId);

	va_start(ap, format);

	/* Now cycle through the provided format specifier. */
	while (*curFormatPos != '\0')
	{
		switch (*curFormatPos++)
		{
		case 'U':

			switch (*curFormatPos++)
			{
			case '1':
				/* 'uint8_t' is promoted to 'int' when passed through '...'. */
				curOutputLoc += sprintf((char*) curOutputLoc, ",%d", va_arg(ap, int));
				break;

			case '2':
				/* 'uint16_t' is promoted to 'int' when passed through '...'. */
				curOutputLoc += sprintf((char*) curOutputLoc, ",%d", va_arg(ap, int));
				break;

			case '4':
				curOutputLoc += sprintf((char*) curOutputLoc, ",%d", va_arg(ap, uint32_t));
				break;
			}

			break;

		case 'F':

			switch (*curFormatPos++)
			{
			case '4':
				/* 'float' is promoted to 'double' when passed through '...'. */
				curOutputLoc += sprintf((char*) curOutputLoc, ",%f", va_arg(ap, double));
				break;

			case '8':
				curOutputLoc += sprintf((char*) curOutputLoc, ",%f", va_arg(ap, double));
				break;
			}

			break;
			
		case 'S':
			curOutputLoc += sprintf((char*) curOutputLoc,",%s",va_arg(ap,char *));
			break;
		}
	}

	#if defined(_MSC_VER)
		#pragma warning(pop)
	#endif

	va_end(ap);

	/* Compute and append the checksum. */
	*curOutputLoc++ = '*';

	if (errorDetectionMode == VNERRORDETECTIONMODE_NONE)
	{
		*curOutputLoc++ = 'X';
		*curOutputLoc++ = 'X';
	}
	else if (errorDetectionMode == VNERRORDETECTIONMODE_CHECKSUM)
	{
		uint8_t checksum = VnChecksum8_compute((char*) (buffer + 1), curOutputLoc - buffer - 2);
		VnUtil_toHexStrFromUint8(checksum, (char*) curOutputLoc);

		curOutputLoc += 2;
	}
	else if (errorDetectionMode == VNERRORDETECTIONMODE_CRC)
	{
		uint16_t crc = VnCrc16_compute((char*) (buffer + 1), curOutputLoc - buffer - 2);
		VnUtil_toHexStrFromUint16(crc, (char*) curOutputLoc);

		curOutputLoc += 4;
	}
	else
	{
		return E_NOT_SUPPORTED;
	}

	*curOutputLoc++ = '\r';
	*curOutputLoc++ = '\n';

	*cmdSize = curOutputLoc - buffer;

	return E_NONE;
}

VnError VnUartPacket_genReadBinaryOutput1(
	uint8_t *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize)
{
	#if VN_HAVE_SECURE_CRT
	*cmdSize = sprintf_s((char*)buffer, bufferSize, "$VNRRG,75");
	#else
	*cmdSize = sprintf((char*) buffer, "$VNRRG,75");
	#endif

	return VnUartPacket_finalizeCommand(errorDetectionMode, buffer, cmdSize);
}

VnError VnUartPacket_genReadBinaryOutput2(
	uint8_t *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize)
{
	#if VN_HAVE_SECURE_CRT
	*cmdSize = sprintf_s((char*)buffer, bufferSize, "$VNRRG,76");
	#else
	*cmdSize = sprintf((char*) buffer, "$VNRRG,76");
	#endif

	return VnUartPacket_finalizeCommand(errorDetectionMode, buffer, cmdSize);
}

VnError VnUartPacket_genReadBinaryOutput3(
	uint8_t *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize)
{
	#if VN_HAVE_SECURE_CRT
	*cmdSize = sprintf_s((char*)buffer, bufferSize, "$VNRRG,77");
	#else
	*cmdSize = sprintf((char*) buffer, "$VNRRG,77");
	#endif

	return VnUartPacket_finalizeCommand(errorDetectionMode, buffer, cmdSize);
}

#ifdef EXTRA

VnError VnUartPacket_genReadBinaryOutput4(
	uint8_t *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize)
{
	#if VN_HAVE_SECURE_CRT
	*cmdSize = sprintf_s(buffer, bufferSize, "$VNRRG,78");
	#else
	*cmdSize = sprintf(buffer, "$VNRRG,78");
	#endif

	return VnUartPacket_finalizeCommand(errorDetectionMode, buffer, cmdSize);
}

VnError VnUartPacket_genReadBinaryOutput5(
	uint8_t *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize)
{
	#if VN_HAVE_SECURE_CRT
	*cmdSize = sprintf_s(buffer, bufferSize, "$VNRRG,79");
	#else
	*cmdSize = sprintf(buffer, "$VNRRG,79");
	#endif

	return VnUartPacket_finalizeCommand(errorDetectionMode, buffer, cmdSize);
}

#endif

VnError VnUartPacket_finalizeCommand(VnErrorDetectionMode errorDetectionMode, uint8_t *packet, size_t *length)
{
	#if defined(_MSC_VER)
		/* Disable warnings regarding using sprintf_s. */
		#pragma warning(push)
		#pragma warning(disable:4996)
	#endif

	if (errorDetectionMode == VNERRORDETECTIONMODE_CHECKSUM)
	{
		*length += sprintf((char*) (packet + *length), "*%02X\r\n", VnChecksum8_compute((char*) packet + 1, *length - 1));
	}
	else if (errorDetectionMode == VNERRORDETECTIONMODE_CRC)
	{
		*length += sprintf((char*) (packet + *length), "*%04X\r\n", VnCrc16_compute((char*) packet + 1, *length - 1));
	}
	else
	{
		*length += sprintf((char*) (packet + *length), "*XX\r\n");
	}

	#if defined(_MSC_VER)
		#pragma warning(pop)
	#endif

	return E_NONE;
}

VnError VnUartPacket_genCmdWriteSettings(
	uint8_t *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize)
{
	#if VN_HAVE_SECURE_CRT
	*cmdSize = sprintf_s((char*)buffer, bufferSize, "$VNWNV");
	#else
	*cmdSize = sprintf((char*) buffer, "$VNWNV");
	#endif

	return VnUartPacket_finalizeCommand(errorDetectionMode, buffer, cmdSize);
}

VnError VnUartPacket_genCmdRestoreFactorySettings(
	uint8_t *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize)
{
	#if VN_HAVE_SECURE_CRT
	*cmdSize = sprintf_s((char*)buffer, bufferSize, "$VNRFS");
	#else
	*cmdSize = sprintf((char*) buffer, "$VNRFS");
	#endif

	return VnUartPacket_finalizeCommand(errorDetectionMode, buffer, cmdSize);
}

VnError VnUartPacket_genCmdReset(
	uint8_t *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize)
{
	#if VN_HAVE_SECURE_CRT
	*cmdSize = sprintf_s((char*)buffer, bufferSize, "$VNRST");
	#else
	*cmdSize = sprintf((char*) buffer, "$VNRST");
	#endif

	return VnUartPacket_finalizeCommand(errorDetectionMode, buffer, cmdSize);
}

VnError VnUartPacket_genCmdFirmwareUpdate(
	uint8_t* buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t* cmdSize)
{
#if VN_HAVE_SECURE_CRT
	* cmdSize = sprintf_s((char*)buffer, bufferSize, "$VNFWU");
#else
	* cmdSize = sprintf((char*)buffer, "$VNFWU");
#endif

	return VnUartPacket_finalizeCommand(errorDetectionMode, buffer, cmdSize);
}


VnError VnUartPacket_genCmdTare(
	uint8_t *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize)
{
	#if VN_HAVE_SECURE_CRT
	*cmdSize = sprintf_s((char*)buffer, bufferSize, "$VNTAR");
	#else
	*cmdSize = sprintf((char*) buffer, "$VNTAR");
	#endif

	return VnUartPacket_finalizeCommand(errorDetectionMode, buffer, cmdSize);
}

VnError VnUartPacket_genCmdSetGyroBias(
	uint8_t *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize)
{
	#if VN_HAVE_SECURE_CRT
	*cmdSize = sprintf_s((char*)buffer, bufferSize, "$VNSGB");
	#else
	*cmdSize = sprintf((char*) buffer, "$VNSGB");
	#endif

	return VnUartPacket_finalizeCommand(errorDetectionMode, buffer, cmdSize);
}

VnError VnUartPacket_genCmdKnownMagneticDisturbance(
	uint8_t *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	bool disturbancePresent,
	size_t *cmdSize)
{
	#if VN_HAVE_SECURE_CRT
	*cmdSize = sprintf_s((char*)buffer, bufferSize, "$VNKMD,%d", disturbancePresent ? 1 : 0);
	#else
	*cmdSize = sprintf((char*) buffer, "$VNKMD,%d", disturbancePresent ? 1 : 0);
	#endif

	return VnUartPacket_finalizeCommand(errorDetectionMode, buffer, cmdSize);
}

VnError VnUartPacket_genCmdKnownAccelerationDisturbance(
	uint8_t *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	bool disturbancePresent,
	size_t *cmdSize)
{
	#if VN_HAVE_SECURE_CRT
	*cmdSize = sprintf_s((char*)buffer, bufferSize, "$VNKAD,%d", disturbancePresent ? 1 : 0);
	#else
	*cmdSize = sprintf((char*) buffer, "$VNKAD,%d", disturbancePresent ? 1 : 0);
	#endif

	return VnUartPacket_finalizeCommand(errorDetectionMode, buffer, cmdSize);
}

VnError VnUartPacket_genReadUserTag(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize)
{
	#if VN_HAVE_SECURE_CRT
	*cmdSize = sprintf_s(buffer, bufferSize, "$VNRRG,00");
	#else
	*cmdSize = sprintf(buffer, "$VNRRG,00");
	#endif

	return VnUartPacket_finalizeCommand(errorDetectionMode, (uint8_t*)buffer, cmdSize);
}

VnError VnUartPacket_genReadModelNumber(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize)
{
	#if VN_HAVE_SECURE_CRT
	*cmdSize = sprintf_s(buffer, bufferSize, "$VNRRG,01");
	#else
	*cmdSize = sprintf(buffer, "$VNRRG,01");
	#endif

	return VnUartPacket_finalizeCommand(errorDetectionMode, (uint8_t*)buffer, cmdSize);
}

VnError VnUartPacket_genReadHardwareRevision(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize)
{
	#if VN_HAVE_SECURE_CRT
	*cmdSize = sprintf_s(buffer, bufferSize, "$VNRRG,02");
	#else
	*cmdSize = sprintf(buffer, "$VNRRG,02");
	#endif

	return VnUartPacket_finalizeCommand(errorDetectionMode, (uint8_t*)buffer, cmdSize);
}

VnError VnUartPacket_genReadSerialNumber(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize)
{
	#if VN_HAVE_SECURE_CRT
	*cmdSize = sprintf_s(buffer, bufferSize, "$VNRRG,03");
	#else
	*cmdSize = sprintf(buffer, "$VNRRG,03");
	#endif

	return VnUartPacket_finalizeCommand(errorDetectionMode, (uint8_t*)buffer, cmdSize);
}

VnError VnUartPacket_genReadFirmwareVersion(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize)
{
	#if VN_HAVE_SECURE_CRT
	*cmdSize = sprintf_s(buffer, bufferSize, "$VNRRG,04");
	#else
	*cmdSize = sprintf(buffer, "$VNRRG,04");
	#endif

	return VnUartPacket_finalizeCommand(errorDetectionMode, (uint8_t*)buffer, cmdSize);
}

VnError VnUartPacket_genReadSerialBaudRate(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize)
{
	#if VN_HAVE_SECURE_CRT
	*cmdSize = sprintf_s(buffer, bufferSize, "$VNRRG,05");
	#else
	*cmdSize = sprintf(buffer, "$VNRRG,05");
	#endif

	return VnUartPacket_finalizeCommand(errorDetectionMode, (uint8_t*)buffer, cmdSize);
}

VnError VnUartPacket_genReadAsyncDataOutputType(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize)
{
	#if VN_HAVE_SECURE_CRT
	*cmdSize = sprintf_s(buffer, bufferSize, "$VNRRG,06");
	#else
	*cmdSize = sprintf(buffer, "$VNRRG,06");
	#endif

	return VnUartPacket_finalizeCommand(errorDetectionMode, (uint8_t*)buffer, cmdSize);
}

VnError VnUartPacket_genReadAsyncDataOutputFrequency(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize)
{
	#if VN_HAVE_SECURE_CRT
	*cmdSize = sprintf_s(buffer, bufferSize, "$VNRRG,07");
	#else
	*cmdSize = sprintf(buffer, "$VNRRG,07");
	#endif

	return VnUartPacket_finalizeCommand(errorDetectionMode, (uint8_t*)buffer, cmdSize);
}

VnError VnUartPacket_genReadYawPitchRoll(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize)
{
	#if VN_HAVE_SECURE_CRT
	*cmdSize = sprintf_s(buffer, bufferSize, "$VNRRG,08");
	#else
	*cmdSize = sprintf(buffer, "$VNRRG,08");
	#endif

	return VnUartPacket_finalizeCommand(errorDetectionMode, (uint8_t*)buffer, cmdSize);
}

VnError VnUartPacket_genReadAttitudeQuaternion(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize)
{
	#if VN_HAVE_SECURE_CRT
	*cmdSize = sprintf_s(buffer, bufferSize, "$VNRRG,09");
	#else
	*cmdSize = sprintf(buffer, "$VNRRG,09");
	#endif

	return VnUartPacket_finalizeCommand(errorDetectionMode, (uint8_t*)buffer, cmdSize);
}

VnError VnUartPacket_genReadQuaternionMagneticAccelerationAndAngularRates(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize)
{
	#if VN_HAVE_SECURE_CRT
	*cmdSize = sprintf_s(buffer, bufferSize, "$VNRRG,15");
	#else
	*cmdSize = sprintf(buffer, "$VNRRG,15");
	#endif

	return VnUartPacket_finalizeCommand(errorDetectionMode, (uint8_t*)buffer, cmdSize);
}

VnError VnUartPacket_genReadMagneticMeasurements(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize)
{
	#if VN_HAVE_SECURE_CRT
	*cmdSize = sprintf_s(buffer, bufferSize, "$VNRRG,17");
	#else
	*cmdSize = sprintf(buffer, "$VNRRG,17");
	#endif

	return VnUartPacket_finalizeCommand(errorDetectionMode, (uint8_t*)buffer, cmdSize);
}

VnError VnUartPacket_genReadAccelerationMeasurements(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize)
{
	#if VN_HAVE_SECURE_CRT
	*cmdSize = sprintf_s(buffer, bufferSize, "$VNRRG,18");
	#else
	*cmdSize = sprintf(buffer, "$VNRRG,18");
	#endif

	return VnUartPacket_finalizeCommand(errorDetectionMode, (uint8_t*)buffer, cmdSize);
}

VnError VnUartPacket_genReadAngularRateMeasurements(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize)
{
	#if VN_HAVE_SECURE_CRT
	*cmdSize = sprintf_s(buffer, bufferSize, "$VNRRG,19");
	#else
	*cmdSize = sprintf(buffer, "$VNRRG,19");
	#endif

	return VnUartPacket_finalizeCommand(errorDetectionMode, (uint8_t*)buffer, cmdSize);
}

VnError VnUartPacket_genReadMagneticAccelerationAndAngularRates(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize)
{
	#if VN_HAVE_SECURE_CRT
	*cmdSize = sprintf_s(buffer, bufferSize, "$VNRRG,20");
	#else
	*cmdSize = sprintf(buffer, "$VNRRG,20");
	#endif

	return VnUartPacket_finalizeCommand(errorDetectionMode, (uint8_t*)buffer, cmdSize);
}

VnError VnUartPacket_genReadMagneticAndGravityReferenceVectors(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize)
{
	#if VN_HAVE_SECURE_CRT
	*cmdSize = sprintf_s(buffer, bufferSize, "$VNRRG,21");
	#else
	*cmdSize = sprintf(buffer, "$VNRRG,21");
	#endif

	return VnUartPacket_finalizeCommand(errorDetectionMode, (uint8_t*)buffer, cmdSize);
}

VnError VnUartPacket_genReadMagnetometerCompensation(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize)
{
	#if VN_HAVE_SECURE_CRT
	*cmdSize = sprintf_s(buffer, bufferSize, "$VNRRG,23");
	#else
	*cmdSize = sprintf(buffer, "$VNRRG,23");
	#endif

	return VnUartPacket_finalizeCommand(errorDetectionMode, (uint8_t*)buffer, cmdSize);
}

VnError VnUartPacket_genReadAccelerationCompensation(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize)
{
	#if VN_HAVE_SECURE_CRT
	*cmdSize = sprintf_s(buffer, bufferSize, "$VNRRG,25");
	#else
	*cmdSize = sprintf(buffer, "$VNRRG,25");
	#endif

	return VnUartPacket_finalizeCommand(errorDetectionMode, (uint8_t*)buffer, cmdSize);
}

VnError VnUartPacket_genReadReferenceFrameRotation(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize)
{
	#if VN_HAVE_SECURE_CRT
	*cmdSize = sprintf_s(buffer, bufferSize, "$VNRRG,26");
	#else
	*cmdSize = sprintf(buffer, "$VNRRG,26");
	#endif

	return VnUartPacket_finalizeCommand(errorDetectionMode, (uint8_t*)buffer, cmdSize);
}

VnError VnUartPacket_genReadYawPitchRollMagneticAccelerationAndAngularRates(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize)
{
	#if VN_HAVE_SECURE_CRT
	*cmdSize = sprintf_s(buffer, bufferSize, "$VNRRG,27");
	#else
	*cmdSize = sprintf(buffer, "$VNRRG,27");
	#endif

	return VnUartPacket_finalizeCommand(errorDetectionMode, (uint8_t*)buffer, cmdSize);
}

VnError VnUartPacket_genReadCommunicationProtocolControl(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize)
{
	#if VN_HAVE_SECURE_CRT
	*cmdSize = sprintf_s(buffer, bufferSize, "$VNRRG,30");
	#else
	*cmdSize = sprintf(buffer, "$VNRRG,30");
	#endif

	return VnUartPacket_finalizeCommand(errorDetectionMode, (uint8_t*)buffer, cmdSize);
}

VnError VnUartPacket_genReadSynchronizationControl(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize)
{
	#if VN_HAVE_SECURE_CRT
	*cmdSize = sprintf_s(buffer, bufferSize, "$VNRRG,32");
	#else
	*cmdSize = sprintf(buffer, "$VNRRG,32");
	#endif

	return VnUartPacket_finalizeCommand(errorDetectionMode, (uint8_t*)buffer, cmdSize);
}

VnError VnUartPacket_genReadSynchronizationStatus(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize)
{
	#if VN_HAVE_SECURE_CRT
	*cmdSize = sprintf_s(buffer, bufferSize, "$VNRRG,33");
	#else
	*cmdSize = sprintf(buffer, "$VNRRG,33");
	#endif

	return VnUartPacket_finalizeCommand(errorDetectionMode, (uint8_t*)buffer, cmdSize);
}

VnError VnUartPacket_genReadVpeBasicControl(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize)
{
	#if VN_HAVE_SECURE_CRT
	*cmdSize = sprintf_s(buffer, bufferSize, "$VNRRG,35");
	#else
	*cmdSize = sprintf(buffer, "$VNRRG,35");
	#endif

	return VnUartPacket_finalizeCommand(errorDetectionMode, (uint8_t*)buffer, cmdSize);
}

VnError VnUartPacket_genReadVpeMagnetometerBasicTuning(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize)
{
	#if VN_HAVE_SECURE_CRT
	*cmdSize = sprintf_s(buffer, bufferSize, "$VNRRG,36");
	#else
	*cmdSize = sprintf(buffer, "$VNRRG,36");
	#endif

	return VnUartPacket_finalizeCommand(errorDetectionMode, (uint8_t*)buffer, cmdSize);
}

VnError VnUartPacket_genReadVpeAccelerometerBasicTuning(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize)
{
	#if VN_HAVE_SECURE_CRT
	*cmdSize = sprintf_s(buffer, bufferSize, "$VNRRG,38");
	#else
	*cmdSize = sprintf(buffer, "$VNRRG,38");
	#endif

	return VnUartPacket_finalizeCommand(errorDetectionMode, (uint8_t*)buffer, cmdSize);
}

VnError VnUartPacket_genReadMagnetometerCalibrationControl(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize)
{
	#if VN_HAVE_SECURE_CRT
	*cmdSize = sprintf_s(buffer, bufferSize, "$VNRRG,44");
	#else
	*cmdSize = sprintf(buffer, "$VNRRG,44");
	#endif

	return VnUartPacket_finalizeCommand(errorDetectionMode, (uint8_t*)buffer, cmdSize);
}

VnError VnUartPacket_genReadCalculatedMagnetometerCalibration(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize)
{
	#if VN_HAVE_SECURE_CRT
	*cmdSize = sprintf_s(buffer, bufferSize, "$VNRRG,47");
	#else
	*cmdSize = sprintf(buffer, "$VNRRG,47");
	#endif

	return VnUartPacket_finalizeCommand(errorDetectionMode, (uint8_t*)buffer, cmdSize);
}

VnError VnUartPacket_genReadVelocityCompensationMeasurement(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize)
{
	#if VN_HAVE_SECURE_CRT
	*cmdSize = sprintf_s(buffer, bufferSize, "$VNRRG,50");
	#else
	*cmdSize = sprintf(buffer, "$VNRRG,50");
	#endif

	return VnUartPacket_finalizeCommand(errorDetectionMode, (uint8_t*)buffer, cmdSize);
}

VnError VnUartPacket_genReadVelocityCompensationControl(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize)
{
	#if VN_HAVE_SECURE_CRT
	*cmdSize = sprintf_s(buffer, bufferSize, "$VNRRG,51");
	#else
	*cmdSize = sprintf(buffer, "$VNRRG,51");
	#endif

	return VnUartPacket_finalizeCommand(errorDetectionMode, (uint8_t*)buffer, cmdSize);
}

VnError VnUartPacket_genReadImuMeasurements(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize)
{
	#if VN_HAVE_SECURE_CRT
	*cmdSize = sprintf_s(buffer, bufferSize, "$VNRRG,54");
	#else
	*cmdSize = sprintf(buffer, "$VNRRG,54");
	#endif

	return VnUartPacket_finalizeCommand(errorDetectionMode, (uint8_t*)buffer, cmdSize);
}

VnError VnUartPacket_genReadGpsConfiguration(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize)
{
	#if VN_HAVE_SECURE_CRT
	*cmdSize = sprintf_s(buffer, bufferSize, "$VNRRG,55");
	#else
	*cmdSize = sprintf(buffer, "$VNRRG,55");
	#endif

	return VnUartPacket_finalizeCommand(errorDetectionMode, (uint8_t*)buffer, cmdSize);
}

VnError VnUartPacket_genReadGpsAntennaOffset(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize)
{
	#if VN_HAVE_SECURE_CRT
	*cmdSize = sprintf_s(buffer, bufferSize, "$VNRRG,57");
	#else
	*cmdSize = sprintf(buffer, "$VNRRG,57");
	#endif

	return VnUartPacket_finalizeCommand(errorDetectionMode, (uint8_t*)buffer, cmdSize);
}

VnError VnUartPacket_genReadGpsSolutionLla(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize)
{
	#if VN_HAVE_SECURE_CRT
	*cmdSize = sprintf_s(buffer, bufferSize, "$VNRRG,58");
	#else
	*cmdSize = sprintf(buffer, "$VNRRG,58");
	#endif

	return VnUartPacket_finalizeCommand(errorDetectionMode, (uint8_t*)buffer, cmdSize);
}

VnError VnUartPacket_genReadGpsSolutionEcef(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize)
{
	#if VN_HAVE_SECURE_CRT
	*cmdSize = sprintf_s(buffer, bufferSize, "$VNRRG,59");
	#else
	*cmdSize = sprintf(buffer, "$VNRRG,59");
	#endif

	return VnUartPacket_finalizeCommand(errorDetectionMode, (uint8_t*)buffer, cmdSize);
}

VnError VnUartPacket_genReadInsSolutionLla(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize)
{
	#if VN_HAVE_SECURE_CRT
	*cmdSize = sprintf_s(buffer, bufferSize, "$VNRRG,63");
	#else
	*cmdSize = sprintf(buffer, "$VNRRG,63");
	#endif

	return VnUartPacket_finalizeCommand(errorDetectionMode, (uint8_t*)buffer, cmdSize);
}

VnError VnUartPacket_genReadInsSolutionEcef(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize)
{
	#if VN_HAVE_SECURE_CRT
	*cmdSize = sprintf_s(buffer, bufferSize, "$VNRRG,64");
	#else
	*cmdSize = sprintf(buffer, "$VNRRG,64");
	#endif

	return VnUartPacket_finalizeCommand(errorDetectionMode, (uint8_t*)buffer, cmdSize);
}

VnError VnUartPacket_genReadInsBasicConfiguration(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize)
{
	#if VN_HAVE_SECURE_CRT
	*cmdSize = sprintf_s(buffer, bufferSize, "$VNRRG,67");
	#else
	*cmdSize = sprintf(buffer, "$VNRRG,67");
	#endif

	return VnUartPacket_finalizeCommand(errorDetectionMode, (uint8_t*)buffer, cmdSize);
}

VnError VnUartPacket_genReadInsStateLla(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize)
{
	#if VN_HAVE_SECURE_CRT
	*cmdSize = sprintf_s(buffer, bufferSize, "$VNRRG,72");
	#else
	*cmdSize = sprintf(buffer, "$VNRRG,72");
	#endif

	return VnUartPacket_finalizeCommand(errorDetectionMode, (uint8_t*)buffer, cmdSize);
}

VnError VnUartPacket_genReadInsStateEcef(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize)
{
	#if VN_HAVE_SECURE_CRT
	*cmdSize = sprintf_s(buffer, bufferSize, "$VNRRG,73");
	#else
	*cmdSize = sprintf(buffer, "$VNRRG,73");
	#endif

	return VnUartPacket_finalizeCommand(errorDetectionMode, (uint8_t*)buffer, cmdSize);
}

VnError VnUartPacket_genReadStartupFilterBiasEstimate(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize)
{
	#if VN_HAVE_SECURE_CRT
	*cmdSize = sprintf_s(buffer, bufferSize, "$VNRRG,74");
	#else
	*cmdSize = sprintf(buffer, "$VNRRG,74");
	#endif

	return VnUartPacket_finalizeCommand(errorDetectionMode, (uint8_t*)buffer, cmdSize);
}

VnError VnUartPacket_genReadDeltaThetaAndDeltaVelocity(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize)
{
	#if VN_HAVE_SECURE_CRT
	*cmdSize = sprintf_s(buffer, bufferSize, "$VNRRG,80");
	#else
	*cmdSize = sprintf(buffer, "$VNRRG,80");
	#endif

	return VnUartPacket_finalizeCommand(errorDetectionMode, (uint8_t*)buffer, cmdSize);
}

VnError VnUartPacket_genReadDeltaThetaAndDeltaVelocityConfiguration(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize)
{
	#if VN_HAVE_SECURE_CRT
	*cmdSize = sprintf_s(buffer, bufferSize, "$VNRRG,82");
	#else
	*cmdSize = sprintf(buffer, "$VNRRG,82");
	#endif

	return VnUartPacket_finalizeCommand(errorDetectionMode, (uint8_t*)buffer, cmdSize);
}

VnError VnUartPacket_genReadReferenceVectorConfiguration(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize)
{
	#if VN_HAVE_SECURE_CRT
	*cmdSize = sprintf_s(buffer, bufferSize, "$VNRRG,83");
	#else
	*cmdSize = sprintf(buffer, "$VNRRG,83");
	#endif

	return VnUartPacket_finalizeCommand(errorDetectionMode, (uint8_t*)buffer, cmdSize);
}

VnError VnUartPacket_genReadGyroCompensation(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize)
{
	#if VN_HAVE_SECURE_CRT
	*cmdSize = sprintf_s(buffer, bufferSize, "$VNRRG,84");
	#else
	*cmdSize = sprintf(buffer, "$VNRRG,84");
	#endif

	return VnUartPacket_finalizeCommand(errorDetectionMode, (uint8_t*)buffer, cmdSize);
}

VnError VnUartPacket_genReadImuFilteringConfiguration(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize)
{
	#if VN_HAVE_SECURE_CRT
	*cmdSize = sprintf_s(buffer, bufferSize, "$VNRRG,85");
	#else
	*cmdSize = sprintf(buffer, "$VNRRG,85");
	#endif

	return VnUartPacket_finalizeCommand(errorDetectionMode, (uint8_t*)buffer, cmdSize);
}

VnError VnUartPacket_genReadGpsCompassBaseline(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize)
{
	#if VN_HAVE_SECURE_CRT
	*cmdSize = sprintf_s(buffer, bufferSize, "$VNRRG,93");
	#else
	*cmdSize = sprintf(buffer, "$VNRRG,93");
	#endif

	return VnUartPacket_finalizeCommand(errorDetectionMode, (uint8_t*)buffer, cmdSize);
}

VnError VnUartPacket_genReadGpsCompassEstimatedBaseline(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize)
{
	#if VN_HAVE_SECURE_CRT
	*cmdSize = sprintf_s(buffer, bufferSize, "$VNRRG,97");
	#else
	*cmdSize = sprintf(buffer, "$VNRRG,97");
	#endif

	return VnUartPacket_finalizeCommand(errorDetectionMode, (uint8_t*)buffer, cmdSize);
}

VnError VnUartPacket_genReadYawPitchRollTrueBodyAccelerationAndAngularRates(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize)
{
	#if VN_HAVE_SECURE_CRT
	*cmdSize = sprintf_s(buffer, bufferSize, "$VNRRG,239");
	#else
	*cmdSize = sprintf(buffer, "$VNRRG,239");
	#endif

	return VnUartPacket_finalizeCommand(errorDetectionMode, (uint8_t*)buffer, cmdSize);
}

VnError VnUartPacket_genReadYawPitchRollTrueInertialAccelerationAndAngularRates(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize)
{
	#if VN_HAVE_SECURE_CRT
	*cmdSize = sprintf_s(buffer, bufferSize, "$VNRRG,240");
	#else
	*cmdSize = sprintf(buffer, "$VNRRG,240");
	#endif

	return VnUartPacket_finalizeCommand(errorDetectionMode, (uint8_t*)buffer, cmdSize);
}

VnError VnUartPacket_genWriteBinaryOutput(
	uint8_t *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	uint8_t binaryOutputNumber,
	uint16_t asyncMode,
	uint16_t rateDivisor,
	uint16_t commonField,
	uint16_t timeField,
	uint16_t imuField,
	uint16_t gpsField,
	uint16_t attitudeField,
	uint16_t insField,
  uint16_t gps2Field)
{
	uint16_t groups = 0;
	#if !VN_HAVE_SECURE_CRT
	char* cbp = (char*)buffer;
	#endif

	/* First determine which groups are present. */
	if (commonField)
		groups |= 0x0001;
	if (timeField)
		groups |= 0x0002;
	if (imuField)
		groups |= 0x0004;
	if (gpsField)
		groups |= 0x0008;
	if (attitudeField)
		groups |= 0x0010;
	if (insField)
		groups |= 0x0020;
  if (gps2Field)
    groups |= 0x0040;

	#if VN_HAVE_SECURE_CRT
	*cmdSize = sprintf_s((char*)buffer, bufferSize, "$VNWRG,%u,%u,%u,%X", 74 + binaryOutputNumber, asyncMode, rateDivisor, groups);
	#else
	*cmdSize = sprintf(cbp, "$VNWRG,%u,%u,%u,%X", 74 + binaryOutputNumber, asyncMode, rateDivisor, groups);
	#endif

	if (commonField)
		#if VN_HAVE_SECURE_CRT
		*cmdSize += sprintf_s((char*)(buffer + *cmdSize), bufferSize - *cmdSize, ",%X", commonField);
		#else
		*cmdSize += sprintf(cbp + *cmdSize, ",%X", commonField);
		#endif
	if (timeField)
		#if VN_HAVE_SECURE_CRT
		*cmdSize += sprintf_s((char*)(buffer + *cmdSize), bufferSize - *cmdSize, ",%X", timeField);
		#else
		*cmdSize += sprintf(cbp + *cmdSize, ",%X", timeField);
		#endif
	if (imuField)
		#if VN_HAVE_SECURE_CRT
		*cmdSize += sprintf_s((char*)(buffer + *cmdSize), bufferSize - *cmdSize, ",%X", imuField);
		#else
		*cmdSize += sprintf(cbp + *cmdSize, ",%X", imuField);
		#endif
	if (gpsField)
		#if VN_HAVE_SECURE_CRT
		*cmdSize += sprintf_s((char*)(buffer + *cmdSize), bufferSize - *cmdSize, ",%X", gpsField);
		#else
		*cmdSize += sprintf(cbp + *cmdSize, ",%X", gpsField);
		#endif
	if (attitudeField)
		#if VN_HAVE_SECURE_CRT
		*cmdSize += sprintf_s((char*)(buffer + *cmdSize), bufferSize - *cmdSize, ",%X", attitudeField);
		#else
		*cmdSize += sprintf(cbp + *cmdSize, ",%X", attitudeField);
		#endif
	if (insField)
		#if VN_HAVE_SECURE_CRT
		*cmdSize += sprintf_s((char*)(buffer + *cmdSize), bufferSize - *cmdSize, ",%X", insField);
		#else
		*cmdSize += sprintf(cbp + *cmdSize, ",%X", insField);
		#endif
  if (gps2Field)
    #if VN_HAVE_SECURE_CRT
    *cmdSize += sprintf_s((char*)(buffer + *cmdSize), bufferSize - *cmdSize, ",%X", gps2Field);
    #else
    *cmdSize += sprintf(cbp + *cmdSize, ",%X", gps2Field);
    #endif

	return VnUartPacket_finalizeCommand(errorDetectionMode, buffer, cmdSize);
}

VnError VnUartPacket_genWriteBinaryOutput1(
	uint8_t *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	uint16_t asyncMode,
	uint16_t rateDivisor,
	uint16_t commonField,
	uint16_t timeField,
	uint16_t imuField,
	uint16_t gpsField,
	uint16_t attitudeField,
  uint16_t insField,
  uint16_t gps2Field)
{
	return VnUartPacket_genWriteBinaryOutput(
		buffer,
		bufferSize,
		errorDetectionMode,
		cmdSize,
		1,
		asyncMode,
		rateDivisor,
		commonField,
		timeField,
		imuField,
		gpsField,
		attitudeField,
		insField,
    gps2Field);
}

VnError VnUartPacket_genWriteBinaryOutput2(
	uint8_t *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	uint16_t asyncMode,
	uint16_t rateDivisor,
	uint16_t commonField,
	uint16_t timeField,
	uint16_t imuField,
	uint16_t gpsField,
	uint16_t attitudeField,
  uint16_t insField,
  uint16_t gps2Field)
{
	return VnUartPacket_genWriteBinaryOutput(
		buffer,
		bufferSize,
		errorDetectionMode,
		cmdSize,
		2,
		asyncMode,
		rateDivisor,
		commonField,
		timeField,
		imuField,
		gpsField,
		attitudeField,
    insField,
    gps2Field);
}

VnError VnUartPacket_genWriteBinaryOutput3(
	uint8_t *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	uint16_t asyncMode,
	uint16_t rateDivisor,
	uint16_t commonField,
	uint16_t timeField,
	uint16_t imuField,
	uint16_t gpsField,
	uint16_t attitudeField,
  uint16_t insField,
  uint16_t gps2Field)
{
	return VnUartPacket_genWriteBinaryOutput(
		buffer,
		bufferSize,
		errorDetectionMode,
		cmdSize,
		3,
		asyncMode,
		rateDivisor,
		commonField,
		timeField,
		imuField,
		gpsField,
		attitudeField,
    insField,
    gps2Field);
}

#ifdef EXTRA

VnError VnUartPacket_genWriteBinaryOutput4(
	uint8_t *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	uint16_t asyncMode,
	uint16_t rateDivisor,
	uint16_t commonField,
	uint16_t timeField,
	uint16_t imuField,
	uint16_t gpsField,
	uint16_t attitudeField,
  uint16_t insField,
  uint16_t gps2Field)
{
	return VnUartPacket_genWriteBinaryOutput(
		buffer,
		bufferSize,
		errorDetectionMode,
		cmdSize,
		4,
		asyncMode,
		rateDivisor,
		commonField,
		timeField,
		imuField,
		gpsField,
		attitudeField,
    insField,
    gps2Field);
}

VnError VnUartPacket_genWriteBinaryOutput5(
	uint8_t *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	uint16_t asyncMode,
	uint16_t rateDivisor,
	uint16_t commonField,
	uint16_t timeField,
	uint16_t imuField,
	uint16_t gpsField,
	uint16_t attitudeField,
  uint16_t insField,
  uint16_t gps2Field)
{
	return VnUartPacket_genWriteBinaryOutput(
		buffer,
		bufferSize,
		errorDetectionMode,
		cmdSize,
		5,
		asyncMode,
		rateDivisor,
		commonField,
		timeField,
		imuField,
		gpsField,
		attitudeField,
		insField,
    gps2Field);
}

#endif

VnError VnUartPacket_genWriteFirmwareUpdate(
	char* buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t* cmdSize,
	char* record)
{
#if VN_HAVE_SECURE_CRT
	* cmdSize = sprintf_s((char*)buffer, bufferSize, "$VNBLD,%s",record);
#else
	* cmdSize = sprintf((char*)buffer, "$VNBLD,%s", record);
#endif

	return VnUartPacket_finalizeCommand(errorDetectionMode, buffer, cmdSize);
}

VnError VnUartPacket_genWriteUserTag(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	char* tag)
{
	return VnUartPacket_genWrite(
		(uint8_t*)buffer,
		bufferSize,
		errorDetectionMode,
		0,
		cmdSize,
		"S",
		tag);
}

VnError VnUartPacket_genWriteSerialBaudRate(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	uint32_t baudrate)
{
	return VnUartPacket_genWrite(
		(uint8_t*)buffer,
		bufferSize,
		errorDetectionMode,
		5,
		cmdSize,
		"U4",
		baudrate);
}

VnError VnUartPacket_genWriteSerialBaudRate_with_options(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	uint32_t baudrate)
{
	return VnUartPacket_genWrite(
		(uint8_t*)buffer,
		bufferSize,
		errorDetectionMode,
		5,
		cmdSize,
		"U4",
		baudrate);
}

VnError VnUartPacket_genWriteAsyncDataOutputType(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	uint32_t ador)
{
	return VnUartPacket_genWrite(
		(uint8_t*)buffer,
		bufferSize,
		errorDetectionMode,
		6,
		cmdSize,
		"U4",
		ador);
}

VnError VnUartPacket_genWriteAsyncDataOutputType_with_options(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	uint32_t ador)
{
	return VnUartPacket_genWrite(
		(uint8_t*)buffer,
		bufferSize,
		errorDetectionMode,
		6,
		cmdSize,
		"U4",
		ador);
}

VnError VnUartPacket_genWriteAsyncDataOutputFrequency(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	uint32_t adof)
{
	return VnUartPacket_genWrite(
		(uint8_t*)buffer,
		bufferSize,
		errorDetectionMode,
		7,
		cmdSize,
		"U4",
		adof);
}

VnError VnUartPacket_genWriteAsyncDataOutputFrequency_with_options(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	uint32_t adof)
{
	return VnUartPacket_genWrite(
		(uint8_t*)buffer,
		bufferSize,
		errorDetectionMode,
		7,
		cmdSize,
		"U4",
		adof);
}

VnError VnUartPacket_genWriteMagneticAndGravityReferenceVectors(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	vec3f magRef,
	vec3f accRef)
{
	return VnUartPacket_genWrite(
		(uint8_t*)buffer,
		bufferSize,
		errorDetectionMode,
		21,
		cmdSize,
		"F4F4F4F4F4F4",
		magRef.c[0],
		magRef.c[1],
		magRef.c[2],
		accRef.c[0],
		accRef.c[1],
		accRef.c[2]);
}

VnError VnUartPacket_genWriteMagnetometerCompensation(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	mat3f c,
	vec3f b)
{
	return VnUartPacket_genWrite(
		(uint8_t*)buffer,
		bufferSize,
		errorDetectionMode,
		23,
		cmdSize,
		"F4F4F4F4F4F4F4F4F4F4F4F4",
		c.e[0],
		c.e[3],
		c.e[6],
		c.e[1],
		c.e[4],
		c.e[7],
		c.e[2],
		c.e[5],
		c.e[8],
		b.c[0],
		b.c[1],
		b.c[2]);
}

VnError VnUartPacket_genWriteAccelerationCompensation(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	mat3f c,
	vec3f b)
{
	return VnUartPacket_genWrite(
		(uint8_t*)buffer,
		bufferSize,
		errorDetectionMode,
		25,
		cmdSize,
		"F4F4F4F4F4F4F4F4F4F4F4F4",
		c.e[0],
		c.e[3],
		c.e[6],
		c.e[1],
		c.e[4],
		c.e[7],
		c.e[2],
		c.e[5],
		c.e[8],
		b.c[0],
		b.c[1],
		b.c[2]);
}

VnError VnUartPacket_genWriteReferenceFrameRotation(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	mat3f c)
{
	return VnUartPacket_genWrite(
		(uint8_t*)buffer,
		bufferSize,
		errorDetectionMode,
		26,
		cmdSize,
		"F4F4F4F4F4F4F4F4F4",
		c.e[0],
		c.e[3],
		c.e[6],
		c.e[1],
		c.e[4],
		c.e[7],
		c.e[2],
		c.e[5],
		c.e[8]);
}

VnError VnUartPacket_genWriteCommunicationProtocolControl(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	uint8_t serialCount,
	uint8_t serialStatus,
	uint8_t spiCount,
	uint8_t spiStatus,
	uint8_t serialChecksum,
	uint8_t spiChecksum,
	uint8_t errorMode)
{
	return VnUartPacket_genWrite(
		(uint8_t*)buffer,
		bufferSize,
		errorDetectionMode,
		30,
		cmdSize,
		"U1U1U1U1U1U1U1",
		serialCount,
		serialStatus,
		spiCount,
		spiStatus,
		serialChecksum,
		spiChecksum,
		errorMode);
}

VnError VnUartPacket_genWriteSynchronizationControl(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
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
	return VnUartPacket_genWrite(
		(uint8_t*)buffer,
		bufferSize,
		errorDetectionMode,
		32,
		cmdSize,
		"U1U1U2U4U1U1U2U4U4",
		syncInMode,
		syncInEdge,
		syncInSkipFactor,
		reserved1,
		syncOutMode,
		syncOutPolarity,
		syncOutSkipFactor,
		syncOutPulseWidth,
		reserved2);
}

VnError VnUartPacket_genWriteSynchronizationStatus(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	uint32_t syncInCount,
	uint32_t syncInTime,
	uint32_t syncOutCount)
{
	return VnUartPacket_genWrite(
		(uint8_t*)buffer,
		bufferSize,
		errorDetectionMode,
		33,
		cmdSize,
		"U4U4U4",
		syncInCount,
		syncInTime,
		syncOutCount);
}

VnError VnUartPacket_genWriteVpeBasicControl(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	uint8_t enable,
	uint8_t headingMode,
	uint8_t filteringMode,
	uint8_t tuningMode)
{
	return VnUartPacket_genWrite(
		(uint8_t*)buffer,
		bufferSize,
		errorDetectionMode,
		35,
		cmdSize,
		"U1U1U1U1",
		enable,
		headingMode,
		filteringMode,
		tuningMode);
}

VnError VnUartPacket_genWriteVpeMagnetometerBasicTuning(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	vec3f baseTuning,
	vec3f adaptiveTuning,
	vec3f adaptiveFiltering)
{
	return VnUartPacket_genWrite(
		(uint8_t*)buffer,
		bufferSize,
		errorDetectionMode,
		36,
		cmdSize,
		"F4F4F4F4F4F4F4F4F4",
		baseTuning.c[0],
		baseTuning.c[1],
		baseTuning.c[2],
		adaptiveTuning.c[0],
		adaptiveTuning.c[1],
		adaptiveTuning.c[2],
		adaptiveFiltering.c[0],
		adaptiveFiltering.c[1],
		adaptiveFiltering.c[2]);
}

VnError VnUartPacket_genWriteVpeAccelerometerBasicTuning(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	vec3f baseTuning,
	vec3f adaptiveTuning,
	vec3f adaptiveFiltering)
{
	return VnUartPacket_genWrite(
		(uint8_t*)buffer,
		bufferSize,
		errorDetectionMode,
		38,
		cmdSize,
		"F4F4F4F4F4F4F4F4F4",
		baseTuning.c[0],
		baseTuning.c[1],
		baseTuning.c[2],
		adaptiveTuning.c[0],
		adaptiveTuning.c[1],
		adaptiveTuning.c[2],
		adaptiveFiltering.c[0],
		adaptiveFiltering.c[1],
		adaptiveFiltering.c[2]);
}

VnError VnUartPacket_genWriteMagnetometerCalibrationControl(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	uint8_t hsiMode,
	uint8_t hsiOutput,
	uint8_t convergeRate)
{
	return VnUartPacket_genWrite(
		(uint8_t*)buffer,
		bufferSize,
		errorDetectionMode,
		44,
		cmdSize,
		"U1U1U1",
		hsiMode,
		hsiOutput,
		convergeRate);
}

VnError VnUartPacket_genWriteVelocityCompensationMeasurement(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	vec3f velocity)
{
	return VnUartPacket_genWrite(
		(uint8_t*)buffer,
		bufferSize,
		errorDetectionMode,
		50,
		cmdSize,
		"F4F4F4",
		velocity.c[0],
		velocity.c[1],
		velocity.c[2]);
}

VnError VnUartPacket_genWriteVelocityCompensationControl(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	uint8_t mode,
	float velocityTuning,
	float rateTuning)
{
	return VnUartPacket_genWrite(
		(uint8_t*)buffer,
		bufferSize,
		errorDetectionMode,
		51,
		cmdSize,
		"U1F4F4",
		mode,
		velocityTuning,
		rateTuning);
}

VnError VnUartPacket_genWriteGpsConfiguration(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	uint8_t mode,
	uint8_t ppsSource,
	uint8_t reserved1,
	uint8_t reserved2,
	uint8_t reserved3)
{
	return VnUartPacket_genWrite(
		(uint8_t*)buffer,
		bufferSize,
		errorDetectionMode,
		55,
		cmdSize,
		"U1U1U1U1U1",
		mode,
		ppsSource,
		reserved1,
		reserved2,
		reserved3);
}

VnError VnUartPacket_genWriteGpsAntennaOffset(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	vec3f position)
{
	return VnUartPacket_genWrite(
		(uint8_t*)buffer,
		bufferSize,
		errorDetectionMode,
		57,
		cmdSize,
		"F4F4F4",
		position.c[0],
		position.c[1],
		position.c[2]);
}

VnError VnUartPacket_genWriteInsBasicConfiguration(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	uint8_t scenario,
	uint8_t ahrsAiding,
	uint8_t estBaseline,
	uint8_t resv2)
{
	return VnUartPacket_genWrite(
		(uint8_t*)buffer,
		bufferSize,
		errorDetectionMode,
		67,
		cmdSize,
		"U1U1U1U1",
		scenario,
		ahrsAiding,
		estBaseline,
		resv2);
}

VnError VnUartPacket_genWriteStartupFilterBiasEstimate(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	vec3f gyroBias,
	vec3f accelBias,
	float pressureBias)
{
	return VnUartPacket_genWrite(
		(uint8_t*)buffer,
		bufferSize,
		errorDetectionMode,
		74,
		cmdSize,
		"F4F4F4F4F4F4F4",
		gyroBias.c[0],
		gyroBias.c[1],
		gyroBias.c[2],
		accelBias.c[0],
		accelBias.c[1],
		accelBias.c[2],
		pressureBias);
}

VnError VnUartPacket_genWriteDeltaThetaAndDeltaVelocityConfiguration(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	uint8_t integrationFrame,
	uint8_t gyroCompensation,
	uint8_t accelCompensation,
	uint8_t reserved1,
	uint16_t reserved2)
{
	return VnUartPacket_genWrite(
		(uint8_t*)buffer,
		bufferSize,
		errorDetectionMode,
		82,
		cmdSize,
		"U1U1U1U1U2",
		integrationFrame,
		gyroCompensation,
		accelCompensation,
		reserved1,
		reserved2);
}

VnError VnUartPacket_genWriteReferenceVectorConfiguration(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	uint8_t useMagModel,
	uint8_t useGravityModel,
	uint8_t resv1,
	uint8_t resv2,
	uint32_t recalcThreshold,
	float year,
	vec3d position)
{
	return VnUartPacket_genWrite(
		(uint8_t*)buffer,
		bufferSize,
		errorDetectionMode,
		83,
		cmdSize,
		"U1U1U1U1U4F4F8F8F8",
		useMagModel,
		useGravityModel,
		resv1,
		resv2,
		recalcThreshold,
		year,
		position.c[0],
		position.c[1],
		position.c[2]);
}

VnError VnUartPacket_genWriteGyroCompensation(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	mat3f c,
	vec3f b)
{
	return VnUartPacket_genWrite(
		(uint8_t*)buffer,
		bufferSize,
		errorDetectionMode,
		84,
		cmdSize,
		"F4F4F4F4F4F4F4F4F4F4F4F4",
		c.e[0],
		c.e[3],
		c.e[6],
		c.e[1],
		c.e[4],
		c.e[7],
		c.e[2],
		c.e[5],
		c.e[8],
		b.c[0],
		b.c[1],
		b.c[2]);
}

VnError VnUartPacket_genWriteImuFilteringConfiguration(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
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
	return VnUartPacket_genWrite(
		(uint8_t*)buffer,
		bufferSize,
		errorDetectionMode,
		85,
		cmdSize,
		"U2U2U2U2U2U1U1U1U1U1",
		magWindowSize,
		accelWindowSize,
		gyroWindowSize,
		tempWindowSize,
		presWindowSize,
		magFilterMode,
		accelFilterMode,
		gyroFilterMode,
		tempFilterMode,
		presFilterMode);
}

VnError VnUartPacket_genWriteGpsCompassBaseline(
	char *buffer,
	size_t bufferSize,
	VnErrorDetectionMode errorDetectionMode,
	size_t *cmdSize,
	vec3f position,
	vec3f uncertainty)
{
	return VnUartPacket_genWrite(
		(uint8_t*)buffer,
		bufferSize,
		errorDetectionMode,
		93,
		cmdSize,
		"F4F4F4F4F4F4",
		position.c[0],
		position.c[1],
		position.c[2],
		uncertainty.c[0],
		uncertainty.c[1],
		uncertainty.c[2]);
}

void VnUartPacket_parseError(VnUartPacket *packet, uint8_t *error)
{
	VnUartPacket_parseErrorRaw(packet->data, error);
}

void VnUartPacket_parseErrorRaw(uint8_t *packet, uint8_t *error)
{
	*error = (uint8_t) strtol((char*)(packet + 7),NULL,16);
}

void VnUartPacket_parseBinaryOutput(
	VnUartPacket *packet,
	uint16_t* asyncMode,
	uint16_t* rateDivisor,
	uint16_t* outputGroup,
	uint16_t* commonField,
	uint16_t* timeField,
	uint16_t* imuField,
	uint16_t* gpsField,
	uint16_t* attitudeField,
	uint16_t* insField,
  uint16_t* gps2Field)
{
	VnUartPacket_parseBinaryOutputRaw(
		packet->data,
		asyncMode,
		rateDivisor,
		outputGroup,
		commonField,
		timeField,
		imuField,
		gpsField,
		attitudeField,
		insField,
    gps2Field);
}

void VnUartPacket_parseBinaryOutputRaw(
	uint8_t *packet,
	uint16_t* asyncMode,
	uint16_t* rateDivisor,
	uint16_t* outputGroup,
	uint16_t* commonField,
	uint16_t* timeField,
	uint16_t* imuField,
	uint16_t* gpsField,
	uint16_t* attitudeField,
	uint16_t* insField,
  uint16_t* gps2Field)
{
	size_t packetIndex;

	char *result = (char*)VnUartPacket_startAsciiResponsePacketParse(packet, &packetIndex);

	*commonField = 0;
	*timeField = 0;
	*imuField = 0;
	*gpsField = 0;
	*attitudeField = 0;
	*insField = 0;
  *gps2Field = 0;

	*asyncMode = ATOU16;
	NEXTRAW
	*rateDivisor = ATOU16;
	NEXTRAW
	*outputGroup = ATOU16X;
	if (*outputGroup & 0x0001)
	{
		NEXTRAW
		*commonField = ATOU16X;
	}
	if (*outputGroup & 0x0002)
	{
		NEXTRAW
		*timeField = ATOU16X;
	}
	if (*outputGroup & 0x0004)
	{
		NEXTRAW
		*imuField = ATOU16X;
	}
	if (*outputGroup & 0x0008)
	{
		NEXTRAW
		*gpsField = ATOU16X;
	}
	if (*outputGroup & 0x0010)
	{
		NEXTRAW
		*attitudeField = ATOU16X;
	}
	if (*outputGroup & 0x0020)
	{
		NEXTRAW
		*insField = ATOU16X;
	}
  if (*outputGroup & 0x0040)
  {
    NEXTRAW
    *gps2Field = ATOU16X;
  }
}

void VnUartPacket_parseUserTag(VnUartPacket *packet, char *tag)
{
	VnUartPacket_parseUserTagRaw((char*)packet->data, tag);
}

void VnUartPacket_parseUserTagRaw(char *packet, char* tag)
{
	size_t packetIndex;
	char* next;

	next = (char*)VnUartPacket_startAsciiPacketParse((uint8_t*)packet, &packetIndex);

	if (*(next + strlen(next) + 1) == '*')
	{
		tag[0] = '\0';
		return;
	}

	next = (char*)VnUartPacket_getNextData((uint8_t*)packet, &packetIndex);

	#if defined(_MSC_VER)
		/* Disable warnings regarding using strcpy_s since this
		 * function's signature does not provide us with information
		 * about the length of 'out'. */
		#pragma warning(push)
		#pragma warning(disable:4996)
	#endif
	strcpy(tag, next);

	#if defined(_MSC_VER)
		#pragma warning(pop)
	#endif
}

void VnUartPacket_parseModelNumber(VnUartPacket *packet, char *productName)
{
	VnUartPacket_parseModelNumberRaw((char*)packet->data, productName);
}

void VnUartPacket_parseModelNumberRaw(char *packet, char* productName)
{
	size_t packetIndex;
	char* next;

	next = (char*)VnUartPacket_startAsciiPacketParse((uint8_t*)packet, &packetIndex);

	if (*(next + strlen(next) + 1) == '*')
	{
		productName[0] = '\0';
		return;
	}

	next = (char*)VnUartPacket_getNextData((uint8_t*)packet, &packetIndex);

	#if defined(_MSC_VER)
		/* Disable warnings regarding using strcpy_s since this
		 * function's signature does not provide us with information
		 * about the length of 'out'. */
		#pragma warning(push)
		#pragma warning(disable:4996)
	#endif
	strcpy(productName, next);

	#if defined(_MSC_VER)
		#pragma warning(pop)
	#endif
}

void VnUartPacket_parseHardwareRevision(VnUartPacket *packet, uint32_t* revision)
{
	VnUartPacket_parseHardwareRevisionRaw((char*)packet->data, revision);
}

void VnUartPacket_parseHardwareRevisionRaw(char *packet, uint32_t* revision)
{
	size_t packetIndex;
	char *result = (char*)VnUartPacket_startAsciiResponsePacketParse((uint8_t*)packet, &packetIndex);

	*revision = ATOU32;
}

void VnUartPacket_parseSerialNumber(VnUartPacket *packet, uint32_t* serialNum)
{
	VnUartPacket_parseSerialNumberRaw((char*)packet->data, serialNum);
}

void VnUartPacket_parseSerialNumberRaw(char *packet, uint32_t* serialNum)
{
	size_t packetIndex;
	char *result = (char*)VnUartPacket_startAsciiResponsePacketParse((uint8_t*)packet, &packetIndex);

	*serialNum = ATOU32;
}

void VnUartPacket_parseFirmwareVersion(VnUartPacket *packet, char *firmwareVersion)
{
	VnUartPacket_parseFirmwareVersionRaw((char*)packet->data, firmwareVersion);
}

void VnUartPacket_parseFirmwareVersionRaw(char *packet, char* firmwareVersion)
{
	size_t packetIndex;
	char* next;

	next = (char*)VnUartPacket_startAsciiPacketParse((uint8_t*)packet, &packetIndex);

	if (*(next + strlen(next) + 1) == '*')
	{
		firmwareVersion[0] = '\0';
		return;
	}

	next = (char*)VnUartPacket_getNextData((uint8_t*)packet, &packetIndex);

	#if defined(_MSC_VER)
		/* Disable warnings regarding using strcpy_s since this
		 * function's signature does not provide us with information
		 * about the length of 'out'. */
		#pragma warning(push)
		#pragma warning(disable:4996)
	#endif
	strcpy(firmwareVersion, next);

	#if defined(_MSC_VER)
		#pragma warning(pop)
	#endif
}

void VnUartPacket_parseSerialBaudRate(VnUartPacket *packet, uint32_t* baudrate)
{
	VnUartPacket_parseSerialBaudRateRaw((char*)packet->data, baudrate);
}

void VnUartPacket_parseSerialBaudRateRaw(char *packet, uint32_t* baudrate)
{
	size_t packetIndex;
	char *result = (char*)VnUartPacket_startAsciiResponsePacketParse((uint8_t*)packet, &packetIndex);

	*baudrate = ATOU32;
}

void VnUartPacket_parseAsyncDataOutputType(VnUartPacket *packet, uint32_t* ador)
{
	VnUartPacket_parseAsyncDataOutputTypeRaw((char*)packet->data, ador);
}

void VnUartPacket_parseAsyncDataOutputTypeRaw(char *packet, uint32_t* ador)
{
	size_t packetIndex;
	char *result = (char*)VnUartPacket_startAsciiResponsePacketParse((uint8_t*)packet, &packetIndex);

	*ador = ATOU32;
}

void VnUartPacket_parseAsyncDataOutputFrequency(VnUartPacket *packet, uint32_t* adof)
{
	VnUartPacket_parseAsyncDataOutputFrequencyRaw((char*)packet->data, adof);
}

void VnUartPacket_parseAsyncDataOutputFrequencyRaw(char *packet, uint32_t* adof)
{
	size_t packetIndex;
	char *result = (char*)VnUartPacket_startAsciiResponsePacketParse((uint8_t*)packet, &packetIndex);

	*adof = ATOU32;
}

void VnUartPacket_parseYawPitchRoll(VnUartPacket *packet, vec3f* yawPitchRoll)
{
	VnUartPacket_parseYawPitchRollRaw((char*)packet->data, yawPitchRoll);
}

void VnUartPacket_parseYawPitchRollRaw(char *packet, vec3f* yawPitchRoll)
{
	size_t packetIndex;
	char *result = (char*)VnUartPacket_startAsciiResponsePacketParse((uint8_t*)packet, &packetIndex);

	yawPitchRoll->c[0] = ATOFF; NEXTRAW
	yawPitchRoll->c[1] = ATOFF; NEXTRAW
	yawPitchRoll->c[2] = ATOFF;
}

void VnUartPacket_parseAttitudeQuaternion(VnUartPacket *packet, vec4f* quat)
{
	VnUartPacket_parseAttitudeQuaternionRaw((char*)packet->data, quat);
}

void VnUartPacket_parseAttitudeQuaternionRaw(char *packet, vec4f* quat)
{
	size_t packetIndex;
	char *result = (char*)VnUartPacket_startAsciiResponsePacketParse((uint8_t*)packet, &packetIndex);

	quat->c[0] = ATOFF; NEXTRAW
	quat->c[1] = ATOFF; NEXTRAW
	quat->c[2] = ATOFF; NEXTRAW
	quat->c[3] = ATOFF;
}

void VnUartPacket_parseQuaternionMagneticAccelerationAndAngularRates(VnUartPacket *packet, vec4f* quat, vec3f* mag, vec3f* accel, vec3f* gyro)
{
	VnUartPacket_parseQuaternionMagneticAccelerationAndAngularRatesRaw((char*)packet->data, quat, mag, accel, gyro);
}

void VnUartPacket_parseQuaternionMagneticAccelerationAndAngularRatesRaw(char *packet, vec4f* quat, vec3f* mag, vec3f* accel, vec3f* gyro)
{
	size_t packetIndex;
	char *result = (char*)VnUartPacket_startAsciiResponsePacketParse((uint8_t*)packet, &packetIndex);

	quat->c[0] = ATOFF; NEXTRAW
	quat->c[1] = ATOFF; NEXTRAW
	quat->c[2] = ATOFF; NEXTRAW
	quat->c[3] = ATOFF; NEXTRAW
	mag->c[0] = ATOFF; NEXTRAW
	mag->c[1] = ATOFF; NEXTRAW
	mag->c[2] = ATOFF; NEXTRAW
	accel->c[0] = ATOFF; NEXTRAW
	accel->c[1] = ATOFF; NEXTRAW
	accel->c[2] = ATOFF; NEXTRAW
	gyro->c[0] = ATOFF; NEXTRAW
	gyro->c[1] = ATOFF; NEXTRAW
	gyro->c[2] = ATOFF;
}

void VnUartPacket_parseMagneticMeasurements(VnUartPacket *packet, vec3f* mag)
{
	VnUartPacket_parseMagneticMeasurementsRaw((char*)packet->data, mag);
}

void VnUartPacket_parseMagneticMeasurementsRaw(char *packet, vec3f* mag)
{
	size_t packetIndex;
	char *result = (char*)VnUartPacket_startAsciiResponsePacketParse((uint8_t*)packet, &packetIndex);

	mag->c[0] = ATOFF; NEXTRAW
	mag->c[1] = ATOFF; NEXTRAW
	mag->c[2] = ATOFF;
}

void VnUartPacket_parseAccelerationMeasurements(VnUartPacket *packet, vec3f* accel)
{
	VnUartPacket_parseAccelerationMeasurementsRaw((char*)packet->data, accel);
}

void VnUartPacket_parseAccelerationMeasurementsRaw(char *packet, vec3f* accel)
{
	size_t packetIndex;
	char *result = (char*)VnUartPacket_startAsciiResponsePacketParse((uint8_t*)packet, &packetIndex);

	accel->c[0] = ATOFF; NEXTRAW
	accel->c[1] = ATOFF; NEXTRAW
	accel->c[2] = ATOFF;
}

void VnUartPacket_parseAngularRateMeasurements(VnUartPacket *packet, vec3f* gyro)
{
	VnUartPacket_parseAngularRateMeasurementsRaw((char*)packet->data, gyro);
}

void VnUartPacket_parseAngularRateMeasurementsRaw(char *packet, vec3f* gyro)
{
	size_t packetIndex;
	char *result = (char*)VnUartPacket_startAsciiResponsePacketParse((uint8_t*)packet, &packetIndex);

	gyro->c[0] = ATOFF; NEXTRAW
	gyro->c[1] = ATOFF; NEXTRAW
	gyro->c[2] = ATOFF;
}

void VnUartPacket_parseMagneticAccelerationAndAngularRates(VnUartPacket *packet, vec3f* mag, vec3f* accel, vec3f* gyro)
{
	VnUartPacket_parseMagneticAccelerationAndAngularRatesRaw((char*)packet->data, mag, accel, gyro);
}

void VnUartPacket_parseMagneticAccelerationAndAngularRatesRaw(char *packet, vec3f* mag, vec3f* accel, vec3f* gyro)
{
	size_t packetIndex;
	char *result = (char*)VnUartPacket_startAsciiResponsePacketParse((uint8_t*)packet, &packetIndex);

	mag->c[0] = ATOFF; NEXTRAW
	mag->c[1] = ATOFF; NEXTRAW
	mag->c[2] = ATOFF; NEXTRAW
	accel->c[0] = ATOFF; NEXTRAW
	accel->c[1] = ATOFF; NEXTRAW
	accel->c[2] = ATOFF; NEXTRAW
	gyro->c[0] = ATOFF; NEXTRAW
	gyro->c[1] = ATOFF; NEXTRAW
	gyro->c[2] = ATOFF;
}

void VnUartPacket_parseMagneticAndGravityReferenceVectors(VnUartPacket *packet, vec3f* magRef, vec3f* accRef)
{
	VnUartPacket_parseMagneticAndGravityReferenceVectorsRaw((char*)packet->data, magRef, accRef);
}

void VnUartPacket_parseMagneticAndGravityReferenceVectorsRaw(char *packet, vec3f* magRef, vec3f* accRef)
{
	size_t packetIndex;
	char *result = (char*)VnUartPacket_startAsciiResponsePacketParse((uint8_t*)packet, &packetIndex);

	magRef->c[0] = ATOFF; NEXTRAW
	magRef->c[1] = ATOFF; NEXTRAW
	magRef->c[2] = ATOFF; NEXTRAW
	accRef->c[0] = ATOFF; NEXTRAW
	accRef->c[1] = ATOFF; NEXTRAW
	accRef->c[2] = ATOFF;
}

void VnUartPacket_parseFilterMeasurementsVarianceParameters(VnUartPacket *packet, float* angularWalkVariance, vec3f* angularRateVariance, vec3f* magneticVariance, vec3f* accelerationVariance)
{
	VnUartPacket_parseFilterMeasurementsVarianceParametersRaw((char*)packet->data, angularWalkVariance, angularRateVariance, magneticVariance, accelerationVariance);
}

void VnUartPacket_parseFilterMeasurementsVarianceParametersRaw(char *packet, float* angularWalkVariance, vec3f* angularRateVariance, vec3f* magneticVariance, vec3f* accelerationVariance)
{
	size_t packetIndex;
	char *result = (char*)VnUartPacket_startAsciiResponsePacketParse((uint8_t*)packet, &packetIndex);

	*angularWalkVariance = ATOFF; NEXTRAW
	angularRateVariance->c[0] = ATOFF; NEXTRAW
	angularRateVariance->c[1] = ATOFF; NEXTRAW
	angularRateVariance->c[2] = ATOFF; NEXTRAW
	magneticVariance->c[0] = ATOFF; NEXTRAW
	magneticVariance->c[1] = ATOFF; NEXTRAW
	magneticVariance->c[2] = ATOFF; NEXTRAW
	accelerationVariance->c[0] = ATOFF; NEXTRAW
	accelerationVariance->c[1] = ATOFF; NEXTRAW
	accelerationVariance->c[2] = ATOFF;
}

void VnUartPacket_parseMagnetometerCompensation(VnUartPacket *packet, mat3f* c, vec3f* b)
{
	VnUartPacket_parseMagnetometerCompensationRaw((char*)packet->data, c, b);
}

void VnUartPacket_parseMagnetometerCompensationRaw(char *packet, mat3f* c, vec3f* b)
{
	size_t packetIndex;
	char *result = (char*)VnUartPacket_startAsciiResponsePacketParse((uint8_t*)packet, &packetIndex);

	c->e[0] = ATOFF; NEXTRAW
	c->e[3] = ATOFF; NEXTRAW
	c->e[6] = ATOFF; NEXTRAW
	c->e[1] = ATOFF; NEXTRAW
	c->e[4] = ATOFF; NEXTRAW
	c->e[7] = ATOFF; NEXTRAW
	c->e[2] = ATOFF; NEXTRAW
	c->e[5] = ATOFF; NEXTRAW
	c->e[8] = ATOFF; NEXTRAW
	b->c[0] = ATOFF; NEXTRAW
	b->c[1] = ATOFF; NEXTRAW
	b->c[2] = ATOFF;
}

void VnUartPacket_parseFilterActiveTuningParameters(VnUartPacket *packet, float* magneticDisturbanceGain, float* accelerationDisturbanceGain, float* magneticDisturbanceMemory, float* accelerationDisturbanceMemory)
{
	VnUartPacket_parseFilterActiveTuningParametersRaw((char*)packet->data, magneticDisturbanceGain, accelerationDisturbanceGain, magneticDisturbanceMemory, accelerationDisturbanceMemory);
}

void VnUartPacket_parseFilterActiveTuningParametersRaw(char *packet, float* magneticDisturbanceGain, float* accelerationDisturbanceGain, float* magneticDisturbanceMemory, float* accelerationDisturbanceMemory)
{
	size_t packetIndex;
	char *result = (char*)VnUartPacket_startAsciiResponsePacketParse((uint8_t*)packet, &packetIndex);

	*magneticDisturbanceGain = ATOFF; NEXTRAW
	*accelerationDisturbanceGain = ATOFF; NEXTRAW
	*magneticDisturbanceMemory = ATOFF; NEXTRAW
	*accelerationDisturbanceMemory = ATOFF;
}

void VnUartPacket_parseAccelerationCompensation(VnUartPacket *packet, mat3f* c, vec3f* b)
{
	VnUartPacket_parseAccelerationCompensationRaw((char*)packet->data, c, b);
}

void VnUartPacket_parseAccelerationCompensationRaw(char *packet, mat3f* c, vec3f* b)
{
	size_t packetIndex;
	char *result = (char*)VnUartPacket_startAsciiResponsePacketParse((uint8_t*)packet, &packetIndex);

	c->e[0] = ATOFF; NEXTRAW
	c->e[3] = ATOFF; NEXTRAW
	c->e[6] = ATOFF; NEXTRAW
	c->e[1] = ATOFF; NEXTRAW
	c->e[4] = ATOFF; NEXTRAW
	c->e[7] = ATOFF; NEXTRAW
	c->e[2] = ATOFF; NEXTRAW
	c->e[5] = ATOFF; NEXTRAW
	c->e[8] = ATOFF; NEXTRAW
	b->c[0] = ATOFF; NEXTRAW
	b->c[1] = ATOFF; NEXTRAW
	b->c[2] = ATOFF;
}

void VnUartPacket_parseReferenceFrameRotation(VnUartPacket *packet, mat3f* c)
{
	VnUartPacket_parseReferenceFrameRotationRaw((char*)packet->data, c);
}

void VnUartPacket_parseReferenceFrameRotationRaw(char *packet, mat3f* c)
{
	size_t packetIndex;
	char *result = (char*)VnUartPacket_startAsciiResponsePacketParse((uint8_t*)packet, &packetIndex);

	c->e[0] = ATOFF; NEXTRAW
	c->e[3] = ATOFF; NEXTRAW
	c->e[6] = ATOFF; NEXTRAW
	c->e[1] = ATOFF; NEXTRAW
	c->e[4] = ATOFF; NEXTRAW
	c->e[7] = ATOFF; NEXTRAW
	c->e[2] = ATOFF; NEXTRAW
	c->e[5] = ATOFF; NEXTRAW
	c->e[8] = ATOFF;
}

void VnUartPacket_parseYawPitchRollMagneticAccelerationAndAngularRates(VnUartPacket *packet, vec3f* yawPitchRoll, vec3f* mag, vec3f* accel, vec3f* gyro)
{
	VnUartPacket_parseYawPitchRollMagneticAccelerationAndAngularRatesRaw((char*)packet->data, yawPitchRoll, mag, accel, gyro);
}

void VnUartPacket_parseYawPitchRollMagneticAccelerationAndAngularRatesRaw(char *packet, vec3f* yawPitchRoll, vec3f* mag, vec3f* accel, vec3f* gyro)
{
	size_t packetIndex;
	char *result = (char*)VnUartPacket_startAsciiResponsePacketParse((uint8_t*)packet, &packetIndex);

	yawPitchRoll->c[0] = ATOFF; NEXTRAW
	yawPitchRoll->c[1] = ATOFF; NEXTRAW
	yawPitchRoll->c[2] = ATOFF; NEXTRAW
	mag->c[0] = ATOFF; NEXTRAW
	mag->c[1] = ATOFF; NEXTRAW
	mag->c[2] = ATOFF; NEXTRAW
	accel->c[0] = ATOFF; NEXTRAW
	accel->c[1] = ATOFF; NEXTRAW
	accel->c[2] = ATOFF; NEXTRAW
	gyro->c[0] = ATOFF; NEXTRAW
	gyro->c[1] = ATOFF; NEXTRAW
	gyro->c[2] = ATOFF;
}

void VnUartPacket_parseCommunicationProtocolControl(VnUartPacket *packet, uint8_t* serialCount, uint8_t* serialStatus, uint8_t* spiCount, uint8_t* spiStatus, uint8_t* serialChecksum, uint8_t* spiChecksum, uint8_t* errorMode)
{
	VnUartPacket_parseCommunicationProtocolControlRaw((char*)packet->data, serialCount, serialStatus, spiCount, spiStatus, serialChecksum, spiChecksum, errorMode);
}

void VnUartPacket_parseCommunicationProtocolControlRaw(char *packet, uint8_t* serialCount, uint8_t* serialStatus, uint8_t* spiCount, uint8_t* spiStatus, uint8_t* serialChecksum, uint8_t* spiChecksum, uint8_t* errorMode)
{
	size_t packetIndex;
	char *result = (char*)VnUartPacket_startAsciiResponsePacketParse((uint8_t*)packet, &packetIndex);

	*serialCount = ATOU8; NEXTRAW
	*serialStatus = ATOU8; NEXTRAW
	*spiCount = ATOU8; NEXTRAW
	*spiStatus = ATOU8; NEXTRAW
	*serialChecksum = ATOU8; NEXTRAW
	*spiChecksum = ATOU8; NEXTRAW
	*errorMode = ATOU8;
}

void VnUartPacket_parseSynchronizationControl(VnUartPacket *packet, uint8_t* syncInMode, uint8_t* syncInEdge, uint16_t* syncInSkipFactor, uint32_t* reserved1, uint8_t* syncOutMode, uint8_t* syncOutPolarity, uint16_t* syncOutSkipFactor, uint32_t* syncOutPulseWidth, uint32_t* reserved2)
{
	VnUartPacket_parseSynchronizationControlRaw((char*)packet->data, syncInMode, syncInEdge, syncInSkipFactor, reserved1, syncOutMode, syncOutPolarity, syncOutSkipFactor, syncOutPulseWidth, reserved2);
}

void VnUartPacket_parseSynchronizationControlRaw(char *packet, uint8_t* syncInMode, uint8_t* syncInEdge, uint16_t* syncInSkipFactor, uint32_t* reserved1, uint8_t* syncOutMode, uint8_t* syncOutPolarity, uint16_t* syncOutSkipFactor, uint32_t* syncOutPulseWidth, uint32_t* reserved2)
{
	size_t packetIndex;
	char *result = (char*)VnUartPacket_startAsciiResponsePacketParse((uint8_t*)packet, &packetIndex);

	*syncInMode = ATOU8; NEXTRAW
	*syncInEdge = ATOU8; NEXTRAW
	*syncInSkipFactor = ATOU16; NEXTRAW
	*reserved1 = ATOU32; NEXTRAW
	*syncOutMode = ATOU8; NEXTRAW
	*syncOutPolarity = ATOU8; NEXTRAW
	*syncOutSkipFactor = ATOU16; NEXTRAW
	*syncOutPulseWidth = ATOU32; NEXTRAW
	*reserved2 = ATOU32;
}

void VnUartPacket_parseSynchronizationStatus(VnUartPacket *packet, uint32_t* syncInCount, uint32_t* syncInTime, uint32_t* syncOutCount)
{
	VnUartPacket_parseSynchronizationStatusRaw((char*)packet->data, syncInCount, syncInTime, syncOutCount);
}

void VnUartPacket_parseSynchronizationStatusRaw(char *packet, uint32_t* syncInCount, uint32_t* syncInTime, uint32_t* syncOutCount)
{
	size_t packetIndex;
	char *result = (char*)VnUartPacket_startAsciiResponsePacketParse((uint8_t*)packet, &packetIndex);

	*syncInCount = ATOU32; NEXTRAW
	*syncInTime = ATOU32; NEXTRAW
	*syncOutCount = ATOU32;
}

void VnUartPacket_parseFilterBasicControl(VnUartPacket *packet, uint8_t* magMode, uint8_t* extMagMode, uint8_t* extAccMode, uint8_t* extGyroMode, vec3f* gyroLimit)
{
	VnUartPacket_parseFilterBasicControlRaw((char*)packet->data, magMode, extMagMode, extAccMode, extGyroMode, gyroLimit);
}

void VnUartPacket_parseFilterBasicControlRaw(char *packet, uint8_t* magMode, uint8_t* extMagMode, uint8_t* extAccMode, uint8_t* extGyroMode, vec3f* gyroLimit)
{
	size_t packetIndex;
	char *result = (char*)VnUartPacket_startAsciiResponsePacketParse((uint8_t*)packet, &packetIndex);

	*magMode = ATOU8; NEXTRAW
	*extMagMode = ATOU8; NEXTRAW
	*extAccMode = ATOU8; NEXTRAW
	*extGyroMode = ATOU8; NEXTRAW
	gyroLimit->c[0] = ATOFF; NEXTRAW
	gyroLimit->c[1] = ATOFF; NEXTRAW
	gyroLimit->c[2] = ATOFF;
}

void VnUartPacket_parseVpeBasicControl(VnUartPacket *packet, uint8_t* enable, uint8_t* headingMode, uint8_t* filteringMode, uint8_t* tuningMode)
{
	VnUartPacket_parseVpeBasicControlRaw((char*)packet->data, enable, headingMode, filteringMode, tuningMode);
}

void VnUartPacket_parseVpeBasicControlRaw(char *packet, uint8_t* enable, uint8_t* headingMode, uint8_t* filteringMode, uint8_t* tuningMode)
{
	size_t packetIndex;
	char *result = (char*)VnUartPacket_startAsciiResponsePacketParse((uint8_t*)packet, &packetIndex);

	*enable = ATOU8; NEXTRAW
	*headingMode = ATOU8; NEXTRAW
	*filteringMode = ATOU8; NEXTRAW
	*tuningMode = ATOU8;
}

void VnUartPacket_parseVpeMagnetometerBasicTuning(VnUartPacket *packet, vec3f* baseTuning, vec3f* adaptiveTuning, vec3f* adaptiveFiltering)
{
	VnUartPacket_parseVpeMagnetometerBasicTuningRaw((char*)packet->data, baseTuning, adaptiveTuning, adaptiveFiltering);
}

void VnUartPacket_parseVpeMagnetometerBasicTuningRaw(char *packet, vec3f* baseTuning, vec3f* adaptiveTuning, vec3f* adaptiveFiltering)
{
	size_t packetIndex;
	char *result = (char*)VnUartPacket_startAsciiResponsePacketParse((uint8_t*)packet, &packetIndex);

	baseTuning->c[0] = ATOFF; NEXTRAW
	baseTuning->c[1] = ATOFF; NEXTRAW
	baseTuning->c[2] = ATOFF; NEXTRAW
	adaptiveTuning->c[0] = ATOFF; NEXTRAW
	adaptiveTuning->c[1] = ATOFF; NEXTRAW
	adaptiveTuning->c[2] = ATOFF; NEXTRAW
	adaptiveFiltering->c[0] = ATOFF; NEXTRAW
	adaptiveFiltering->c[1] = ATOFF; NEXTRAW
	adaptiveFiltering->c[2] = ATOFF;
}

void VnUartPacket_parseVpeMagnetometerAdvancedTuning(VnUartPacket *packet, vec3f* minFiltering, vec3f* maxFiltering, float* maxAdaptRate, float* disturbanceWindow, float* maxTuning)
{
	VnUartPacket_parseVpeMagnetometerAdvancedTuningRaw((char*)packet->data, minFiltering, maxFiltering, maxAdaptRate, disturbanceWindow, maxTuning);
}

void VnUartPacket_parseVpeMagnetometerAdvancedTuningRaw(char *packet, vec3f* minFiltering, vec3f* maxFiltering, float* maxAdaptRate, float* disturbanceWindow, float* maxTuning)
{
	size_t packetIndex;
	char *result = (char*)VnUartPacket_startAsciiResponsePacketParse((uint8_t*)packet, &packetIndex);

	minFiltering->c[0] = ATOFF; NEXTRAW
	minFiltering->c[1] = ATOFF; NEXTRAW
	minFiltering->c[2] = ATOFF; NEXTRAW
	maxFiltering->c[0] = ATOFF; NEXTRAW
	maxFiltering->c[1] = ATOFF; NEXTRAW
	maxFiltering->c[2] = ATOFF; NEXTRAW
	*maxAdaptRate = ATOFF; NEXTRAW
	*disturbanceWindow = ATOFF; NEXTRAW
	*maxTuning = ATOFF;
}

void VnUartPacket_parseVpeAccelerometerBasicTuning(VnUartPacket *packet, vec3f* baseTuning, vec3f* adaptiveTuning, vec3f* adaptiveFiltering)
{
	VnUartPacket_parseVpeAccelerometerBasicTuningRaw((char*)packet->data, baseTuning, adaptiveTuning, adaptiveFiltering);
}

void VnUartPacket_parseVpeAccelerometerBasicTuningRaw(char *packet, vec3f* baseTuning, vec3f* adaptiveTuning, vec3f* adaptiveFiltering)
{
	size_t packetIndex;
	char *result = (char*)VnUartPacket_startAsciiResponsePacketParse((uint8_t*)packet, &packetIndex);

	baseTuning->c[0] = ATOFF; NEXTRAW
	baseTuning->c[1] = ATOFF; NEXTRAW
	baseTuning->c[2] = ATOFF; NEXTRAW
	adaptiveTuning->c[0] = ATOFF; NEXTRAW
	adaptiveTuning->c[1] = ATOFF; NEXTRAW
	adaptiveTuning->c[2] = ATOFF; NEXTRAW
	adaptiveFiltering->c[0] = ATOFF; NEXTRAW
	adaptiveFiltering->c[1] = ATOFF; NEXTRAW
	adaptiveFiltering->c[2] = ATOFF;
}

void VnUartPacket_parseVpeAccelerometerAdvancedTuning(VnUartPacket *packet, vec3f* minFiltering, vec3f* maxFiltering, float* maxAdaptRate, float* disturbanceWindow, float* maxTuning)
{
	VnUartPacket_parseVpeAccelerometerAdvancedTuningRaw((char*)packet->data, minFiltering, maxFiltering, maxAdaptRate, disturbanceWindow, maxTuning);
}

void VnUartPacket_parseVpeAccelerometerAdvancedTuningRaw(char *packet, vec3f* minFiltering, vec3f* maxFiltering, float* maxAdaptRate, float* disturbanceWindow, float* maxTuning)
{
	size_t packetIndex;
	char *result = (char*)VnUartPacket_startAsciiResponsePacketParse((uint8_t*)packet, &packetIndex);

	minFiltering->c[0] = ATOFF; NEXTRAW
	minFiltering->c[1] = ATOFF; NEXTRAW
	minFiltering->c[2] = ATOFF; NEXTRAW
	maxFiltering->c[0] = ATOFF; NEXTRAW
	maxFiltering->c[1] = ATOFF; NEXTRAW
	maxFiltering->c[2] = ATOFF; NEXTRAW
	*maxAdaptRate = ATOFF; NEXTRAW
	*disturbanceWindow = ATOFF; NEXTRAW
	*maxTuning = ATOFF;
}

void VnUartPacket_parseVpeGyroBasicTuning(VnUartPacket *packet, vec3f* angularWalkVariance, vec3f* baseTuning, vec3f* adaptiveTuning)
{
	VnUartPacket_parseVpeGyroBasicTuningRaw((char*)packet->data, angularWalkVariance, baseTuning, adaptiveTuning);
}

void VnUartPacket_parseVpeGyroBasicTuningRaw(char *packet, vec3f* angularWalkVariance, vec3f* baseTuning, vec3f* adaptiveTuning)
{
	size_t packetIndex;
	char *result = (char*)VnUartPacket_startAsciiResponsePacketParse((uint8_t*)packet, &packetIndex);

	angularWalkVariance->c[0] = ATOFF; NEXTRAW
	angularWalkVariance->c[1] = ATOFF; NEXTRAW
	angularWalkVariance->c[2] = ATOFF; NEXTRAW
	baseTuning->c[0] = ATOFF; NEXTRAW
	baseTuning->c[1] = ATOFF; NEXTRAW
	baseTuning->c[2] = ATOFF; NEXTRAW
	adaptiveTuning->c[0] = ATOFF; NEXTRAW
	adaptiveTuning->c[1] = ATOFF; NEXTRAW
	adaptiveTuning->c[2] = ATOFF;
}

void VnUartPacket_parseFilterStartupGyroBias(VnUartPacket *packet, vec3f* bias)
{
	VnUartPacket_parseFilterStartupGyroBiasRaw((char*)packet->data, bias);
}

void VnUartPacket_parseFilterStartupGyroBiasRaw(char *packet, vec3f* bias)
{
	size_t packetIndex;
	char *result = (char*)VnUartPacket_startAsciiResponsePacketParse((uint8_t*)packet, &packetIndex);

	bias->c[0] = ATOFF; NEXTRAW
	bias->c[1] = ATOFF; NEXTRAW
	bias->c[2] = ATOFF;
}

void VnUartPacket_parseMagnetometerCalibrationControl(VnUartPacket *packet, uint8_t* hsiMode, uint8_t* hsiOutput, uint8_t* convergeRate)
{
	VnUartPacket_parseMagnetometerCalibrationControlRaw((char*)packet->data, hsiMode, hsiOutput, convergeRate);
}

void VnUartPacket_parseMagnetometerCalibrationControlRaw(char *packet, uint8_t* hsiMode, uint8_t* hsiOutput, uint8_t* convergeRate)
{
	size_t packetIndex;
	char *result = (char*)VnUartPacket_startAsciiResponsePacketParse((uint8_t*)packet, &packetIndex);

	*hsiMode = ATOU8; NEXTRAW
	*hsiOutput = ATOU8; NEXTRAW
	*convergeRate = ATOU8;
}

void VnUartPacket_parseCalculatedMagnetometerCalibration(VnUartPacket *packet, mat3f* c, vec3f* b)
{
	VnUartPacket_parseCalculatedMagnetometerCalibrationRaw((char*)packet->data, c, b);
}

void VnUartPacket_parseCalculatedMagnetometerCalibrationRaw(char *packet, mat3f* c, vec3f* b)
{
	size_t packetIndex;
	char *result = (char*)VnUartPacket_startAsciiResponsePacketParse((uint8_t*)packet, &packetIndex);

	c->e[0] = ATOFF; NEXTRAW
	c->e[3] = ATOFF; NEXTRAW
	c->e[6] = ATOFF; NEXTRAW
	c->e[1] = ATOFF; NEXTRAW
	c->e[4] = ATOFF; NEXTRAW
	c->e[7] = ATOFF; NEXTRAW
	c->e[2] = ATOFF; NEXTRAW
	c->e[5] = ATOFF; NEXTRAW
	c->e[8] = ATOFF; NEXTRAW
	b->c[0] = ATOFF; NEXTRAW
	b->c[1] = ATOFF; NEXTRAW
	b->c[2] = ATOFF;
}

void VnUartPacket_parseIndoorHeadingModeControl(VnUartPacket *packet, float* maxRateError, uint8_t* reserved1)
{
	VnUartPacket_parseIndoorHeadingModeControlRaw((char*)packet->data, maxRateError, reserved1);
}

void VnUartPacket_parseIndoorHeadingModeControlRaw(char *packet, float* maxRateError, uint8_t* reserved1)
{
	size_t packetIndex;
	char *result = (char*)VnUartPacket_startAsciiResponsePacketParse((uint8_t*)packet, &packetIndex);

	*maxRateError = ATOFF; NEXTRAW
	*reserved1 = ATOU8;
}

void VnUartPacket_parseVelocityCompensationMeasurement(VnUartPacket *packet, vec3f* velocity)
{
	VnUartPacket_parseVelocityCompensationMeasurementRaw((char*)packet->data, velocity);
}

void VnUartPacket_parseVelocityCompensationMeasurementRaw(char *packet, vec3f* velocity)
{
	size_t packetIndex;
	char *result = (char*)VnUartPacket_startAsciiResponsePacketParse((uint8_t*)packet, &packetIndex);

	velocity->c[0] = ATOFF; NEXTRAW
	velocity->c[1] = ATOFF; NEXTRAW
	velocity->c[2] = ATOFF;
}

void VnUartPacket_parseVelocityCompensationControl(VnUartPacket *packet, uint8_t* mode, float* velocityTuning, float* rateTuning)
{
	VnUartPacket_parseVelocityCompensationControlRaw((char*)packet->data, mode, velocityTuning, rateTuning);
}

void VnUartPacket_parseVelocityCompensationControlRaw(char *packet, uint8_t* mode, float* velocityTuning, float* rateTuning)
{
	size_t packetIndex;
	char *result = (char*)VnUartPacket_startAsciiResponsePacketParse((uint8_t*)packet, &packetIndex);

	*mode = ATOU8; NEXTRAW
	*velocityTuning = ATOFF; NEXTRAW
	*rateTuning = ATOFF;
}

void VnUartPacket_parseVelocityCompensationStatus(VnUartPacket *packet, float* x, float* xDot, vec3f* accelOffset, vec3f* omega)
{
	VnUartPacket_parseVelocityCompensationStatusRaw((char*)packet->data, x, xDot, accelOffset, omega);
}

void VnUartPacket_parseVelocityCompensationStatusRaw(char *packet, float* x, float* xDot, vec3f* accelOffset, vec3f* omega)
{
	size_t packetIndex;
	char *result = (char*)VnUartPacket_startAsciiResponsePacketParse((uint8_t*)packet, &packetIndex);

	*x = ATOFF; NEXTRAW
	*xDot = ATOFF; NEXTRAW
	accelOffset->c[0] = ATOFF; NEXTRAW
	accelOffset->c[1] = ATOFF; NEXTRAW
	accelOffset->c[2] = ATOFF; NEXTRAW
	omega->c[0] = ATOFF; NEXTRAW
	omega->c[1] = ATOFF; NEXTRAW
	omega->c[2] = ATOFF;
}

void VnUartPacket_parseImuMeasurements(VnUartPacket *packet, vec3f* mag, vec3f* accel, vec3f* gyro, float* temp, float* pressure)
{
	VnUartPacket_parseImuMeasurementsRaw((char*)packet->data, mag, accel, gyro, temp, pressure);
}

void VnUartPacket_parseImuMeasurementsRaw(char *packet, vec3f* mag, vec3f* accel, vec3f* gyro, float* temp, float* pressure)
{
	size_t packetIndex;
	char *result = (char*)VnUartPacket_startAsciiResponsePacketParse((uint8_t*)packet, &packetIndex);

	mag->c[0] = ATOFF; NEXTRAW
	mag->c[1] = ATOFF; NEXTRAW
	mag->c[2] = ATOFF; NEXTRAW
	accel->c[0] = ATOFF; NEXTRAW
	accel->c[1] = ATOFF; NEXTRAW
	accel->c[2] = ATOFF; NEXTRAW
	gyro->c[0] = ATOFF; NEXTRAW
	gyro->c[1] = ATOFF; NEXTRAW
	gyro->c[2] = ATOFF; NEXTRAW
	*temp = ATOFF; NEXTRAW
	*pressure = ATOFF;
}

void VnUartPacket_parseGpsConfiguration(VnUartPacket *packet, uint8_t* mode, uint8_t* ppsSource, uint8_t* reserved1, uint8_t* reserved2, uint8_t* reserved3)
{
	VnUartPacket_parseGpsConfigurationRaw((char*)packet->data, mode, ppsSource, reserved1, reserved2, reserved3);
}

void VnUartPacket_parseGpsConfigurationRaw(char *packet, uint8_t* mode, uint8_t* ppsSource, uint8_t* reserved1, uint8_t* reserved2, uint8_t* reserved3)
{
	size_t packetIndex;
	char *result = (char*)VnUartPacket_startAsciiResponsePacketParse((uint8_t*)packet, &packetIndex);

	*mode = ATOU8; NEXTRAW
	*ppsSource = ATOU8; NEXTRAW
	*reserved1 = ATOU8; NEXTRAW
	*reserved2 = ATOU8; NEXTRAW
	*reserved3 = ATOU8;
}

void VnUartPacket_parseGpsAntennaOffset(VnUartPacket *packet, vec3f* position)
{
	VnUartPacket_parseGpsAntennaOffsetRaw((char*)packet->data, position);
}

void VnUartPacket_parseGpsAntennaOffsetRaw(char *packet, vec3f* position)
{
	size_t packetIndex;
	char *result = (char*)VnUartPacket_startAsciiResponsePacketParse((uint8_t*)packet, &packetIndex);

	position->c[0] = ATOFF; NEXTRAW
	position->c[1] = ATOFF; NEXTRAW
	position->c[2] = ATOFF;
}

void VnUartPacket_parseGpsSolutionLla(VnUartPacket *packet, double* time, uint16_t* week, uint8_t* gpsFix, uint8_t* numSats, vec3d* lla, vec3f* nedVel, vec3f* nedAcc, float* speedAcc, float* timeAcc)
{
	VnUartPacket_parseGpsSolutionLlaRaw((char*)packet->data, time, week, gpsFix, numSats, lla, nedVel, nedAcc, speedAcc, timeAcc);
}

void VnUartPacket_parseGpsSolutionLlaRaw(char *packet, double* time, uint16_t* week, uint8_t* gpsFix, uint8_t* numSats, vec3d* lla, vec3f* nedVel, vec3f* nedAcc, float* speedAcc, float* timeAcc)
{
	size_t packetIndex;
	char *result = (char*)VnUartPacket_startAsciiResponsePacketParse((uint8_t*)packet, &packetIndex);

	*time = ATOFD; NEXTRAW
	*week = ATOU16; NEXTRAW
	*gpsFix = ATOU8; NEXTRAW
	*numSats = ATOU8; NEXTRAW
	lla->c[0] = ATOFD; NEXTRAW
	lla->c[1] = ATOFD; NEXTRAW
	lla->c[2] = ATOFD; NEXTRAW
	nedVel->c[0] = ATOFF; NEXTRAW
	nedVel->c[1] = ATOFF; NEXTRAW
	nedVel->c[2] = ATOFF; NEXTRAW
	nedAcc->c[0] = ATOFF; NEXTRAW
	nedAcc->c[1] = ATOFF; NEXTRAW
	nedAcc->c[2] = ATOFF; NEXTRAW
	*speedAcc = ATOFF; NEXTRAW
	*timeAcc = ATOFF;
}

void VnUartPacket_parseGpsSolutionEcef(VnUartPacket *packet, double* tow, uint16_t* week, uint8_t* gpsFix, uint8_t* numSats, vec3d* position, vec3f* velocity, vec3f* posAcc, float* speedAcc, float* timeAcc)
{
	VnUartPacket_parseGpsSolutionEcefRaw((char*)packet->data, tow, week, gpsFix, numSats, position, velocity, posAcc, speedAcc, timeAcc);
}

void VnUartPacket_parseGpsSolutionEcefRaw(char *packet, double* tow, uint16_t* week, uint8_t* gpsFix, uint8_t* numSats, vec3d* position, vec3f* velocity, vec3f* posAcc, float* speedAcc, float* timeAcc)
{
	size_t packetIndex;
	char *result = (char*)VnUartPacket_startAsciiResponsePacketParse((uint8_t*)packet, &packetIndex);

	*tow = ATOFD; NEXTRAW
	*week = ATOU16; NEXTRAW
	*gpsFix = ATOU8; NEXTRAW
	*numSats = ATOU8; NEXTRAW
	position->c[0] = ATOFD; NEXTRAW
	position->c[1] = ATOFD; NEXTRAW
	position->c[2] = ATOFD; NEXTRAW
	velocity->c[0] = ATOFF; NEXTRAW
	velocity->c[1] = ATOFF; NEXTRAW
	velocity->c[2] = ATOFF; NEXTRAW
	posAcc->c[0] = ATOFF; NEXTRAW
	posAcc->c[1] = ATOFF; NEXTRAW
	posAcc->c[2] = ATOFF; NEXTRAW
	*speedAcc = ATOFF; NEXTRAW
	*timeAcc = ATOFF;
}

void VnUartPacket_parseInsSolutionLla(VnUartPacket *packet, double* time, uint16_t* week, uint16_t* status, vec3f* yawPitchRoll, vec3d* position, vec3f* nedVel, float* attUncertainty, float* posUncertainty, float* velUncertainty)
{
	VnUartPacket_parseInsSolutionLlaRaw((char*)packet->data, time, week, status, yawPitchRoll, position, nedVel, attUncertainty, posUncertainty, velUncertainty);
}

void VnUartPacket_parseInsSolutionLlaRaw(char *packet, double* time, uint16_t* week, uint16_t* status, vec3f* yawPitchRoll, vec3d* position, vec3f* nedVel, float* attUncertainty, float* posUncertainty, float* velUncertainty)
{
	size_t packetIndex;
	char *result = (char*)VnUartPacket_startAsciiResponsePacketParse((uint8_t*)packet, &packetIndex);

	*time = ATOFD; NEXTRAW
	*week = ATOU16; NEXTRAW
	*status = ATOU16X; NEXTRAW
	yawPitchRoll->c[0] = ATOFF; NEXTRAW
	yawPitchRoll->c[1] = ATOFF; NEXTRAW
	yawPitchRoll->c[2] = ATOFF; NEXTRAW
	position->c[0] = ATOFD; NEXTRAW
	position->c[1] = ATOFD; NEXTRAW
	position->c[2] = ATOFD; NEXTRAW
	nedVel->c[0] = ATOFF; NEXTRAW
	nedVel->c[1] = ATOFF; NEXTRAW
	nedVel->c[2] = ATOFF; NEXTRAW
	*attUncertainty = ATOFF; NEXTRAW
	*posUncertainty = ATOFF; NEXTRAW
	*velUncertainty = ATOFF;
}

void VnUartPacket_parseInsSolutionEcef(VnUartPacket *packet, double* time, uint16_t* week, uint16_t* status, vec3f* yawPitchRoll, vec3d* position, vec3f* velocity, float* attUncertainty, float* posUncertainty, float* velUncertainty)
{
	VnUartPacket_parseInsSolutionEcefRaw((char*)packet->data, time, week, status, yawPitchRoll, position, velocity, attUncertainty, posUncertainty, velUncertainty);
}

void VnUartPacket_parseInsSolutionEcefRaw(char *packet, double* time, uint16_t* week, uint16_t* status, vec3f* yawPitchRoll, vec3d* position, vec3f* velocity, float* attUncertainty, float* posUncertainty, float* velUncertainty)
{
	size_t packetIndex;
	char *result = (char*)VnUartPacket_startAsciiResponsePacketParse((uint8_t*)packet, &packetIndex);

	*time = ATOFD; NEXTRAW
	*week = ATOU16; NEXTRAW
	*status = ATOU16X; NEXTRAW
	yawPitchRoll->c[0] = ATOFF; NEXTRAW
	yawPitchRoll->c[1] = ATOFF; NEXTRAW
	yawPitchRoll->c[2] = ATOFF; NEXTRAW
	position->c[0] = ATOFD; NEXTRAW
	position->c[1] = ATOFD; NEXTRAW
	position->c[2] = ATOFD; NEXTRAW
	velocity->c[0] = ATOFF; NEXTRAW
	velocity->c[1] = ATOFF; NEXTRAW
	velocity->c[2] = ATOFF; NEXTRAW
	*attUncertainty = ATOFF; NEXTRAW
	*posUncertainty = ATOFF; NEXTRAW
	*velUncertainty = ATOFF;
}

void VnUartPacket_parseInsBasicConfiguration(VnUartPacket *packet, uint8_t* scenario, uint8_t* ahrsAiding, uint8_t* estBaseline, uint8_t* resv2)
{
	VnUartPacket_parseInsBasicConfigurationRaw((char*)packet->data, scenario, ahrsAiding, estBaseline, resv2);
}

void VnUartPacket_parseInsBasicConfigurationRaw(char *packet, uint8_t* scenario, uint8_t* ahrsAiding, uint8_t* estBaseline, uint8_t* resv2)
{
	size_t packetIndex;
	char *result = (char*)VnUartPacket_startAsciiResponsePacketParse((uint8_t*)packet, &packetIndex);

	*scenario = ATOU8; NEXTRAW
	*ahrsAiding = ATOU8; NEXTRAW
	*estBaseline = ATOU8; NEXTRAW
	*resv2 = ATOU8;
}

void VnUartPacket_parseInsAdvancedConfiguration(VnUartPacket *packet, uint8_t* useMag, uint8_t* usePres, uint8_t* posAtt, uint8_t* velAtt, uint8_t* velBias, uint8_t* useFoam, uint8_t* gpsCovType, uint8_t* velCount, float* velInit, float* moveOrigin, float* gpsTimeout, float* deltaLimitPos, float* deltaLimitVel, float* minPosUncertainty, float* minVelUncertainty)
{
	VnUartPacket_parseInsAdvancedConfigurationRaw((char*)packet->data, useMag, usePres, posAtt, velAtt, velBias, useFoam, gpsCovType, velCount, velInit, moveOrigin, gpsTimeout, deltaLimitPos, deltaLimitVel, minPosUncertainty, minVelUncertainty);
}

void VnUartPacket_parseInsAdvancedConfigurationRaw(char *packet, uint8_t* useMag, uint8_t* usePres, uint8_t* posAtt, uint8_t* velAtt, uint8_t* velBias, uint8_t* useFoam, uint8_t* gpsCovType, uint8_t* velCount, float* velInit, float* moveOrigin, float* gpsTimeout, float* deltaLimitPos, float* deltaLimitVel, float* minPosUncertainty, float* minVelUncertainty)
{
	size_t packetIndex;
	char *result = (char*)VnUartPacket_startAsciiResponsePacketParse((uint8_t*)packet, &packetIndex);

	*useMag = ATOU8; NEXTRAW
	*usePres = ATOU8; NEXTRAW
	*posAtt = ATOU8; NEXTRAW
	*velAtt = ATOU8; NEXTRAW
	*velBias = ATOU8; NEXTRAW
	*useFoam = ATOU8; NEXTRAW
	*gpsCovType = ATOU8; NEXTRAW
	*velCount = ATOU8; NEXTRAW
	*velInit = ATOFF; NEXTRAW
	*moveOrigin = ATOFF; NEXTRAW
	*gpsTimeout = ATOFF; NEXTRAW
	*deltaLimitPos = ATOFF; NEXTRAW
	*deltaLimitVel = ATOFF; NEXTRAW
	*minPosUncertainty = ATOFF; NEXTRAW
	*minVelUncertainty = ATOFF;
}

void VnUartPacket_parseInsStateLla(VnUartPacket *packet, vec3f* yawPitchRoll, vec3d* position, vec3f* velocity, vec3f* accel, vec3f* angularRate)
{
	VnUartPacket_parseInsStateLlaRaw((char*)packet->data, yawPitchRoll, position, velocity, accel, angularRate);
}

void VnUartPacket_parseInsStateLlaRaw(char *packet, vec3f* yawPitchRoll, vec3d* position, vec3f* velocity, vec3f* accel, vec3f* angularRate)
{
	size_t packetIndex;
	char *result = (char*)VnUartPacket_startAsciiResponsePacketParse((uint8_t*)packet, &packetIndex);

	yawPitchRoll->c[0] = ATOFF; NEXTRAW
	yawPitchRoll->c[1] = ATOFF; NEXTRAW
	yawPitchRoll->c[2] = ATOFF; NEXTRAW
	position->c[0] = ATOFD; NEXTRAW
	position->c[1] = ATOFD; NEXTRAW
	position->c[2] = ATOFD; NEXTRAW
	velocity->c[0] = ATOFF; NEXTRAW
	velocity->c[1] = ATOFF; NEXTRAW
	velocity->c[2] = ATOFF; NEXTRAW
	accel->c[0] = ATOFF; NEXTRAW
	accel->c[1] = ATOFF; NEXTRAW
	accel->c[2] = ATOFF; NEXTRAW
	angularRate->c[0] = ATOFF; NEXTRAW
	angularRate->c[1] = ATOFF; NEXTRAW
	angularRate->c[2] = ATOFF;
}

void VnUartPacket_parseInsStateEcef(VnUartPacket *packet, vec3f* yawPitchRoll, vec3d* position, vec3f* velocity, vec3f* accel, vec3f* angularRate)
{
	VnUartPacket_parseInsStateEcefRaw((char*)packet->data, yawPitchRoll, position, velocity, accel, angularRate);
}

void VnUartPacket_parseInsStateEcefRaw(char *packet, vec3f* yawPitchRoll, vec3d* position, vec3f* velocity, vec3f* accel, vec3f* angularRate)
{
	size_t packetIndex;
	char *result = (char*)VnUartPacket_startAsciiResponsePacketParse((uint8_t*)packet, &packetIndex);

	yawPitchRoll->c[0] = ATOFF; NEXTRAW
	yawPitchRoll->c[1] = ATOFF; NEXTRAW
	yawPitchRoll->c[2] = ATOFF; NEXTRAW
	position->c[0] = ATOFD; NEXTRAW
	position->c[1] = ATOFD; NEXTRAW
	position->c[2] = ATOFD; NEXTRAW
	velocity->c[0] = ATOFF; NEXTRAW
	velocity->c[1] = ATOFF; NEXTRAW
	velocity->c[2] = ATOFF; NEXTRAW
	accel->c[0] = ATOFF; NEXTRAW
	accel->c[1] = ATOFF; NEXTRAW
	accel->c[2] = ATOFF; NEXTRAW
	angularRate->c[0] = ATOFF; NEXTRAW
	angularRate->c[1] = ATOFF; NEXTRAW
	angularRate->c[2] = ATOFF;
}

void VnUartPacket_parseStartupFilterBiasEstimate(VnUartPacket *packet, vec3f* gyroBias, vec3f* accelBias, float* pressureBias)
{
	VnUartPacket_parseStartupFilterBiasEstimateRaw((char*)packet->data, gyroBias, accelBias, pressureBias);
}

void VnUartPacket_parseStartupFilterBiasEstimateRaw(char *packet, vec3f* gyroBias, vec3f* accelBias, float* pressureBias)
{
	size_t packetIndex;
	char *result = (char*)VnUartPacket_startAsciiResponsePacketParse((uint8_t*)packet, &packetIndex);

	gyroBias->c[0] = ATOFF; NEXTRAW
	gyroBias->c[1] = ATOFF; NEXTRAW
	gyroBias->c[2] = ATOFF; NEXTRAW
	accelBias->c[0] = ATOFF; NEXTRAW
	accelBias->c[1] = ATOFF; NEXTRAW
	accelBias->c[2] = ATOFF; NEXTRAW
	*pressureBias = ATOFF;
}

void VnUartPacket_parseDeltaThetaAndDeltaVelocity(VnUartPacket *packet, float* deltaTime, vec3f* deltaTheta, vec3f* deltaVelocity)
{
	VnUartPacket_parseDeltaThetaAndDeltaVelocityRaw((char*)packet->data, deltaTime, deltaTheta, deltaVelocity);
}

void VnUartPacket_parseDeltaThetaAndDeltaVelocityRaw(char *packet, float* deltaTime, vec3f* deltaTheta, vec3f* deltaVelocity)
{
	size_t packetIndex;
	char *result = (char*)VnUartPacket_startAsciiResponsePacketParse((uint8_t*)packet, &packetIndex);

	*deltaTime = ATOFF; NEXTRAW
	deltaTheta->c[0] = ATOFF; NEXTRAW
	deltaTheta->c[1] = ATOFF; NEXTRAW
	deltaTheta->c[2] = ATOFF; NEXTRAW
	deltaVelocity->c[0] = ATOFF; NEXTRAW
	deltaVelocity->c[1] = ATOFF; NEXTRAW
	deltaVelocity->c[2] = ATOFF;
}

void VnUartPacket_parseDeltaThetaAndDeltaVelocityConfiguration(VnUartPacket *packet, uint8_t* integrationFrame, uint8_t* gyroCompensation, uint8_t* accelCompensation, uint8_t* reserved1, uint16_t* reserved2)
{
	VnUartPacket_parseDeltaThetaAndDeltaVelocityConfigurationRaw((char*)packet->data, integrationFrame, gyroCompensation, accelCompensation, reserved1, reserved2);
}

void VnUartPacket_parseDeltaThetaAndDeltaVelocityConfigurationRaw(char *packet, uint8_t* integrationFrame, uint8_t* gyroCompensation, uint8_t* accelCompensation, uint8_t* reserved1, uint16_t* reserved2)
{
	size_t packetIndex;
	char *result = (char*)VnUartPacket_startAsciiResponsePacketParse((uint8_t*)packet, &packetIndex);

	*integrationFrame = ATOU8; NEXTRAW
	*gyroCompensation = ATOU8; NEXTRAW
	*accelCompensation = ATOU8; NEXTRAW
	*reserved1 = ATOU8; NEXTRAW
	*reserved2 = ATOU16;
}

void VnUartPacket_parseReferenceVectorConfiguration(VnUartPacket *packet, uint8_t* useMagModel, uint8_t* useGravityModel, uint8_t* resv1, uint8_t* resv2, uint32_t* recalcThreshold, float* year, vec3d* position)
{
	VnUartPacket_parseReferenceVectorConfigurationRaw((char*)packet->data, useMagModel, useGravityModel, resv1, resv2, recalcThreshold, year, position);
}

void VnUartPacket_parseReferenceVectorConfigurationRaw(char *packet, uint8_t* useMagModel, uint8_t* useGravityModel, uint8_t* resv1, uint8_t* resv2, uint32_t* recalcThreshold, float* year, vec3d* position)
{
	size_t packetIndex;
	char *result = (char*)VnUartPacket_startAsciiResponsePacketParse((uint8_t*)packet, &packetIndex);

	*useMagModel = ATOU8; NEXTRAW
	*useGravityModel = ATOU8; NEXTRAW
	*resv1 = ATOU8; NEXTRAW
	*resv2 = ATOU8; NEXTRAW
	*recalcThreshold = ATOU32; NEXTRAW
	*year = ATOFF; NEXTRAW
	position->c[0] = ATOFD; NEXTRAW
	position->c[1] = ATOFD; NEXTRAW
	position->c[2] = ATOFD;
}

void VnUartPacket_parseGyroCompensation(VnUartPacket *packet, mat3f* c, vec3f* b)
{
	VnUartPacket_parseGyroCompensationRaw((char*)packet->data, c, b);
}

void VnUartPacket_parseGyroCompensationRaw(char *packet, mat3f* c, vec3f* b)
{
	size_t packetIndex;
	char *result = (char*)VnUartPacket_startAsciiResponsePacketParse((uint8_t*)packet, &packetIndex);

	c->e[0] = ATOFF; NEXTRAW
	c->e[3] = ATOFF; NEXTRAW
	c->e[6] = ATOFF; NEXTRAW
	c->e[1] = ATOFF; NEXTRAW
	c->e[4] = ATOFF; NEXTRAW
	c->e[7] = ATOFF; NEXTRAW
	c->e[2] = ATOFF; NEXTRAW
	c->e[5] = ATOFF; NEXTRAW
	c->e[8] = ATOFF; NEXTRAW
	b->c[0] = ATOFF; NEXTRAW
	b->c[1] = ATOFF; NEXTRAW
	b->c[2] = ATOFF;
}

void VnUartPacket_parseImuFilteringConfiguration(VnUartPacket *packet, uint16_t* magWindowSize, uint16_t* accelWindowSize, uint16_t* gyroWindowSize, uint16_t* tempWindowSize, uint16_t* presWindowSize, uint8_t* magFilterMode, uint8_t* accelFilterMode, uint8_t* gyroFilterMode, uint8_t* tempFilterMode, uint8_t* presFilterMode)
{
	VnUartPacket_parseImuFilteringConfigurationRaw((char*)packet->data, magWindowSize, accelWindowSize, gyroWindowSize, tempWindowSize, presWindowSize, magFilterMode, accelFilterMode, gyroFilterMode, tempFilterMode, presFilterMode);
}

void VnUartPacket_parseImuFilteringConfigurationRaw(char *packet, uint16_t* magWindowSize, uint16_t* accelWindowSize, uint16_t* gyroWindowSize, uint16_t* tempWindowSize, uint16_t* presWindowSize, uint8_t* magFilterMode, uint8_t* accelFilterMode, uint8_t* gyroFilterMode, uint8_t* tempFilterMode, uint8_t* presFilterMode)
{
	size_t packetIndex;
	char *result = (char*)VnUartPacket_startAsciiResponsePacketParse((uint8_t*)packet, &packetIndex);

	*magWindowSize = ATOU16; NEXTRAW
	*accelWindowSize = ATOU16; NEXTRAW
	*gyroWindowSize = ATOU16; NEXTRAW
	*tempWindowSize = ATOU16; NEXTRAW
	*presWindowSize = ATOU16; NEXTRAW
	*magFilterMode = ATOU8; NEXTRAW
	*accelFilterMode = ATOU8; NEXTRAW
	*gyroFilterMode = ATOU8; NEXTRAW
	*tempFilterMode = ATOU8; NEXTRAW
	*presFilterMode = ATOU8;
}

void VnUartPacket_parseGpsCompassBaseline(VnUartPacket *packet, vec3f* position, vec3f* uncertainty)
{
	VnUartPacket_parseGpsCompassBaselineRaw((char*)packet->data, position, uncertainty);
}

void VnUartPacket_parseGpsCompassBaselineRaw(char *packet, vec3f* position, vec3f* uncertainty)
{
	size_t packetIndex;
	char *result = (char*)VnUartPacket_startAsciiResponsePacketParse((uint8_t*)packet, &packetIndex);

	position->c[0] = ATOFF; NEXTRAW
	position->c[1] = ATOFF; NEXTRAW
	position->c[2] = ATOFF; NEXTRAW
	uncertainty->c[0] = ATOFF; NEXTRAW
	uncertainty->c[1] = ATOFF; NEXTRAW
	uncertainty->c[2] = ATOFF;
}

void VnUartPacket_parseGpsCompassEstimatedBaseline(VnUartPacket *packet, uint8_t* estBaselineUsed, uint8_t* resv, uint16_t* numMeas, vec3f* position, vec3f* uncertainty)
{
	VnUartPacket_parseGpsCompassEstimatedBaselineRaw((char*)packet->data, estBaselineUsed, resv, numMeas, position, uncertainty);
}

void VnUartPacket_parseGpsCompassEstimatedBaselineRaw(char *packet, uint8_t* estBaselineUsed, uint8_t* resv, uint16_t* numMeas, vec3f* position, vec3f* uncertainty)
{
	size_t packetIndex;
	char *result = (char*)VnUartPacket_startAsciiResponsePacketParse((uint8_t*)packet, &packetIndex);

	*estBaselineUsed = ATOU8; NEXTRAW
	*resv = ATOU8; NEXTRAW
	*numMeas = ATOU16; NEXTRAW
	position->c[0] = ATOFF; NEXTRAW
	position->c[1] = ATOFF; NEXTRAW
	position->c[2] = ATOFF; NEXTRAW
	uncertainty->c[0] = ATOFF; NEXTRAW
	uncertainty->c[1] = ATOFF; NEXTRAW
	uncertainty->c[2] = ATOFF;
}

void VnUartPacket_parseImuRateConfiguration(VnUartPacket *packet, uint16_t* imuRate, uint16_t* navDivisor, float* filterTargetRate, float* filterMinRate)
{
	VnUartPacket_parseImuRateConfigurationRaw((char*)packet->data, imuRate, navDivisor, filterTargetRate, filterMinRate);
}

void VnUartPacket_parseImuRateConfigurationRaw(char *packet, uint16_t* imuRate, uint16_t* navDivisor, float* filterTargetRate, float* filterMinRate)
{
	size_t packetIndex;
	char *result = (char*)VnUartPacket_startAsciiResponsePacketParse((uint8_t*)packet, &packetIndex);

	*imuRate = ATOU16; NEXTRAW
	*navDivisor = ATOU16; NEXTRAW
	*filterTargetRate = ATOFF; NEXTRAW
	*filterMinRate = ATOFF;
}

void VnUartPacket_parseYawPitchRollTrueBodyAccelerationAndAngularRates(VnUartPacket *packet, vec3f* yawPitchRoll, vec3f* bodyAccel, vec3f* gyro)
{
	VnUartPacket_parseYawPitchRollTrueBodyAccelerationAndAngularRatesRaw((char*)packet->data, yawPitchRoll, bodyAccel, gyro);
}

void VnUartPacket_parseYawPitchRollTrueBodyAccelerationAndAngularRatesRaw(char *packet, vec3f* yawPitchRoll, vec3f* bodyAccel, vec3f* gyro)
{
	size_t packetIndex;
	char *result = (char*)VnUartPacket_startAsciiResponsePacketParse((uint8_t*)packet, &packetIndex);

	yawPitchRoll->c[0] = ATOFF; NEXTRAW
	yawPitchRoll->c[1] = ATOFF; NEXTRAW
	yawPitchRoll->c[2] = ATOFF; NEXTRAW
	bodyAccel->c[0] = ATOFF; NEXTRAW
	bodyAccel->c[1] = ATOFF; NEXTRAW
	bodyAccel->c[2] = ATOFF; NEXTRAW
	gyro->c[0] = ATOFF; NEXTRAW
	gyro->c[1] = ATOFF; NEXTRAW
	gyro->c[2] = ATOFF;
}

void VnUartPacket_parseYawPitchRollTrueInertialAccelerationAndAngularRates(VnUartPacket *packet, vec3f* yawPitchRoll, vec3f* inertialAccel, vec3f* gyro)
{
	VnUartPacket_parseYawPitchRollTrueInertialAccelerationAndAngularRatesRaw((char*)packet->data, yawPitchRoll, inertialAccel, gyro);
}

void VnUartPacket_parseYawPitchRollTrueInertialAccelerationAndAngularRatesRaw(char *packet, vec3f* yawPitchRoll, vec3f* inertialAccel, vec3f* gyro)
{
	size_t packetIndex;
	char *result = (char*)VnUartPacket_startAsciiResponsePacketParse((uint8_t*)packet, &packetIndex);

	yawPitchRoll->c[0] = ATOFF; NEXTRAW
	yawPitchRoll->c[1] = ATOFF; NEXTRAW
	yawPitchRoll->c[2] = ATOFF; NEXTRAW
	inertialAccel->c[0] = ATOFF; NEXTRAW
	inertialAccel->c[1] = ATOFF; NEXTRAW
	inertialAccel->c[2] = ATOFF; NEXTRAW
	gyro->c[0] = ATOFF; NEXTRAW
	gyro->c[1] = ATOFF; NEXTRAW
	gyro->c[2] = ATOFF;
}

void strFromVnAsciiAsync(char *out, VnAsciiAsync val)
{
	#if defined(_MSC_VER)
		/* Disable warnings regarding using strcpy_s since this
		 * function's signature does not provide us with information
		 * about the length of 'out'. */
		#pragma warning(push)
		#pragma warning(disable:4996)
	#endif

	switch (val)
	{
		case VNOFF:
			strcpy(out, "Off");
			break;
		case VNYPR:
			strcpy(out, "Yaw, Pitch, Roll");
			break;
		case VNQTN:
			strcpy(out, "Quaterion");
			break;
		#ifdef EXTRA
		case VNQTM:
			strcpy(out, "Quaternion and Magnetic");
			break;
		case VNQTA:
			strcpy(out, "Quaternion and Acceleration");
			break;
		case VNQTR:
			strcpy(out, "Quaternion and Angular Rates");
			break;
		case VNQMA:
			strcpy(out, "Quaternion, Magnetic and Acceleration");
			break;
		case VNQAR:
			strcpy(out, "Quaternion, Acceleration and Angular Rates");
			break;
		#endif
		case VNQMR:
			strcpy(out, "Quaternion, Magnetic, Acceleration and Angular Rates");
			break;
		#ifdef EXTRA
		case VNDCM:
			strcpy(out, "Directional Cosine Orientation Matrix");
			break;
		#endif
		case VNMAG:
			strcpy(out, "Magnetic Measurements");
			break;
		case VNACC:
			strcpy(out, "Acceleration Measurements");
			break;
		case VNGYR:
			strcpy(out, "Angular Rate Measurements");
			break;
		case VNMAR:
			strcpy(out, "Magnetic, Acceleration, and Angular Rate Measurements");
			break;
		case VNYMR:
			strcpy(out, "Yaw, Pitch, Roll, Magnetic, Acceleration, and Angular Rate Measurements");
			break;
		#ifdef EXTRA
		case VNYCM:
			strcpy(out, "Yaw, Pitch, Roll, and Calibrated Measurements");
			break;
		#endif
		case VNYBA:
			strcpy(out, "Yaw, Pitch, Roll, Body True Acceleration");
			break;
		case VNYIA:
			strcpy(out, "Yaw, Pitch, Roll, Inertial True Acceleration");
			break;
		#ifdef EXTRA
		case VNICM:
			strcpy(out, "Yaw, Pitch, Roll, Inertial Magnetic/Acceleration, and Angular Rate Measurements");
			break;
		#endif
		case VNIMU:
			strcpy(out, "Calibrated Inertial Measurements");
			break;
		case VNGPS:
			strcpy(out, "GPS LLA");
			break;
		case VNGPE:
			strcpy(out, "GPS ECEF");
			break;
		case VNINS:
			strcpy(out, "INS LLA solution");
			break;
		case VNINE:
			strcpy(out, "INS ECEF solution");
			break;
		case VNISL:
			strcpy(out, "INS LLA 2 solution");
			break;
		case VNISE:
			strcpy(out, "INS ECEF 2 solution");
			break;
		case VNDTV:
			strcpy(out, "Delta Theta and Delta Velocity");
			break;
		case VNRTK:
			strcpy(out, "RTK");
			break;
#ifdef EXTRA
		case VNRAW:
			strcpy(out, "Raw Voltage Measurements");
			break;
		case VNCMV:
			strcpy(out, "Calibrated Measurements");
			break;
		case VNSTV:
			strcpy(out, "Kalman Filter State Vector");
			break;
		case VNCOV:
			strcpy(out, "Kalman Filter Covariance Matrix Diagonal");
			break;
		#endif
		default:
			strcpy(out, "Unknown");
			break;
	}

	#if defined(_MSC_VER)
		#pragma warning(pop)
	#endif
}
