#include "vn/util.h"

#include <string.h>
#include <stdio.h>

#if (defined _M_IX86 || defined __i386__ || defined __x86_64)
	/* Compiling for x86 processor. */
	#define VN_LITTLE_ENDIAN	1
#elif defined __linux__
	/* Don't know what processor we are compiling for but we have endian.h. */
	#define VN_HAVE_ENDIAN_H	1
	#include <endian.h>
#elif defined __ORDER_LITTLE_ENDIAN__
	#define VN_LITTLE_ENDIAN	1
#elif defined __ORDER_BIG_ENDIAN__
	#define VN_BIG_ENDIAN	1
#else
	#error "Unknown System"
#endif

int VnApi_major(void)
{
	return VNAPI_MAJOR;
}

int VnApi_minor(void)
{
	return VNAPI_MINOR;
}

int VnApi_patch(void)
{
	return VNAPI_PATCH;
}

int VnApi_revision(void)
{
	return VNAPI_REVISION;
}

VnError VnApi_getVersion(char *out, size_t outLength)
{
	#if defined(_MSC_VER)
		/* Disable warnings regarding using strcpy_s since this
		 * function's signature does not provide us with information
		 * about the length of 'out'. */
		#pragma warning(push)
		#pragma warning(disable:4996)
	#endif

	outLength = sprintf(out, "%d.%d.%d.%d", VNAPI_MAJOR, VNAPI_MINOR, VNAPI_PATCH, VNAPI_REVISION);

	#if defined(_MSC_VER)
		#pragma warning(pop)
	#endif

	return E_NONE;
}

uint8_t VnUtil_toUint8FromHexChar(char c)
{
	if (c < ':')
		return c - '0';

	if (c < 'G')
		return c - '7';

	return c - 'W';
}

uint8_t VnUtil_toUint8FromHexStr(char const *str)
{
	uint8_t result;

	result = VnUtil_toUint8FromHexChar(str[0]) << 4;
	result += VnUtil_toUint8FromHexChar(str[1]);

	return result;
}

uint16_t VnUtil_toUint16FromHexStr(char const *str)
{
	uint16_t result;

	result = VnUtil_toUint8FromHexChar(str[0]) << 12;
	result += VnUtil_toUint8FromHexChar(str[1]) << 8;
	result += VnUtil_toUint8FromHexChar(str[2]) << 4;
	result += VnUtil_toUint8FromHexChar(str[3]);

	return result;
}

void VnUtil_toHexStrFromUint8(uint8_t toConvert, char* output)
{
	char const digits[] = "0123456789ABCDEF";

	*output++ = digits[(toConvert / 16) % 16];
	*output = digits[toConvert % 16];
}

void VnUtil_toHexStrFromUint16(uint16_t toConvert, char* output)
{
	char const digits[] = "0123456789ABCDEF";

	*output++ = digits[(toConvert / (16*16*16)) % 16];
	*output++ = digits[(toConvert / (16 * 16)) % 16];
	*output++ = digits[(toConvert / 16) % 16];
	*output = digits[toConvert % 16];
}

size_t VnUtil_toStrFromUint16(uint16_t toConvert, char *output)
{
	char const digits[] = "0123456789";

	uint16_t shifter = toConvert;
	size_t numOfChars = 0;
	char* p = output;

	/* Move to the representation end. */
	do
	{
		++p;
		shifter /= 10;
		numOfChars++;
	} while (shifter);

	/* Move back to the beginning inserting our converted digits as we go. */
	do
	{
		*--p = digits[toConvert % 10];
		toConvert /= 10;
	} while (toConvert);

	return numOfChars;
}

uint8_t VnUtil_countSetBitsUint8(uint8_t data)
{
	uint8_t count = 0;

	while (data)
	{
		data &= (data - 1);

		count++;
	}

	return count;
}

void strFromBool(char *out, bool val)
{
	#if defined(_MSC_VER)
		/* Disable warnings regarding using strcpy_s since this
		 * function's signature does not provide us with information
		 * about the length of 'out'. */
		#pragma warning(push)
		#pragma warning(disable:4996)
	#endif

	if (val == 0)
		strcpy(out, "false");
	else
		strcpy(out, "true");

	#if defined(_MSC_VER)
		#pragma warning(pop)
	#endif
}

uint16_t stoh16(uint16_t sensorOrdered)
{
	#if VN_LITTLE_ENDIAN
	return sensorOrdered;
	#elif VN_BIG_ENDIAN
	uint16_t host;
	host = ((sensorOrdered >> 0) & 0xFF) * 0x0100;
	host += ((sensorOrdered >> 8) & 0xFF) * 0x0001;
	return host;
	#elif VN_HAVE_ENDIAN_H
	return le16toh(sensorOrdered);
	#else
	#error("Unknown system")
	#endif
}

uint32_t stoh32(uint32_t sensorOrdered)
{
	#if VN_LITTLE_ENDIAN
	return sensorOrdered;
	#elif VN_BIG_ENDIAN
	uint32_t host;
	host = ((sensorOrdered >> 0) & 0xFF) * 0x01000000;
	host += ((sensorOrdered >> 8) & 0xFF) * 0x00010000;
	host += ((sensorOrdered >> 16) & 0xFF) * 0x00000100;
	host += ((sensorOrdered >> 24) & 0xFF) * 0x00000001;
	return host;
	#elif VN_HAVE_ENDIAN_H
	return le32toh(sensorOrdered);
	#else
	#error("Unknown system")
	#endif
}

uint64_t stoh64(uint64_t sensorOrdered)
{
	#if VN_LITTLE_ENDIAN
	return sensorOrdered;
	#elif VN_BIG_ENDIAN
	uint64_t host;
	host = ((sensorOrdered >> 0) & 0xFF) * 0x0100000000000000;
	host += ((sensorOrdered >> 8) & 0xFF) * 0x0001000000000000;
	host += ((sensorOrdered >> 16) & 0xFF) * 0x0000010000000000;
	host += ((sensorOrdered >> 24) & 0xFF) * 0x0000000100000000;
	host += ((sensorOrdered >> 32) & 0xFF) * 0x0000000001000000;
	host += ((sensorOrdered >> 40) & 0xFF) * 0x0000000000010000;
	host += ((sensorOrdered >> 48) & 0xFF) * 0x0000000000000100;
	host += ((sensorOrdered >> 56) & 0xFF) * 0x0000000000000001;
	return host;
	#elif VN_HAVE_ENDIAN_H
	return le64toh(sensorOrdered);
	#else
	#error("Unknown system")
	#endif
}

uint16_t htos16(uint16_t hostOrdered)
{
	return stoh16(hostOrdered);
}

uint32_t htos32(uint32_t hostOrdered)
{
	return stoh32(hostOrdered);
}

uint64_t htos64(uint64_t hostOrdered)
{
	return stoh64(hostOrdered);
}

float htosf4(float hostOrdered)
{
	uint32_t* v = (uint32_t*) &hostOrdered;

	*v = stoh32(*v);

	return hostOrdered;
}

double htosf8(double hostOrdered)
{
	uint64_t* v = (uint64_t*) &hostOrdered;

	*v = stoh64(*v);

	return hostOrdered;
}

uint16_t VnUtil_extractUint16(const char* pos)
{
	uint16_t d;

	memcpy(&d, pos, sizeof(uint16_t));

	return stoh16(d);
}

uint32_t VnUtil_extractUint32(const char* pos)
{
	uint32_t d;

	memcpy(&d, pos, sizeof(uint32_t));

	return stoh32(d);
}

float VnUtil_extractFloat(const char* pos)
{
	float f;
	uint32_t tmp;

	memcpy(&tmp, pos, sizeof(float));
	tmp = stoh32(tmp);
	memcpy(&f, &tmp, sizeof(float));

	return f;
}

double VnUtil_extractDouble(const char* pos)
{
	double f;
	uint64_t tmp;

	memcpy(&tmp, pos, sizeof(double));
	tmp = stoh64(tmp);
	memcpy(&f, &tmp, sizeof(double));

	return f;
}

vec3f VnUtil_extractVec3f(const char* pos)
{
	vec3f r;

	r.c[0] = VnUtil_extractFloat(pos);
	r.c[1] = VnUtil_extractFloat(pos + sizeof(float));
	r.c[2] = VnUtil_extractFloat(pos + 2 * sizeof(float));

	return r;
}

vec4f VnUtil_extractVec4f(const char* pos)
{
	vec4f r;

	r.c[0] = VnUtil_extractFloat(pos);
	r.c[1] = VnUtil_extractFloat(pos + sizeof(float));
	r.c[2] = VnUtil_extractFloat(pos + 2 * sizeof(float));
	r.c[3] = VnUtil_extractFloat(pos + 3 * sizeof(float));

	return r;
}

vec3d VnUtil_extractVec3d(const char* pos)
{
	vec3d r;

	r.c[0] = VnUtil_extractDouble(pos);
	r.c[1] = VnUtil_extractDouble(pos + sizeof(double));
	r.c[2] = VnUtil_extractDouble(pos + 2 * sizeof(double));

	return r;
}

mat3f VnUtil_extractMat3f(const char* pos)
{
	mat3f r;

	r.e[0] = VnUtil_extractFloat(pos);
	r.e[1] = VnUtil_extractFloat(pos + sizeof(float));
	r.e[2] = VnUtil_extractFloat(pos + 2 * sizeof(float));
	r.e[3] = VnUtil_extractFloat(pos + 3 * sizeof(float));
	r.e[4] = VnUtil_extractFloat(pos + 4 * sizeof(float));
	r.e[5] = VnUtil_extractFloat(pos + 5 * sizeof(float));
	r.e[6] = VnUtil_extractFloat(pos + 6 * sizeof(float));
	r.e[7] = VnUtil_extractFloat(pos + 7 * sizeof(float));
	r.e[8] = VnUtil_extractFloat(pos + 8 * sizeof(float));

	return r;
}

GpsDop VnUtil_extractGpsDop(const char* pos)
{
  GpsDop dop;

  dop.gDOP = VnUtil_extractFloat(pos);
  dop.pDOP = VnUtil_extractFloat(pos + sizeof(float));
  dop.tDOP = VnUtil_extractFloat(pos + 2 * sizeof(float));
  dop.vDOP = VnUtil_extractFloat(pos + 3 * sizeof(float));
  dop.hDOP = VnUtil_extractFloat(pos + 4 * sizeof(float));
  dop.nDOP = VnUtil_extractFloat(pos + 5 * sizeof(float));
  dop.eDOP = VnUtil_extractFloat(pos + 6 * sizeof(float));

  return dop;
}

TimeUtc VnUtil_extractTimeUtc(const char* pos)
{
  TimeUtc timeUtc;

  timeUtc.year  = (int8_t)pos[0];
  timeUtc.month = (uint8_t)pos[1];
  timeUtc.day   = (uint8_t)pos[2];
  timeUtc.hour  = (uint8_t)pos[3];
  timeUtc.min   = (uint8_t)pos[4];
  timeUtc.sec   = (uint8_t)pos[5];
  timeUtc.ms = VnUtil_extractUint16(pos + 6);

  return timeUtc;
}

TimeInfo VnUtil_extractTimeInfo(const char* pos)
{
  TimeInfo timeInfo;

  timeInfo.timeStatus = (uint8_t)pos[0];
  timeInfo.leapSeconds = (int8_t)pos[1];

  return timeInfo;
}
