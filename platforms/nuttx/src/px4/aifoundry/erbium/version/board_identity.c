/**
 * @file board_identity.c
 * Implementation of the Erbium board identity API
 * Implementation of the Erbium board identity API
 */

#include <px4_platform_common/px4_config.h>
#include <stdio.h>
#include <string.h>

/* Erbium has no unique-ID register. All Erbium boards report the fixed
 * 12-byte ID "ERBIUMMINION" until an ID from OTP is used.
 */

static const uint32_t erbium_uuid[3] = {
	0x49425245, /* "ERBI", little-endian */
	0x494d4d55, /* "UMMI" */
	0x4e4f494e  /* "NION" */
};

#define CPU_UUID_BYTE_FORMAT_ORDER          {3, 2, 1, 0, 7, 6, 5, 4, 11, 10, 9, 8}
#define SWAP_UINT32(x) (((x) >> 24) | (((x) & 0x00ff0000) >> 8) | (((x) & 0x0000ff00) << 8) | ((x) << 24))

static const uint16_t soc_arch_id = PX4_SOC_ARCH_ID;

/* A type suitable for holding the reordering array for the byte format of the UUID
 */

typedef const uint8_t uuid_uint8_reorder_t[PX4_CPU_UUID_BYTE_LENGTH];


void board_get_uuid(uuid_byte_t uuid_bytes)
{
	uuid_uint8_reorder_t reorder = CPU_UUID_BYTE_FORMAT_ORDER;
	union {
		uuid_byte_t b;
		uuid_uint32_t w;
	} id;

	/* Copy the serial from the chips non-write memory */

	board_get_uuid32(id.w);

	/* swap endianess */

	for (int i = 0; i < PX4_CPU_UUID_BYTE_LENGTH; i++) {
		uuid_bytes[i] = id.b[reorder[i]];
	}
}

__EXPORT void board_get_uuid32(uuid_uint32_t uuid_words)
{
	const uint32_t *chip_uuid = erbium_uuid;

	for (unsigned i = 0; i < PX4_CPU_UUID_WORD32_LENGTH; i++) {
		uuid_words[i] = chip_uuid[i];
	}
}

int board_get_uuid32_formated(char *format_buffer, int size,
			      const char *format,
			      const char *seperator)
{
	uuid_uint32_t uuid;
	board_get_uuid32(uuid);
	int offset = 0;
	int sep_size = seperator ? strlen(seperator) : 0;

	for (unsigned i = 0; (offset < size - 1) && (i < PX4_CPU_UUID_WORD32_LENGTH); i++) {
		offset += snprintf(&format_buffer[offset], size - offset, format, uuid[i]);

		if (sep_size && (offset < size - sep_size - 1) && (i < PX4_CPU_UUID_WORD32_LENGTH - 1)) {
			strncat(&format_buffer[offset], seperator, size - offset);
			offset += sep_size;
		}
	}

	return 0;
}

int board_get_mfguid(mfguid_t mfgid)
{
	const uint32_t *chip_uuid = erbium_uuid;
	uint8_t  *rv = &mfgid[0];

	for (unsigned i = 0; i < PX4_CPU_UUID_WORD32_LENGTH; i++) {
		uint32_t uuid_bytes = SWAP_UINT32(chip_uuid[(PX4_CPU_UUID_WORD32_LENGTH - 1) - i]);
		memcpy(rv, &uuid_bytes, sizeof(uint32_t));
		rv += sizeof(uint32_t);
	}

	return PX4_CPU_MFGUID_BYTE_LENGTH;
}

int board_get_mfguid_formated(char *format_buffer, int size)
{
	mfguid_t mfguid;

	board_get_mfguid(mfguid);
	int offset  = 0;

	for (unsigned i = 0; offset < size && i < PX4_CPU_MFGUID_BYTE_LENGTH; i++) {
		offset += snprintf(&format_buffer[offset], size - offset, "%02x", mfguid[i]);
	}

	return offset;
}

int board_get_px4_guid(px4_guid_t px4_guid)
{
	uint8_t  *pb = (uint8_t *) &px4_guid[0];
	*pb++ = (soc_arch_id >> 8) & 0xff;
	*pb++ = (soc_arch_id & 0xff);

	for (unsigned i = 0; i < PX4_GUID_BYTE_LENGTH - (sizeof(soc_arch_id) + PX4_CPU_UUID_BYTE_LENGTH); i++) {
		*pb++ = 0;
	}

	const uint32_t *chip_uuid = erbium_uuid;

	for (unsigned i = 0; i < PX4_CPU_UUID_WORD32_LENGTH; i++) {
		uint32_t uuid_bytes = SWAP_UINT32(chip_uuid[(PX4_CPU_UUID_WORD32_LENGTH - 1) - i]);
		memcpy(pb, &uuid_bytes, sizeof(uint32_t));
		pb += sizeof(uint32_t);
	}

	return PX4_GUID_BYTE_LENGTH;
}

int board_get_px4_guid_formated(char *format_buffer, int size)
{
	px4_guid_t px4_guid;
	board_get_px4_guid(px4_guid);
	int offset  = 0;

	/* size should be 2 per byte + 1 for termination
	 * So it needs to be odd
	 */
	size = size & 1 ? size : size - 1;

	/* Discard from MSD */
	for (unsigned i = PX4_GUID_BYTE_LENGTH - size / 2; offset < size && i < PX4_GUID_BYTE_LENGTH; i++) {
		offset += snprintf(&format_buffer[offset], size - offset, "%02x", px4_guid[i]);
	}

	return offset;
}
