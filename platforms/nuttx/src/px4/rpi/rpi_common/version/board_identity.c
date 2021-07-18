/**
 * @file board_identity.c
 * Implementation of STM32 based Board identity API
 */

#include <px4_platform_common/px4_config.h>
#include <stdio.h>
#include <string.h>

// RP2040 doesn't really have a cpu register with unique id.
// However, there is a function in pico-sdk which can provide
// a device unique id from its flash which is 64 bits in length.
// For now, a fixed value of 8 bytes "MYFC2040" is used.
uint32_t myUUID[2] = {'M' << 0 | 'Y' << 8 | 'F' << 16 | 'C' << 24,'2' << 0 | '0' << 8 | '4' << 16 | '0' << 24};
#define MYFC_SYSTEM_UID	((uint32_t)myUUID)

#define CPU_UUID_BYTE_FORMAT_ORDER          {3, 2, 1, 0, 7, 6, 5, 4}
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
	uint32_t *chip_uuid = (uint32_t *) MYFC_SYSTEM_UID;

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

	for (unsigned i = 0; i < PX4_CPU_UUID_WORD32_LENGTH; i++) {
		offset += snprintf(&format_buffer[offset], size - offset, format, uuid[i]);

		if (sep_size && i < PX4_CPU_UUID_WORD32_LENGTH - 1) {
			strcat(&format_buffer[offset], seperator);
			offset += sep_size;
		}
	}

	return 0;
}

int board_get_mfguid(mfguid_t mfgid)
{
	uint32_t *chip_uuid = (uint32_t *) MYFC_SYSTEM_UID;
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

	uint32_t *chip_uuid = (uint32_t *) MYFC_SYSTEM_UID;

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
