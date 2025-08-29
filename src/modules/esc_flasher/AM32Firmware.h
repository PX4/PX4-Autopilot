#pragma once

#include <px4_platform_common/defines.h>

#define INCLUDE_AM32_FIRMWARE

#define AM32_FIRMWARE_ADDR        (0x08001000)
#define AM32_FIRMWARE_TAG_ADDR    (0x08007BE0)

__EXPORT extern const uint8_t am32_fw_version_major;
__EXPORT extern const uint8_t am32_fw_version_minor;

__EXPORT extern const uint8_t am32_firmware[20752];
__EXPORT extern const uint8_t am32_firmware_tag[16];
