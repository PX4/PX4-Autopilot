#ifndef _BOARD_TYPE_H_
#define _BOARD_TYPE_H_

#include <px4_platform_common/crypto_algorithms.h>

#define BOARD_TYPE 1500

#define BOOTLOADER_SIGNING_ALGORITHM CRYPTO_ED25519
#define BOOTLOADER_VERIFY_UBOOT 0
#define BOOTLOADER_BOOT_HART_1 0
#define BOOTLOADER_BOOT_HART_4 0

#define TOC_VERIFICATION_KEY 0
#define BOOT_VERIFICATION_KEY 0

#endif
