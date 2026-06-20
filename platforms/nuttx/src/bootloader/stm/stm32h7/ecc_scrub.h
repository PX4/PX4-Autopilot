/*
 * STM32H7 flash ECC scrub for the bootloader.
 *
 */

#pragma once

/* All supported H7 parts have uniform 128 KiB sectors. */
#if !defined(FLASH_SECTOR_SIZE)
#define FLASH_SECTOR_SIZE   (128u * 1024u)
#endif

void check_ecc_errors(void);
