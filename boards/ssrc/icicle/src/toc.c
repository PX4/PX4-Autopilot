/****************************************************************************
 *
 *   Copyright (C) 2020 Technology Innovation Institute. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
#include <image_toc.h>
#include "board_config.h"
#include "board_type.h"

/* Size of the signature */

#define SIGNATURE_SIZE PX4_SIGNATURE_SIZE(BOOTLOADER_SIGNING_ALGORITHM)

/* ToC area boundaries */
extern const uintptr_t _toc_start;
extern const uintptr_t _toc_end;

#define TOC_ADDR &_toc_start
#define TOC_END ((const void *)&_toc_end)

/* ToC signature */
extern const uintptr_t _toc_signature;

#define TOCSIG_ADDR ((const void *)&_toc_signature)
#define TOCSIG_END ((const void *)((const uint8_t *)TOCSIG_ADDR+SIGNATURE_SIZE))

/* Boot image starts at __start and ends at
 * the beginning of signature, but for protected/kernel mode we don't know
 * their locations. Assume binary file start and binary file end ?
*/
extern const uintptr_t _app_start;
extern const uintptr_t _app_end;

#define BOOT_ADDR &_app_start
#define BOOT_END ((const void *)&_app_end)

/* Boot signature start and end are defined by the
 * signature definition below
*/
extern const uintptr_t _boot_signature;

#define BOOTSIG_ADDR ((const void *)&_boot_signature)
#define BOOTSIG_END ((const void *)((const uint8_t *)BOOTSIG_ADDR+SIGNATURE_SIZE))

/* RD certifcate may follow boot signature */

#define RDCT_ADDR BOOTSIG_END
#define RDCT_END ((const void *)((const uint8_t*)BOOTSIG_END+sizeof(image_cert_t)))

/* RD certificate signature follows the certificate */

#define RDCTSIG_ADDR RDCT_END
#define RDCTSIG_END ((const void *)((const uint8_t*)RDCTSIG_ADDR+SIGNATURE_SIZE))

/* The table of contents */

IMAGE_MAIN_TOC(6) = {
	{TOC_START_MAGIC, TOC_VERSION},
	{
		{"TOC",  TOC_ADDR, TOC_END, 0, 1, TOC_VERIFICATION_KEY, 0, TOC_FLAG1_CHECK_SIGNATURE},
		{"SIG0", TOCSIG_ADDR, TOCSIG_END, 0, 0, 0, 0, 0},
		{"BOOT", BOOT_ADDR, BOOT_END, 0, 3, BOOT_VERIFICATION_KEY, 0, TOC_FLAG1_BOOT | TOC_FLAG1_CHECK_SIGNATURE, PX4_VENDOR_BOOT_FLAGS},
		{"SIG1", BOOTSIG_ADDR, BOOTSIG_END, 0, 0, 0, 0, 0},
		{"RDCT", RDCT_ADDR, RDCT_END, 0, 5, 0, 0, TOC_FLAG1_RDCT | TOC_FLAG1_CHECK_SIGNATURE},
		{"RDSG", RDCTSIG_ADDR, RDCTSIG_END, 0, 0, 0, 0, 0},
	},
	TOC_END_MAGIC
};

/* Define a signature area, just for sizing the ToC area */

const char _main_toc_sig[SIGNATURE_SIZE] __attribute__((section(".main_toc_sig")));
