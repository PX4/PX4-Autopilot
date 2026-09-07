/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
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

/*
 * Table of contents for the standalone signed TOC block prepended to
 * the px4_fmu-v6xrt_secureboot image. Layout on flash after upload:
 *
 *   0x30020000  +-------------------------------+  <- APP_LOAD_ADDRESS
 *               | TOC struct (this file)        |
 *               | 0xff padding                  |
 *   0x30020fc0  +-------------------------------+
 *               | 64B TOC signature (SIG0)      |
 *   0x30021000  +-------------------------------+  <- _app_addr
 *               | app boot_hdr / vectors / text |
 *               | .../ .rodata / .data image    |
 *   ...         +-------------------------------+
 *               | 64B app signature (SIG1)      |
 *               +-------------------------------+
 *
 * The TOC / SIG0 entries use TOC_FLAG2_RELATIVE_ADDRESSES so their
 * start / end are byte offsets into the 4 KiB TOC block (which the
 * bootloader locates at APP_LOAD_ADDRESS by default). The BOOT / SIG1
 * entries use absolute XIP-flash addresses: since the app payload is
 * memory-mapped and executed in place, no staging into RAM is needed
 * and the bootloader can hash the payload directly from flash.
 */

#include <image_toc.h>

/* ed25519 signature size, matches _sig_size in toc.ld and sign_firmware.py. */
#define SIGNATURE_SIZE 64

/* TOC block byte-offset symbols come from toc.ld. */
extern const uintptr_t _toc_start;
extern const uintptr_t _toc_end;
extern const uintptr_t _toc_sig_start;
extern const uintptr_t _toc_sig_end;

#define TOC_START    ((const void *)&_toc_start)
#define TOC_END      ((const void *)&_toc_end)
#define TOCSIG_START ((const void *)&_toc_sig_start)
#define TOCSIG_END   ((const void *)&_toc_sig_end)

/* Absolute XIP addresses for the app payload and its signature, computed
 * from the .incbin'd unsigned app .bin in toc.ld.
 */
extern const uintptr_t _app_addr;
extern const uintptr_t _boot_sig_addr;
extern const uintptr_t _boot_sig_end;

#define BOOT_START    ((const void *)&_app_addr)
#define BOOT_END      ((const void *)&_boot_sig_addr)
#define BOOTSIG_START ((const void *)&_boot_sig_addr)
#define BOOTSIG_END   ((const void *)&_boot_sig_end)

IMAGE_MAIN_TOC(4) = {
	{TOC_START_MAGIC, TOC_VERSION},
	{
		{"TOC",  TOC_START,    TOC_END,    0, 1, 0, 0, TOC_FLAG1_CHECK_SIGNATURE, TOC_FLAG2_RELATIVE_ADDRESSES},
		{"SIG0", TOCSIG_START, TOCSIG_END, 0, 0, 0, 0, 0,                         TOC_FLAG2_RELATIVE_ADDRESSES},
		{"BOOT", BOOT_START,   BOOT_END,   0, 3, 0, 0, TOC_FLAG1_BOOT | TOC_FLAG1_CHECK_SIGNATURE, 0},
		{"SIG1", BOOTSIG_START, BOOTSIG_END, 0, 0, 0, 0, 0,                       0},
	},
	TOC_END_MAGIC
};

/* Placeholder used by toc.ld to size the TOC-signature slot inside the
 * standalone block. sign_firmware.py fills the real bytes after signing.
 */
const char _main_toc_sig[SIGNATURE_SIZE] __attribute__((section(".main_toc_sig")));
