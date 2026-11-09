/****************************************************************************
 * boards/px4/fmu-v6xrt/nuttx-config/scripts/toc.ld
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 *
 * Linker script for the standalone signed TOC block that is prepended to
 * the signed px4_fmu-v6xrt_secureboot app image.
 *
 * The output binary is a fixed 4 KiB block laid out on flash as:
 *
 *   [ .main_toc          ]  <- table of contents struct (from src/toc.c)
 *   [ 0xff padding       ]
 *   [ 64B TOC signature  ]  <- appended by sign_firmware.py
 *
 * The unsigned app .bin is pulled in via .incbin by
 * platforms/nuttx/toc/fw_image.c (into section .firmware) so that
 * _app_start / _app_end reflect the real payload size, and the boot
 * signature's runtime absolute address (_boot_sig_addr) can be computed
 * from _app_addr + payload size.
 *
 ****************************************************************************/

/* ed25519 signature size */
_sig_size = 64;

/* Reserved size on flash for the standalone TOC block (TOC + padding +
 * signature).
 */
_toc_block_size = 0x1000;

/* Absolute flash address of the app payload after the prepended TOC
 * block.
 */
_app_addr = 0x30020000 + _toc_block_size;

MEMORY
{
	progmem (r) : ORIGIN = 0, LENGTH = 8M
}

OUTPUT_ARCH(arm)

EXTERN(_main_toc)

SECTIONS
{
	/* TOC and TOC signature */
	.toc : {
		_toc_start = ABSOLUTE(.);
		KEEP(*(.main_toc));
		FILL(0xff);
		. = _toc_block_size - _sig_size;
		_toc_end = ABSOLUTE(.);
	} > progmem

	_toc_sig_start = ABSOLUTE(.);

	.toc_sig (NOLOAD) : {
		KEEP(*(.main_toc_sig));
	} > progmem

	_toc_sig_end = _toc_sig_start + _sig_size;

	/* PX4 image payload placeholder. */
	.app (NOLOAD) : {
		_app_start = ABSOLUTE(.);
		*(.firmware)
		. = ALIGN(4);
		_app_end = ABSOLUTE(.);
	} > progmem

	/* Absolute address of the px4 image signature. */
	_boot_sig_addr = _app_addr + (_app_end - _app_start);
	_boot_sig_end  = _boot_sig_addr + _sig_size;
}
