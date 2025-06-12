/****************************************************************************
 *
 *   Copyright (c) 2018-2019, 2023 PX4 Development Team. All rights reserved.
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

/**
 * @file imxrt_ocram_initialize.c
 *
 * PX4 fmu-v6xrt RAM startup early startup code.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "arm_internal.h"
#include "imxrt_iomuxc.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

__BEGIN_DECLS
extern const uint64_t _fitcmfuncs;   /* Copy source address in FLASH */
extern uint64_t _sitcmfuncs;         /* Copy destination start address in ITCM */
extern uint64_t _eitcmfuncs;         /* Copy destination end address in ITCM */
extern uint64_t _sdtcm;              /* Copy destination start address in DTCM */
extern uint64_t _edtcm;              /* Copy destination end address in DTCM */
__END_DECLS

/****************************************************************************
 * Name: imxrt_ocram_initialize
 *
 * Description:
 *   Called off reset vector to reconfigure the flexRAM
 *   and finish the FLASH to RAM Copy.
 *   CMakeLists.txt Forces compiler not to use builtin functions using -fno-builtin
 *
 ****************************************************************************/

__EXPORT void imxrt_ocram_initialize(void)
{
	uint32_t regval;
	register uint64_t *src;
	register uint64_t *dest;

	/* FlexRAM Configuration
	 *    F = 64K ITCM
	 *    A = 64K DTCM
	 *    5 = 64K OCRAM
	 *    So 0xFFFFFFAA is
	 *    384K FlexRAM ITCM
	 *    128K FlexRAM DTCM
	 * */

	putreg32(0xFFFFFFAA, IMXRT_IOMUXC_GPR_GPR17);
	regval = getreg32(IMXRT_IOMUXC_GPR_GPR16);
	putreg32(regval | GPR_GPR16_FLEXRAM_BANK_CFG_SEL, IMXRT_IOMUXC_GPR_GPR16);

	/* Copy any necessary code sections from FLASH to ITCM. The process is the
	* same as the code copying from FLASH to RAM above. */
	for (src = (uint64_t *)&_fitcmfuncs, dest = (uint64_t *)&_sitcmfuncs;
	     dest < (uint64_t *)&_eitcmfuncs;) {
		*dest++ = *src++;
	}

	/* Clear .dtcm.  We'll do this inline (vs. calling memset) just to be
	* certain that there are no issues with the state of global variables.
	*/

	for (dest = &_sdtcm; dest < &_edtcm;) {
		*dest++ = 0;
	}
}
