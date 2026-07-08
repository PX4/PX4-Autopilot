/****************************************************************************
 *
 *   Copyright (c) 2021 Technology Innovation Institute. All rights reserved.
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

#include <stdbool.h>
#include "image_toc.h"
#include "hw_config.h"

#ifdef BOOTLOADER_USE_SECURITY

#include <px4_platform_common/crypto_backend.h>
#include <stddef.h>
#include <stdint.h>
#include <nuttx/arch.h>

/* sw_crypto unconditionally references px4_get_secure_random from the
 * XCHACHA20 path in crypto_open(). The bootloader only verifies
 * ed25519 signatures and has no reason to generate randomness, but
 * the linker still demands the symbol. Supplying it here avoids
 * pulling NuttX's random pool (CONFIG_CRYPTO_RANDOM_POOL) into the
 * bootloader build just to resolve an unreachable call site.
 *
 * The implementation panics rather than returning zeros: if anyone
 * ever enables bootloader-side encryption without also wiring up a
 * real RNG, silently handing out predictable bytes would be a
 * serious security bug. up_assert() makes that mistake loud.
 */
size_t px4_get_secure_random(uint8_t *out, size_t outlen)
{
	(void)out;
	(void)outlen;

	up_assert(__FILE__, __LINE__);

	return 0;
}

bool verify_app(uint16_t idx, const image_toc_entry_t *toc_entries)
{
	volatile uint8_t *app_signature_ptr = NULL;
	volatile size_t len = 0;
	bool ret;

	uint8_t sig_idx = toc_entries[idx].signature_idx;
	uint8_t sig_key = toc_entries[idx].signature_key;
	crypto_session_handle_t handle = crypto_open(BOOTLOADER_SIGNING_ALGORITHM);
	app_signature_ptr = (volatile uint8_t *)toc_entries[sig_idx].start;
	len = (size_t)toc_entries[idx].end - (size_t)toc_entries[idx].start;

	ret =  crypto_signature_check(handle, sig_key, (const uint8_t *)app_signature_ptr,
				      (const uint8_t *)toc_entries[idx].start, len);

	crypto_close(&handle);
	return ret;
}

bool decrypt_app(uint16_t idx, const image_toc_entry_t *toc_entries)
{
	/*
	 * Not implemented yet.
	 */
	return false;
}

#endif //BOOTLOADER_USE_SECURITY
