/****************************************************************************
 *
 *   Copyright (c) 2020 Technology Innovation Institute. All rights reserved.
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
 * @file crypto.c
 *
 * Wrapper for the monocypher crypto
 *
 */

#include <inttypes.h>
#include <stdbool.h>

#include <px4_platform_common/arch_crypto.h>
#include <lib/crypto/monocypher/src/optional/monocypher-ed25519.h>

/* room for 16 keys */
#define KEY_CACHE_LEN 16

#ifndef SECMEM_ALLOC
#define SECMEM_ALLOC malloc
#endif

typedef struct {
	size_t key_size;
	uint8_t *key;
} volatile_key_t;

static volatile_key_t key_cache[KEY_CACHE_LEN];

/* Clear key cache */
static void clear_key_cache(void)
{
	for (int i = 0; i < KEY_CACHE_LEN ; i++) {
		key_cache[i].key = NULL;
		key_cache[i].key_size = 0;
	}
}

/* Retrieve a direct pointer to the cached temporary/public key */
static const uint8_t *arch_get_key_ptr(uint8_t key_idx,
				       size_t *len)
{
	uint8_t *ret = key_cache[key_idx].key;

	/* if the key doesn't exist in the key cache, try to read it in there from keystore */
	if (ret == NULL) {

		/* First check if the key exists in the keystore and retrieve its length */
		*len = arch_keystore_get_key(key_idx, NULL, 0);

		if (*len > 0) {

			/* Allocate memory for the key in the cache */
			ret = SECMEM_ALLOC(*len);

			/* Retrieve the key from the keystore */
			if (ret) {
				if (arch_keystore_get_key(key_idx, ret, *len) > 0) {
					/* Success, store the key in cache */
					key_cache[key_idx].key_size = *len;
					key_cache[key_idx].key = ret;

				} else {
					/* key retrieval failed, free the memory */
					free(ret);
				}
			}
		}
	}

	*len = key_cache[key_idx].key_size;

	return ret;
}


void arch_crypto_init()
{
	arch_keystore_init();
	clear_key_cache();
}

arch_crypto_session_handle_t arch_crypto_open(px4_crypto_algorithm_t algorithm)
{
	arch_crypto_session_handle_t ret;
	ret.algorithm = algorithm;
	ret.handle = 1;
	return ret;
}

void arch_crypto_close(arch_crypto_session_handle_t *handle)
{
	handle->handle = 0;
}

bool arch_signature_check(arch_crypto_session_handle_t handle,
			  uint8_t  key_index,
			  const uint8_t  *signature,
			  const uint8_t *message,
			  size_t message_size)
{
	bool ret;
	size_t keylen;
	const uint8_t *public_key = arch_get_key_ptr(key_index, &keylen);

	if (keylen == 0) {
		return false;
	}

	switch (handle.algorithm) {
	case CRYPTO_ED25519:
		ret = crypto_ed25519_check(signature, public_key, message, message_size) == 0 ? true : false;
		break;

	default:
		ret = false;
	}

	return ret;
}
