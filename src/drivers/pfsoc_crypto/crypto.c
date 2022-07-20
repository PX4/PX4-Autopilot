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
 * Implementation for PF SOC FPGA crypto engine.
 *
 */

#include <inttypes.h>
#include <stdbool.h>

#include <px4_platform_common/crypto_backend.h>
#include <lib/crypto/monocypher/src/optional/monocypher-ed25519.h>

/*
 * For now, this is just a dummy up/down counter for tracking open/close calls
 */
static int crypto_open_count = 0;

typedef struct {
	uint8_t nonce[24];
	uint64_t ctr;
} chacha20_context_t;

void crypto_init()
{
	keystore_init();
}

crypto_session_handle_t crypto_open(px4_crypto_algorithm_t algorithm)
{
	crypto_session_handle_t ret;
	ret.keystore_handle = keystore_open();

	if (keystore_session_handle_valid(ret.keystore_handle)) {
		ret.algorithm = algorithm;
		ret.handle = ++crypto_open_count;

	} else {
		ret.handle = 0;
		ret.context = NULL;
		ret.algorithm = CRYPTO_NONE;
		return ret;
	}

	switch (algorithm) {
	default:
		ret.context = NULL;
	}

	return ret;
}

void crypto_close(crypto_session_handle_t *handle)
{

	// Not open?
	if (!crypto_session_handle_valid(*handle)) {
		return;
	}

	crypto_open_count--;
	handle->handle = 0;
	keystore_close(&handle->keystore_handle);
	handle->context = NULL;
}

bool crypto_signature_check(crypto_session_handle_t handle,
			    uint8_t  key_index,
			    const uint8_t  *signature,
			    const uint8_t *message,
			    size_t message_size)
{
	bool ret = false;
	size_t keylen = 0;
	uint8_t public_key[32];

	if (crypto_session_handle_valid(handle)) {
		keylen = keystore_get_key(handle.keystore_handle, key_index, public_key, sizeof(public_key));
	}

	if (keylen == 0) {
		return false;
	}

	switch (handle.algorithm) {
	case CRYPTO_ED25519:
		ret = crypto_ed25519_check(signature, public_key, message, message_size) == 0;
		break;

	default:
		ret = false;
	}

	return ret;
}

bool crypto_encrypt_data(crypto_session_handle_t handle,
			 uint8_t  key_idx,
			 const uint8_t *message,
			 size_t message_size,
			 uint8_t *cipher,
			 size_t *cipher_size)
{

	bool ret = false;

	if (!crypto_session_handle_valid(handle)) {
		return ret;
	}

	switch (handle.algorithm) {
	default:
		break;
	}

	return ret;
}

bool crypto_generate_key(crypto_session_handle_t handle,
			 uint8_t idx, bool persistent)
{
	bool ret = false;

	switch (handle.algorithm) {
	default:
		break;
	}

	if (ret && persistent) {
		/* Store the generated key persistently */
	}

	return ret;
}

bool crypto_get_encrypted_key(crypto_session_handle_t handle,
			      uint8_t key_idx,
			      uint8_t *key,
			      size_t *max_len,
			      uint8_t encryption_key_idx)
{
	bool ret = false;
	return ret;
}


bool crypto_get_nonce(crypto_session_handle_t handle,
		      uint8_t *nonce,
		      size_t *nonce_len)
{
	switch (handle.algorithm) {
	default:
		*nonce_len = 0;
	}

	return true;
}

size_t crypto_get_min_blocksize(crypto_session_handle_t handle, uint8_t key_idx)
{
	size_t ret;

	switch (handle.algorithm) {
	default:
		ret = 1;
	}

	return ret;
}
