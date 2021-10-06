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

#if defined(PX4_CRYPTO)

#include <px4_platform_common/crypto.h>
#include <px4_platform_common/crypto_backend.h>
#include <sys/boardctl.h>

bool PX4Crypto::_initialized = false;

void PX4Crypto::px4_crypto_init()
{
	PX4Crypto::_initialized = true;
}

PX4Crypto::PX4Crypto()
{
	// Initialize an empty handle
	crypto_session_handle_init(&_crypto_handle);
}

PX4Crypto::~PX4Crypto()
{
	close();
}

bool PX4Crypto::open(px4_crypto_algorithm_t algorithm)
{
	// HW specific crypto already open? Just close before proceeding
	close();

	// Open the HW specific crypto handle
	cryptoiocopen_t data = {algorithm, &_crypto_handle};
	boardctl(CRYPTOIOCOPEN, reinterpret_cast<unsigned long>(&data));

	return crypto_session_handle_valid(_crypto_handle);
}

void PX4Crypto::close()
{
	if (!crypto_session_handle_valid(_crypto_handle)) {
		return;
	}

	boardctl(CRYPTOIOCCLOSE, reinterpret_cast<unsigned long>(&_crypto_handle));
}

bool PX4Crypto::encrypt_data(uint8_t  key_index,
			     const uint8_t *message,
			     size_t message_size,
			     uint8_t *cipher,
			     size_t *cipher_size)
{
	cryptoiocencrypt_t data = {&_crypto_handle, key_index, message, message_size, cipher, cipher_size, false};
	boardctl(CRYPTOIOCENCRYPT, reinterpret_cast<unsigned long>(&data));
	return data.ret;
}

bool  PX4Crypto::generate_key(uint8_t idx,
			      bool persistent)
{
	cryptoiocgenkey_t data = {&_crypto_handle, idx, persistent, false};
	boardctl(CRYPTOIOCGENKEY, reinterpret_cast<unsigned long>(&data));
	return data.ret;
}

bool  PX4Crypto::get_nonce(uint8_t *nonce,
			   size_t *nonce_len)
{
	cryptoiocgetnonce_t data = {&_crypto_handle, nonce, nonce_len, false};
	boardctl(CRYPTOIOCGETNONCE, reinterpret_cast<unsigned long>(&data));
	return data.ret;
}

bool  PX4Crypto::get_encrypted_key(uint8_t key_idx,
				   uint8_t *key,
				   size_t *key_len,
				   uint8_t encryption_key_idx)
{
	cryptoiocgetkey_t data = {&_crypto_handle, key_idx, key, key_len, encryption_key_idx, false};
	boardctl(CRYPTOIOCGETKEY, reinterpret_cast<unsigned long>(&data));
	return data.ret;
}

size_t PX4Crypto::get_min_blocksize(uint8_t key_idx)
{
	cryptoiocgetblocksz_t data = {&_crypto_handle, key_idx, 0};
	boardctl(CRYPTOIOCGETBLOCKSZ, reinterpret_cast<unsigned long>(&data));
	return data.ret;
}

#endif
