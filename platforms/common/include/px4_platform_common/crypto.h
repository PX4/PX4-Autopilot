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

#pragma once

#ifdef PX4_CRYPTO

#include <stdbool.h>
#include <stdint.h>

#include <px4_random.h>
#include <px4_platform_common/crypto_algorithms.h>
#include <px4_platform_common/sem.h>
#include "crypto_backend_definitions.h"

/*
 * Crypto API interface class
 */

class PX4Crypto
{
public:
	/*
	 * Constructor & destructor
	 */

	PX4Crypto();
	~PX4Crypto();

	/*
	 * Class function for crypto api initialization, called only once at
	 * boot
	 */

	static void px4_crypto_init(void);

	/*
	 * Open crypto API for a specific algorithm
	 * algorithm: The crypto algorithm to be used
	 * returns true on success, false on failure
	 */

	bool open(px4_crypto_algorithm_t algorithm);

	/*
	 * Close the crypto API. Optional, it is also closed by destructing the
	 * interface object
	 */

	void close();

	/*
	 * Keystore access functions
	 */

	/*
	 * Generate a single random key for symmetric-key encryption
	 *
	 * idx: the index in keystore where the key will be stored
	 * persistent: whether the key need to be stored persistently
	 * returns true on success, false on failure
	 */

	bool generate_key(uint8_t idx,
			  bool persistent);

	/*
	 * Generate a key pair for asymmetric-key encryption
	 *
	 * key_size: size of the key in bytes
	 * key_idx: the keys will be stored in this index in the keystore
	 * persistent: whether the keys need to be stored persistently
	 * returns true on success, false on failure
	*/

	bool generate_keypair(size_t key_size,
			      uint8_t key_idx,
			      bool persistent);


	/*
	 * Re-create or set nonce.
	 *
	 * A nonce or intialization vector value for the selected algortithm is
	 * automatically generated when the crypto session is opened. If needed, the
	 * nonce can be set by this function.
	 * If this is called with NULL pointer, a new nonce is automatically random
	 * generated
	 */

	bool renew_nonce(const uint8_t *nonce, size_t nonce_size);


	/*
	 * Get current crypto session nonce
	 *
	 * This function returns the current nonce for the session
	 * If the "nonce" is NULL, only nonse legth will be provided
	 * nonce: pointer to the buffer where the nonce will be written
	 * nonce_len: length of the current nonce vector for the session
	 * returns true on success, false on failure
	 */

	bool get_nonce(uint8_t *nonce,
		       size_t *nonce_len);

	/*
	 * Store a key into keystore
	 *
	 * encryption_idx: The key index in keystore to be used in decrypting and
	 *   authenticating the key before storing
	 * key: The pointer to the key
	 * key_idx: Index where the key will be stored in keystore
	*/


	bool set_key(uint8_t encryption_idx,
		     const uint8_t *key,
		     uint8_t key_idx);

	/*
	 * Get a key from keystore. Key can be encrypted
	 *
	 * key_idx: Index of the requested key in the keystore
	 * key: The provided buffer to the key. If NULL, the function only provides
	 *          the length of the key.
	 * key_len: input: the size of the provided "key" buffer.
	            output: the actual size of the key
	 * encryption_key_idx: The key index in keystore to be used for encrypting
	 * returns true on success, false on failure
	 *
	 */

	bool get_encrypted_key(uint8_t key_idx,
			       uint8_t *key,
			       size_t *key_len,
			       uint8_t encryption_key_idx);

	/*
	 * PX4 Crypto API functions
	 */

	/*
	 * Message signing
	 *
	 * key_index: private key index in keystore
	 * signature: pointer to the output signature
	 * message: pointer to the data to be signed
	 * message_size: size of the message in bytes
	*/

	bool sign(uint8_t key_index,
		  uint8_t *signature,
		  const uint8_t *message,
		  size_t message_size);

	/*
	 * Verify signature
	 *
	 * key_index: public key index in keystore
	 * signature: pointer to the signature
	 * message: pointer to the data to be verified
	 * message_size: size of the message in bytes
	*/

	bool signature_check(uint8_t key_index,
			     const uint8_t *signature,
			     const uint8_t *message,
			     size_t message_size);


	/*
	 * Encrypt data. This always supports encryption in-place
	 *
	 * key_index: key index in keystore
	 * message: pointer to the message
	 * message_size: size of the message in bytes
	 * cipher: pointer to a buffer for encrypted data
	 * cipher_size: size of the buffer reserved for cipher and actual cipher length
	 *   after the encryption
	 * mac: pointer to the buffer for authentication code
	 * mac_size: pointer to the size of the authentication code buffer
	 *   the size is updated to match the actual size after encryption
	 * returns true on success, false on failure
	 */

	bool encrypt_data(uint8_t key_index,
			  const uint8_t *message,
			  size_t message_size,
			  uint8_t *cipher,
			  size_t *cipher_size,
			  uint8_t *mac,
			  size_t *mac_size);

	/*
	 * Decrypt data. This always supports decryption in place
	 *
	 * De-crypts the given cipher using the given nonce and key index.
	 * handle: session handle, returned by oepn
	 * key_index: index to the key used for decryption
	 * cipher: the ciphertext to be decrypted
	 * mac: pointer to the buffer for authentication code
	 * mac_size: size of the authentication code buffer
	 * message: output buffer given by the client
	 * message_size: in: size of "message" buffer, out: actual result size
	 * returns
	 */
	bool decrypt_data(uint8_t key_index,
			  const uint8_t *cipher,
			  size_t cipher_size,
			  const uint8_t *mac,
			  size_t mac_size,
			  uint8_t *message,
			  size_t *message_size);

	size_t get_min_blocksize(uint8_t key_idx);

	static int crypto_ioctl(unsigned int cmd, unsigned long arg);

private:
	crypto_session_handle_t _crypto_handle;
	static px4_sem_t _lock;
	static bool _initialized;
	static void lock() { do {} while (px4_sem_wait(&PX4Crypto::_lock) != 0); }
	static void unlock() { px4_sem_post(&PX4Crypto::_lock); }
};

#endif
