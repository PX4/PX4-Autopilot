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

#if defined(__cplusplus)
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <px4_platform_common/crypto_algorithms.h>
#include "crypto_backend_definitions.h"

/*
 * Keystore access functions
 */

void keystore_init(void);
void keystore_deinit(void);

/*
 * Open a session for accessing security keys
 */

keystore_session_handle_t keystore_open(void);

/*
 * Close the keystore session
 * handle: handle to the session to be closed
 */

void keystore_close(keystore_session_handle_t *handle);

/*
 * Get a key from keystore
 * handle: a handle to an open keystore
 * idx: key index in keystore
 * key_buf: output buffer for the key
 * key_buf_size: size of the provided output buffer
 * returns the size of the key in keystore
 */
size_t keystore_get_key(keystore_session_handle_t handle, uint8_t idx, uint8_t *key_buf, size_t key_buf_size);

/*
 * Store a key persistently into the keystore
 * handle: a handle to an open keystore
 * idx: key index in keystore
 * key: pointer to the key
 * key_size: size of the key
 */
bool keystore_put_key(keystore_session_handle_t handle, uint8_t idx, uint8_t *key, size_t key_size);

/*
 * Modify a key in keystore
 * handle: a handle to an open keystore
 * idx: key index in keystore
 * key_buf: work buffer for the key
 * key_buf_size: size of the provided work buffer
 * cb: callback function to perform the modification of the key
 * arg: pointer to callback custom argument
 */
bool keystore_modify_key(keystore_session_handle_t handle, uint8_t idx, uint8_t *key_buf, size_t key_buf_size,
			 keystore_callback_t cb, void *arg);

/*
 * Architecture specific PX4 Crypto API functions
 */

/*
 * Initialize hw level crypto
 * This has to be called before any other crypto operations
 */

void crypto_init(void);

/*
 * De-initialize hw level crypto
 * This may be called to shut down hw level crypto
 */

void crypto_deinit(void);

/*
 * Open a session for performing crypto functions
 * algorithm: The crypto algorithm to be used in this session
 */

crypto_session_handle_t crypto_open(px4_crypto_algorithm_t algorithm);

/*
 * Close the session for performing crypto operations
 * handle: handle to the session to be closed
 */

void crypto_close(crypto_session_handle_t *handle);

/*
 * Generate a key
 * handle: Open handle for the crypto session. The key will be generated for
 *         the crypto algorithm used by this session
 * idx: The key index, by which the key can be used
 * persistent: if set to "true", the key will be stored into the keystore
 */
bool crypto_generate_key(crypto_session_handle_t handle,
			 uint8_t idx,
			 bool persistent);

/*
 * Get a key from keystore, possibly encrypted
 *
 * handle: an open crypto context; the returned key will be encrypted
 *   according to this context
 * key_idx: Index of the requested key in the keystore
 * key: The provided buffer to the key
 * max_len: the length of the provided key buffer. Returns the actual key size
 * encryption_key_idx: The key index in keystore to be used for encrypting
 * returns true on success, false on failure
 */
bool crypto_get_encrypted_key(crypto_session_handle_t handle,
			      uint8_t key_idx,
			      uint8_t *key,
			      size_t *max_len,
			      uint8_t encryption_key_idx);

/*
 * Re-create or set nonce.
 *
 * A nonce or intialization vector value for the selected algortithm is
 * automatically generated when the crypto session is opened. If needed, the
 * nonce can be set by this function.
 * If this is called with NULL pointer, a new nonce is automatically random
 * generated
 */
bool crypto_renew_nonce(crypto_session_handle_t handle,
			const uint8_t *nonce,
			size_t nonce_size);

/*
 * Get the generated nonce value
 *
 * handle: an open crypto context; the returned nonce is the one associsated
 *   with this context/algorithm
 * nonce: The provided buffer to the key. If NULL, only length is returned
 * nonce_len: the length of the nonce value
 * encryption_key_idx: The key index in keystore to be used for encrypting
 * returns true on success, false on failure
 */
bool crypto_get_nonce(crypto_session_handle_t handle,
		      uint8_t *nonce,
		      size_t *nonce_len);

/*
 * Perform signature check using an open session to crypto
 * handle: session handle, returned by open
 * key_index: index to the key used for signature check
 * message: pointer to the data to be checked
 * message_size: size of the data
 */

bool crypto_signature_check(crypto_session_handle_t handle,
			    uint8_t  key_index,
			    const uint8_t  *signature,
			    const uint8_t *message,
			    size_t message_size);

bool crypto_encrypt_data(crypto_session_handle_t handle,
			 uint8_t  key_index,
			 const uint8_t *message,
			 size_t message_size,
			 uint8_t *cipher,
			 size_t *cipher_size,
			 uint8_t *mac,
			 size_t *mac_size);

/*
 * Returns a minimum data block size on which the crypto operations can be
 *   performed. Performing encryption on sizes which are not multiple of this
 *   are valid, but the output will be padded to the multiple of this value
 *   Input for decryption must be multiple of this value.
 * handle: session handle, returned by open
 * key_idx: key to be used for encryption/decryption
 * returs the block size
 */

size_t crypto_get_min_blocksize(crypto_session_handle_t handle, uint8_t key_idx);

/*
 * Decrypt data. This always supports decryption in place
 *
 * De-crypts the given cipher using the given nonce and key index.
 * handle: session handle, returned by oepn
 * key_index: index to the key used for decryption
 * cipher: the ciphertext to be decrypted
 * nonce: the nonce used for decryption. If NULL, using the previously set nonce
 * nonce_size: size of the given nonce value (note. caller is responsible of giving a suitable nonce for the algorithm)
 * message: output buffer given by the client
 * message_size: in: size of "message" buffer, out: actual result size
 * returns
 */
bool crypto_decrypt_data(crypto_session_handle_t handle,
			 uint8_t key_index,
			 const uint8_t *cipher,
			 size_t cipher_size,
			 const uint8_t *mac,
			 size_t mac_size,
			 uint8_t *message,
			 size_t *message_size);

/* Crypto IOCTLs, to access backend from user space */

#define _CRYPTOIOC(_n)		(_IOC(_CRYPTOIOCBASE, _n))

#define CRYPTOIOCOPEN _CRYPTOIOC(1)
typedef struct cryptoiocopen {
	px4_crypto_algorithm_t algorithm;
	crypto_session_handle_t *handle;
} cryptoiocopen_t;

#define CRYPTOIOCCLOSE _CRYPTOIOC(2)

#define CRYPTOIOCENCRYPT _CRYPTOIOC(3)
typedef struct cryptoiocencrypt {
	crypto_session_handle_t *handle;
	uint8_t  key_index;
	const uint8_t *message;
	size_t message_size;
	uint8_t *cipher;
	size_t *cipher_size;
	uint8_t *mac;
	size_t *mac_size;
	bool ret;
} cryptoiocencrypt_t;

#define CRYPTOIOCGENKEY _CRYPTOIOC(4)
typedef struct cryptoiocgenkey {
	crypto_session_handle_t *handle;
	uint8_t idx;
	bool persistent;
	bool ret;
} cryptoiocgenkey_t;

#define CRYPTOIOCGETNONCE _CRYPTOIOC(5)
typedef struct cryptoiocgetnonce {
	crypto_session_handle_t *handle;
	uint8_t *nonce;
	size_t *nonce_len;
	bool ret;
} cryptoiocgetnonce_t;

#define CRYPTOIOCGETKEY _CRYPTOIOC(6)
typedef struct cryptoiocgetkey {
	crypto_session_handle_t *handle;
	uint8_t key_idx;
	uint8_t *key;
	size_t *max_len;
	uint8_t encryption_key_idx;
	bool ret;
} cryptoiocgetkey_t;

#define CRYPTOIOCGETBLOCKSZ _CRYPTOIOC(7)
typedef struct cryptoiocgetblocksz {
	crypto_session_handle_t *handle;
	uint8_t key_idx;
	size_t ret;
} cryptoiocgetblocksz_t;

#define CRYPTOIOCRENEWNONCE _CRYPTOIOC(8)
typedef struct cryptoiocrenewnonce {
	crypto_session_handle_t *handle;
	const uint8_t *nonce;
	size_t nonce_size;
	size_t ret;
} cryptoiocrenewnonce_t;

#define CRYPTOIOCSIGNATURECHECK _CRYPTOIOC(9)
typedef struct cryptoiocsignaturecheck {
	crypto_session_handle_t *handle;
	uint8_t  key_index;
	const uint8_t  *signature;
	const uint8_t *message;
	size_t message_size;
	size_t ret;
} cryptoiocsignaturecheck_t;

#define CRYPTOIOCDECRYPTDATA _CRYPTOIOC(10)
typedef struct cryptoiocdecryptdata {
	crypto_session_handle_t *handle;
	uint8_t key_index;
	const uint8_t *cipher;
	size_t cipher_size;
	const uint8_t *mac;
	size_t mac_size;
	uint8_t *message;
	size_t *message_size;
	size_t ret;
} cryptoiocdecryptdata_t;

#if defined(__cplusplus)
} // extern "C"
#endif
