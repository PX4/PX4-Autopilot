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

#include <stdint.h>

/* List of all the supported crypto algorithms in PX4 */

typedef enum {
	CRYPTO_NONE = 0,
	CRYPTO_ED25519 = 1,
	CRYPTO_XCHACHA20 = 2,
	CRYPTO_AES = 3,
	CRYPTO_RSA_OAEP = 4,
	CRYPTO_ECDSA_P256 = 5,
	CRYPTO_RSA_SIG = 6,    /* openssl dgst -sha256 -sign */
} px4_crypto_algorithm_t;

/* Define the expected size of the signature for signing algorithms */

#define PX4_SIGNATURE_SIZE(x) PX4_SIGNATURE_SIZE_(x)
#define PX4_SIGNATURE_SIZE_(x) PX4_SIGNATURE_SIZE_##x

#define PX4_SIGNATURE_SIZE_CRYPTO_ED25519 64
#define PX4_SIGNATURE_SIZE_CRYPTO_ECDSA_P256 64 /* Raw R+S, both 32 bytes */
#define PX4_SIGNATURE_SIZE_CRYPTO_RSA_SIG 256   /* Assume key length of 2048 bits */
