/****************************************************************************
 *
 *  Copyright (C) 2012-2019 PX4 Development Team. All rights reserved.
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
 * @file test_crypto.cpp
 * Tests for crypto interfaces.
 */

#include <stdio.h>
#include <unit_test.h>
#include <px4_platform_common/crypto.h>
#include <px4_random.h>
#include <stdint.h>

// Input for encryption (e.g. plaintext message)
const uint32_t test_input[256 / 4] = {
	0xf625d5fc, 0x0253fd3b, 0xe427adf4, 0x824e292e, 0x8f903040, 0xf89a67b3, 0x1ab7cfe4, 0x9a23929c,
	0xb25f43bb, 0xa009a087, 0xa8e13de5, 0x58fc0026, 0x012e9c22, 0xf33be7a2, 0x47e0ce21, 0x596fc612,
	0x6e2d5ece, 0xcdb9aa7c, 0xa2cd3538, 0xac4eb414, 0x51236c03, 0x61704a6f, 0x1992d2a0, 0xb68d766c,
	0xbbfa2886, 0x04e09c5d, 0x1508d501, 0x15f4c9e4, 0xb7f0ce5c, 0x8de0f57d, 0xdd1c9335, 0xb773af67,
	0x8be4289e, 0x98a2cdf3, 0xf6b47000, 0x0d5bca6f, 0x31d062ab, 0x9856426a, 0x00fd1376, 0xa21e101e,
	0xe9907670, 0x61bde6d8, 0x8f2edc79, 0x21839eca, 0x868eb807, 0x7109f785, 0xd5781dd3, 0x87f578a3,
	0x3f445f25, 0x657c2d3b, 0x7d4a8374, 0x7a932bcc, 0xfa0ac524, 0xda203efd, 0x4ff8b7ab, 0x04b3ed62,
	0x5f06f9fe, 0x42f265cd, 0x736487b1, 0x17ffc170, 0x45c37f4f, 0x82f5aeb7, 0xad5906fb, 0xb98b6825,
};

// Another input for encryption (e.g. test key)
static const uint32_t test_key[256 / 4] = {
	0x60b6c82d, 0xb51c908f, 0xe67c05cb, 0x1f728c4f, 0x29e4c6b2, 0x5ecb5c27, 0x618f4b47, 0x8be711cd,
	0xf537275d, 0xbfe729db, 0x17e32ac7, 0x832c5e15, 0x1324c3a0, 0xe38a6e44, 0x7b082c31, 0xe26fed61,
	0xa43bafa2, 0xa3183b71, 0xbe3a89fc, 0x7e02d124, 0x5569dc48, 0xc5aeb667, 0x5f6eee55, 0xed528371,
	0x65c15683, 0xfd264695, 0x4f64ea7b, 0x5bdb5278, 0x5780405b, 0x6e0bef86, 0xd847e9ce, 0x1fbcf9e1,
	0x6405ef96, 0x44a143da, 0x6f7f9945, 0xd1355623, 0x98430078, 0xa2e9973a, 0xca5e934a, 0x4e382f89,
	0x13fce2b8, 0x41108c34, 0xc0794ce2, 0x054d56a3, 0x872eda91, 0x511795db, 0x60069323, 0x9607502a,
	0xb96dcf7b, 0x94b77bc0, 0x7a39948a, 0x7d6d3ab0, 0xf1b50de7, 0xb5d201e3, 0xa60e0754, 0x798dcdb4,
	0x4fa58770, 0x8b7e833f, 0x57e0dc52, 0x4f321168, 0x92b7e26c, 0xb3c0e5e3, 0xc0199112, 0x69ea7691,
};

// Another input for encryption (e.g. nonce)
static uint32_t test_nonce[64 / 4] = {
	0x9fb7a36e, 0xdafd7f9d, 0x1848d166, 0x398b3c9d, 0xcae67c98, 0x9b1d77e3, 0xbffed534, 0x02dffc3a,
};

// Output of the encryption
static uint32_t test_output[2048 / 4];

static const uint8_t test_color[4] = {0xef, 0xbe, 0xad, 0xde};

class CryptoTest : public UnitTest
{
public:
	virtual bool run_tests();

	bool test_aes();
	bool test_chacha();

private:
	void color_output(void *buf, size_t sz);
	bool check_color(const void *p1, const void *p2);
	void dump_data(const void *d, size_t sz);
	void dump_data32(const void *d, size_t sz);
};

static bool output_error(PX4Crypto &crypto, const char *str)
{
	printf("%s\n", str);
	crypto.close();
	return false;
}

#define ERROUT(x) return output_error(crypto, x);

bool CryptoTest::test_aes()
{
	bool ret = true;
	const uint8_t *key = (uint8_t *)test_key;

	const uint8_t *plaintext = (uint8_t *)test_input;
	uint8_t *enc_data = (uint8_t *)&test_output[512 / 4];
	uint8_t *dec_data = (uint8_t *)&test_output[1024 / 4];
	const void *outbuf_end = (uint8_t *)&test_output[sizeof(test_output) / 4];
	PX4Crypto crypto;

	if (!crypto.open(CRYPTO_AES)) {
		ERROUT("Crypto open failed");
	}

	if (!crypto.generate_key(4, false)) {
		ERROUT("Generate key failed");
	}

	color_output(test_output, sizeof(test_output));

	/* Encrypt data (in-place) */

	uint32_t mac[4];
	size_t mac_size = sizeof(mac);
	size_t out;

	crypto.renew_nonce((uint8_t *)test_nonce, 12);

	/* Encrypt */

	memcpy(enc_data, plaintext, 247);

	out = 247;
	ret = crypto.encrypt_data(
		      4,
		      enc_data,
		      out,
		      enc_data,
		      &out,
		      (uint8_t *)mac,
		      &mac_size);

	if (!ret) {
		ERROUT("Encrypt failed");
	}

	/* Check for memory corruption */

	if (!check_color(test_output, enc_data) ||
	    !check_color(enc_data + 256, outbuf_end)) {
		dump_data32(enc_data - 32, 512);
		ERROUT("Memory corruption in encryption");
	}

	if (out != 247) {
		ERROUT("Encrypt: output size mismatch");
	}

	/* Decrypt data */

	crypto.renew_nonce((const uint8_t *)test_nonce, 12);

	ret = crypto.decrypt_data(
		      4,
		      enc_data,
		      out,
		      (uint8_t *)mac,
		      mac_size,
		      dec_data,
		      &out);

	if (!ret) {
		ERROUT("Decrypt failed");
	}

	if (!check_color(enc_data + 256, dec_data) ||
	    !check_color(dec_data + 256, outbuf_end)) {
		dump_data(dec_data - 16, 256 + 32);
		ERROUT("Memory corruption in decryption");
	}

	if (out != 247) {
		ERROUT("Decrypt: output size mismatch");
	}

	if (memcmp(test_input, dec_data, out)) {
		ERROUT("Data doesn't match");
	}

	crypto.close();
	return true;
}

bool CryptoTest::test_chacha()
{
	bool ret = true;
	const uint8_t *key = (uint8_t *)test_key;

	const uint8_t *plaintext = (uint8_t *)test_input;
	uint8_t *enc_data = (uint8_t *)&test_output[512 / 4];
	uint8_t *dec_data = (uint8_t *)&test_output[1024 / 4];
	const void *outbuf_end = (uint8_t *)&test_output[sizeof(test_output) / 4];
	PX4Crypto crypto;

	if (!crypto.open(CRYPTO_XCHACHA20)) {
		ERROUT("Crypto open failed");
	}

	if (!crypto.generate_key(4, false)) {
		ERROUT("Generate key failed");
	}

	color_output(test_output, sizeof(test_output));

	/* Encrypt data (in-place) */

	uint32_t mac[4];
	size_t mac_size = sizeof(mac);
	size_t out;

	crypto.renew_nonce((const uint8_t *)test_nonce, 24);

	/* Encrypt */

	memcpy(enc_data, plaintext, 256);

	out = 256;
	ret = crypto.encrypt_data(
		      4,
		      enc_data,
		      out,
		      enc_data,
		      &out,
		      (uint8_t *)mac,
		      &mac_size);

	if (!ret) {
		ERROUT("Encrypt failed");
	}

	if (out != 256) {
		ERROUT("Encrypt: output size mismatch");
	}

	/* Check for memory corruption */

	if (!check_color(test_output, enc_data) ||
	    !check_color(enc_data + 256, outbuf_end)) {
		ERROUT("Memory corruption in encryption");
	}

	/* Decrypt data */

	/* Use the same generated key, but re-initialize */

	crypto.renew_nonce((const uint8_t *)test_nonce, 24);

	out = 256;
	ret = crypto.decrypt_data(
		      4,
		      enc_data,
		      out,
		      (uint8_t *)mac,
		      mac_size,
		      dec_data,
		      &out);

	if (!ret) {
		ERROUT("Decrypt failed");
	}

	if (out != 256) {
		ERROUT("Decrypt: output size mismatch");
	}

	if (!check_color(enc_data + 256, dec_data) ||
	    !check_color(dec_data + 256, outbuf_end)) {
		dump_data(dec_data - 16, 256 + 32);
		ERROUT("Memory corruption in decryption");
	}

	if (memcmp(test_input, dec_data, 256)) {
		ERROUT("Data doesn't match");
	}

	crypto.close();
	return true;
}

bool CryptoTest::run_tests()
{
	ut_run_test(test_aes);
	ut_run_test(test_chacha);

	return (_tests_failed == 0);
}

void CryptoTest::dump_data(const void *d, size_t sz)
{
	const uint8_t *data = (const uint8_t *)d;

	for (size_t i = 0; i < sz; i += 16) {
		for (size_t j = 0; j < 16 && i + j < sz; j++) {
			printf(" 0x%02x,", data[i + j]);
		}

		printf("\n");
	}
}

void CryptoTest::dump_data32(const void *d, size_t sz)
{
	const uint32_t *data = (const uint32_t *)d;
	const size_t sz_w = sz / sizeof(uint32_t);

	for (size_t i = 0; i < sz_w; i += 8) {
		for (size_t j = 0; j < 8 && i + j < sz_w; j++) {
			printf(" 0x%08x,", data[i + j]);
		}

		printf("\n");
	}
}

void CryptoTest::color_output(void *buf, size_t sz)
{
	uint8_t *p = (uint8_t *)buf;
	uint8_t *end = &p[sz];

	for (; p < end; p++) {
		*p = test_color[(uintptr_t)p & 0x3];
	}
}

bool CryptoTest::check_color(const void *p1, const void *p2)
{
	uint8_t *p = (uint8_t *)p1;
	uint8_t *end = (uint8_t *)p2;
	bool res = true;

	for (; res && p < end; p++) {
		if (*p != test_color[(uintptr_t)p & 0x3]) {
			res = false;
		}
	}

	return res;
}

ut_declare_test_c(test_crypto, CryptoTest)
