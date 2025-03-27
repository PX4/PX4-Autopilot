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
#include <board_config.h>

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


const uint8_t test_rsa_pubkey_der[294] = {
	0x30, 0x82, 0x01, 0x22, 0x30, 0x0d, 0x06, 0x09, 0x2a, 0x86, 0x48, 0x86, 0xf7, 0x0d, 0x01, 0x01,
	0x01, 0x05, 0x00, 0x03, 0x82, 0x01, 0x0f, 0x00, 0x30, 0x82, 0x01, 0x0a, 0x02, 0x82, 0x01, 0x01,
	0x00, 0xaa, 0x43, 0x61, 0x20, 0xc7, 0x04, 0x42, 0x1b, 0x55, 0xaa, 0x5f, 0x59, 0x70, 0xf0, 0xe8,
	0x47, 0x9b, 0x95, 0xd6, 0x5a, 0xbf, 0x5f, 0x53, 0xd3, 0x97, 0xb8, 0x5e, 0xf0, 0x36, 0x0f, 0x41,
	0x21, 0x9f, 0x62, 0x14, 0xe4, 0x56, 0xa2, 0x9f, 0xe3, 0x19, 0xbe, 0x27, 0xb2, 0x60, 0x18, 0x4f,
	0xed, 0xb5, 0x50, 0x62, 0xb5, 0xc6, 0x61, 0xf7, 0xf1, 0x55, 0x4d, 0xd1, 0xf7, 0x46, 0x57, 0xf6,
	0x81, 0x60, 0x3a, 0xe7, 0x93, 0x41, 0x04, 0xff, 0xe7, 0xc2, 0xd9, 0x59, 0x0d, 0xe4, 0x88, 0x8c,
	0xf9, 0xf7, 0x46, 0xb4, 0xcc, 0xe2, 0x78, 0xc2, 0x13, 0x4b, 0xb4, 0xd1, 0x0a, 0x14, 0xca, 0x35,
	0x25, 0x3f, 0x3c, 0x4f, 0x4f, 0x45, 0x8a, 0xe3, 0xc2, 0x4c, 0x04, 0x04, 0xc9, 0x99, 0x20, 0xb7,
	0x53, 0xe6, 0x5a, 0x6b, 0x6c, 0x5d, 0x99, 0xee, 0x1d, 0xb0, 0x85, 0x0d, 0x0b, 0xee, 0x2a, 0xa0,
	0x0d, 0x80, 0x8c, 0xf2, 0x7e, 0x4a, 0x49, 0xa4, 0xa4, 0x3e, 0xd0, 0x4a, 0xf7, 0x4e, 0x16, 0x0e,
	0xe3, 0xc3, 0xb8, 0x83, 0xd7, 0x5d, 0xc9, 0x1e, 0x8c, 0x7d, 0x48, 0x62, 0x9c, 0x18, 0x4c, 0x28,
	0xde, 0xc0, 0x2a, 0xcc, 0x3d, 0xa6, 0xa2, 0x79, 0xfc, 0x1c, 0xc6, 0x1e, 0xdf, 0xce, 0x8b, 0x62,
	0xfd, 0xdd, 0x76, 0x5f, 0x42, 0xb2, 0x67, 0xb0, 0x42, 0x73, 0x93, 0xf1, 0x2f, 0x02, 0x98, 0xed,
	0x71, 0x00, 0x2a, 0x99, 0x2a, 0x65, 0x93, 0x54, 0x82, 0xf5, 0x0c, 0x8f, 0x3f, 0x50, 0x51, 0x06,
	0xf6, 0x35, 0xc4, 0x63, 0x06, 0x96, 0x14, 0xcf, 0x68, 0xcc, 0x5d, 0x34, 0x61, 0x3a, 0x5d, 0xde,
	0x74, 0xe4, 0x68, 0x7f, 0xbb, 0x63, 0x86, 0x51, 0x7e, 0xe4, 0xf7, 0x5e, 0x30, 0xb9, 0xf5, 0x59,
	0x8a, 0xa5, 0xc6, 0xe4, 0x01, 0x20, 0x04, 0x3e, 0xb4, 0xaf, 0xe5, 0x13, 0x86, 0x6a, 0xff, 0xeb,
	0xab, 0x02, 0x03, 0x01, 0x00, 0x01,
};

const uint8_t test_data_rsa_signature[256] = {
	0x43, 0x4c, 0x6d, 0xdf, 0x91, 0x69, 0xc3, 0x69, 0x5e, 0x0a, 0x83, 0xa7, 0x88, 0x70, 0x77, 0xf4,
	0xe6, 0xf1, 0xb7, 0x69, 0x9c, 0x44, 0x6d, 0x47, 0x4f, 0x0f, 0x48, 0xff, 0x83, 0xb4, 0xf4, 0xa8,
	0x38, 0x02, 0xc5, 0x3c, 0xa8, 0x98, 0xe6, 0x54, 0x2b, 0xab, 0x89, 0xb9, 0x02, 0x7b, 0xaa, 0x89,
	0x3a, 0x91, 0x25, 0x7a, 0x00, 0x46, 0xfc, 0x96, 0x59, 0xe2, 0x89, 0x4a, 0x49, 0xf1, 0xa9, 0x0e,
	0xf0, 0xc7, 0x8e, 0xf2, 0x80, 0xcd, 0x54, 0x80, 0x02, 0xed, 0xbe, 0x1a, 0x2f, 0x6e, 0x21, 0xdb,
	0xdc, 0x41, 0x50, 0xa6, 0x8e, 0x78, 0x27, 0xaf, 0x19, 0x7b, 0xa0, 0xbf, 0x23, 0xb0, 0x8e, 0xd1,
	0x31, 0xd2, 0x3f, 0x12, 0x19, 0xa7, 0xf7, 0x5b, 0x3a, 0x97, 0xab, 0xc9, 0x42, 0x06, 0xe3, 0xe3,
	0x28, 0x27, 0x0b, 0x8b, 0xf2, 0xea, 0xef, 0x25, 0xe0, 0x3c, 0x7a, 0x92, 0xa5, 0x84, 0x17, 0x81,
	0x0a, 0x18, 0xf0, 0xe2, 0x8a, 0xd1, 0x62, 0x84, 0x17, 0x81, 0x5d, 0x58, 0x5d, 0xc6, 0x0b, 0x9b,
	0x22, 0x16, 0x95, 0xdf, 0xb6, 0x09, 0x2e, 0xe6, 0xc2, 0x57, 0x11, 0xcb, 0xa1, 0x45, 0x18, 0x63,
	0x27, 0x4e, 0x51, 0x5b, 0xdd, 0x46, 0x1f, 0x74, 0x55, 0x4a, 0x41, 0x22, 0xdc, 0x95, 0xe9, 0x41,
	0xad, 0x6f, 0x35, 0x17, 0x0d, 0x04, 0x3a, 0x8f, 0x98, 0x4a, 0x28, 0x9f, 0xab, 0x0d, 0xf1, 0x2a,
	0x3b, 0x45, 0xe3, 0x28, 0xb5, 0x48, 0x62, 0xe9, 0xa4, 0x35, 0x60, 0x62, 0x8c, 0xbf, 0xce, 0xb7,
	0x76, 0xfd, 0x7b, 0x0b, 0xb6, 0x57, 0x4d, 0x21, 0x2f, 0xff, 0x71, 0xd6, 0xf1, 0x55, 0x9b, 0xc3,
	0x41, 0x1e, 0x3b, 0x43, 0x88, 0x27, 0x79, 0x13, 0x8e, 0xdf, 0x6b, 0xb3, 0x00, 0x8d, 0x7a, 0xa3,
	0xb1, 0x05, 0xe6, 0x29, 0x4f, 0x43, 0x08, 0x87, 0x2c, 0x95, 0x3b, 0x6f, 0x73, 0x4f, 0xf8, 0xd4,
};

// Output of the encryption
static uint32_t test_output1[2048 / 4];
static uint32_t test_output2[2048 / 4];

static const uint8_t test_color[4] = {0xef, 0xbe, 0xad, 0xde};

static uint32_t rsa_sig[] = {
	0x8ad828c5, 0x7f38a88e, 0x8cd2ea5d, 0xb7b879f0, 0xe40f7fb9, 0x38b63aad, 0xa65cd35b, 0x48762e55,
	0x60544615, 0xf5c97d41, 0xb11661dd, 0x57aadc70, 0xa484687b, 0x329fbde0, 0xb49aa5a6, 0xb77d944b,
	0xe44ceef9, 0xdec23633, 0x624ec5c0, 0xee80b11f, 0xc1bf23f6, 0x0f5179a2, 0xed23a7f0, 0xca15f9cd,
	0xdfde03ca, 0xf969102d, 0x55747617, 0xb8f8c10c, 0xf2593e3a, 0xd7a4741f, 0xe87c87be, 0xae66829f,
	0x3d82069a, 0x0edb316a, 0x9f125c10, 0xbe98f1c1, 0x1fe76e6f, 0x38999bf3, 0xed4f6f7f, 0x80390541,
	0x0ca57ab5, 0x7ee0422b, 0xbf00044f, 0x23c396a3, 0x78465678, 0xfea6d77f, 0xefad237f, 0x363c889b,
	0xc841ddae, 0x6586b9b7, 0xcbf66c5d, 0x831cc395, 0xd0078a79, 0xec833561, 0x261f91eb, 0x2f640291,
	0x9303ebe0, 0x8414b231, 0xbeec5418, 0x088cc7bb, 0x5e345fe3, 0xb7c954a3, 0x54d08308, 0xcaae3c41,
};

static const uint8_t ed25519_test_key_index = 14;
static const uint8_t rsa_pub_test_key_index = 15;

class CryptoTest : public UnitTest
{
public:
	virtual bool run_tests();

	bool test_aes();
	bool test_chacha();
	bool test_rsa_sign();
	bool test_ed25519_key_gen();
	bool test_ed25519_sign();
	bool test_rsa_verif_set_key();

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
	uint8_t *enc_data = (uint8_t *)&test_output1[512 / 4];
	uint8_t *dec_data = (uint8_t *)&test_output2[512 / 4];
	const void *outbuf1_end = (uint8_t *)&test_output1[sizeof(test_output1) / 4];
	const void *outbuf2_end = (uint8_t *)&test_output2[sizeof(test_output2) / 4];
	PX4Crypto crypto;

	if (!crypto.open(CRYPTO_AES)) {
		ERROUT("Crypto open failed");
	}

	if (!crypto.generate_key(4, false)) {
		ERROUT("Generate key failed");
	}

	color_output(test_output1, sizeof(test_output1));
	color_output(test_output2, sizeof(test_output2));

	/* Encrypt data (in-place) */

	uint32_t mac[4];
	size_t mac_size = sizeof(mac);
	size_t out;

	crypto.renew_nonce((uint8_t *)test_nonce, 12);

	const size_t text_size = 247;
	const size_t aligned_text_sz = (text_size + 0x3) & (~0x3);

	/* Encrypt */

	memcpy(enc_data, plaintext, text_size);

	out = 1024;
	ret = crypto.encrypt_data(
		      4,
		      enc_data,
		      text_size,
		      enc_data,
		      &out,
		      (uint8_t *)mac,
		      &mac_size);

	if (!ret) {
		ERROUT("Encrypt failed");
	}

	/* Check for memory corruption */

	if (!check_color(test_output1, enc_data) ||
	    !check_color(enc_data + aligned_text_sz, outbuf1_end)) {
		dump_data32(enc_data - 16, aligned_text_sz + 32);
		ERROUT("Memory corruption in encryption");
	}

	if (out != text_size) {
		ERROUT("Encrypt: output size mismatch");
	}

	/* Decrypt data */

	crypto.renew_nonce((const uint8_t *)test_nonce, 12);

	out = 1024;
	ret = crypto.decrypt_data(
		      4,
		      enc_data,
		      text_size,
		      (uint8_t *)mac,
		      mac_size,
		      dec_data,
		      &out);

	if (!ret) {
		ERROUT("Decrypt failed");
	}

	if (!check_color(test_output2, dec_data) ||
	    !check_color(dec_data + aligned_text_sz, outbuf2_end)) {
		dump_data(dec_data - 16, aligned_text_sz + 32);
		ERROUT("Memory corruption in decryption");
	}

	if (out != text_size) {
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
	uint8_t *enc_data = (uint8_t *)&test_output1[512 / 4];
	uint8_t *dec_data = (uint8_t *)&test_output2[512 / 4];
	const void *outbuf1_end = (uint8_t *)&test_output1[sizeof(test_output1) / 4];
	const void *outbuf2_end = (uint8_t *)&test_output2[sizeof(test_output2) / 4];
	PX4Crypto crypto;

	if (!crypto.open(CRYPTO_XCHACHA20)) {
		ERROUT("Crypto open failed");
	}

	if (!crypto.generate_key(4, false)) {
		ERROUT("Generate key failed");
	}

	color_output(test_output1, sizeof(test_output1));
	color_output(test_output2, sizeof(test_output2));

	/* Encrypt data (in-place) */

	size_t out;

	const size_t text_size = 247;

	crypto.renew_nonce((const uint8_t *)test_nonce, 24);

	/* Encrypt */

	memcpy(enc_data, plaintext, text_size);

	out = 1024;
	ret = crypto.encrypt_data(
		      4,
		      enc_data,
		      text_size,
		      enc_data,
		      &out,
		      NULL,
		      0);

	if (!ret) {
		ERROUT("Encrypt failed");
	}

	if (out != text_size) {
		ERROUT("Encrypt: output size mismatch");
	}

	/* Check for memory corruption */

	if (!check_color(test_output1, enc_data) ||
	    !check_color(enc_data + text_size, outbuf1_end)) {
		dump_data(enc_data - 16, text_size + 32);
		ERROUT("Memory corruption in encryption");
	}

	/* Decrypt data */

	/* Use the same generated key, but re-initialize */

	crypto.renew_nonce((const uint8_t *)test_nonce, 24);

	out = 1024;
	ret = crypto.decrypt_data(
		      4,
		      enc_data,
		      text_size,
		      NULL,
		      0,
		      dec_data,
		      &out);

	if (!ret) {
		ERROUT("Decrypt failed");
	}

	if (out != text_size) {
		ERROUT("Decrypt: output size mismatch");
	}

	if (!check_color(test_output2, dec_data) ||
	    !check_color(dec_data + text_size, outbuf2_end)) {
		dump_data(dec_data - 16, text_size + 32);
		ERROUT("Memory corruption in decryption");
	}

	if (memcmp(test_input, dec_data, text_size)) {
		ERROUT("Data doesn't match");
	}

	crypto.close();
	return true;
}

// openssl dgst -sha256 -sign Tools/saluki-sec-scripts/test_keys/saluki-v2/rsa2048_test_key.pem sig_data.bin > rsa_sig.bin
// openssl dgst -sha256 -verify rsa2048.pub -signature rsa_sig.bin sig_data.bin

bool CryptoTest::test_rsa_sign()
{
	bool ret = true;

	PX4Crypto crypto;

	if (!crypto.open(CRYPTO_RSA_SIG)) {
		ERROUT("Crypto open failed");
	}

	ret = crypto.signature_check(2,
				     (uint8_t *)rsa_sig,
				     (uint8_t *)test_input,
				     5);

	crypto.close();

	return ret;
}

bool CryptoTest::test_ed25519_key_gen()
{
	bool ret = true;

	PX4Crypto crypto;

	if (!crypto.open(CRYPTO_ED25519)) {
		ERROUT("Crypto open failed");
	}

	ret = crypto.generate_keypair(32, ed25519_test_key_index, false);

	crypto.close();

	return ret;
}

bool CryptoTest::test_ed25519_sign()
{
	bool ret = true;
	uint8_t signature[64] = {0xde, 0xad, 0xbe, 0xef};

	PX4Crypto crypto;

	if (!crypto.open(CRYPTO_ED25519)) {
		ERROUT("Crypto open failed");
	}

	ret = crypto.sign(ed25519_test_key_index, signature, (const uint8_t *)test_input, sizeof(test_input));

	if (!ret) {
		return ret;
	}

	return crypto.signature_check(ed25519_test_key_index, signature, (const uint8_t *)test_input, sizeof(test_input));
}

bool CryptoTest::test_rsa_verif_set_key()
{
	bool ret = true;
	uint8_t *board_specific_sign = NULL;

	PX4Crypto crypto;

	if (!crypto.open(CRYPTO_RSA_SIG)) {
		ERROUT("Crypto open failed");
	}

	/* Set key with signature.
	 * The signature is given from board specific sources,
	 * BOARD_TEST_RSA_PUBKEY_SIG shall point to signature data.
	 * If board specific signature not given, the NULL pointer is
	 * use, menaing the pubkey is assumed to be not signed and
	 * no verification is done.
	 */

#if defined(BOARD_TEST_RSA_PUBKEY_SIG)
	board_specific_sign = (uint8_t *)(BOARD_TEST_RSA_PUBKEY_SIG);
	printf("INFO  [tests] Using board specific signature\n");
#endif

	if (!crypto.set_key(2,
			    board_specific_sign,
			    (uint8_t *)test_rsa_pubkey_der,
			    294,
			    rsa_pub_test_key_index)) {
		crypto.close();
		ERROUT("Verifying and storing RSA public key failed");
	}

	// Verify another signature with new stored public key
	if (!crypto.signature_check(rsa_pub_test_key_index,
				    (uint8_t *)test_data_rsa_signature,
				    (uint8_t *)test_rsa_pubkey_der,
				    294)) {
		crypto.close();
		ERROUT("Verifying and storing RSA public key failed");
	}

	crypto.close();

	return ret;
}

bool CryptoTest::run_tests()
{
	ut_run_test(test_aes);
	ut_run_test(test_chacha);
	ut_run_test(test_ed25519_key_gen);
	ut_run_test(test_ed25519_sign);
	ut_run_test(test_rsa_sign);
	ut_run_test(test_rsa_verif_set_key);

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
