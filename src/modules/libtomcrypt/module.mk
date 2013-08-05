############################################################################
#
#   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

#
# LibTom Crypt
#
# Cryptography library by Tom St Denis, licensed WTFPL / public domain.
#
# http://libtom.org/?page=download&newsitems=5&whatfile=crypt
#
# Selectively compiling only the parts we actually use. Right now this includes:
#
# - PKCS #1
# - RSA
#

SRCS		=	encauth/gcm/gcm_add_aad.c \
			encauth/gcm/gcm_add_iv.c \
			encauth/gcm/gcm_done.c \
			encauth/gcm/gcm_gf_mult.c \
			encauth/gcm/gcm_init.c \
			encauth/gcm/gcm_memory.c \
			encauth/gcm/gcm_mult_h.c \
			encauth/gcm/gcm_process.c \
			encauth/gcm/gcm_reset.c \
			encauth/gcm/gcm_test.c \
			hashes/sha1.c \
			pk/ecc/ecc_ansi_x963_export.c \
			pk/ecc/ecc_ansi_x963_import.c \
			pk/ecc/ecc_decrypt_key.c \
			pk/ecc/ecc_encrypt_key.c \
			pk/ecc/ecc_export.c \
			pk/ecc/ecc_free.c \
			pk/ecc/ecc_get_size.c \
			pk/ecc/ecc_import.c \
			pk/ecc/ecc_make_key.c \
			pk/ecc/ecc_shared_secret.c \
			pk/ecc/ecc_sign_hash.c \
			pk/ecc/ecc_sizes.c \
			pk/ecc/ecc_test.c \
			pk/ecc/ecc_verify_hash.c \
			pk/ecc/ecc.c \
			pk/ecc/ltc_ecc_is_valid_idx.c \
			pk/ecc/ltc_ecc_map.c \
			pk/ecc/ltc_ecc_mul2add.c \
			pk/ecc/ltc_ecc_mulmod_timing.c \
			pk/ecc/ltc_ecc_mulmod.c \
			pk/ecc/ltc_ecc_points.c \
			pk/ecc/ltc_ecc_projective_add_point.c \
			pk/ecc/ltc_ecc_projective_dbl_point.c \
			pk/pkcs1/pkcs_1_i2osp.c \
			pk/pkcs1/pkcs_1_mgf1.c \
			pk/pkcs1/pkcs_1_oaep_decode.c \
			pk/pkcs1/pkcs_1_oaep_encode.c \
			pk/pkcs1/pkcs_1_os2ip.c \
			pk/pkcs1/pkcs_1_pss_decode.c \
			pk/pkcs1/pkcs_1_pss_encode.c \
			pk/pkcs1/pkcs_1_v1_5_decode.c \
			pk/pkcs1/pkcs_1_v1_5_encode.c \
			pk/rsa/rsa_decrypt_key.c \
			pk/rsa/rsa_encrypt_key.c \
			pk/rsa/rsa_export.c \
			pk/rsa/rsa_exptmod.c \
			pk/rsa/rsa_free.c \
			pk/rsa/rsa_import.c \
			pk/rsa/rsa_make_key.c \
			pk/rsa/rsa_sign_hash.c \
			pk/rsa/rsa_verify_hash.c \
			pk/asn1/der/bit/der_decode_bit_string.c \
			pk/asn1/der/bit/der_encode_bit_string.c \
			pk/asn1/der/bit/der_length_bit_string.c \
			pk/asn1/der/boolean/der_decode_boolean.c \
			pk/asn1/der/boolean/der_encode_boolean.c \
			pk/asn1/der/boolean/der_length_boolean.c \
			pk/asn1/der/choice/der_decode_choice.c \
			pk/asn1/der/ia5/der_decode_ia5_string.c \
			pk/asn1/der/ia5/der_encode_ia5_string.c \
			pk/asn1/der/ia5/der_length_ia5_string.c \
			pk/asn1/der/integer/der_decode_integer.c \
			pk/asn1/der/integer/der_encode_integer.c \
			pk/asn1/der/integer/der_length_integer.c \
			pk/asn1/der/object_identifier/der_decode_object_identifier.c \
			pk/asn1/der/object_identifier/der_encode_object_identifier.c \
			pk/asn1/der/object_identifier/der_length_object_identifier.c \
			pk/asn1/der/octet/der_decode_octet_string.c \
			pk/asn1/der/octet/der_encode_octet_string.c \
			pk/asn1/der/octet/der_length_octet_string.c \
			pk/asn1/der/printable_string/der_decode_printable_string.c \
			pk/asn1/der/printable_string/der_encode_printable_string.c \
			pk/asn1/der/printable_string/der_length_printable_string.c \
			pk/asn1/der/sequence/der_decode_sequence_ex.c \
			pk/asn1/der/sequence/der_decode_sequence_flexi.c \
			pk/asn1/der/sequence/der_decode_sequence_multi.c \
			pk/asn1/der/sequence/der_encode_sequence_ex.c \
			pk/asn1/der/sequence/der_encode_sequence_multi.c \
			pk/asn1/der/sequence/der_length_sequence.c \
			pk/asn1/der/sequence/der_sequence_free.c \
			pk/asn1/der/set/der_encode_set.c \
			pk/asn1/der/set/der_encode_setof.c \
			pk/asn1/der/short_integer/der_decode_short_integer.c \
			pk/asn1/der/short_integer/der_encode_short_integer.c \
			pk/asn1/der/short_integer/der_length_short_integer.c \
			pk/asn1/der/utctime/der_decode_utctime.c \
			pk/asn1/der/utctime/der_encode_utctime.c \
			pk/asn1/der/utctime/der_length_utctime.c \
			pk/asn1/der/utf8/der_decode_utf8_string.c \
			pk/asn1/der/utf8/der_encode_utf8_string.c \
			pk/asn1/der/utf8/der_length_utf8_string.c \
			ciphers/aes/aes_tab.c \
			ciphers/aes/aes.c \
			hashes/sha2/sha256.c \
			hashes/helper/hash_memory.c \
			prngs/yarrow.c \
			prngs/rng_get_bytes.c \
			prngs/rng_make_prng.c \
			misc/base64/base64_decode.c \
			misc/base64/base64_encode.c \
			misc/crypt/crypt_argchk.c \
			misc/crypt/crypt_cipher_descriptor.c \
			misc/crypt/crypt_cipher_is_valid.c \
			misc/crypt/crypt_find_cipher_any.c \
			misc/crypt/crypt_find_cipher_id.c \
			misc/crypt/crypt_find_cipher.c \
			misc/crypt/crypt_find_hash_any.c \
			misc/crypt/crypt_find_hash_id.c \
			misc/crypt/crypt_find_hash_oid.c \
			misc/crypt/crypt_find_hash.c \
			misc/crypt/crypt_find_prng.c \
			misc/crypt/crypt_fsa.c \
			misc/crypt/crypt_hash_descriptor.c \
			misc/crypt/crypt_hash_is_valid.c \
			misc/crypt/crypt_ltc_mp_descriptor.c \
			misc/crypt/crypt_prng_descriptor.c \
			misc/crypt/crypt_prng_is_valid.c \
			misc/crypt/crypt_register_cipher.c \
			misc/crypt/crypt_register_hash.c \
			misc/crypt/crypt_register_prng.c \
			misc/crypt/crypt_unregister_cipher.c \
			misc/crypt/crypt_unregister_hash.c \
			misc/crypt/crypt_unregister_prng.c \
			misc/crypt/crypt.c \
			misc/error_to_string.c \
			misc/zeromem.c \
			math/fp/ltc_ecc_fp_mulmod.c \
			math/gmp_desc.c \
			math/ltm_desc.c \
			math/multi.c \
			math/rand_prime.c \
			math/tfm_desc.c \
			modes/ctr/ctr_decrypt.c \
			modes/ctr/ctr_done.c \
			modes/ctr/ctr_encrypt.c \
			modes/ctr/ctr_getiv.c \
			modes/ctr/ctr_setiv.c \
			modes/ctr/ctr_start.c \
			modes/ctr/ctr_test.c

INCLUDE_DIRS	+=	$(PX4_INCLUDE_DIR)/libtomcrypt \
			$(PX4_INCLUDE_DIR)/libtomfastmath

#
# Make all symbols of this library visible globally
#
DEFAULT_VISIBILITY	= y
