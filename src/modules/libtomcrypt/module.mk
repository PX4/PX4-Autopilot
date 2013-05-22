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

SRCS		=	pk/pkcs1/pkcs_1_i2osp.c \
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
			pk/rsa/rsa_verify_hash.c

INCLUDE_DIRS	+= $(PX4_INCLUDE_DIR)/libtomcrypt
