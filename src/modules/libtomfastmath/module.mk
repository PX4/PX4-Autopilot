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

SRCS		=	addsub/fp_add_d.c \
			addsub/fp_add.c \
			addsub/fp_addmod.c \
			addsub/fp_cmp_d.c \
			addsub/fp_cmp_mag.c \
			addsub/fp_cmp.c \
			addsub/fp_sub_d.c \
			addsub/fp_sub.c \
			addsub/fp_submod.c \
			addsub/s_fp_add.c \
			addsub/s_fp_sub.c \
			bin/fp_radix_size.c \
			bin/fp_read_radix.c \
			bin/fp_read_signed_bin.c \
			bin/fp_read_unsigned_bin.c \
			bin/fp_reverse.c \
			bin/fp_s_rmap.c \
			bin/fp_signed_bin_size.c \
			bin/fp_to_signed_bin.c \
			bin/fp_to_unsigned_bin.c \
			bin/fp_toradix.c \
			bin/fp_unsigned_bin_size.c \
			bit/fp_cnt_lsb.c \
			bit/fp_count_bits.c \
			bit/fp_div_2.c \
			bit/fp_div_2d.c \
			bit/fp_lshd.c \
			bit/fp_mod_2d.c \
			bit/fp_rshd.c \
			divide/fp_div.c \
			divide/fp_div_d.c \
			divide/fp_mod.c \
			divide/fp_mod_d.c \
			exptmod/fp_2expt.c \
			exptmod/fp_exptmod.c \
			misc/fp_set.c \
			mont/fp_montgomery_calc_normalization.c \
			mont/fp_montgomery_reduce.c \
			mont/fp_montgomery_setup.c \
			mul/fp_mul_2.c \
			mul/fp_mul_2d.c \
			mul/fp_mul_comba_3.c \
			mul/fp_mul_comba_4.c \
			mul/fp_mul_comba_6.c \
			mul/fp_mul_comba_7.c \
			mul/fp_mul_comba_8.c \
			mul/fp_mul_comba_9.c \
			mul/fp_mul_comba_12.c \
			mul/fp_mul_comba_17.c \
			mul/fp_mul_comba_20.c \
			mul/fp_mul_comba_24.c \
			mul/fp_mul_comba_28.c \
			mul/fp_mul_comba_32.c \
			mul/fp_mul_comba_48.c \
			mul/fp_mul_comba_64.c \
			mul/fp_mul_comba_small_set.c \
			mul/fp_mul_comba.c \
			mul/fp_mul_d.c \
			mul/fp_mul.c \
			mul/fp_mulmod.c \
			numtheory/fp_gcd.c \
			numtheory/fp_isprime.c \
			numtheory/fp_invmod.c \
			numtheory/fp_lcm.c \
			numtheory/fp_prime_miller_rabin.c \
			sqr/fp_sqr_comba_3.c \
			sqr/fp_sqr_comba_4.c \
			sqr/fp_sqr_comba_6.c \
			sqr/fp_sqr_comba_7.c \
			sqr/fp_sqr_comba_8.c \
			sqr/fp_sqr_comba_9.c \
			sqr/fp_sqr_comba_12.c \
			sqr/fp_sqr_comba_17.c \
			sqr/fp_sqr_comba_20.c \
			sqr/fp_sqr_comba_24.c \
			sqr/fp_sqr_comba_28.c \
			sqr/fp_sqr_comba_32.c \
			sqr/fp_sqr_comba_48.c \
			sqr/fp_sqr_comba_64.c \
			sqr/fp_sqr_comba_generic.c \
			sqr/fp_sqr_comba_small_set.c \
			sqr/fp_sqr_comba.c \
			sqr/fp_sqr.c \
			sqr/fp_sqrmod.c


INCLUDE_DIRS	+= $(PX4_INCLUDE_DIR)/libtomfastmath

#
# Make all symbols of this library visible globally
#
DEFAULT_VISIBILITY	= y

MAXOPTIMIZATION	 = -Os
