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
 * @file public_key.h
 *
 * File holds public keys for signed firmware.
 *
 *
 */

#pragma once

#define XSTR(x) #x
#define STR(x) XSTR(x)

#ifndef PUBLIC_KEY0
#error "At least one key (PUBLIC_KEY0) must be defined"
#endif

typedef struct {
	const size_t key_size;
	const uint8_t *key;
} persistent_key_t;

/* This constant only exists to calculate size of the
   key. It will be removed by the linker */
static const uint8_t public_key0[] = {
#include STR(PUBLIC_KEY0)
};

#ifdef PUBLIC_KEY1

static const uint8_t public_key1[] = {
#include STR(PUBLIC_KEY1)
};
#endif

#ifdef PUBLIC_KEY2
static const uint8_t public_key2[] = {
#include STR(PUBLIC_KEY2)
};
#endif

#ifdef PUBLIC_KEY3
static const uint8_t public_key3[] = {
#include STR(PUBLIC_KEY3)
};
#endif


static const persistent_key_t public_keys[] = {
	{
		.key = public_key0,
		.key_size = sizeof(public_key0)
	}

#ifdef PUBLIC_KEY1
	,
	{
		.key = public_key1,
		.key_size = sizeof(public_key1)
	}
#endif

#ifdef PUBLIC_KEY2
	,
	{
		.key = public_key2,
		.key_size = sizeof(public_key2)
	}
#endif

#ifdef PUBLIC_KEY3
	{
		.key = public_key3,
		.key_size = sizeof(public_key3)
	}
#endif

};

#define NPERSISTENT_KEYS (sizeof(public_keys) / sizeof(persistent_key_t))
