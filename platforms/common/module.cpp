/****************************************************************************
 *
 * Copyright (C) 2017 PX4 Development Team. All rights reserved.
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
 * @file module.cpp
 * Implementation of the API declared in px4_module.h.
 */

#ifndef MODULE_NAME
#define MODULE_NAME "module"
#endif

#include <px4_platform_common/module.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>

pthread_mutex_t px4_modules_mutex = PTHREAD_MUTEX_INITIALIZER;

#ifndef __PX4_NUTTX

void PRINT_MODULE_DESCRIPTION(const char *description)
{
	// TODO: the output could be improved by:
	// - mark titles in bold (lines starting with ##)
	// - highlight commands (lines starting with $, or `cmd`)
	PX4_INFO_RAW("%s\n\n", description);
}

#endif /* __PX4_NUTTX */

void PRINT_MODULE_USAGE_NAME(const char *executable_name, const char *category)
{
	PX4_INFO_RAW("Usage: %s <command> [arguments...]\n", executable_name);
	PX4_INFO_RAW(" Commands:\n");
}

void PRINT_MODULE_USAGE_SUBCATEGORY(const char *subcategory)
{
	(void)subcategory;
}

void PRINT_MODULE_USAGE_NAME_SIMPLE(const char *executable_name, const char *category)
{
	PX4_INFO_RAW("Usage: %s [arguments...]\n", executable_name);
}

void PRINT_MODULE_USAGE_COMMAND_DESCR(const char *name, const char *description)
{
	if (description) {
		PX4_INFO_RAW("\n   %-13s %s\n", name, description);

	} else {
		PX4_INFO_RAW("\n   %s\n", name);
	}
}

void PRINT_MODULE_USAGE_PARAM_COMMENT(const char *comment)
{
	PX4_INFO_RAW("\n %s\n", comment);
}

void PRINT_MODULE_USAGE_PARAM_INT(char option_char, int default_val, int min_val, int max_val,
				  const char *description, bool is_optional)
{
	if (is_optional) {
		PX4_INFO_RAW("     [-%c <val>]  %s\n", option_char, description);

		if (default_val != -1) {
			PX4_INFO_RAW("                 default: %i\n", default_val);
		}

	} else {
		PX4_INFO_RAW("     -%c <val>    %s\n", option_char, description);
	}
}

void PRINT_MODULE_USAGE_PARAM_FLOAT(char option_char, float default_val, float min_val, float max_val,
				    const char *description, bool is_optional)
{
	if (is_optional) {
		PX4_INFO_RAW("     [-%c <val>]  %s\n", option_char, description);

		if (PX4_ISFINITE(default_val)) {
			PX4_INFO_RAW("                 default: %.1f\n", (double)default_val);
		}

	} else {
		PX4_INFO_RAW("     -%c <val>    %s\n", option_char, description);
	}
}

void PRINT_MODULE_USAGE_PARAM_FLAG(char option_char, const char *description, bool is_optional)
{
	if (is_optional) {
		PX4_INFO_RAW("     [-%c]        %s\n", option_char, description);

	} else {
		PX4_INFO_RAW("     -%c          %s\n", option_char, description);
	}
}

void PRINT_MODULE_USAGE_PARAM_STRING(char option_char, const char *default_val, const char *values,
				     const char *description, bool is_optional)
{
	if (is_optional) {
		PX4_INFO_RAW("     [-%c <val>]  %s\n", option_char, description);

	} else {
		PX4_INFO_RAW("     -%c <val>    %s\n", option_char, description);
	}

	if (values) {
		if (default_val) {
			PX4_INFO_RAW("                 values: %s, default: %s\n", values, default_val);

		} else {
			PX4_INFO_RAW("                 values: %s\n", values);
		}

	} else {
		if (default_val) {
			PX4_INFO_RAW("                 default: %s\n", default_val);
		}
	}
}


void PRINT_MODULE_USAGE_ARG(const char *values, const char *description, bool is_optional)
{
	if (is_optional) {
		PX4_INFO_RAW("     [%-9s] %s\n", values, description);

	} else {
		PX4_INFO_RAW("     %-11s %s\n", values, description);
	}
}

