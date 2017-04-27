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

#include <px4_module.h>

pthread_mutex_t px4_modules_mutex = PTHREAD_MUTEX_INITIALIZER;

#ifndef __PX4_NUTTX

void PRINT_MODULE_DESCRIPTION(const char *description)
{
	printf("%s\n", description);
}

#endif /* __PX4_NUTTX */

void PRINT_MODULE_USAGE_NAME(const char *executable_name, const char *category)
{
	printf("Usage: %s <command> [arguments...]\n", executable_name);
	printf(" Commands:\n");
}

void PRINT_MODULE_USAGE_NAME_SIMPLE(const char *executable_name, const char *category)
{
	printf("Usage: %s [arguments...]\n", executable_name);
}

void PRINT_MODULE_USAGE_COMMAND_DESCR(const char *name, const char *description)
{
	if (description) {
		printf("\n   %-13s %s\n", name, description);

	} else {
		printf("\n   %s\n", name);
	}
}

void PRINT_MODULE_USAGE_PARAM_COMMENT(const char *comment)
{
	printf("\n %s\n", comment);
}

void PRINT_MODULE_USAGE_PARAM_INT(char option_char, int default_val, int min_val, int max_val,
				  const char *description, bool is_optional)
{
	if (is_optional) {
		printf("     [-%c <val>]  %s\n", option_char, description);
		printf("                 default: %i\n", default_val);

	} else {
		printf("     -%c <val>    %s\n", option_char, description);
	}
}

void PRINT_MODULE_USAGE_PARAM_FLOAT(char option_char, float default_val, float min_val, float max_val,
				    const char *description, bool is_optional)
{
	if (is_optional) {
		printf("     [-%c <val>]  %s\n", option_char, description);
		printf("                 default: %.1f\n", (double)default_val);

	} else {
		printf("     -%c <val>    %s\n", option_char, description);
	}
}

void PRINT_MODULE_USAGE_PARAM_FLAG(char option_char, const char *description, bool is_optional)
{
	if (is_optional) {
		printf("     [-%c]        %s\n", option_char, description);

	} else {
		printf("     -%c          %s\n", option_char, description);
	}
}

void PRINT_MODULE_USAGE_PARAM_STRING(char option_char, const char *default_val, const char *values,
				     const char *description, bool is_optional)
{
	if (is_optional) {
		printf("     [-%c <val>]  %s\n", option_char, description);

	} else {
		printf("     -%c <val>    %s\n", option_char, description);
	}

	if (values) {
		if (default_val) {
			printf("                 values: %s, default: %s\n", values, default_val);

		} else {
			printf("                 values: %s\n", values);
		}

	} else {
		if (default_val) {
			printf("                 default: %s\n", default_val);
		}
	}
}


void PRINT_MODULE_USAGE_ARG(const char *values, const char *description, bool is_optional)
{
	if (is_optional) {
		printf("     [%-9s] %s\n", values, description);

	} else {
		printf("     %-11s %s\n", values, description);
	}
}

