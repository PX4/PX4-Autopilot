/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include "state_sharing/args_parser.hpp"

#define min(a,b) (((a) < (b)) ? (a) : (b))

void ArgParser::trim(char *str)
{
	if (!str || *str == '\0') { return; }

	char *start = str;

	while (isspace((unsigned char)*start)) { start++; }

	char *end = start + strlen(start) - 1;

	while (end > start && isspace((unsigned char)*end)) { end--; }

	*(end + 1) = '\0';

	if (start != str) {
		memmove(str, start, end - start + 2);
	}
}

void ArgParser::toLowerCase(char *str)
{
	for (; *str; str++) {
		*str = tolower((unsigned char) * str);
	}
}

ArgParser::ArgParser(const char *data) : _arg_count(0)
{
	char input[256];
	strncpy(input, data, sizeof(input) - 1);
	input[sizeof(input) - 1] = '\0';
	char *token = strtok(input, ",");

	while (token && _arg_count < MAX_ARGS) {
		char *equalPos = strchr(token, '=');

		if (equalPos) {
			*equalPos = '\0';
			strncpy(_args[_arg_count].key, token, min(strlen(token), MAX_KEY_LEN - 1));
			_args[_arg_count].key[min(strlen(token), MAX_KEY_LEN - 1)] = '\0';
			trim(_args[_arg_count].key);

			strncpy(_args[_arg_count].value, equalPos + 1, min(strlen(equalPos + 1), MAX_VALUE_LEN - 1));
			_args[_arg_count].value[min(strlen(equalPos + 1), MAX_VALUE_LEN - 1)] = '\0';
			trim(_args[_arg_count].value);

			_arg_count++;
		}

		token = strtok(NULL, ",");
	}
}

bool ArgParser::hasArgument(const char *arg) const
{
	for (size_t i = 0; i < _arg_count; i++) {
		if (strcmp(_args[i].key, arg) == 0) {
			return true;
		}
	}

	return false;
}

const char *ArgParser::getArgument(const char *arg, const char *defaultValue) const
{
	for (size_t i = 0; i < _arg_count; i++) {
		if (strcmp(_args[i].key, arg) == 0) {
			return _args[i].value;
		}
	}

	PX4_WARN("Argument '%s' not found.", arg);
	return defaultValue;
}

int ArgParser::getInt(const char *arg, int defaultValue) const
{
	auto arg_val = getArgument(arg, nullptr);

	if (arg_val) {
		char *endptr;
		long value = strtol(arg_val, &endptr, 10);

		if (*endptr == '\0') {
			return (int)value;

		} else {
			PX4_WARN("Error converting argument '%s' to int.", arg);
			return defaultValue;
		}
	}

	PX4_WARN("Argument '%s' not found.", arg);
	return defaultValue;
}

float ArgParser::getFloat(const char *arg, float defaultValue) const
{
	auto arg_val = getArgument(arg, nullptr);

	if (arg_val) {
		char *endptr;
		float value = strtof(arg_val, &endptr);

		if (*endptr == '\0') {
			return value;

		} else {
			PX4_WARN("Error converting argument '%s' to float.", arg);
			return defaultValue;
		}
	}

	PX4_WARN("Argument '%s' not found.", arg);
	return defaultValue;
}

bool ArgParser::getBool(const char *arg, bool defaultValue) const
{
	auto arg_val = getArgument(arg, nullptr);

	if (arg_val) {
		char value[MAX_VALUE_LEN];
		strncpy(value, arg_val, MAX_VALUE_LEN - 1);
		value[MAX_VALUE_LEN - 1] = '\0';
		toLowerCase(value);

		if (strcmp(value, "true") == 0 || strcmp(value, "1") == 0) {
			return true;
		}

		if (strcmp(value, "false") == 0 || strcmp(value, "0") == 0) {
			return false;
		}

		PX4_WARN("Argument '%s' has invalid boolean value: '%s'.", arg, value);
		return defaultValue;
	}

	PX4_WARN("Argument '%s' not found.", arg);
	return defaultValue;
}

void ArgParser::printArguments() const
{
	PX4_DEBUG("-------------Received arguments:---------");

	for (size_t i = 0; i < _arg_count; i++) {
		PX4_DEBUG("%s = %s", _args[i].key, _args[i].value);
	}

	PX4_DEBUG("-----------------------------------------");
}
