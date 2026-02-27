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

#pragma once

#include <px4_platform_common/log.h>
#include <string.h>
#include <ctype.h>

#define MAX_ARGS 10
#define MAX_KEY_LEN 32
#define MAX_VALUE_LEN 32

typedef struct {
	char key[MAX_KEY_LEN];
	char value[MAX_VALUE_LEN];
} ArgPair;

class ArgParser
{
private:
	ArgPair _args[MAX_ARGS];
	size_t _arg_count;

	/**
	 * @brief Trim whitespace from a string.
	 *
	 * @param[in,out] str String to trim
	 */
	static void trim(char *str);

	/**
	 * @brief Convert a string to lowercase.
	 *
	 * @param[in,out] str String to convert
	 */
	static void toLowerCase(char *str);

public:
	/**
	 * @brief Construct from a comma-separated key=value string.
	 *
	 * @param[in] data Input string to parse
	 */
	ArgParser(const char *data);

	/**
	 * @brief Check if argument exists.
	 *
	 * @param[in] arg Argument name to check
	 * @return true if argument exists, false otherwise
	 */
	bool hasArgument(const char *arg) const;

	/**
	 * @brief Get argument value as string.
	 *
	 * @param[in] arg Argument name to get
	 * @param[in] defaultValue Default value if argument not found
	 * @return Argument value as string
	 */
	const char *getArgument(const char *arg, const char *defaultValue = "") const;

	/**
	 * @brief Get argument value as int.
	 *
	 * @param[in] arg Argument name to get
	 * @param[in] defaultValue Default value if argument not found
	 * @return Argument value as integer
	 */
	int getInt(const char *arg, int defaultValue = 0) const;

	/**
	 * @brief Get argument value as float.
	 *
	 * @param[in] arg Argument name to get
	 * @param[in] defaultValue Default value if argument not found
	 * @return Argument value as float
	 */
	float getFloat(const char *arg, float defaultValue = 0.0f) const;

	/**
	 * @brief Get argument value as bool.
	 *
	 * @param[in] arg Argument name to get
	 * @param[in] defaultValue Default value if argument not found
	 * @return Argument value as boolean
	 */
	bool getBool(const char *arg, bool defaultValue = false) const;

	/**
	 * @brief Print all parsed arguments (for debug).
	 */
	void printArguments() const;
};
