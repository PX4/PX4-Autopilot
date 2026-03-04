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

/**
 * @file module.h
 */

#pragma once

#ifdef __cplusplus

#include <cstring>

#include "rclcpp/rclcpp.hpp"

#endif /* __cplusplus */


__BEGIN_DECLS

/**
 * @brief Module documentation and command usage help methods.
 *        These are extracted with the Tools/px_process_module_doc.py
 *        script and must be kept in sync.
 */

#ifdef __PX4_NUTTX
/**
 * @note Disable module description on NuttX to reduce Flash usage.
 *       There's a GCC bug (https://gcc.gnu.org/bugzilla/show_bug.cgi?id=55971), preventing us to use
 *       a macro, but GCC will remove the string as well with this empty inline method.
 * @param description The provided functionality of the module and potentially the most important parameters.
 */
static inline void PRINT_MODULE_DESCRIPTION(const char *description) {}
#else

/**
 * @brief Prints module documentation (will also be used for online documentation). It uses Markdown syntax
 *        and should include these sections:
 * - ### Description
 *   Provided functionality of the module and potentially the most important parameters.
 * - ### Implementation
 *   High-level implementation overview
 * - ### Examples
 *   Examples how to use the CLI interface (if it's non-trivial)
 *
 * In addition to the Markdown syntax, a line beginning with '$ ' can be used to mark a command:
 * $ module start -p param
 */
__EXPORT void PRINT_MODULE_DESCRIPTION(const char *description);
#endif

/**
 * @brief Prints the command name.
 * @param executable_name: command name used in scripts & CLI
 * @param category one of: driver, estimator, controller, system, communication, command, template
 */
__EXPORT void PRINT_MODULE_USAGE_NAME(const char *executable_name, const char *category);

/**
 * @brief Specify a subcategory (optional).
 * @param subcategory e.g. if the category is 'driver', subcategory can be 'distance_sensor'
 */
__EXPORT void PRINT_MODULE_USAGE_SUBCATEGORY(const char *subcategory);

/**
 * @brief Prints the name for a command without any sub-commands (@see PRINT_MODULE_USAGE_NAME()).
 */
__EXPORT void PRINT_MODULE_USAGE_NAME_SIMPLE(const char *executable_name, const char *category);


/**
 * @brief Prints a command with a short description what it does.
 */
__EXPORT void PRINT_MODULE_USAGE_COMMAND_DESCR(const char *name, const char *description);

#define PRINT_MODULE_USAGE_COMMAND(name) \
	PRINT_MODULE_USAGE_COMMAND_DESCR(name, NULL);

/**
 * @brief Prints the default commands: stop & status.
 */
#define PRINT_MODULE_USAGE_DEFAULT_COMMANDS() \
	PRINT_MODULE_USAGE_COMMAND("stop"); \
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "print status info");

/**
 * Print default params for I2C or SPI drivers
 * @param i2c_support true if the driver supports I2C
 * @param spi_support true if the driver supports SPI
 */
__EXPORT void PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(bool i2c_support, bool spi_support);

/**
 * Configurable I2C address (via -a <address>)
 */
__EXPORT void PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(uint8_t default_address);

/**
 * -k flag
 */
__EXPORT void PRINT_MODULE_USAGE_PARAMS_I2C_KEEP_RUNNING_FLAG(void);

/** @note Each of the PRINT_MODULE_USAGE_PARAM_* methods apply to the previous PRINT_MODULE_USAGE_COMMAND_DESCR(). */

/**
 * @brief Prints an integer parameter.
 * @param option_char The option character.
 * @param default_val The parameter default value (set to -1 if not applicable).
 * @param min_val The parameter minimum value.
 * @param max_val The parameter value.
 * @param description Pointer to the usage description.
 * @param is_optional true if this parameter is optional
 */
__EXPORT void PRINT_MODULE_USAGE_PARAM_INT(char option_char, int default_val, int min_val, int max_val,
		const char *description, bool is_optional);

/**
 * @brief Prints a float parameter.
 * @note See PRINT_MODULE_USAGE_PARAM_INT().
 * @param default_val The parameter default value (set to NaN if not applicable).
 * @param min_val The parameter minimum value.
 * @param max_val The parameter maximum value.
 * @param description Pointer to the usage description. Pointer to the usage description.
 * @param is_optional true if this parameter is optional
 */
__EXPORT void PRINT_MODULE_USAGE_PARAM_FLOAT(char option_char, float default_val, float min_val, float max_val,
		const char *description, bool is_optional);

/**
 * @brief Prints a flag parameter, without any value.
 * @note See PRINT_MODULE_USAGE_PARAM_INT().
 * @param option_char The option character.
 * @param description Pointer to the usage description.
 * @param is_optional true if this parameter is optional
 */
__EXPORT void PRINT_MODULE_USAGE_PARAM_FLAG(char option_char, const char *description, bool is_optional);

/**
 * @brief Prints a string parameter.
 * @param option_char The option character.
 * @param default_val The default value, can be nullptr.
 * @param values The valid values, it has one of the following forms:
 *               - nullptr: leave unspecified, or any value is valid
 *               - "<file>" or "<file:dev>": a file or more specifically a device file (eg. serial device)
 *               - "<topic_name>": uORB topic name
 *               - "<value1> [<value2>]": a list of values
 *               - "on|off": a concrete set of valid strings separated by "|".
 * @param description Pointer to the usage description.
 * @param is_optional True iff this parameter is optional.
 */
__EXPORT void PRINT_MODULE_USAGE_PARAM_STRING(char option_char, const char *default_val, const char *values,
		const char *description, bool is_optional);

/**
 * @brief Prints a comment, that applies to the next arguments or parameters. For example to indicate that
 *        a parameter applies to several or all commands.
 * @param comment
 */
__EXPORT void PRINT_MODULE_USAGE_PARAM_COMMENT(const char *comment);


/**
 * @brief Prints the definition for an argument, which does not have the typical -p <val> form,
 *        but for example 'param set <param> <value>'
 * @param values eg. "<file>", "<param> <value>" or "<value1> [<value2>]"
 * @param description Pointer to the usage description.
 * @param is_optional true if this parameter is optional
 */
__EXPORT void PRINT_MODULE_USAGE_ARG(const char *values, const char *description, bool is_optional);


__END_DECLS
