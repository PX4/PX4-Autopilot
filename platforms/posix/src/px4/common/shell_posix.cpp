/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/shell.h>

#define MODULE_NAME "px4"
#include <px4_platform_common/log.h>

#include <cstdlib>
#include <cstring>
#include <string>

namespace px4
{

static std::string shell_quote(const std::string &value)
{
	std::string quoted = "'";

	for (char ch : value) {
		if (ch == '\'') {
			quoted += "'\\''";

		} else {
			quoted += ch;
		}
	}

	quoted += "'";
	return quoted;
}

static const char *startup_shell_wrapper = R"sh(
__px4_echo() {
	__px4_line=$*
	__px4_level=
	case "$__px4_line" in
		"DEBUG ["*) __px4_level=DEBUG; __px4_rest=${__px4_line#DEBUG } ;;
		"DEBUG  ["*) __px4_level=DEBUG; __px4_rest=${__px4_line#DEBUG  } ;;
		"INFO ["*) __px4_level=INFO; __px4_rest=${__px4_line#INFO } ;;
		"INFO  ["*) __px4_level=INFO; __px4_rest=${__px4_line#INFO  } ;;
		"WARN ["*) __px4_level=WARN; __px4_rest=${__px4_line#WARN } ;;
		"WARN  ["*) __px4_level=WARN; __px4_rest=${__px4_line#WARN  } ;;
		"ERROR ["*) __px4_level=ERROR; __px4_rest=${__px4_line#ERROR } ;;
		"ERROR  ["*) __px4_level=ERROR; __px4_rest=${__px4_line#ERROR  } ;;
		"PANIC ["*) __px4_level=PANIC; __px4_rest=${__px4_line#PANIC } ;;
		"PANIC  ["*) __px4_level=PANIC; __px4_rest=${__px4_line#PANIC  } ;;
	esac

	if [ -z "$__px4_level" ]; then
		command echo "$@"
		return
	fi

	__px4_module=${__px4_rest#\[}
	__px4_module=${__px4_module%%\]*}
	__px4_message=${__px4_rest#*\]}
	__px4_message=${__px4_message# }

	if { [ -t 1 ] || [ "${PX4_FORCE_COLOR:-}" = 1 ]; } && [ -z "${NO_COLOR:-}" ]; then
		case "$__px4_level" in
			DEBUG) __px4_color='\033[32m' ;;
			WARN) __px4_color='\033[33m' ;;
			ERROR|PANIC) __px4_color='\033[31m' ;;
			*) __px4_color='\033[0m' ;;
		esac

		printf '%b%-5s %b[%s] %b%s%b\n' "$__px4_color" "$__px4_level" '\033[37m' "$__px4_module" "$__px4_color" "$__px4_message" '\033[0m'
	else
		printf '%s\n' "$__px4_line"
	fi
}

echo() {
	__px4_echo "$@"
}

__px4_script=$1
set -- "$2"
. "$__px4_script"
)sh";

int run_shell_script(const std::string &script_path,
			     const std::string &binary_dir,
			     int instance)
{
	// Prepend binary_dir to PATH so the script's `. px4-alias.sh` and
	// px4-<module> invocations resolve regardless of the user's cwd.
	const char *path = getenv("PATH");

	if (path && strstr(path, binary_dir.c_str()) == nullptr) {
		std::string updated = binary_dir + ':' + path;
		setenv("PATH", updated.c_str(), 1);
	}

	PX4_INFO("startup script: /bin/sh %s %d", script_path.c_str(), instance);
	const std::string cmd = "/bin/sh -c " + shell_quote(startup_shell_wrapper)
				+ " px4-rc " + shell_quote(script_path) + ' ' + std::to_string(instance);
	return system(cmd.c_str());
}

} // namespace px4
