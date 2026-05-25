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

// Windows needs a real in-process shell backend if px4.exe is to be
// redistributed as a self-contained component. The minimal parser below is
// only a stopgap for trivial generated scripts and should not be used for
// full rcS semantics.

#include <px4_platform_common/shell.h>

#define MODULE_NAME "px4"
#include <px4_platform_common/log.h>

#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <fstream>
#include <string>

#include "../../common/px4_daemon/pxh.h"
#include "embedded_backend.h"

namespace px4
{

static int set_env(const char *key, const char *value)
{
	return _putenv_s(key, value);
}

static bool allow_external_shell_fallback()
{
	const char *value = getenv("PX4_ALLOW_EXTERNAL_SH");

	return value != nullptr && strcmp(value, "1") == 0;
}

static int run_shell_script_fallback(const std::string &script_path)
{
	std::ifstream file(script_path);

	if (!file.is_open()) {
		PX4_ERR("Could not open startup script %s", script_path.c_str());
		return -1;
	}

	PX4_WARN("falling back to the minimal Windows shell parser for %s", script_path.c_str());

	int last_ret = 0;
	std::string line;

	while (std::getline(file, line)) {
		// strip CR (files written with CRLF line endings)
		while (!line.empty() && (line.back() == '\r' || line.back() == '\n')) {
			line.pop_back();
		}

		std::size_t first = line.find_first_not_of(" \t");

		if (first == std::string::npos) {
			continue; // blank line
		}

		std::string trimmed = line.substr(first);

		if (trimmed[0] == '#') {
			continue; // comment or shebang
		}

		if (trimmed.rfind(". ", 0) == 0) {
			// Skip POSIX `source`; the only script init files do this
			// for is px4-alias.sh, whose aliases are redundant here
			// because Pxh dispatches bare command names via the apps map.
			continue;
		}

		if (trimmed.rfind("echo ", 0) == 0 || trimmed == "echo") {
			// Minimal `echo` so scripts can emit PASS/FAIL markers that
			// ctest PASS_REGULAR_EXPRESSION looks for. Strips a single
			// pair of surrounding quotes but otherwise prints verbatim.
			std::string arg = trimmed.size() > 4 ? trimmed.substr(5) : "";

			if (arg.size() >= 2 &&
			    ((arg.front() == '"' && arg.back() == '"') ||
			     (arg.front() == '\'' && arg.back() == '\''))) {
				arg = arg.substr(1, arg.size() - 2);
			}

			const std::string output = arg + '\n';
			px4_log_write_text(stdout, output.data(), output.size());
			last_ret = 0;
			continue;
		}

		last_ret = px4_daemon::Pxh::process_line(trimmed, true);
	}

	return last_ret;
}

int run_shell_script(const std::string &script_path,
		     const std::string &binary_dir,
		     int instance)
{
	const char *path = getenv("PATH");

	if (path && strstr(path, binary_dir.c_str()) == nullptr) {
		std::string updated = binary_dir + ':' + path;
		set_env("PATH", updated.c_str());

	} else if (!path) {
		set_env("PATH", binary_dir.c_str());
	}

	if (embedded_shell::is_available()) {
		PX4_INFO("startup script (%s): %s",
			 embedded_shell::backend_name(),
			 script_path.c_str());
		return embedded_shell::run_script(script_path, binary_dir, instance);
	}

	if (allow_external_shell_fallback()) {
		std::string cmd = "sh \"" + script_path + "\" " + std::to_string(instance);
		PX4_WARN("no embedded shell backend compiled in, using external shell fallback: %s", cmd.c_str());

		const int ret = system(cmd.c_str());

		if (ret != -1) {
			return ret;
		}
	}

	PX4_WARN("no embedded shell backend is available for %s", script_path.c_str());
	PX4_WARN("build a real backend into px4.exe for full rcS support");
	return run_shell_script_fallback(script_path);
}

} // namespace px4
