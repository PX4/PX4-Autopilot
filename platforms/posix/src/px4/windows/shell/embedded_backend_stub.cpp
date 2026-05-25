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

#include "embedded_backend.h"

#define MODULE_NAME "px4"
#include <px4_platform_common/log.h>
#include <px4_daemon/server_io.h>

#include "../../common/px4_daemon/pxh.h"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <regex>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <io.h>
#include <unistd.h>

#ifdef _WIN32
#include <windows.h>  // SearchPathA, MAX_PATH
#endif

namespace px4::embedded_shell
{
namespace
{

namespace fs = std::filesystem;

struct CommandResult {
	int status{0};
	std::string output{};
};

#ifdef _WIN32
// Returns true if `command` resolves to an executable on PATH (or as a literal
// path). Used to skip cmd.exe spawn for unrecognized commands so its localized
// "path not found" stderr never reaches the user. Real programs that exist run
// normally and their stderr passes through unfiltered.
bool host_command_exists_on_path(const std::string &command)
{
	if (command.empty()) {
		return false;
	}

	// SearchPathA also accepts paths with directory components and verifies
	// they exist; .exe is the default extension probed when none is given.
	char buffer[MAX_PATH];
	const DWORD len = SearchPathA(nullptr, command.c_str(), ".exe",
				      static_cast<DWORD>(sizeof(buffer)), buffer, nullptr);
	return len > 0 && len < sizeof(buffer);
}
#endif

struct ScanDepth {
	int single_quotes{0};
	int double_quotes{0};
	int paren_depth{0};
};

static bool is_px4_command(const std::string &command)
{
	static apps_map_type px4_apps{};

	if (px4_apps.empty()) {
		init_app_map(px4_apps);
	}

	return px4_apps.find(command) != px4_apps.end();
}

static bool looks_like_glob_or_pattern(const std::string &command)
{
	return command.find_first_of("*?[") != std::string::npos;
}

static std::string ltrim(std::string value)
{
	value.erase(value.begin(),
	std::find_if(value.begin(), value.end(), [](unsigned char ch) { return !std::isspace(ch); }));
	return value;
}

static std::string rtrim(std::string value)
{
	value.erase(std::find_if(value.rbegin(), value.rend(),
	[](unsigned char ch) { return !std::isspace(ch); }).base(),
	value.end());
	return value;
}

static std::string trim(std::string value)
{
	return rtrim(ltrim(std::move(value)));
}

static bool starts_with(const std::string &value, const std::string &prefix)
{
	return value.rfind(prefix, 0) == 0;
}

static std::string strip_trailing_newlines(std::string value)
{
	while (!value.empty() && (value.back() == '\n' || value.back() == '\r')) {
		value.pop_back();
	}

	return value;
}

static bool is_identifier(const std::string &value)
{
	if (value.empty() || !(std::isalpha(value[0]) || value[0] == '_')) {
		return false;
	}

	for (char ch : value) {
		if (!(std::isalnum(static_cast<unsigned char>(ch)) || ch == '_')) {
			return false;
		}
	}

	return true;
}

static bool is_digits(const std::string &value)
{
	return !value.empty() && std::all_of(value.begin(), value.end(),
	[](unsigned char ch) { return std::isdigit(ch); });
}

static std::vector<std::string> split_lines(const std::string &value)
{
	std::vector<std::string> lines;
	std::stringstream stream(value);
	std::string line;

	while (std::getline(stream, line)) {
		lines.push_back(strip_trailing_newlines(line));
	}

	return lines;
}

static bool glob_match_impl(const char *pattern, const char *value)
{
	if (*pattern == '\0') {
		return *value == '\0';
	}

	if (*pattern == '*') {
		return glob_match_impl(pattern + 1, value) || (*value != '\0' && glob_match_impl(pattern, value + 1));
	}

	if (*pattern == '?') {
		return *value != '\0' && glob_match_impl(pattern + 1, value + 1);
	}

	if (*pattern == '\\' && pattern[1] != '\0') {
		return *value == pattern[1] && glob_match_impl(pattern + 2, value + 1);
	}

	return *pattern == *value && glob_match_impl(pattern + 1, value + 1);
}

static bool glob_match(const std::string &pattern, const std::string &value)
{
	return glob_match_impl(pattern.c_str(), value.c_str());
}

static std::string escape_for_regex(const std::string &value)
{
	static const std::regex special{R"([-[\]{}()*+?.,\^$|#\s])"};
	return std::regex_replace(value, special, R"(\$&)");
}

static std::string quote_for_host_command(const std::string &arg)
{
	if (arg.find_first_of(" \t\"") == std::string::npos) {
		return arg;
	}

	std::string quoted = "\"";

	for (char ch : arg) {
		if (ch == '"') {
			quoted += "\\\"";

		} else {
			quoted += ch;
		}
	}

	quoted += "\"";
	return quoted;
}

class Shell
{
public:
	Shell(std::string binary_dir, int instance) :
		_binary_dir(std::move(binary_dir)),
		_instance(instance)
	{
		char cwd_buffer[4096] {};
		::getcwd(cwd_buffer, sizeof(cwd_buffer));
		_cwd = cwd_buffer;
		set_variable("PWD", _cwd);
		set_variable("R", _cwd + "/");
		set_variable("px4_instance", std::to_string(_instance));
		set_variable("1", std::to_string(_instance));
		_trace_enabled = std::getenv("PX4_WIN_SHELL_TRACE") != nullptr;
	}

	int run_script(const std::string &script_path)
	{
		const std::string resolved = resolve_script_path(script_path);

		if (resolved.empty()) {
			PX4_ERR("embedded shell could not resolve %s", script_path.c_str());
			return -1;
		}

		return execute_script_file(resolved);
	}

private:
	std::unordered_map<std::string, std::string> _variables{};
	std::string _binary_dir;
	std::string _cwd;
	int _instance{0};
	bool _exit_requested{false};
	int _exit_status{0};
	bool _trace_enabled{false};

	void trace_line(const char *prefix, const std::string &line) const
	{
		if (_trace_enabled) {
			PX4_INFO("[embedded-shell] %s%s", prefix, line.c_str());
		}
	}

	void set_variable(const std::string &name, const std::string &value)
	{
		_variables[name] = value;

		// Keep the POSIX shell search path separate from the host Windows
		// search path. rcS appends colon-separated entries to PATH, which is
		// valid for the embedded shell but corrupts cmd.exe path lookup when a
		// host fallback command is run.
		if (name == "PATH") {
			return;
		}

		_putenv_s(name.c_str(), value.c_str());
	}

	std::string get_variable(const std::string &name) const
	{
		if (auto it = _variables.find(name); it != _variables.end()) {
			return it->second;
		}

		if (const char *env = std::getenv(name.c_str())) {
			return env;
		}

		return {};
	}

	std::string resolve_script_path(const std::string &path) const
	{
		fs::path candidate(path);

		if (candidate.is_absolute()) {
			return fs::exists(candidate) ? candidate.lexically_normal().string() : std::string{};
		}

		const fs::path from_cwd = fs::path(_cwd) / candidate;

		if (fs::exists(from_cwd)) {
			return from_cwd.lexically_normal().string();
		}

		const std::string path_var = get_variable("PATH");
		std::stringstream stream(path_var);
		std::string entry;

		while (std::getline(stream, entry, ':')) {
			if (entry.empty()) {
				continue;
			}

			fs::path search(entry);

			if (!search.is_absolute()) {
				search = fs::path(_cwd) / search;
			}

			search /= candidate;

			if (fs::exists(search)) {
				return search.lexically_normal().string();
			}
		}

		return {};
	}

	static std::string strip_comments(const std::string &line)
	{
		std::string output;
		bool single = false;
		bool dbl = false;

		for (std::size_t i = 0; i < line.size(); ++i) {
			const char ch = line[i];

			if (ch == '\'' && !dbl) {
				single = !single;
				output += ch;
				continue;
			}

			if (ch == '"' && !single) {
				dbl = !dbl;
				output += ch;
				continue;
			}

			if (!single && !dbl && ch == '#'
			    && (i == 0 || std::isspace(static_cast<unsigned char>(line[i - 1])))) {
				break;
			}

			output += ch;
		}

		return output;
	}

	static std::vector<std::string> split_semicolons(const std::string &line)
	{
		std::vector<std::string> parts;
		std::string current;
		bool single = false;
		bool dbl = false;

		for (std::size_t i = 0; i < line.size(); ++i) {
			const char ch = line[i];

			if (ch == '\'' && !dbl) {
				single = !single;
				current += ch;
				continue;
			}

			if (ch == '"' && !single) {
				dbl = !dbl;
				current += ch;
				continue;
			}

			if (!single && !dbl && ch == ';') {
				// `;;` is the case-terminator, not two empty statements.
				// Emit the accumulated part, then push `;;` as its own
				// token, and skip both characters. A lone `;` still
				// splits as a statement separator.
				if (i + 1 < line.size() && line[i + 1] == ';') {
					parts.push_back(trim(current));
					current.clear();
					parts.push_back(";;");
					++i;
					continue;
				}

				parts.push_back(trim(current));
				current.clear();
				continue;
			}

			current += ch;
		}

		parts.push_back(trim(current));
		parts.erase(std::remove_if(parts.begin(), parts.end(),
		[](const std::string & part) { return part.empty(); }),
		parts.end());
		return parts;
	}

	std::vector<std::string> load_script_lines(const std::string &path) const
	{
		std::ifstream file(path);
		std::vector<std::string> lines;

		if (!file.is_open()) {
			return lines;
		}

		std::string raw;
		std::string accumulated;

		while (std::getline(file, raw)) {
			raw = strip_trailing_newlines(raw);

			if (!raw.empty() && raw.back() == '\r') {
				raw.pop_back();
			}

			if (!accumulated.empty()) {
				accumulated += raw;

			} else {
				accumulated = raw;
			}

			if (!accumulated.empty() && accumulated.back() == '\\') {
				accumulated.pop_back();
				continue;
			}

			const std::string stripped = trim(strip_comments(accumulated));
			accumulated.clear();

			if (stripped.empty() || stripped[0] == '#') {
				continue;
			}

			for (std::string &part : split_semicolons(stripped)) {
				lines.push_back(std::move(part));
			}
		}

		return lines;
	}

	int execute_script_file(const std::string &path)
	{
		if (fs::path(path).filename() == "px4-alias.sh") {
			set_variable("R", _cwd + "/");
			set_variable("px4_instance", std::to_string(_instance));
			set_variable("1", std::to_string(_instance));
			return 0;
		}

		std::vector<std::string> lines = load_script_lines(path);
		std::size_t index = 0;
		return execute_range(lines, index, lines.size());
	}

	int execute_range(const std::vector<std::string> &lines, std::size_t &index, std::size_t end)
	{
		int last_status = 0;

		while (index < end && !_exit_requested) {
			const std::string &line = lines[index];

			if (starts_with(line, "if ")) {
				last_status = execute_if(lines, index);
				continue;
			}

			if (starts_with(line, "for ")) {
				last_status = execute_for(lines, index);
				continue;
			}

			if (starts_with(line, "case ")) {
				last_status = execute_case(lines, index);
				continue;
			}

			if (line.find("| while ") != std::string::npos) {
				last_status = execute_pipe_while(lines, index);
				continue;
			}

			if (line == "then" || line == "fi" || line == "do" || line == "done" || line == "esac"
			    || line == "else" || starts_with(line, "elif ")) {
				return last_status;
			}

			trace_line("", line);
			last_status = execute_command_chain(line, false).status;
			++index;
		}

		return _exit_requested ? _exit_status : last_status;
	}

	static bool is_nested_block_start(const std::string &line)
	{
		return starts_with(line, "if ") || starts_with(line, "for ") || starts_with(line, "case ")
		       || line.find("| while ") != std::string::npos;
	}

	static bool is_nested_block_end(const std::string &line)
	{
		return line == "fi" || line == "done" || line == "esac";
	}

	std::size_t find_matching_fi(const std::vector<std::string> &lines, std::size_t start) const
	{
		int depth = 0;

		for (std::size_t i = start; i < lines.size(); ++i) {
			const std::string &line = lines[i];

			if (starts_with(line, "if ")) {
				++depth;
				continue;
			}

			if (line == "fi") {
				if (depth == 0) {
					return i;
				}

				--depth;
				continue;
			}

			if ((starts_with(line, "for ") || starts_with(line, "case ") || line.find("| while ") != std::string::npos)) {
				++depth;
				continue;
			}

			if ((line == "done" || line == "esac") && depth > 0) {
				--depth;
			}
		}

		return lines.size();
	}

	std::size_t find_matching_done(const std::vector<std::string> &lines, std::size_t body_start) const
	{
		int depth = 0;

		for (std::size_t i = body_start; i < lines.size(); ++i) {
			if (is_nested_block_start(lines[i])) {
				++depth;
				continue;
			}

			if (is_nested_block_end(lines[i])) {
				if (depth == 0) {
					return i;
				}

				--depth;
			}
		}

		return lines.size();
	}

	int execute_if(const std::vector<std::string> &lines, std::size_t &index)
	{
		std::string condition = trim(lines[index].substr(3));
		++index;

		if (index < lines.size() && lines[index] == "then") {
			++index;
		}

		while (index <= lines.size()) {
			const std::size_t branch_start = index;
			int depth = 0;

			while (index < lines.size()) {
				const std::string &line = lines[index];

				if (is_nested_block_start(line)) {
					++depth;
					++index;
					continue;
				}

				if (is_nested_block_end(line)) {
					if (depth > 0) {
						--depth;
						++index;
						continue;
					}
				}

				if (depth == 0 && (line == "else" || line == "fi" || starts_with(line, "elif "))) {
					break;
				}

				++index;
			}

			const std::size_t branch_end = index;

			if (condition.empty() || execute_command_chain(condition, false).status == 0) {
				std::size_t exec_index = branch_start;
				const int status = execute_range(lines, exec_index, branch_end);
				index = find_matching_fi(lines, index);

				if (index < lines.size()) {
					++index;
				}

				return status;
			}

			if (index >= lines.size()) {
				return -1;
			}

			if (lines[index] == "else") {
				condition.clear();
				++index;
				continue;
			}

			if (starts_with(lines[index], "elif ")) {
				condition = trim(lines[index].substr(5));
				++index;

				if (index < lines.size() && lines[index] == "then") {
					++index;
				}

				continue;
			}

			if (lines[index] == "fi") {
				++index;
				return 0;
			}
		}

		return 0;
	}

	int execute_for(const std::vector<std::string> &lines, std::size_t &index)
	{
		const std::string header = trim(lines[index].substr(4));
		const std::size_t in_pos = header.find(" in ");

		if (in_pos == std::string::npos) {
			PX4_ERR("unsupported for syntax: %s", lines[index].c_str());
			return -1;
		}

		const std::string variable_name = trim(header.substr(0, in_pos));
		const std::string word_list = trim(header.substr(in_pos + 4));
		++index;

		if (index < lines.size() && lines[index] == "do") {
			++index;
		}

		const std::size_t body_start = index;
		const std::size_t body_end = find_matching_done(lines, body_start);

		if (body_end >= lines.size()) {
			return -1;
		}

		std::vector<std::string> items = tokenize_words(word_list);
		std::vector<std::string> expanded_items;

		for (const std::string &item : items) {
			const auto globbed = expand_glob(item);
			expanded_items.insert(expanded_items.end(), globbed.begin(), globbed.end());
		}

		int status = 0;

		for (const std::string &item : expanded_items) {
			set_variable(variable_name, item);
			std::size_t exec_index = body_start;
			status = execute_range(lines, exec_index, body_end);

			if (_exit_requested) {
				break;
			}
		}

		index = body_end + 1;
		return status;
	}

	int execute_case(const std::vector<std::string> &lines, std::size_t &index)
	{
		std::string header = trim(lines[index].substr(5));

		if (header.size() >= 3 && header.compare(header.size() - 3, 3, " in") == 0) {
			header = trim(header.substr(0, header.size() - 3));
		}

		const std::string subject = expand_to_string(header);
		++index;
		int status = 0;
		bool matched = false;

		while (index < lines.size()) {
			if (lines[index] == "esac") {
				++index;
				return status;
			}

			std::string pattern_line = lines[index];
			++index;

			if (!pattern_line.empty() && pattern_line.back() == ')') {
				pattern_line.pop_back();
			}

			const std::vector<std::string> patterns = split_patterns(pattern_line);
			const std::size_t body_start = index;

			while (index < lines.size() && lines[index] != ";;" && lines[index] != "esac") {
				++index;
			}

			if (!matched) {
				for (const std::string &pattern : patterns) {
					if (glob_match(pattern, subject)) {
						std::size_t exec_index = body_start;
						status = execute_range(lines, exec_index, index);
						matched = true;
						break;
					}
				}
			}

			if (index < lines.size() && lines[index] == ";;") {
				++index;
			}
		}

		return status;
	}

	int execute_pipe_while(const std::vector<std::string> &lines, std::size_t &index)
	{
		const std::string line = lines[index];
		const std::size_t marker = line.find("| while ");

		if (marker == std::string::npos) {
			return -1;
		}

		const std::string producer = trim(line.substr(0, marker));
		const std::string consumer = trim(line.substr(marker + 2));
		++index;

		if (index < lines.size() && lines[index] == "do") {
			++index;
		}

		const std::size_t body_start = index;
		const std::size_t body_end = find_matching_done(lines, body_start);

		if (body_end >= lines.size()) {
			return -1;
		}

		CommandResult produced = execute_command_chain(producer, true);
		std::vector<std::string> input_lines = split_lines(produced.output);
		int status = 0;

		for (const std::string &input : input_lines) {
			status = execute_read_consumer(consumer, input);

			if (status != 0) {
				break;
			}

			std::size_t exec_index = body_start;
			status = execute_range(lines, exec_index, body_end);

			if (_exit_requested) {
				break;
			}
		}

		index = body_end + 1;
		return status;
	}

	int execute_read_consumer(const std::string &consumer, const std::string &input)
	{
		std::vector<std::string> words = tokenize_words(consumer);

		if (words.empty()) {
			return -1;
		}

		std::size_t command_index = 0;

		if (words[command_index] == "while") {
			++command_index;
		}

		while (command_index < words.size() && words[command_index].find('=') != std::string::npos) {
			++command_index;
		}

		if (command_index >= words.size() || words[command_index] != "read") {
			return -1;
		}

		++command_index;

		if (command_index < words.size() && words[command_index] == "-r") {
			++command_index;
		}

		if (command_index < words.size()) {
			set_variable(words[command_index], input);
			return 0;
		}

		return -1;
	}

	CommandResult execute_command_chain(const std::string &line, bool capture_output)
	{
		const std::vector<std::pair<std::string, std::string>> chain = split_and_or(line);
		CommandResult result{};
		bool first = true;

		for (const auto &entry : chain) {
			if (!first) {
				if (entry.first == "&&" && result.status != 0) {
					continue;
				}

				if (entry.first == "||" && result.status == 0) {
					continue;
				}
			}

			result = execute_pipeline(entry.second, capture_output);
			first = false;
		}

		return result;
	}

	std::vector<std::pair<std::string, std::string>> split_and_or(const std::string &line) const
	{
		std::vector<std::pair<std::string, std::string>> parts;
		std::string current;
		std::string pending_operator;
		bool single = false;
		bool dbl = false;

		for (std::size_t i = 0; i < line.size(); ++i) {
			const char ch = line[i];

			if (ch == '\'' && !dbl) {
				single = !single;
				current += ch;
				continue;
			}

			if (ch == '"' && !single) {
				dbl = !dbl;
				current += ch;
				continue;
			}

			if (!single && !dbl && i + 1 < line.size()) {
				const std::string op = line.substr(i, 2);

				if (op == "&&" || op == "||") {
					parts.emplace_back(pending_operator, trim(current));
					pending_operator = op;
					current.clear();
					i += 1;
					continue;
				}
			}

			current += ch;
		}

		parts.emplace_back(pending_operator, trim(current));
		parts.erase(std::remove_if(parts.begin(), parts.end(),
		[](const auto & entry) { return entry.second.empty(); }),
		parts.end());
		return parts;
	}

	CommandResult execute_pipeline(const std::string &line, bool capture_output)
	{
		std::vector<std::string> segments = split_pipeline(line);
		CommandResult result{};
		std::string input;
		bool allow_output = capture_output || segments.size() > 1;

		for (std::size_t i = 0; i < segments.size(); ++i) {
			const bool stage_capture = allow_output || i + 1 < segments.size();
			result = execute_simple_command(segments[i], input, stage_capture);

			if (result.status != 0 && i + 1 < segments.size()) {
				return result;
			}

			input = result.output;
		}

		return result;
	}

	std::vector<std::string> split_pipeline(const std::string &line) const
	{
		std::vector<std::string> parts;
		std::string current;
		bool single = false;
		bool dbl = false;
		int paren_depth = 0;

		for (std::size_t i = 0; i < line.size(); ++i) {
			const char ch = line[i];

			if (ch == '\'' && !dbl) {
				single = !single;
				current += ch;
				continue;
			}

			if (ch == '"' && !single) {
				dbl = !dbl;
				current += ch;
				continue;
			}

			if (!single && !dbl) {
				if (ch == '(') {
					++paren_depth;

				} else if (ch == ')' && paren_depth > 0) {
					--paren_depth;
				}

				if (ch == '|' && paren_depth == 0 && !(i + 1 < line.size() && line[i + 1] == '|')) {
					parts.push_back(trim(current));
					current.clear();
					continue;
				}
			}

			current += ch;
		}

		parts.push_back(trim(current));
		parts.erase(std::remove_if(parts.begin(), parts.end(),
		[](const std::string & part) { return part.empty(); }),
		parts.end());
		return parts;
	}

	CommandResult execute_simple_command(const std::string &segment, const std::string &stdin_data, bool capture_output)
	{
		std::vector<std::string> words = tokenize_words(segment);

		if (words.empty()) {
			return {};
		}

		bool negate = false;

		if (words.front() == "!") {
			negate = true;
			words.erase(words.begin());
		}

		bool background = false;

		if (!words.empty() && words.back() == "&") {
			background = true;
			words.pop_back();
		}

		std::string stdout_file;
		bool stdout_discard = false;
		bool stderr_to_stdout = false;

		std::vector<std::string> filtered;

		for (std::size_t i = 0; i < words.size(); ++i) {
			const std::string &word = words[i];

			if (word == "2>&1") {
				stderr_to_stdout = true;
				continue;
			}

			if (word == ">" || word == "1>" || word == "2>") {
				if (i + 1 < words.size()) {
					if (words[i + 1] == "/dev/null") {
						stdout_discard = true;

					} else {
						stdout_file = words[i + 1];
					}

					++i;
					continue;
				}
			}

			if (starts_with(word, "2>")) {
				if (word.substr(2) == "/dev/null") {
					continue;
				}
			}

			if (starts_with(word, ">")) {
				if (word.substr(1) == "/dev/null") {
					stdout_discard = true;
					continue;
				}

				stdout_file = word.substr(1);
				continue;
			}

			filtered.push_back(word);
		}

		words = filtered;

		if (words.empty()) {
			return {};
		}

		std::unordered_map<std::string, std::string> temporary_assignments;
		std::size_t command_index = 0;

		while (command_index < words.size()) {
			const std::size_t eq = words[command_index].find('=');

			if (eq == std::string::npos || !is_identifier(words[command_index].substr(0, eq))) {
				break;
			}

			temporary_assignments[words[command_index].substr(0, eq)] = words[command_index].substr(eq + 1);
			++command_index;
		}

		if (command_index >= words.size()) {
			for (const auto &entry : temporary_assignments) {
				set_variable(entry.first, entry.second);
			}

			return {};
		}

		const std::string command = words[command_index];
		std::vector<std::string> args(words.begin() + command_index + 1, words.end());
		CommandResult result{};

		if (command == "[") {
			result.status = evaluate_test(args) ? 0 : 1;

		} else if (command == "set") {
			result.status = execute_set(args);

		} else if (command == ".") {
			if (args.empty()) {
				result.status = -1;

			} else {
				result.status = execute_script_file(resolve_script_path(args[0]));
			}

		} else if (command == "exit") {
			_exit_requested = true;
			_exit_status = args.empty() ? 0 : std::atoi(args[0].c_str());
			result.status = _exit_status;

		} else if (command == "echo") {
			result.output = join(args, " ");
			result.output += '\n';
			result.status = 0;

		} else if (command == "pwd") {
			result.output = _cwd + '\n';
			result.status = 0;

		} else if (command == "basename") {
			result.output = fs::path(args.empty() ? std::string{} : args[0]).filename().string();
			result.output += '\n';
			result.status = 0;

		} else if (command == "env") {
			for (const auto &entry : _variables) {
				result.output += entry.first + "=" + entry.second + "\n";
			}

			result.status = 0;

		} else if (command == "cp") {
			result.status = execute_copy(args);

		} else if (command == "sleep") {
			double seconds = args.empty() ? 0.0 : std::atof(args[0].c_str());
			std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
			result.status = 0;

		} else if (command == "true") {
			result.status = 0;

		} else if (command == "false") {
			result.status = 1;

		} else if (command == "ls") {
			result = execute_ls(args);

		} else if (command == "sed") {
			result = execute_sed(args, stdin_data);

		} else if (command == "bc") {
			result = execute_bc(stdin_data);

		} else if (command == "grep") {
			result = execute_grep(args, stdin_data);

		} else if (command == "head") {
			result = execute_head(args, stdin_data);

		} else if (command == "tr") {
			result = execute_tr(args, stdin_data);

		} else if (command == "sort") {
			result = execute_sort(args, stdin_data);

		} else if (command == "printf") {
			result = execute_printf(args);

		} else if (command == "awk") {
			result = execute_awk(args, stdin_data);

		} else if (command == "read") {
			result.status = execute_read(args, stdin_data);

		} else if (capture_output) {
			result = capture_px4_or_host(command, args, stderr_to_stdout);

		} else {
			result.status = run_px4_or_host(command, args, background);
		}

		if (!stdout_file.empty()) {
			std::ofstream out(stdout_file, std::ios::binary);
			out << result.output;
			result.output.clear();

		} else if (stdout_discard) {
			result.output.clear();

		} else if (!capture_output && !result.output.empty()) {
			px4_log_write_text(stdout, result.output.data(), result.output.size());
			result.output.clear();
		}

		if (negate) {
			result.status = result.status == 0 ? 1 : 0;
		}

		return result;
	}

	int run_px4_or_host(const std::string &command, const std::vector<std::string> &args, bool background)
	{
		std::string line = command;

		for (const std::string &arg : args) {
			line += ' ';
			line += arg;
		}

		const int px4_status = px4_daemon::Pxh::process_line(line, true);

		if (px4_status != -1 || is_px4_command(command)) {
			return px4_status;
		}

		if (looks_like_glob_or_pattern(command)) {
			return -1;
		}

		std::string host_command = quote_for_host_command(command);

		for (const std::string &arg : args) {
			host_command += ' ';
			host_command += quote_for_host_command(arg);
		}

#ifdef _WIN32

		// Skip the cmd.exe spawn for unrecognized commands so its localized
		// "path not found" stderr never reaches the user. Real programs that
		// exist on PATH run normally and their stderr passes through.
		if (!host_command_exists_on_path(command)) {
			return -1;
		}

#endif

		if (background) {
			std::thread([host_command]() {
				std::system(host_command.c_str());
			}).detach();
			return 0;
		}

		return std::system(host_command.c_str());
	}

	CommandResult capture_px4_or_host(const std::string &command, const std::vector<std::string> &args, bool merge_stderr)
	{
		std::string line = command;

		for (const std::string &arg : args) {
			line += ' ';
			line += arg;
		}

		const CommandResult px4_captured = capture_px4_command(line);

		if (px4_captured.status != -1 || is_px4_command(command)) {
			return px4_captured;
		}

		if (looks_like_glob_or_pattern(command)) {
			return px4_captured;
		}

		std::string host_command = quote_for_host_command(command);

		for (const std::string &arg : args) {
			host_command += ' ';
			host_command += quote_for_host_command(arg);
		}

		if (merge_stderr) {
			host_command += " 2>&1";
		}

#ifdef _WIN32

		// Skip the cmd.exe spawn for unrecognized commands; see the matching
		// pre-check in run_px4_or_host() for rationale.
		if (!host_command_exists_on_path(command)) {
			CommandResult missing{};
			missing.status = -1;
			return missing;
		}

		FILE *pipe = _popen(host_command.c_str(), "r");
#else
		FILE *pipe = popen(host_command.c_str(), "r");
#endif

		CommandResult result{};

		if (!pipe) {
			result.status = -1;
			return result;
		}

		char buffer[512];

		while (std::fgets(buffer, sizeof(buffer), pipe) != nullptr) {
			result.output += buffer;
		}

#ifdef _WIN32
		result.status = _pclose(pipe);
#else
		result.status = pclose(pipe);
#endif
		return result;
	}

	CommandResult capture_px4_command(const std::string &line)
	{
		CommandResult result{};
		std::fflush(stdout);
		const int stdout_copy = dup(STDOUT_FILENO);
		FILE *tmp = std::tmpfile();

		if (stdout_copy < 0 || !tmp) {
			result.status = -1;
			return result;
		}

		// Command substitutions and pipeline stages consume captured bytes, not terminal output.
		// Keep them plain so ANSI resets do not become part of shell variables or glob patterns.
		const char *previous_no_color = std::getenv("NO_COLOR");
		const bool had_no_color = previous_no_color != nullptr;
		const std::string saved_no_color = had_no_color ? previous_no_color : "";
		setenv("NO_COLOR", "1", 1);
		set_stdout_isatty_override(false);
		dup2(fileno(tmp), STDOUT_FILENO);
		result.status = px4_daemon::Pxh::process_line(line, true);
		std::fflush(stdout);
		dup2(stdout_copy, STDOUT_FILENO);
		clear_stdout_isatty_override();

		if (had_no_color) {
			setenv("NO_COLOR", saved_no_color.c_str(), 1);

		} else {
			unsetenv("NO_COLOR");
		}

		close(stdout_copy);

		std::rewind(tmp);
		char buffer[512];

		while (std::fgets(buffer, sizeof(buffer), tmp) != nullptr) {
			result.output += buffer;
		}

		std::fclose(tmp);
		result.output = strip_trailing_newlines(result.output);
		return result;
	}

	int execute_set(const std::vector<std::string> &args)
	{
		if (args.empty()) {
			return 0;
		}

		if (args[0] == "+e" || args[0] == "-e" || args[0] == "-x" || args[0] == "+x") {
			return 0;
		}

		if (args.size() == 1) {
			const std::size_t eq = args[0].find('=');

			if (eq != std::string::npos) {
				set_variable(args[0].substr(0, eq), args[0].substr(eq + 1));
			}

			return 0;
		}

		set_variable(args[0], join(std::vector<std::string>(args.begin() + 1, args.end()), " "));
		return 0;
	}

	int execute_copy(const std::vector<std::string> &args)
	{
		if (args.size() < 2) {
			return -1;
		}

		std::error_code ec;
		fs::copy_file(args[0], args[1], fs::copy_options::overwrite_existing, ec);
		return ec ? -1 : 0;
	}

	CommandResult execute_ls(const std::vector<std::string> &args)
	{
		CommandResult result{};
		const fs::path target = args.empty() ? fs::path(_cwd) : fs::path(args[0]);
		std::vector<std::string> entries;

		if (!fs::exists(target)) {
			result.status = 1;
			return result;
		}

		for (const auto &entry : fs::directory_iterator(target)) {
			entries.push_back(entry.path().filename().string());
		}

		std::sort(entries.begin(), entries.end());

		for (const std::string &entry : entries) {
			result.output += entry + "\n";
		}

		return result;
	}

	CommandResult execute_bc(const std::string &stdin_data)
	{
		CommandResult result{};
		const double value = evaluate_numeric_expression(trim(stdin_data), false);
		result.output = format_number(value);
		result.output += '\n';
		return result;
	}

	CommandResult execute_head(const std::vector<std::string> &args, const std::string &stdin_data)
	{
		CommandResult result{};
		int count = 10;

		if (args.size() >= 2 && args[0] == "-n") {
			count = std::atoi(args[1].c_str());
		}

		std::vector<std::string> lines = split_lines(stdin_data);

		for (int i = 0; i < count && i < static_cast<int>(lines.size()); ++i) {
			result.output += lines[i] + "\n";
		}

		return result;
	}

	CommandResult execute_tr(const std::vector<std::string> &args, const std::string &stdin_data)
	{
		CommandResult result{};
		result.output = stdin_data;

		if (args.size() >= 2 && args[0] == "-d") {
			for (char ch : args[1]) {
				result.output.erase(std::remove(result.output.begin(), result.output.end(), ch), result.output.end());
			}
		}

		return result;
	}

	CommandResult execute_sort(const std::vector<std::string> &args, const std::string &stdin_data)
	{
		CommandResult result{};
		std::vector<std::string> lines = split_lines(stdin_data);
		const bool version_sort = !args.empty() && args[0] == "-V";

		auto comparator = [version_sort](const std::string & lhs, const std::string & rhs) {
			if (!version_sort) {
				return lhs < rhs;
			}

			std::regex token_re(R"(([0-9]+|[^0-9]+))");
			auto lhs_begin = std::sregex_iterator(lhs.begin(), lhs.end(), token_re);
			auto rhs_begin = std::sregex_iterator(rhs.begin(), rhs.end(), token_re);
			const auto end = std::sregex_iterator();

			for (auto lit = lhs_begin, rit = rhs_begin; lit != end && rit != end; ++lit, ++rit) {
				const std::string ltoken = lit->str();
				const std::string rtoken = rit->str();
				const bool lnum = is_digits(ltoken);
				const bool rnum = is_digits(rtoken);

				if (lnum && rnum) {
					const long long lvalue = std::atoll(ltoken.c_str());
					const long long rvalue = std::atoll(rtoken.c_str());

					if (lvalue != rvalue) {
						return lvalue < rvalue;
					}

				} else if (ltoken != rtoken) {
					return ltoken < rtoken;
				}
			}

			return lhs < rhs;
		};

		std::sort(lines.begin(), lines.end(), comparator);

		for (const std::string &line : lines) {
			result.output += line + "\n";
		}

		return result;
	}

	CommandResult execute_printf(const std::vector<std::string> &args)
	{
		CommandResult result{};

		if (args.empty()) {
			return result;
		}

		if (args[0] == "%s\\n") {
			for (std::size_t i = 1; i < args.size(); ++i) {
				result.output += args[i] + "\n";
			}
		}

		return result;
	}

	CommandResult execute_awk(const std::vector<std::string> &args, const std::string &stdin_data)
	{
		CommandResult result{};
		std::string delimiter = " ";
		std::string program;

		for (std::size_t i = 0; i < args.size(); ++i) {
			if (args[i] == "-F" && i + 1 < args.size()) {
				delimiter = args[++i];

			} else {
				program = args[i];
			}
		}

		std::smatch match;
		const std::regex print_field_re(R"(\{print \$([0-9]+)\})");

		if (!std::regex_match(program, match, print_field_re)) {
			result.status = 1;
			return result;
		}

		const std::size_t field_index = static_cast<std::size_t>(std::stoul(match[1].str())) - 1;

		for (const std::string &line : split_lines(stdin_data)) {
			std::vector<std::string> fields;

			if (delimiter == " ") {
				std::stringstream stream(line);
				std::string field;

				while (stream >> field) {
					fields.push_back(field);
				}

			} else {
				std::size_t start = 0;

				while (start <= line.size()) {
					const std::size_t end = line.find(delimiter, start);

					if (end == std::string::npos) {
						fields.push_back(line.substr(start));
						break;
					}

					fields.push_back(line.substr(start, end - start));
					start = end + delimiter.size();
				}
			}

			if (field_index < fields.size()) {
				result.output += fields[field_index] + "\n";
			}
		}

		return result;
	}

	CommandResult execute_grep(const std::vector<std::string> &args, const std::string &stdin_data)
	{
		CommandResult result{};
		bool ignore_case = false;
		bool quiet = false;
		int max_matches = -1;
		std::string pattern;

		for (std::size_t i = 0; i < args.size(); ++i) {
			if (args[i] == "-i") {
				ignore_case = true;

			} else if (args[i] == "-q") {
				quiet = true;

			} else if (args[i] == "-m" && i + 1 < args.size()) {
				max_matches = std::atoi(args[++i].c_str());

			} else if (args[i] == "-e" && i + 1 < args.size()) {
				pattern = args[++i];

			} else if (pattern.empty()) {
				pattern = args[i];
			}
		}

		const std::regex re(pattern, ignore_case ? std::regex::icase : std::regex::ECMAScript);
		int matches = 0;

		for (const std::string &line : split_lines(stdin_data)) {
			if (std::regex_search(line, re)) {
				++matches;

				if (!quiet) {
					result.output += line + "\n";
				}

				if (max_matches > 0 && matches >= max_matches) {
					break;
				}
			}
		}

		result.status = matches > 0 ? 0 : 1;
		return result;
	}

	CommandResult execute_sed(const std::vector<std::string> &args, const std::string &stdin_data)
	{
		CommandResult result{};
		bool quiet = false;
		std::string script;

		for (const std::string &arg : args) {
			if (arg == "-n") {
				quiet = true;

			} else {
				script = arg;
			}
		}

		const std::vector<std::string> programs = split_sed_programs(script);

		for (const std::string &line : split_lines(stdin_data)) {
			std::string current = line;
			bool printed = !quiet;

			for (const std::string &program : programs) {
				apply_sed_program(program, current, quiet, printed, result.output);
			}

			if (printed) {
				result.output += current + "\n";
			}
		}

		return result;
	}

	static std::vector<std::string> split_sed_programs(const std::string &script)
	{
		std::vector<std::string> programs;
		std::string current;
		bool single = false;

		for (char ch : script) {
			if (ch == '\'') {
				single = !single;
				continue;
			}

			if (!single && ch == ';') {
				programs.push_back(trim(current));
				current.clear();
				continue;
			}

			current += ch;
		}

		if (!current.empty()) {
			programs.push_back(trim(current));
		}

		return programs;
	}

	static void replace_all(std::string &target, const std::string &from, const std::string &to)
	{
		std::size_t pos = 0;

		while ((pos = target.find(from, pos)) != std::string::npos) {
			target.replace(pos, from.size(), to);
			pos += to.size();
		}
	}

	static std::string convert_sed_regex(std::string pattern)
	{
		replace_all(pattern, R"(\()", "(");
		replace_all(pattern, R"(\))", ")");
		return pattern;
	}

	static std::string convert_sed_replacement(std::string replacement)
	{
		for (int i = 1; i <= 9; ++i) {
			replace_all(replacement, "\\" + std::to_string(i), "$" + std::to_string(i));
		}

		return replacement;
	}

	static void apply_sed_program(const std::string &program, std::string &line, bool quiet, bool &printed,
				      std::string &output)
	{
		if (program.empty() || program[0] != 's') {
			return;
		}

		const char delimiter = program[1];
		std::vector<std::string> pieces;
		std::string current;

		for (std::size_t i = 2; i < program.size(); ++i) {
			if (program[i] == delimiter && (i == 0 || program[i - 1] != '\\')) {
				pieces.push_back(current);
				current.clear();
				continue;
			}

			current += program[i];
		}

		pieces.push_back(current);

		if (pieces.size() < 3) {
			return;
		}

		const std::regex re(convert_sed_regex(pieces[0]));
		const bool global = pieces[2].find('g') != std::string::npos;
		const bool print = pieces[2].find('p') != std::string::npos;
		const std::string replacement = convert_sed_replacement(pieces[1]);
		const std::string replaced = global ? std::regex_replace(line, re, replacement)
					     : std::regex_replace(line, re, replacement,
							     std::regex_constants::format_first_only);

		if (replaced != line) {
			line = replaced;

			if (quiet && print) {
				output += line + "\n";
				printed = false;
			}
		}
	}

	int execute_read(const std::vector<std::string> &args, const std::string &stdin_data)
	{
		std::size_t index = 0;

		if (index < args.size() && args[index] == "-r") {
			++index;
		}

		if (index >= args.size()) {
			return -1;
		}

		set_variable(args[index], strip_trailing_newlines(stdin_data));
		return 0;
	}

	bool evaluate_test(std::vector<std::string> args)
	{
		if (!args.empty() && args.back() == "]") {
			args.pop_back();
		}

		if (args.empty()) {
			return false;
		}

		if (args[0] == "!") {
			args.erase(args.begin());
			return !evaluate_test(args);
		}

		if (args.size() == 1) {
			return !args[0].empty();
		}

		if (args.size() == 2) {
			if (args[0] == "-n") {
				return !args[1].empty();
			}

			if (args[0] == "-z") {
				return args[1].empty();
			}

			if (args[0] == "-f") {
				return fs::is_regular_file(args[1]);
			}

			if (args[0] == "-e") {
				return fs::exists(args[1]);
			}
		}

		if (args.size() >= 3) {
			const std::string &lhs = args[0];
			const std::string &op = args[1];
			const std::string &rhs = args[2];

			if (op == "=" || op == "==") {
				return lhs == rhs;
			}

			if (op == "!=") {
				return lhs != rhs;
			}

			const long long lvalue = std::atoll(lhs.c_str());
			const long long rvalue = std::atoll(rhs.c_str());

			if (op == "-eq") { return lvalue == rvalue; }

			if (op == "-ne") { return lvalue != rvalue; }

			if (op == "-gt") { return lvalue > rvalue; }

			if (op == "-lt") { return lvalue < rvalue; }

			if (op == "-ge") { return lvalue >= rvalue; }

			if (op == "-le") { return lvalue <= rvalue; }
		}

		return false;
	}

	std::vector<std::string> tokenize_words(const std::string &input)
	{
		std::vector<std::string> words;
		std::string current;
		bool token_active = false;
		bool single = false;
		bool dbl = false;

		auto flush = [&]() {
			if (token_active) {
				words.push_back(current);
				current.clear();
				token_active = false;
			}
		};

		auto append_unquoted = [&](const std::string & text) {
			for (char ch : text) {
				if (std::isspace(static_cast<unsigned char>(ch))) {
					flush();

				} else {
					current += ch;
					token_active = true;
				}
			}
		};

		for (std::size_t i = 0; i < input.size(); ++i) {
			const char ch = input[i];

			if (single) {
				if (ch == '\'') {
					single = false;

				} else {
					current += ch;
					token_active = true;
				}

				continue;
			}

			if (dbl) {
				if (ch == '"') {
					dbl = false;
					token_active = true;
					continue;
				}

				if (ch == '$') {
					current += expand_inline(input, i, true);
					token_active = true;
					continue;
				}

				current += ch;
				token_active = true;
				continue;
			}

			if (std::isspace(static_cast<unsigned char>(ch))) {
				flush();
				continue;
			}

			if (ch == '\'') {
				single = true;
				token_active = true;
				continue;
			}

			if (ch == '"') {
				dbl = true;
				token_active = true;
				continue;
			}

			if (ch == '$') {
				const std::string expanded = expand_inline(input, i, false);

				// In real shells, the RHS of a variable assignment (`name=$expansion`)
				// is never subject to word-splitting, regardless of quoting. Only the
				// arguments of a command are. Without this exception a value like
				// PATH=C:\Program Files\... would be split into a bogus token list
				// (`value=C:\Program`, `Files\...`, ...) and the second word would
				// be dispatched as a command. Detect the `<identifier>=` prefix in
				// the in-progress token and keep the expansion intact.
				if (token_active && current_starts_with_assignment(current)) {
					current += expanded;

				} else {
					append_unquoted(expanded);
				}

				continue;
			}

			current += ch;
			token_active = true;
		}

		flush();
		return words;
	}

	static bool current_starts_with_assignment(const std::string &token)
	{
		const std::size_t eq = token.find('=');

		if (eq == std::string::npos || eq == 0) {
			return false;
		}

		if (!std::isalpha(static_cast<unsigned char>(token[0])) && token[0] != '_') {
			return false;
		}

		for (std::size_t i = 1; i < eq; ++i) {
			const unsigned char ch = static_cast<unsigned char>(token[i]);

			if (!std::isalnum(ch) && ch != '_') {
				return false;
			}
		}

		return true;
	}

	std::string expand_to_string(const std::string &input)
	{
		std::string output;
		bool single = false;
		bool dbl = false;

		for (std::size_t i = 0; i < input.size(); ++i) {
			const char ch = input[i];

			if (single) {
				if (ch == '\'') {
					single = false;

				} else {
					output += ch;
				}

				continue;
			}

			if (dbl) {
				if (ch == '"') {
					dbl = false;
					continue;
				}

				if (ch == '$') {
					output += expand_inline(input, i, true);
					continue;
				}

				output += ch;
				continue;
			}

			if (ch == '\'') {
				single = true;
				continue;
			}

			if (ch == '"') {
				dbl = true;
				continue;
			}

			if (ch == '$') {
				output += expand_inline(input, i, false);
				continue;
			}

			output += ch;
		}

		return output;
	}

	std::string expand_inline(const std::string &input, std::size_t &index, bool quoted)
	{
		(void)quoted;

		if (index + 1 >= input.size()) {
			return "$";
		}

		if (input[index + 1] == '{') {
			std::size_t end = input.find('}', index + 2);

			if (end == std::string::npos) {
				return "$";
			}

			const std::string expression = input.substr(index + 2, end - (index + 2));
			index = end;
			return expand_braced(expression);
		}

		if (input[index + 1] == '(') {
			if (index + 2 < input.size() && input[index + 2] == '(') {
				std::size_t end = input.find("))", index + 3);

				if (end == std::string::npos) {
					return {};
				}

				const std::string expression = input.substr(index + 3, end - (index + 3));
				index = end + 1;
				return std::to_string(static_cast<long long>(evaluate_numeric_expression(expression, true)));
			}

			std::size_t depth = 1;
			std::size_t pos = index + 2;

			for (; pos < input.size(); ++pos) {
				if (input[pos] == '(') {
					++depth;

				} else if (input[pos] == ')') {
					--depth;

					if (depth == 0) {
						break;
					}
				}
			}

			const std::string command = input.substr(index + 2, pos - (index + 2));
			index = pos;
			return strip_trailing_newlines(execute_command_chain(command, true).output);
		}

		std::size_t end = index + 1;

		if (std::isdigit(static_cast<unsigned char>(input[end]))) {
			++end;

		} else {
			while (end < input.size()
			       && (std::isalnum(static_cast<unsigned char>(input[end])) || input[end] == '_')) {
				++end;
			}
		}

		const std::string name = input.substr(index + 1, end - (index + 1));
		index = end - 1;
		return get_variable(name);
	}

	std::string expand_braced(const std::string &expression)
	{
		if (const std::size_t pos = expression.find(":-"); pos != std::string::npos) {
			const std::string name = expression.substr(0, pos);
			const std::string value = get_variable(name);
			return value.empty() ? expression.substr(pos + 2) : value;
		}

		if (const std::size_t pos = expression.find(":="); pos != std::string::npos) {
			const std::string name = expression.substr(0, pos);
			std::string value = get_variable(name);

			if (value.empty()) {
				value = expression.substr(pos + 2);
				set_variable(name, value);
			}

			return value;
		}

		if (const std::size_t pos = expression.find('+'); pos != std::string::npos) {
			const std::string name = expression.substr(0, pos);
			return get_variable(name).empty() ? std::string{} : expression.substr(pos + 1);
		}

		if (const std::size_t pos = expression.find("%%"); pos != std::string::npos) {
			return remove_pattern_suffix(get_variable(expression.substr(0, pos)), expression.substr(pos + 2), true);
		}

		if (const std::size_t pos = expression.find('%'); pos != std::string::npos) {
			return remove_pattern_suffix(get_variable(expression.substr(0, pos)), expression.substr(pos + 1), false);
		}

		if (const std::size_t pos = expression.find("##"); pos != std::string::npos) {
			return remove_pattern_prefix(get_variable(expression.substr(0, pos)), expression.substr(pos + 2), true);
		}

		if (const std::size_t pos = expression.find('#'); pos != std::string::npos) {
			return remove_pattern_prefix(get_variable(expression.substr(0, pos)), expression.substr(pos + 1), false);
		}

		return get_variable(expression);
	}

	static std::string remove_pattern_prefix(const std::string &value, const std::string &pattern, bool longest)
	{
		std::size_t match_index = std::string::npos;

		for (std::size_t i = 0; i <= value.size(); ++i) {
			if (glob_match(pattern, value.substr(0, i))) {
				match_index = i;

				if (!longest) {
					break;
				}
			}
		}

		return match_index == std::string::npos ? value : value.substr(match_index);
	}

	static std::string remove_pattern_suffix(const std::string &value, const std::string &pattern, bool longest)
	{
		std::size_t match_index = std::string::npos;

		for (std::size_t i = value.size(); i > 0; --i) {
			if (glob_match(pattern, value.substr(i - 1))) {
				match_index = i - 1;

				if (!longest) {
					break;
				}
			}
		}

		return match_index == std::string::npos ? value : value.substr(0, match_index);
	}

	std::vector<std::string> expand_glob(const std::string &pattern)
	{
		if (pattern.find_first_of("*?") == std::string::npos) {
			return {pattern};
		}

		fs::path path_pattern(pattern);
		fs::path directory = path_pattern.parent_path();

		if (directory.empty()) {
			directory = _cwd;
		}

		std::vector<std::string> matches;
		const std::string file_pattern = path_pattern.filename().string();

		if (fs::exists(directory)) {
			for (const auto &entry : fs::directory_iterator(directory)) {
				if (glob_match(file_pattern, entry.path().filename().string())) {
					matches.push_back(entry.path().lexically_normal().string());
				}
			}
		}

		if (matches.empty()) {
			return {pattern};
		}

		std::sort(matches.begin(), matches.end());
		return matches;
	}

	static std::vector<std::string> split_patterns(const std::string &pattern_line)
	{
		std::vector<std::string> patterns;
		std::string current;

		for (char ch : pattern_line) {
			if (ch == '|') {
				patterns.push_back(trim(current));
				current.clear();

			} else {
				current += ch;
			}
		}

		if (!current.empty()) {
			patterns.push_back(trim(current));
		}

		return patterns;
	}

	static std::string join(const std::vector<std::string> &values, const std::string &separator)
	{
		std::string joined;

		for (std::size_t i = 0; i < values.size(); ++i) {
			if (i > 0) {
				joined += separator;
			}

			joined += values[i];
		}

		return joined;
	}

	static std::string format_number(double value)
	{
		std::ostringstream stream;
		stream.precision(12);
		stream << std::fixed << value;
		std::string text = stream.str();

		while (text.size() > 1 && text.back() == '0') {
			text.pop_back();
		}

		if (!text.empty() && text.back() == '.') {
			text.pop_back();
		}

		return text.empty() ? "0" : text;
	}

	double evaluate_numeric_expression(const std::string &expression, bool integer_mode)
	{
		struct Parser {
			const Shell &shell;
			const std::string expr;
			std::size_t pos{0};
			bool integer{false};

			void skip_ws()
			{
				while (pos < expr.size() && std::isspace(static_cast<unsigned char>(expr[pos]))) {
					++pos;
				}
			}

			double parse_number_or_variable()
			{
				skip_ws();

				if (pos < expr.size() && expr[pos] == '(') {
					++pos;
					double value = parse_expression();
					skip_ws();

					if (pos < expr.size() && expr[pos] == ')') {
						++pos;
					}

					return value;
				}

				const std::size_t start = pos;

				if (pos < expr.size() && (std::isdigit(static_cast<unsigned char>(expr[pos])) || expr[pos] == '.')) {
					while (pos < expr.size()
					       && (std::isdigit(static_cast<unsigned char>(expr[pos])) || expr[pos] == '.')) {
						++pos;
					}

					return std::atof(expr.substr(start, pos - start).c_str());
				}

				while (pos < expr.size()
				       && (std::isalnum(static_cast<unsigned char>(expr[pos])) || expr[pos] == '_')) {
					++pos;
				}

				const std::string variable = expr.substr(start, pos - start);
				const std::string value = shell.get_variable(variable);
				return value.empty() ? 0.0 : std::atof(value.c_str());
			}

			double parse_factor()
			{
				skip_ws();

				if (pos < expr.size() && expr[pos] == '-') {
					++pos;
					return -parse_factor();
				}

				return parse_number_or_variable();
			}

			double parse_term()
			{
				double value = parse_factor();

				while (true) {
					skip_ws();

					if (pos >= expr.size()) {
						return value;
					}

					const char op = expr[pos];

					if (op != '*' && op != '/' && op != '%') {
						return value;
					}

					++pos;
					double rhs = parse_factor();

					if (op == '*') {
						value *= rhs;

					} else if (op == '/') {
						value /= rhs;

					} else if (op == '%') {
						value = std::fmod(value, rhs);
					}
				}
			}

			double parse_expression()
			{
				double value = parse_term();

				while (true) {
					skip_ws();

					if (pos >= expr.size()) {
						return value;
					}

					const char op = expr[pos];

					if (op != '+' && op != '-') {
						return value;
					}

					++pos;
					double rhs = parse_term();
					value = op == '+' ? value + rhs : value - rhs;
				}
			}
		};

		Parser parser{*this, expression, 0, integer_mode};
		double value = parser.parse_expression();
		return integer_mode ? static_cast<long long>(value) : value;
	}
};

} // namespace

bool is_available()
{
	return true;
}

const char *backend_name()
{
	return "embedded-windows";
}

int run_script(const std::string &script_path,
	       const std::string &binary_dir,
	       int instance)
{
	Shell shell(binary_dir, instance);
	return shell.run_script(script_path);
}

} // namespace px4::embedded_shell
