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

#include "mavlink_ftp_path.h"

#include <cstddef>
#include <cstring>

#include <px4_platform_common/defines.h>

namespace
{
// Consumed by ROMFS/px4fmu_common/init.d/rcS on the next boot.
//
// This list must be kept in sync with the source points in rcS: if rcS starts sourcing
// another file from storage, add it here.
//
// Whole directories are protected where possible, which also covers staged copies (rcS
// moves ext_autostart_new into place).
static constexpr const char *kProtectedPaths[] = {
	PX4_STORAGEDIR "/etc/config.txt",     // rcS: parameter overrides
	PX4_STORAGEDIR "/etc/extras.txt",     // rcS: additional startup apps
	PX4_STORAGEDIR "/etc/rc.txt",         // rcS: replaces the whole boot
	PX4_STORAGEDIR "/ext_autostart",      // rcS: external airframe script
	PX4_STORAGEDIR "/ext_autostart_new",  // rcS: staged copy, moved into place during boot
};

enum class SegmentResult {
	Ok,        ///< a component that can be compared as text
	End,       ///< no component left
	Ambiguous, ///< a component that can name a protected entry without spelling it
};

char to_lower(char c)
{
	return (c >= 'A' && c <= 'Z') ? static_cast<char>(c + ('a' - 'A')) : c;
}

/**
 * Return the next path component, folded the way the storage filesystem folds it.
 *
 * The path is never rewritten: this only reports where a component starts and how long it
 * is, so the caller can compare it against another path. That keeps the string handed to
 * open()/rename()/unlink() identical to the string checked here.
 */
SegmentResult next_segment(const char **cursor, const char *&begin, size_t &len)
{
	const char *p = *cursor;

	for (;;) {
		// Empty components ("//", and leading or trailing '/') name nothing.
		while (*p == '/') {
			++p;
		}

		if (*p == '\0') {
			*cursor = p;
			return SegmentResult::End;
		}

		begin = p;

		while (*p != '\0' && *p != '/') {
			++p;
		}

		len = static_cast<size_t>(p - begin);
		*cursor = p;

		// "." is the current directory and names nothing.
		if (len == 1 && begin[0] == '.') {
			continue;
		}

		break;
	}

	// FAT drops trailing dots and spaces, so "extras.txt." and "extras.txt" are the same
	// entry. Fold them away for the comparison only.
	while (len > 0 && (begin[len - 1] == '.' || begin[len - 1] == ' ')) {
		--len;
	}

	// What is left of ".", "..", "..." or "   " names a directory that is not spelled
	// out. ".." in particular is not resolved here: resolving it needs the filesystem,
	// and the path we check has to stay the path that open() receives.
	if (len == 0) {
		return SegmentResult::Ambiguous;
	}

	// NuttX FAT keeps an 8.3 alias for every long name (CONFIG_FAT_LFN with
	// ALIAS_HASH), shaped "XXXXXX~1" with the '~' in position 1..6. The alias resolves
	// to the same entry, so a long-named protected directory could be reached without
	// spelling it. Refuse the alias namespace rather than individual aliases, which are
	// derived from a hash.
	for (size_t i = 1; i < 7 && i < len; ++i) {
		if (begin[i] == '~') {
			return SegmentResult::Ambiguous;
		}
	}

	return SegmentResult::Ok;
}

bool same_segment(const char *a, size_t a_len, const char *b, size_t b_len)
{
	if (a_len != b_len) {
		return false;
	}

	for (size_t i = 0; i < a_len; ++i) {
		// Storage is FAT, so the comparison is case-insensitive. Folded by hand
		// instead of strncasecmp() to keep this usable on NuttX.
		if (to_lower(a[i]) != to_lower(b[i])) {
			return false;
		}
	}

	return true;
}

/// True when @p path is @p protected_path or below it, comparing component by component.
bool is_same_or_below(const char *path, const char *protected_path)
{
	const char *path_cursor = path;
	const char *protected_cursor = protected_path;

	for (;;) {
		const char *protected_begin = nullptr;
		const char *path_begin = nullptr;
		size_t protected_len = 0;
		size_t path_len = 0;

		const SegmentResult protected_result = next_segment(&protected_cursor, protected_begin, protected_len);

		if (protected_result != SegmentResult::Ok) {
			// The protected path is exhausted, so @p path is the entry itself or a child
			// of it. Ambiguous here would mean the table above is malformed; fail closed.
			return true;
		}

		const SegmentResult path_result = next_segment(&path_cursor, path_begin, path_len);

		if (path_result != SegmentResult::Ok) {
			return path_result == SegmentResult::Ambiguous;
		}

		if (!same_segment(path_begin, path_len, protected_begin, protected_len)) {
			return false;
		}
	}
}
} // namespace

bool mavlink_ftp_path_is_write_protected(const char *path)
{
	if (path == nullptr) {
		return true;
	}

	// Every component has to be comparable as text before any of them is matched: a
	// component that resolves to a directory that is not spelled out, or that is a FAT
	// short-name alias, can name a protected entry without containing its name.
	for (const char *cursor = path;;) {
		const char *begin = nullptr;
		size_t len = 0;
		const SegmentResult result = next_segment(&cursor, begin, len);

		if (result == SegmentResult::End) {
			break;
		}

		if (result == SegmentResult::Ambiguous) {
			return true;
		}
	}

	for (const char *protected_path : kProtectedPaths) {
		if (is_same_or_below(path, protected_path)) {
			return true;
		}
	}

	return false;
}
