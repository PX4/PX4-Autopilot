/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
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
 * @file test_main_silence.cpp
 *
 * Suppress the MSVC Debug CRT invalid-parameter handler so unit tests that
 * intentionally exercise bad-fd / bad-handle paths (e.g. fcntl(0, 0xBAD),
 * flock(-1, ...), _open on a closed fd) do not abort with a debug-time
 * assertion dialog. Without this hook the Debug CRT raises Watson and
 * terminates the process before gtest can record the EXPECT_* result.
 *
 * Linked into every shim unit test target via
 * platforms/posix/src/px4/windows/tests/CMakeLists.txt; takes effect at
 * static-init time before main() runs gtest.
 */

#ifdef _WIN32
#include <stdlib.h>
#include <crtdbg.h>

namespace
{
static void silent_invalid_param(const wchar_t *, const wchar_t *, const wchar_t *,
				 unsigned, uintptr_t) {}
struct InstallSilencer {
	InstallSilencer()
	{
		_set_invalid_parameter_handler(silent_invalid_param);
		_CrtSetReportMode(_CRT_ASSERT, 0);
		_CrtSetReportMode(_CRT_ERROR, 0);
		_CrtSetReportMode(_CRT_WARN, 0);
	}
};
static InstallSilencer kInstallSilencer;
} // namespace
#endif
