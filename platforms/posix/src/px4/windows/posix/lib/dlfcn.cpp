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
 * @file dlfcn.cpp
 *
 * POSIX dlfcn(3): dlopen / dlsym / dlclose / dlerror backed by
 * Win32 LoadLibraryA / GetProcAddress / FreeLibrary.
 *
 * Windows has no `RTLD_*` resolution modes that match ELF. The flag argument
 * is intentionally ignored; PX4 uses this for optional runtime lookups, where
 * loading by DLL name and resolving an exported symbol is enough.
 */

#include "px4_windows_internal.h"

/* --------------------------------------------------------------------------
 * dlfcn: LoadLibrary-backed.
 * -------------------------------------------------------------------------- */
static thread_local char _dl_last_error[256] = {};

extern "C" void *dlopen(const char *filename, int /*flag*/)
{
	/* POSIX allows dlopen(NULL, ...) to reference the main program. On Windows
	 * the equivalent is the current process module. */
	HMODULE h = filename ? LoadLibraryA(filename) : GetModuleHandleA(nullptr);

	if (!h) { snprintf(_dl_last_error, sizeof(_dl_last_error), "LoadLibrary(%s) failed: 0x%lx", filename ? filename : "(self)", (unsigned long)GetLastError()); }

	return (void *)h;
}
extern "C" int dlclose(void *handle)
{
	if (!handle) { return 0; }

	return FreeLibrary((HMODULE)handle) ? 0 : -1;
}
extern "C" void *dlsym(void *handle, const char *symbol)
{
	FARPROC p = GetProcAddress(handle ? (HMODULE)handle : GetModuleHandleA(nullptr), symbol);

	if (!p) { snprintf(_dl_last_error, sizeof(_dl_last_error), "GetProcAddress(%s) failed: 0x%lx", symbol, (unsigned long)GetLastError()); }

	return (void *)p;
}
extern "C" char *dlerror(void)
{
	if (_dl_last_error[0] == 0) { return nullptr; }

	static thread_local char out[sizeof(_dl_last_error)];
	memcpy(out, _dl_last_error, sizeof(out));
	_dl_last_error[0] = 0;
	return out;
}
extern "C" int dladdr(const void *addr, Dl_info *info)
{
	if (!addr || !info) {
		errno = EINVAL;
		return 0;
	}

	MEMORY_BASIC_INFORMATION mbi {};

	if (VirtualQuery(addr, &mbi, sizeof(mbi)) == 0 || !mbi.AllocationBase) {
		return 0;
	}

	HMODULE module = (HMODULE)mbi.AllocationBase;
	static thread_local char module_name[MAX_PATH];

	/* dladdr() on ELF can usually report the nearest symbol too. Windows does
	 * not expose that cheaply without walking debug data, and PX4 only needs
	 * the containing module, so dli_sname/dli_saddr stay null. */
	DWORD len = GetModuleFileNameA(module, module_name, sizeof(module_name));

	if (len == 0 || len >= sizeof(module_name)) {
		module_name[0] = '\0';
	}

	info->dli_fname = module_name[0] ? module_name : nullptr;
	info->dli_fbase = mbi.AllocationBase;
	info->dli_sname = nullptr;
	info->dli_saddr = nullptr;
	return 1;
}

extern "C" void *dlvsym(void *handle, const char *symbol, const char *version)
{
	(void)version;
	return dlsym(handle, symbol);
}
