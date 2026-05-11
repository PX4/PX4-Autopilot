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
 * @file dlfcn.h
 *
 * The systemcmds/dyn module uses dlopen/dlsym to load .so plugins at
 * runtime. On Windows we translate to LoadLibraryA / GetProcAddress /
 * FreeLibrary. Modes are mostly ignored - Windows has no lazy/now,
 * global/local distinction comparable to RTLD_* at the API level.
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#define RTLD_LAZY   0x0001
#define RTLD_NOW    0x0002
#define RTLD_GLOBAL 0x0100
#define RTLD_LOCAL  0x0000
#define RTLD_NODELETE 0x1000
#define RTLD_NOLOAD   0x0004
#define RTLD_DEEPBIND 0x0008
#define RTLD_DEFAULT ((void *)0)
#define RTLD_NEXT    ((void *)-1)

/**
 * @brief Load a dynamic library using Windows LoadLibraryA semantics.
 *
 * @param filename Library path. A NULL filename is accepted for source
 *                 compatibility but has no portable Windows equivalent.
 * @param flag POSIX RTLD_* mode bits. The shim accepts them for API
 *             compatibility; Windows binding semantics are selected by the
 *             loader, not by these flags.
 * @return Library handle on success, NULL on failure. Use dlerror() for text.
 */
void *dlopen(const char *filename, int flag);

/**
 * @brief Unload a library handle returned by dlopen().
 *
 * @return 0 on success, -1 on failure with dlerror() text set.
 */
int   dlclose(void *handle);

/**
 * @brief Look up an exported symbol in a library.
 *
 * @param handle Handle returned by dlopen().
 * @param symbol Export name to resolve.
 * @return Symbol address on success, NULL on failure.
 */
void *dlsym(void *handle, const char *symbol);

/**
 * @brief Fetch and clear the thread-local dynamic-loader error string.
 *
 * @return Last loader error string, or NULL if no error is pending.
 */
char *dlerror(void);

/**
 * @brief POSIX dladdr information container.
 *
 * Windows does not expose all fields with POSIX fidelity, but keeping this
 * type allows diagnostics and third-party code to compile.
 */
typedef struct {
	const char *dli_fname;
	void       *dli_fbase;
	const char *dli_sname;
	void       *dli_saddr;
} Dl_info;

/**
 * @brief Resolve best-effort module/symbol information for an address.
 *
 * @return Non-zero on success, 0 on failure.
 */
int   dladdr(const void *addr, Dl_info *info);

/**
 * @brief Versioned symbol lookup compatibility entry point.
 *
 * Windows exports are not ELF-versioned, so the implementation ignores
 * @p version and delegates to dlsym().
 */
void *dlvsym(void *handle, const char *symbol, const char *version);

#ifdef __cplusplus
}
#endif
