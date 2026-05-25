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
 * @file if_query.cpp
 *
 * net/if.h surface: if_nameindex / if_freenameindex backed by
 * Win32 GetAdaptersAddresses. MAVLink uses this to build its
 * default network-discovery list.
 *
 * The returned POSIX array owns duplicated UTF-8 adapter names. Keep that
 * allocation shape in sync with if_freenameindex(); callers expect one malloc
 * family allocation per name plus one array allocation.
 */

#include "px4_windows_internal.h"

/* --------------------------------------------------------------------------
 * net/if.h surface. PX4's MAVLink module enumerates interfaces via
 * if_nameindex() to build its default network discovery list. Windows
 * provides the same information through GetAdaptersAddresses(). We
 * stuff the adapter's friendly name into POSIX-style entries and
 * return them through a heap-allocated array that if_freenameindex()
 * releases.
 * -------------------------------------------------------------------------- */
/* `if_nametoindex` and `if_indextoname` are provided by MinGW's
 * <netioapi.h> / iphlpapi; no need to reimplement. We only fill in
 * the POSIX `if_nameindex` / `if_freenameindex` helpers that MinGW
 * omits. */
extern "C" struct if_nameindex *if_nameindex(void)
{
	ULONG flags = GAA_FLAG_SKIP_ANYCAST | GAA_FLAG_SKIP_MULTICAST |
		      GAA_FLAG_SKIP_DNS_SERVER;
	ULONG size  = 0;
	GetAdaptersAddresses(AF_UNSPEC, flags, nullptr, nullptr, &size);

	if (size == 0) { errno = ENODEV; return nullptr; }

	IP_ADAPTER_ADDRESSES *adapters = (IP_ADAPTER_ADDRESSES *)malloc(size);

	if (!adapters) { errno = ENOMEM; return nullptr; }

	if (GetAdaptersAddresses(AF_UNSPEC, flags, nullptr, adapters, &size) != NO_ERROR) {
		free(adapters);
		errno = ENODEV;
		return nullptr;
	}

	unsigned int count = 0;

	for (IP_ADAPTER_ADDRESSES *a = adapters; a; a = a->Next) { count++; }

	struct if_nameindex *result = (struct if_nameindex *)calloc(count + 1, sizeof(*result));

	if (!result) { free(adapters); errno = ENOMEM; return nullptr; }

	unsigned int i = 0;

	for (IP_ADAPTER_ADDRESSES *a = adapters; a; a = a->Next, i++) {
		char name[IF_NAMESIZE] = {0};
		/* FriendlyName is UTF-16. POSIX callers get an IF_NAMESIZE-bounded
		 * UTF-8 string, which is what MAVLink logs and compares. */
		WideCharToMultiByte(CP_UTF8, 0, a->FriendlyName, -1, name, IF_NAMESIZE, nullptr, nullptr);
		result[i].if_index = a->IfIndex;
		result[i].if_name  = _strdup(name);

		/* if_freenameindex() uses a NULL if_name as the array sentinel and stops
		 * walking. A silent _strdup() failure mid-loop would truncate the iteration
		 * and leak the remaining names; bail out instead so the caller sees ENOMEM. */
		if (!result[i].if_name) {
			for (unsigned int j = 0; j < i; j++) { free(result[j].if_name); }

			free(result);
			free(adapters);
			errno = ENOMEM;
			return nullptr;
		}
	}

	free(adapters);
	return result;
}

extern "C" void if_freenameindex(struct if_nameindex *ptr)
{
	if (!ptr) { return; }

	for (struct if_nameindex *p = ptr; p->if_name; p++) {
		free(p->if_name);
	}

	free(ptr);
}
