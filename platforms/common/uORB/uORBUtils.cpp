/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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

#include "uORBUtils.hpp"
#include <stdio.h>
#include <errno.h>
#include <string.h>

int uORB::Utils::node_mkpath(char *buf, const struct orb_metadata *meta, int *instance,
			     const char *namespace_prefix)
{
	unsigned len;

	unsigned index = 0;

	if (instance != nullptr) {
		index = *instance;
	}

#if defined(__PX4_POSIX) && !defined(POSIX_SHM_DISABLED)
	len = snprintf(buf, orb_maxpath, "%s%s%s%d", namespace_prefix, orb_name_prefix, meta->o_name, index);
#else
	(void)namespace_prefix;
	len = snprintf(buf, orb_maxpath, "%s%d", meta->o_name, index);
#endif

	if (len >= orb_maxpath) {
		return -ENAMETOOLONG;
	}

	return OK;
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
int uORB::Utils::node_mkpath(char *buf, const char *orbMsgName, const char *namespace_prefix)
{
	unsigned len;

	unsigned index = 0;

#if defined(__PX4_POSIX) && !defined(POSIX_SHM_DISABLED)
	len = snprintf(buf, orb_maxpath, "%s%s%s%d", namespace_prefix, orb_name_prefix, orbMsgName, index);
#else
	(void)namespace_prefix;
	len = snprintf(buf, orb_maxpath, "%s%d", orbMsgName, index);
#endif

	if (len >= orb_maxpath) {
		return -ENAMETOOLONG;
	}

	return OK;
}

int uORB::Utils::manager_mkpath(char *buf, const char *namespace_prefix)
{
#if defined(__PX4_POSIX) && !defined(POSIX_SHM_DISABLED)
	const unsigned len = snprintf(buf, orb_maxpath, "%s%s", namespace_prefix, orb_manager_name);
#else
	(void)namespace_prefix;
	const unsigned len = snprintf(buf, orb_maxpath, "%s", orb_manager_name);
#endif

	if (len >= orb_maxpath) {
		return -ENAMETOOLONG;
	}

	return OK;
}

bool uORB::Utils::is_uorb_node_path(const char *path, const char *namespace_prefix)
{
	if (path == nullptr || namespace_prefix == nullptr) {
		return false;
	}

#if defined(__PX4_POSIX) && !defined(POSIX_SHM_DISABLED)
	const size_t namespace_len = strlen(namespace_prefix);

	if (strncmp(path, namespace_prefix, namespace_len) != 0) {
		return false;
	}

	const char *name = path + namespace_len;

	return strncmp(name, orb_name_prefix, strlen(orb_name_prefix)) == 0;
#else
	(void)namespace_prefix;
	return path[0] != '\0' && path[0] != '_';
#endif
}
