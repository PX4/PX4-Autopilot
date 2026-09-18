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

#ifndef _uORBCommon_hpp_
#define _uORBCommon_hpp_

#include <drivers/drv_orb_dev.h>
#include <systemlib/err.h>
#include "uORB.h"
#include <uORB/topics/uORBTopics.hpp>
#include <drivers/drv_hrt.h>


namespace uORB
{
static constexpr char orb_manager_name[] = "_uORB_Manager";
static constexpr unsigned orb_manager_name_length = sizeof(orb_manager_name) - 1;

#if defined(__PX4_POSIX) && !defined(POSIX_SHM_DISABLED)
static constexpr char orb_name_prefix[] = "_orb_";
static constexpr unsigned orb_max_namespace_prefix_length = 20;
#else
static constexpr char orb_name_prefix[] = "";
static constexpr unsigned orb_max_namespace_prefix_length = 0;
#endif

static constexpr unsigned orb_name_prefix_length = sizeof(orb_name_prefix) - 1;
static constexpr unsigned orb_node_name_length = ORB_MAX_TOPIC_NODE_NAME_LENGTH + orb_name_prefix_length;
static constexpr unsigned orb_maxpath =
	orb_max_namespace_prefix_length
	+ (orb_node_name_length > orb_manager_name_length ? orb_node_name_length : orb_manager_name_length)
	+ 1;

#if defined(CONFIG_NAME_MAX) && defined(CONFIG_FS_SHMFS)
static_assert(CONFIG_NAME_MAX >= (orb_maxpath - 1), "CONFIG_NAME_MAX too small for uORB node names");
#endif

struct orb_advertdata {
	const struct orb_metadata *meta;
	int *instance;
};

}
#endif // _uORBCommon_hpp_
