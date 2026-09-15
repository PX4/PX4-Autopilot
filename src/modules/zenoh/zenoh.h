/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

#ifndef ZENOH_MODULE_H
#define ZENOH_MODULE_H

#include <termios.h>
#include <poll.h>
#include <sys/select.h>
#include <sys/time.h>

#include <px4_platform_common/module_params.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/atomic.h>
#include <drivers/drv_hrt.h>
#include <perf/perf_counter.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/parameter_update.h>

#include "zenoh_config.hpp"
#include "publishers/uorb_publisher.hpp"
#include "subscribers/uorb_subscriber.hpp"


using namespace time_literals;

class ZENOH : public ModuleBase, public ModuleParams
{
public:
	static Descriptor desc;

	ZENOH();

	~ZENOH();


	/**
	 * @see ModuleBase::custom_command
	 */
	static int custom_command(int argc, char *argv[]);

	/**
	 * @see ModuleBase::print_usage
	 */
	static int print_usage(const char *reason = nullptr);

	/**
	 * @see ModuleBase::print_usage
	 */
	int print_status();

	/**
	 * @see ModuleBase::task_spawn
	 */
	static int task_spawn(int argc, char *argv[]);

	static int run_trampoline(int argc, char *argv[]);

	static ZENOH *instantiate(int argc, char *argv[]);

	void run() override;

private:
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::ZENOH_DOMAIN_ID>) _zenoh_domain_id,
		(ParamInt<px4::params::ZENOH_PUB_CC>) _zenoh_pub_cc,
		(ParamInt<px4::params::ZENOH_PUB_REL>) _zenoh_pub_rel,
		(ParamInt<px4::params::ZENOH_PUB_EXPR>) _zenoh_pub_expr,
		(ParamInt<px4::params::ZENOH_PUB_PRIO>) _zenoh_pub_prio
	)

	int generate_rmw_zenoh_node_liveliness_keyexpr(const z_id_t *id, char *keyexpr);
	int generate_rmw_zenoh_topic_keyexpr(const char *topic, const uint8_t *rihs_hash, char *type, char *keyexpr);
	int generate_rmw_zenoh_topic_liveliness_keyexpr(const z_id_t *id, const char *topic, const uint8_t *rihs_hash,
			char *type, char *keyexpr, const char *entity_str);
	int setupSession();
	int setupTopics(px4_pollfd_struct_t *pfds);
	void cleanupSession();

	/**
	 * @brief Waits until the interface behind the locator reports a link (NuttX only).
	 *
	 * Polls IFF_UP | IFF_RUNNING of the interface named by the locator's "iface=" entry
	 * (default eth0), 50 polls with 100 ms sleeps (5 s nominal), see zenoh_link_wait.hpp.
	 * IFF_RUNNING is what the driver reports through netdev_carrier_on(). The imxrt driver only
	 * does so once apache/nuttx a4315fbb is backported to PX4/NuttX, and then at ifup, so even
	 * with the backport it does not prove a physical link; without it the wait always times out
	 * on fmu-v6xrt. On timeout the session open goes ahead anyway: a missing carrier report or
	 * a companion that is still booting must not keep the module from ever trying. On POSIX
	 * there is no wait.
	 * @param[in] locator zenoh locator, e.g. "tcp/10.41.10.1:7447#iface=eth0"
	 * @return false if a stop was requested while waiting
	 */
	bool waitForLink(const char *locator);

	/**
	 * @brief Sleeps in slices of kStopCheckInterval, returning early on a stop request.
	 * @param[in] duration Time to sleep [us]
	 * @return false if a stop was requested
	 */
	bool sleepInterruptible(hrt_abstime duration);

	// [us] Delay between session open attempts after a failed z_open()
	static constexpr hrt_abstime kSessionRetryDelay = 5_s;
	// [us] Granularity at which waits check for a stop request
	static constexpr hrt_abstime kStopCheckInterval = 100_ms;

	Zenoh_Config _config;

	int _pub_count;
	uORB_Zenoh_Publisher **_zenoh_publishers = nullptr;
	int _sub_count;
	Zenoh_Subscriber **_zenoh_subscribers = nullptr;

	z_owned_session_t _s;
	// written by the module task, read by print_status() from the shell task
	px4::atomic_bool _connected{false};
	px4::atomic_bool _waiting_for_link{false};

	px4_guid_t _px4_guid{};

};

#endif //ZENOH_MODULE_H
