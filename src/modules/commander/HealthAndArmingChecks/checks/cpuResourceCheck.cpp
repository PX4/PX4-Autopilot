/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#include "cpuResourceCheck.hpp"

using namespace time_literals;

CpuResourceChecks::CpuResourceChecks()
{
	_high_cpu_load_hysteresis.set_hysteresis_time_from(false, 2_s);
	_high_cpu_load_hysteresis.set_hysteresis_time_from(true, 2_s);
}

void CpuResourceChecks::checkAndReport(const Context &context, Report &reporter)
{
	const bool cpu_load_check_enabled = _param_com_cpu_max.get() > FLT_EPSILON;
	const bool ram_usage_check_enabled = _param_com_ram_max.get() > FLT_EPSILON;

	if (!cpu_load_check_enabled && !ram_usage_check_enabled) {
		return;
	}

	cpuload_s cpuload;

	if (!_cpuload_sub.copy(&cpuload) || hrt_elapsed_time(&cpuload.timestamp) > 2_s) {

		/* EVENT
		 * @description
		 * <profile name="dev">
		 * If the system does not provide any CPU and RAM load information, use the parameters <param>COM_CPU_MAX</param>
		 * and <param>COM_RAM_MAX</param> to disable the checks.
		 * </profile>
		 */
		reporter.healthFailure(NavModes::All, health_component_t::system, events::ID("check_missing_cpuload"),
				       events::Log::Error, "No CPU and RAM load information");

		if (reporter.mavlink_log_pub()) {
			mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: No CPU and RAM load information");
		}

	} else {
		const float cpuload_percent = cpuload.load * 100.f;
		const bool high_cpu_load = cpuload_percent > _param_com_cpu_max.get();
		_high_cpu_load_hysteresis.set_state_and_update(high_cpu_load, hrt_absolute_time());

		// fail check if CPU load is above the threshold for 2 seconds
		if (cpu_load_check_enabled && _high_cpu_load_hysteresis.get_state()) {
			/* EVENT
			 * @description
			 * The CPU load can be reduced for example by disabling unused modules (e.g. mavlink instances) or reducing the gyro update
			 * rate via <param>IMU_GYRO_RATEMAX</param>.
			 *
			 * <profile name="dev">
			 * The threshold can be adjusted via <param>COM_CPU_MAX</param> parameter.
			 * </profile>
			 */
			reporter.healthFailure<float>(NavModes::All, health_component_t::system, events::ID("check_cpuload_too_high"),
						      events::Log::Error, "CPU load too high: {1:.1}%", cpuload_percent);

			if (reporter.mavlink_log_pub()) {
				mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: CPU load too high: %3.1f%%", (double)cpuload_percent);
			}
		}

		const float ram_usage_percent = cpuload.ram_usage * 100.f;
		const bool high_ram_usage = ram_usage_percent > _param_com_ram_max.get();

		if (ram_usage_check_enabled && high_ram_usage) {
			/* EVENT
			 * @description
			 * The RAM usage can be reduced for example by disabling unused modules (e.g. mavlink instances).
			 *
			 * <profile name="dev">
			 * The threshold can be adjusted via <param>COM_RAM_MAX</param> parameter.
			 * </profile>
			 */
			reporter.healthFailure<float>(NavModes::All, health_component_t::system, events::ID("check_ram_usage_too_high"),
						      events::Log::Error, "RAM usage too high: {1:.1}%", ram_usage_percent);

			if (reporter.mavlink_log_pub()) {
				mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: RAM usage too high: %3.1f%%",
						     (double)ram_usage_percent);
			}
		}
	}
}
