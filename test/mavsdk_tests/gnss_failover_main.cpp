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

// gnss_failover: injects one GNSS failure from a companion computer while a pilot flies, and records what the
// vehicle does. It never arms or changes the mode, refuses to inject unless the vehicle is flying in a
// position-controlled mode with both receivers at a 3D fix, and ends the injection on exit and on SIGINT or SIGTERM.

#include "gnss_failover.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

using namespace mavsdk;

namespace
{

std::atomic<bool> stop_requested{false};

void handle_signal(int)
{
	stop_requested = true;
}

void usage(const char *name)
{
	std::cout << "Usage: " << name << " --instance N --type off|wrong|slow|stuck [options]\n"
		  << "\n"
		  << "  --url URL          MAVLink connection (default udpin://0.0.0.0:14540)\n"
		  << "  --instance N       receiver to fail, 1-based (sensor_gnss instance + 1)\n"
		  << "  --type TYPE        off, wrong, slow or stuck\n"
		  << "  --duration S       seconds of vehicle time to keep the failure (default 20)\n"
		  << "  --observe S        seconds to keep recording after the failure ends (default 20)\n"
		  << "  --record FILE      CSV record: wall time [us], vehicle time [us], what (default stdout)\n"
		  << "  --ground           allow injecting on the ground or disarmed (bench test, props off)\n"
		  << "  wrong:  --fix N (SYS_FAIL_GPS_WRG, 0 = unchanged, default 2) --eph M --epv M --sacc M\n"
		  << "          --sats N --jamming N --spoofing N (0 = unchanged)\n"
		  << "  slow:   --divider N (SYS_FAIL_GPS_DIV, default 4)\n";
}

bool parse_type(const std::string &text, Failure::FailureType &type)
{
	if (text == "off") { type = Failure::FailureType::Off; return true; }

	if (text == "wrong") { type = Failure::FailureType::Wrong; return true; }

	if (text == "slow") { type = Failure::FailureType::Slow; return true; }

	if (text == "stuck") { type = Failure::FailureType::Stuck; return true; }

	return false;
}

} // namespace

int main(int argc, char **argv)
{
	std::string url = "udpin://0.0.0.0:14540";
	std::string record_path;
	double duration_s = 20.;
	double observe_s = 20.;
	bool ground = false;
	bool type_given = false;
	GnssFailover::Injection injection{};
	injection.instance = -1;

	for (int i = 1; i < argc; i++) {
		const std::string arg = argv[i];
		const bool has_value = (i + 1 < argc);
		const std::string value = has_value ? argv[i + 1] : "";

		try {
			if (arg == "-h" || arg == "--help") { usage(argv[0]); return 0; }

			else if (arg == "--ground") { ground = true; continue; }

			else if (!has_value) { usage(argv[0]); return 1; }

			else if (arg == "--url") { url = value; }

			else if (arg == "--instance") { injection.instance = std::stoi(value); }

			else if (arg == "--type") {
				if (!parse_type(value, injection.type)) { usage(argv[0]); return 1; }

				type_given = true;
			}

			else if (arg == "--duration") { duration_s = std::stod(value); }

			else if (arg == "--observe") { observe_s = std::stod(value); }

			else if (arg == "--record") { record_path = value; }

			else if (arg == "--fix") { injection.wrong.fix_type = std::stoi(value); }

			else if (arg == "--eph") { injection.wrong.eph = std::stof(value); }

			else if (arg == "--epv") { injection.wrong.epv = std::stof(value); }

			else if (arg == "--sacc") { injection.wrong.speed_accuracy = std::stof(value); }

			else if (arg == "--sats") { injection.wrong.satellites = std::stoi(value); }

			else if (arg == "--jamming") { injection.wrong.jamming_state = std::stoi(value); }

			else if (arg == "--spoofing") { injection.wrong.spoofing_state = std::stoi(value); }

			else if (arg == "--divider") { injection.slow_divider = std::stoi(value); }

			else { usage(argv[0]); return 1; }

		} catch (const std::exception &) {
			std::cerr << "Invalid value for " << arg << ": " << value << std::endl;
			return 1;
		}

		i++;
	}

	// One receiver at a time: failing all of them is a total loss, not a failover
	if (!type_given || injection.instance < 1) {
		usage(argv[0]);
		return 1;
	}

	std::ofstream record_file;

	if (!record_path.empty()) {
		record_file.open(record_path, std::ios::app);

		if (!record_file) {
			std::cerr << "Cannot open " << record_path << std::endl;
			return 1;
		}
	}

	std::ostream &record = record_path.empty() ? std::cout : record_file;

	Mavsdk mavsdk{Mavsdk::Configuration{ComponentType::CompanionComputer}};

	if (mavsdk.add_any_connection(url) != ConnectionResult::Success) {
		std::cerr << "Cannot connect to " << url << std::endl;
		return 1;
	}

	const std::optional<std::shared_ptr<System>> system = mavsdk.first_autopilot(10.);

	if (!system) {
		std::cerr << "No autopilot on " << url << std::endl;
		return 1;
	}

	GnssFailover gnss(*system, true);
	gnss.set_record(&record);

	if (!gnss.request_streams()) {
		std::cerr << "No ODOMETRY from the vehicle" << std::endl;
		return 1;
	}

	// Both receivers' messages, at their requested rate
	gnss.sleep(2.);

	const std::string refusal = gnss.injection_refusal(!ground);

	if (!refusal.empty()) {
		std::cerr << "Not injecting: " << refusal << std::endl;
		gnss.record("refused: " + refusal);
		return 1;
	}

	std::signal(SIGINT, handle_signal);
	std::signal(SIGTERM, handle_signal);

	const int64_t start_us = gnss.vehicle_time_us();
	const unsigned resets_before = gnss.reset_count();
	const bool injected = gnss.inject(injection);

	const auto record_receivers = [&gnss]() {
		std::ostringstream text;

		for (int i = 0; i < 2; i++) {
			const GnssFailover::Receiver receiver = gnss.receiver(i);
			text << (i == 0 ? "GPS_RAW_INT" : " GPS2_RAW") << " fix " << static_cast<int>(receiver.fix_type)
			     << " sats " << static_cast<int>(receiver.satellites) << " eph " << receiver.eph
			     << " age " << (gnss.vehicle_time_us() - receiver.vehicle_time_us) / 1000 << " ms";
		}

		gnss.record(text.str());
	};

	const auto run_for = [&](double seconds) {
		const int64_t end_us = gnss.vehicle_time_us() + static_cast<int64_t>(seconds * 1e6);

		while (!stop_requested && gnss.vehicle_time_us() < end_us) {
			record_receivers();
			gnss.wait_until([]() { return stop_requested.load(); }, 1.);
		}
	};

	if (injected) {
		run_for(duration_s);
	}

	// The failure stays latched in the vehicle until cleared, so clear it however the run ends
	const bool cleared = gnss.clear(injection.instance);
	const bool restored = gnss.restore_parameters();

	if (!stop_requested) {
		run_for(observe_s);
	}

	const bool position_ok = gnss.position_ok_since(start_us);
	std::ostringstream summary;
	summary << "summary: injected " << (injected ? "yes" : "no") << ", cleared " << (cleared ? "yes" : "no")
		<< ", parameters restored " << (restored ? "yes" : "no") << ", estimator resets " << gnss.reset_count() - resets_before
		<< ", position ok throughout " << (position_ok ? "yes" : "no") << ", modes";

	for (const Telemetry::FlightMode mode : gnss.modes_since(start_us)) {
		summary << " " << mode;
	}

	gnss.record(summary.str());
	std::cerr << summary.str() << std::endl;

	return (injected && cleared && restored) ? 0 : 1;
}
