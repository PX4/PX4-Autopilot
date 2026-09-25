#pragma once
#include <lib/sitl_faults/SitlFaults.hpp>
#if defined(CONFIG_ARCH_BOARD_PX4_SITL)
#include <cerrno>
#include <cstdlib>
#include <cstring>

namespace sitl_faults
{
inline bool number(const char *text, unsigned &value)
{
	if (!text || text[0] < '0' || text[0] > '9') { return false; }

	char *end = nullptr;
	errno = 0;
	const auto n = strtoul(text, &end, 10);

	if (errno || *end || n > MaxDurationMs) { return false; }

	value = static_cast<unsigned>(n);
	return true;
}

inline unsigned direction(const char *text)
{
	if (strcmp(text, "tx") == 0) { return Tx; }

	if (strcmp(text, "rx") == 0) { return Rx; }

	if (strcmp(text, "both") == 0) { return Both; }

	return 0;
}

inline int command(int argc, char *argv[], bool enabled)
{
	if (argc == 3 && strcmp(argv[2], "reset") == 0) {
		state().reset();
		PX4_INFO("lab faults reset");
		return 0;
	}

	if (argc == 3 && strcmp(argv[2], "status") == 0) {
		const char *names[] = {"mavlink", "dds", "storage"};

		for (unsigned i = 0; i < 3; ++i) {
			const auto r = state().snapshot(static_cast<Lane>(i));
			const auto now = Clock::now();
			const auto remaining = now < r.end ? std::chrono::duration_cast<std::chrono::milliseconds>(r.end - now).count() : 0;
			PX4_INFO("%s %s remaining_ms=%ld port=%u direction=%u tx_drop=%llu rx_drop=%llu writes=%llu",
				 names[i], now < r.start ? "pending" : now < r.end ? "active" : "idle",
				 static_cast<long>(remaining), r.port, r.direction,
				 static_cast<unsigned long long>(r.tx_drops), static_cast<unsigned long long>(r.rx_drops),
				 static_cast<unsigned long long>(r.delayed_writes));
		}

		return 0;
	}

	if (!enabled) { PX4_ERR("Set SYS_FAILURE_EN=1 before configuring lab faults"); return 1; }

	unsigned port = 0, duration = 0, on = 0, period = 0, delay = 0, dir = Both;
	Lane lane = Lane::Mavlink;
	bool valid = false;

	if (argc == 8 && strcmp(argv[2], "fade") == 0) {
		dir = direction(argv[4]);
		valid = number(argv[3], port) && number(argv[5], duration) && number(argv[6], on) && number(argv[7], period);

	} else if (argc == 5 && strcmp(argv[2], "dds") == 0) {
		lane = Lane::Dds;
		dir = direction(argv[3]);
		valid = number(argv[4], duration);
		on = period = duration;

	} else if (argc == 5 && strcmp(argv[2], "storage") == 0) {
		lane = Lane::Storage;
		valid = number(argv[3], delay) && number(argv[4], duration);
	}

	if (!valid || !state().configure(lane, port, dir, duration, on, period, delay)) {
		PX4_ERR("Invalid request or lane busy. Durations 1..120000 ms; write delay 1..500 ms");
		PX4_INFO("failure lab fade PORT tx|rx|both DURATION_MS ON_MS PERIOD_MS");
		PX4_INFO("failure lab dds tx|rx|both DURATION_MS");
		PX4_INFO("failure lab storage DELAY_MS DURATION_MS; failure lab status|reset");
		return 1;
	}

	PX4_INFO("lab fault scheduled: starts in 1000 ms, expires automatically");
	return 0;
}
} // namespace sitl_faults

#endif
