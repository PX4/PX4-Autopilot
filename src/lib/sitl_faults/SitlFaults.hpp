#pragma once

// Deliberately absent from hardware builds. Deadlines use wall time: a paused
// lockstep simulator must not leave a fault permanently armed.
#if defined(CONFIG_ARCH_BOARD_PX4_SITL)
#include <chrono>
#include <cstdint>
#include <mutex>

namespace sitl_faults
{
using Clock = std::chrono::steady_clock;
enum class Lane { Mavlink, Dds, Storage };
enum Direction : unsigned { Tx = 1, Rx = 2, Both = 3 };
constexpr unsigned MaxDurationMs = 120000;
constexpr unsigned StartDelayMs = 1000;
constexpr unsigned MaxWriteDelayMs = 500;

struct Rule {
	Clock::time_point start{}, end{};
	unsigned port{}, direction{}, on_ms{}, period_ms{}, delay_ms{};
	uint64_t tx_drops{}, rx_drops{}, delayed_writes{};
};

class Faults
{
public:
	bool configure(Lane lane, unsigned port, unsigned direction, unsigned duration_ms,
		       unsigned on_ms, unsigned period_ms, unsigned delay_ms, Clock::time_point now = Clock::now())
	{
		if (duration_ms == 0 || duration_ms > MaxDurationMs || direction < Tx || direction > Both
		    || delay_ms > MaxWriteDelayMs || (lane == Lane::Mavlink && (port == 0 || port > 65535))
		    || (lane != Lane::Storage && (on_ms == 0 || period_ms < on_ms))
		    || (lane == Lane::Storage && delay_ms == 0)) {
			return false;
		}

		std::lock_guard<std::mutex> lock(_mutex);
		auto &r = _rules[static_cast<unsigned>(lane)];

		if (now < r.end) { return false; }

		r = {};
		r.start = now + std::chrono::milliseconds(StartDelayMs);
		r.end = r.start + std::chrono::milliseconds(duration_ms);
		r.port = port;
		r.direction = direction;
		r.on_ms = on_ms;
		r.period_ms = period_ms;
		r.delay_ms = delay_ms;
		return true;
	}

	bool drop(Lane lane, unsigned direction, unsigned port = 0, Clock::time_point now = Clock::now())
	{
		std::lock_guard<std::mutex> lock(_mutex);
		auto &r = _rules[static_cast<unsigned>(lane)];

		if (lane == Lane::Storage || now < r.start || now >= r.end || !(r.direction & direction)
		    || (lane == Lane::Mavlink && r.port != port)) { return false; }

		const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - r.start).count();

		if (r.period_ms == 0 || elapsed % r.period_ms >= r.on_ms) { return false; }

		if (direction == Tx) { ++r.tx_drops; } else { ++r.rx_drops; }

		return true;
	}

	unsigned write_delay(Clock::time_point now = Clock::now())
	{
		std::lock_guard<std::mutex> lock(_mutex);
		auto &r = _rules[static_cast<unsigned>(Lane::Storage)];

		if (now < r.start || now >= r.end) { return 0; }

		++r.delayed_writes;
		const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(r.end - now).count();
		return remaining < r.delay_ms ? static_cast<unsigned>(remaining) : r.delay_ms;
	}

	Rule snapshot(Lane lane)
	{
		std::lock_guard<std::mutex> lock(_mutex);
		return _rules[static_cast<unsigned>(lane)];
	}

	void reset()
	{
		std::lock_guard<std::mutex> lock(_mutex);

		for (auto &r : _rules) { r = {}; }
	}

private:
	std::mutex _mutex;
	Rule _rules[3] {};
};

// One process-wide object shared by the built-in SITL modules.
inline Faults &state()
{
	static Faults faults;
	return faults;
}
} // namespace sitl_faults

#endif
