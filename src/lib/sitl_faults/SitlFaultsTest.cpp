#include "SitlFaults.hpp"
#include <cassert>
#include <thread>
#include <vector>

// Standalone test: c++ -std=c++14 -pthread -DCONFIG_ARCH_BOARD_PX4_SITL SitlFaultsTest.cpp
int main()
{
	using namespace sitl_faults;
	Faults faults;
	const Clock::time_point now{};
	const auto begin = now + std::chrono::milliseconds(StartDelayMs);
	assert(faults.configure(Lane::Mavlink, 14601, Rx, 5000, 200, 1000, 0, now));
	assert(!faults.configure(Lane::Mavlink, 14601, Rx, 5000, 200, 1000, 0, now));
	assert(!faults.drop(Lane::Mavlink, Rx, 14601, now));
	assert(!faults.drop(Lane::Mavlink, Tx, 14601, begin));
	assert(!faults.drop(Lane::Mavlink, Rx, 14550, begin));
	assert(faults.drop(Lane::Mavlink, Rx, 14601, begin));
	assert(!faults.drop(Lane::Mavlink, Rx, 14601, begin + std::chrono::milliseconds(200)));
	assert(faults.drop(Lane::Mavlink, Rx, 14601, begin + std::chrono::milliseconds(1000)));
	assert(!faults.drop(Lane::Mavlink, Rx, 14601, begin + std::chrono::milliseconds(5000)));
	assert(faults.snapshot(Lane::Mavlink).rx_drops == 2);
	assert(faults.configure(Lane::Dds, 0, Both, 10000, 10000, 10000, 0, now));
	assert(faults.drop(Lane::Dds, Tx, 0, begin));
	assert(faults.drop(Lane::Dds, Rx, 0, begin));
	assert(!faults.configure(Lane::Storage, 0, Both, 1000, 0, 0, 501, now));
	assert(faults.configure(Lane::Storage, 0, Both, 1000, 0, 0, 100, now));
	assert(faults.write_delay(now) == 0);
	assert(faults.write_delay(begin) == 100);
	assert(faults.write_delay(begin + std::chrono::milliseconds(999)) == 1);
	assert(faults.write_delay(begin + std::chrono::milliseconds(1000)) == 0);
	faults.reset();
	assert(!faults.drop(Lane::Dds, Tx, 0, begin));
	assert(!faults.configure(Lane::Mavlink, 0, Tx, 1000, 1, 1, 0, now));
	assert(!faults.configure(Lane::Dds, 0, 0, 1000, 1, 1, 0, now));
	assert(!faults.configure(Lane::Dds, 0, Both, 120001, 1, 1, 0, now));
	assert(!faults.configure(Lane::Dds, 0, Both, 1000, 2, 1, 0, now));
	// Concurrent receiver/sender/status/reset accesses share one lock.
	std::vector<std::thread> workers;

	for (unsigned i = 0; i < 4; ++i) {
		workers.emplace_back([&] {
			for (unsigned n = 0; n < 1000; ++n)
			{
				faults.configure(Lane::Dds, 0, Both, 1000, 1000, 1000, 0, now);
				faults.drop(Lane::Dds, Tx, 0, begin);
				faults.snapshot(Lane::Dds);
				faults.reset();
			}
		});
	}

	for (auto &worker : workers) { worker.join(); }
}
