// SPDX-License-Identifier: BSD-3-Clause

#include <lib/dragonfly_turtle/ArmingInterlock.hpp>
#include <cassert>
#include <atomic>
#include <thread>
#include <iostream>
using namespace dragonfly_trtl;
Interlock *other_translation_unit();
int main()
{
	assert(other_translation_unit() == &interlock());
	uint8_t state = 1;
	{
		ArmGuard a(state, 2, true);
		assert(!a.allowed());
	}
	interlock().setReady(true);
	{
		ArmGuard a(state, 2, true);        // failed arm frees reservation
		assert(a.allowed());
		assert(!interlock().acquireRecovery());
	}
	assert(interlock().owner() == Owner::NONE);
	{
		ArmGuard a(state, 2, true);
		assert(a.allowed());
		state = 2;
	}
	assert(interlock().owner() == Owner::FLIGHT);
	{
		DisarmGuard a(state, 2);
		state = 1;
	}
	assert(interlock().owner() == Owner::NONE);
	assert(interlock().acquireRecovery());
	{
		ArmGuard a(state, 2, false);
		assert(!a.allowed());
	}
	interlock().releaseRecovery();
	// A default, non-capable installation retains the existing output/arming path.
	{
		Interlock plain;
		assert(!plain.directionRequired());
		assert(!plain.directionPending(false, false, false));
		assert(plain.acquireFlight(false));
		plain.releaseFlight();
		assert(plain.directionPending(true, false, false));
	}

	// A boot-time capability obligation survives runtime recovery disable. Neither
	// an absent configuration nor an incomplete command sequence permits output.
	{
		Interlock capable;
		capable.requireDirection();
		assert(capable.directionRequired());
		assert(capable.directionPending(false, false, false));
		assert(capable.directionPending(false, false, true));
		assert(capable.directionPending(false, true, false));
		assert(!capable.acquireFlight(false));
		assert(capable.owner() == Owner::NONE);
		capable.setReady(true);
		assert(!capable.directionPending(false, true, true));

		// Restoration readiness remains valid across repeated arm/disarm cycles.
		for (unsigned n = 0; n < 100; ++n) {
			assert(capable.acquireFlight(false));
			assert(!capable.directionPending(false, true, true));
			capable.releaseFlight();
		}

		capable.setReady(false);
		assert(capable.directionRequired());
		assert(!capable.acquireFlight(false));
		capable.setReady(true);
		assert(capable.directionRequired());
		assert(capable.acquireRecovery());
		assert(!capable.acquireFlight(false));
		capable.releaseRecovery();
	}

	// Driver discovery can follow an early arm reservation. The physical throttle
	// gate must still report pending; owner=FLIGHT is not a bypass.
	{
		Interlock late;
		assert(late.acquireFlight(false));
		late.requireDirection();
		assert(late.owner() == Owner::FLIGHT);
		assert(late.directionPending(false, false, false));
		assert(late.directionPending(false, true, false));
		late.releaseFlight();
		assert(!late.acquireFlight(false));
		assert(late.acquireRecovery());
		late.setReady(true);
		late.releaseRecovery();
		assert(!late.directionPending(false, true, true));
		assert(late.acquireFlight(false));
		late.releaseFlight();
	}

	// Configuration validation cannot steal a flight reservation. Early returns
	// release a successful reservation; a fault deliberately survives the guard.
	{
		Interlock configuration;
		configuration.setReady(true);
		assert(configuration.acquireFlight(false));
		{
			ConfigurationGuard guard(configuration);
			assert(!guard.allowed());
		}
		assert(configuration.owner() == Owner::FLIGHT);
		configuration.releaseFlight();
		{
			ConfigurationGuard guard(configuration);
			assert(guard.allowed());
			assert(!configuration.acquireFlight(false));
		}
		assert(configuration.owner() == Owner::NONE);
		{
			ConfigurationGuard guard(configuration);
			assert(guard.allowed());
			configuration.latchFault();
		}
		assert(configuration.owner() == Owner::FAULT);
		assert(!configuration.acquireFlight(false));
	}

	Interlock lock;
	lock.setReady(true);
	std::atomic<int> owner_count{0};
	auto flight = [&]() {
		for (int i = 0; i < 100000; ++i) {
			if (lock.acquireFlight(true)) {
				assert(owner_count.fetch_add(1) == 0);
				assert(owner_count.fetch_sub(1) == 1);
				lock.releaseFlight();
			}
		}
	};
	auto recovery = [&]() {
		for (int i = 0; i < 100000; ++i) {
			ConfigurationGuard guard(lock);

			if (guard.allowed()) {
				assert(owner_count.fetch_add(1) == 0);
				assert(owner_count.fetch_sub(1) == 1);
			}
		}
	};
	std::thread a(flight), b(recovery);
	a.join();
	b.join();
	std::cout <<
		  "PASS: Commander guard lifetimes, sticky direction/output gates, cross-translation-unit identity, 200000 concurrent reservation attempts\n";
}
