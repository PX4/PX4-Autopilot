// SPDX-License-Identifier: BSD-3-Clause

#include "DragonflyTurtleCore.hpp"
#include <cassert>
#include <iostream>
using namespace dragonfly_trtl;
struct Rig {
	Interlock lock;
	Core core{lock};
	Input in;
	uint64_t now = 1000000;
	Rig()
	{
		Config c;
		c.enabled = 1;
		c.esc = 1;
		c.outputs = 15;
		assert(core.configure(c));
		in.disarmed = in.hardware_ok = in.mapping_ok = in.rc_ok = in.throttle_low = in.attitude_ok = in.switch_off = true;
		in.vertical = -1;
		in.selected = 5;
		in.strength = 1;

		for (unsigned n = 0; n < 10000 && !core.synced(); ++n) {
			tick();
		}

		assert(core.synced());

		for (unsigned n = 0; n < 301; ++n) {
			tick();
		}
	}
	Frame tick()
	{
		now += 1000;
		return core.step(now, in);
	}
	void go(Phase p)
	{
		in.switch_on = true;
		in.switch_off = false;

		for (unsigned n = 0; n < 10000 && core.phase != p; ++n) {
			tick();
		}

		assert(core.phase == p);
	}
	void release()
	{
		in.disarmed = in.hardware_ok = in.mapping_ok = in.rc_ok = in.throttle_low = in.attitude_ok = in.switch_off = true;
		in.switch_on = false;

		for (unsigned n = 0; n < 10000 && core.busy(); ++n) {
			tick();
		}

		assert(core.phase == Phase::IDLE);
		assert(lock.ready());
	}
};
int main()
{
	unsigned cases = 0;

	for (Phase phase : {Phase::HOLD, Phase::PRESTOP, Phase::REVERSE, Phase::SETTLE, Phase::DRIVE}) {
		for (unsigned fault = 0; fault < 8; ++fault) {
			Rig r;
			r.go(phase);

			switch (fault) {
			case 0:
				r.in.rc_ok = false;
				break;

			case 1:
				r.in.switch_on = false;
				break;

			case 2:
				r.in.throttle_low = false;
				break;

			case 3:
				r.in.disarmed = false;
				break;

			case 4:
				r.in.hardware_ok = false;
				break;

			case 5:
				r.in.attitude_ok = false;
				break;

			case 6:
				r.in.vertical = 1;
				break;

			case 7:
				r.in.mapping_ok = false;
				break;
			}

			const auto frame = r.tick();
			assert(frame.action == Action::STOP);
			assert(!r.lock.acquireFlight(false));

			if (fault == 7) {
				assert(r.core.phase == Phase::FAULT);

			} else {
				r.release();
			}

			++cases;
		}
	}

	for (Phase p : {Phase::POSTSTOP, Phase::RESTORE, Phase::COOLDOWN, Phase::WAIT_OFF}) {
		Rig r;
		r.go(Phase::DRIVE);
		r.in.switch_on = false;

		for (unsigned n = 0; n < 10000 && r.core.phase != p; ++n) {
			r.tick();
		}

		assert(r.core.phase == p);

		r.in.hardware_ok = false;

		for (unsigned n = 0; n < 100; ++n) {
			assert(r.tick().action == Action::STOP);
			assert(!r.lock.acquireFlight(false));
		}

		r.release();
		++cases;
	}
	Rig r;

	for (unsigned n = 0; n < 1000; ++n) {
		r.go(Phase::DRIVE);

		for (unsigned i = 0; i < 50; ++i) {
			assert(r.tick().action == Action::THROTTLE);
		}

		r.release();
		assert(r.core.attempts == n + 1);
		assert(r.core.restore_sequences_sent == n + 2);
		assert(r.lock.acquireFlight(true));
		r.lock.releaseFlight();
	}

	std::cout << "PASS: " << cases << " phase/fault cases and 1000 repeat/release/arming cycles\n";
}
