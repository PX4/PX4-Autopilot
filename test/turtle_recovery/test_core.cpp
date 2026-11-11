// SPDX-License-Identifier: BSD-3-Clause

#include "DragonflyTurtleCore.hpp"
#include "DragonflyTurtleInput.hpp"
#include <cassert>
#include <iostream>
#include <limits>
using namespace dragonfly_trtl;
struct Rig {
	Interlock lock;
	Core core{lock};
	Input in;
	uint64_t now{1000000};
	Config cfg;
	Rig()
	{
		cfg.enabled = 1;
		cfg.esc = 1;
		cfg.outputs = 15;
		assert(core.configure(cfg));
		in.disarmed = in.hardware_ok = in.mapping_ok = in.rc_ok = in.throttle_low = in.attitude_ok = in.switch_off = true;
		in.vertical = -1;
		in.strength = 1;
		in.selected = 5;
	}
	Frame tick(unsigned us = 1000)
	{
		now += us;
		return core.step(now, in);
	}
	void boot(unsigned us = 1000)
	{
		for (unsigned i = 0; i < 10000000 / us && !core.synced(); ++i) {
			tick(us);
		}

		assert(core.synced());
		assert(core.phase == Phase::IDLE);
		assert(lock.ready());

		for (unsigned i = 0; i <= 300000 / us + 1; ++i) {
			tick(us);
		}
	}
	void on()
	{
		in.switch_on = true;
		in.switch_off = false;
	}
	void off()
	{
		in.switch_on = false;
		in.switch_off = true;
	}
	void drive(unsigned us = 1000)
	{
		on();

		for (unsigned i = 0; i < 10000000 / us && core.phase != Phase::DRIVE; ++i) {
			tick(us);
		}

		assert(core.phase == Phase::DRIVE);
	}
	void released(unsigned us = 1000)
	{
		off();

		for (unsigned i = 0; i < 15000000 / us && core.busy(); ++i) {
			tick(us);
		}

		assert(core.phase == Phase::IDLE);
		assert(lock.owner() == Owner::NONE);
	}
};
int main()
{
	unsigned scenarios = 0;

	for (unsigned period : {100u, 1000u, 2500u, 10000u, 20000u, 50000u}) {
		Rig r;
		r.boot(period);
		assert(r.core.restore_frames >= 24);
		assert(r.core.restore_sequences_sent == 1);
		r.drive(period);
		assert(!r.lock.acquireFlight(true));
		auto f = r.tick(period);
		assert(f.action == Action::THROTTLE && f.outputs == 5);
		r.in.vertical = std::cos(100.f * 0.0174532925f);
		f = r.tick(period);
		assert(f.action == Action::THROTTLE); // below entry still drives
		r.in.vertical = std::cos(40.f * 0.0174532925f);
		f = r.tick(period);
		assert(f.action == Action::THROTTLE);
		r.in.vertical = std::cos(30.f * 0.0174532925f);
		f = r.tick(period);
		assert(f.action == Action::STOP);
		assert(r.core.reason == Reason::UPRIGHT);
		r.released(period);
		assert(r.core.restore_sequences_sent == 2);
		r.in.vertical = -1;
		r.drive(period);
		uint64_t start = r.now;

		while (r.core.phase == Phase::DRIVE) {
			r.tick(period);
		}

		assert(r.core.reason == Reason::TIMEOUT);
		assert(r.core.last_drive_ms >= 1000 && r.core.last_drive_ms <= 1000 + period / 1000);
		assert(r.now - start <= 1000000 + period);
		r.released(period);
		assert(r.core.attempts == 2);
		++scenarios;
	}

	{
		Rig r;
		r.cfg.entry_deg = 85;
		r.cfg.exit_deg = 25;
		assert(r.core.configure(r.cfg));
		r.in.vertical = 0;
		r.boot();
		r.drive();
		assert(r.core.phase == Phase::DRIVE);
		++scenarios;
	}

	{
		Rig r;
		r.boot();
		r.in.vertical = 1;
		r.on();

		for (int i = 0; i < 1000; ++i) {
			r.tick();
		}

		assert(r.core.attempts == 0);

		r.in.vertical = -1;

		for (int i = 0; i < 2000; ++i) {
			r.tick();
		}

		assert(r.core.attempts == 0); // ON while upright cannot trigger when tilted later

		r.off();

		for (int i = 0; i < 400; ++i) {
			r.tick();
		}

		r.drive();
		++scenarios;
	}

	{
		Rig r;
		r.boot();
		r.in.disarmed = false;
		r.on();

		for (int i = 0; i < 500; ++i) {
			r.tick();
		}

		r.in.disarmed = true;

		for (int i = 0; i < 2000; ++i) {
			r.tick();
		}

		assert(r.core.attempts == 0);
		++scenarios;
	}

	{
		Rig r;
		r.boot();
		r.on();
		r.in.rc_ok = false;
		r.tick();
		r.in.rc_ok = true;

		for (int i = 0; i < 2000; ++i) {
			r.tick();
		}

		assert(r.core.attempts == 0);
		++scenarios;
	}

	for (int reason = 0; reason < 7; ++reason) {
		Rig r;
		r.boot();
		r.drive();

		if (reason == 0) {
			r.in.rc_ok = false;
		}

		if (reason == 1) {
			r.in.throttle_low = false;
		}

		if (reason == 2) {
			r.in.hardware_ok = false;
		}

		if (reason == 3) {
			r.in.disarmed = false;
		}

		if (reason == 4) {
			r.in.attitude_ok = false;
		}

		if (reason == 5) {
			r.in.switch_on = false;        // center is not OFF
		}

		if (reason == 6) {
			r.in.mapping_ok = false;
		}

		const auto f = r.tick();
		assert(f.action == Action::STOP);
		assert(r.core.phase != Phase::DRIVE);
		assert(!r.lock.acquireFlight(true));

		if (reason == 6) {
			assert(r.core.phase == Phase::FAULT);

		} else {
			r.in.rc_ok = r.in.throttle_low = r.in.hardware_ok = r.in.disarmed = r.in.attitude_ok = true;
			r.released();
		}

		++scenarios;
	}

	{
		Rig r;
		r.boot();
		r.drive();
		auto f = r.tick(101000);
		assert(f.action == Action::STOP);
		assert(r.core.reason == Reason::CALLBACK_GAP);
		r.released();
		++scenarios;
	}

	{
		Rig r;
		r.boot();
		r.drive();
		r.now = 1;
		auto f = r.core.step(r.now, r.in);
		assert(f.action == Action::STOP && r.core.phase == Phase::FAULT);
		++scenarios;
	}

	{
		Rig r;
		r.boot();
		r.drive();

		for (int i = 0; i < 350; ++i) {
			r.tick();
		}

		auto f = r.tick();
		assert(f.level == 1500);

		r.in.selected = 10;
		f = r.tick();
		assert(f.level == 850 && f.outputs == 10);
		r.in.selected = 0;
		assert(r.tick().action == Action::STOP);
		r.in.selected = 5;
		assert(r.tick().level == 850);

		for (int i = 0; i < 270; ++i) {
			r.tick();
		}

		r.in.strength = .6f;
		f = r.tick();
		assert(f.level >= 1174

		       && f.level <= 1176); // half above 20% deadband

		r.released();
		++scenarios;
	}

	{
		Rig r;
		r.cfg.drive_ms = 3000;
		r.cfg.ramp_ms = 1500;
		r.cfg.pre_ms = 5000;
		r.cfg.post_ms = 5000;
		assert(r.core.configure(r.cfg));
		r.boot(20000);
		r.drive(20000);

		while (r.core.phase == Phase::DRIVE) {
			r.tick(20000);
		}

		assert(r.core.last_drive_ms == 3000);
		r.released(20000);
		++scenarios;
	}

	{
		Rig r;
		r.boot();
		r.drive();
		Config b = r.cfg;
		b.peak = 1800;
		assert(!r.core.configure(b));
		assert(r.core.config().peak == 1500);
		r.released();
		assert(r.core.configure(b));
		++scenarios;
	}

	{
		Rig r;
		r.boot();
		r.on();
		r.tick();
		r.in.hardware_ok = false;

		for (int i = 0; i < 20000; ++i) {
			r.tick();
		}

		assert(r.core.phase == Phase::FAULT);
		assert(!r.lock.acquireFlight(false));
		++scenarios;
	}

	{
		Rig r;
		r.cfg.enabled = 0;
		assert(r.core.configure(r.cfg));
		r.on();
		r.boot();
		assert(r.core.restore_sequences_sent == 1);

		for (int i = 0; i < 10000; ++i) {
			assert(r.tick().action == Action::NORMAL);
		}

		assert(r.core.attempts == 0);
		++scenarios;
	}

	{
		Rig r;
		r.cfg.esc = 0;
		assert(r.core.configure(r.cfg));
		r.on();

		for (int i = 0; i < 2000; ++i) {
			assert(r.tick().action == Action::NORMAL);
		}

		++scenarios;
	}

	// Inconsistent bounds reject rather than silently changing the requested setting.
	Config c;
	c.outputs = 15;
	assert(c.valid());
	c.start = 1600;
	c.peak = 1500;
	assert(!c.valid());
	c.start = 850;
	c.ramp_ms = 1100;
	c.drive_ms = 1000;
	assert(!c.valid());
	c.ramp_ms = 250;
	c.entry_deg = 60;
	c.exit_deg = 55;
	assert(!c.valid());
	c.exit_deg = 35;
	c.polarity = 0;
	assert(!c.valid());
	assert(quadCorner(0, 1, -1, 0, 0, -1));
	assert(!quadCorner(0, 1, 0, 0, 0, -1));
	assert(!quadCorner(0, 1, -1, 0.2f, 0, -1));
	assert(quadCorner(0, 1, -1, 0, 0, -0.5f));
	assert(quadCorner(0, 1, -1, 0, 0, -100));
	assert(!quadCorner(0, 1, -1, 0, 0, 0));
	assert(!quadCorner(0, 1, -1, 0, 0, 1));
	float q[4] {0, 1, 0, 0};
	assert(vertical(q) == -1);
	q[0] = NAN;
	assert(!std::isfinite(vertical(q)));
	std::cout << "PASS: " << scenarios << " controller scenarios, 6 callback rates, config/RC/geometry/attitude validation\n";
}
