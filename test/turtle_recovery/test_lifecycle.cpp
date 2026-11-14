// SPDX-License-Identifier: BSD-3-Clause

#include "DragonflyTurtleCore.hpp"
#include "DragonflyTurtleInput.hpp"
#include "DragonflyTurtleRc.hpp"

#include <cassert>
#include <cstdio>
#include <initializer_list>
#include <limits>

using namespace dragonfly_trtl;

struct Rig {
	Interlock lock;
	Core core{lock};
	Config cfg;
	Input input;
	uint64_t now{1000000};

	Rig()
	{
		cfg.enabled = 1;
		cfg.esc = 1;
		cfg.outputs = 15;
		assert(core.configure(cfg));
		input.disarmed = input.hardware_ok = input.mapping_ok = input.rc_ok = true;
		input.throttle_low = input.attitude_ok = input.switch_off = true;
		input.vertical = -1.f;
		input.selected = 5;
		input.strength = 1.f;
	}

	Frame tick(uint64_t elapsed_us = 1000)
	{
		now += elapsed_us;
		return core.step(now, input);
	}

	void off()
	{
		input.switch_on = false;
		input.switch_off = true;
	}

	void on()
	{
		input.switch_on = true;
		input.switch_off = false;
	}

	void until(Phase phase)
	{
		for (unsigned n = 0; n < 15000 && core.phase != phase; ++n) {
			tick();
		}

		assert(core.phase == phase);
	}

	void boot()
	{
		for (unsigned n = 0; n < 10000 && !core.synced(); ++n) {
			tick();
		}

		assert(core.synced() && core.phase == Phase::IDLE && lock.ready());

		for (unsigned n = 0; n < 301; ++n) {
			tick();
		}
	}

	void drive()
	{
		on();
		tick();
		until(Phase::DRIVE);
	}

	void release()
	{
		off();
		tick();
		until(Phase::IDLE);
		assert(lock.owner() == Owner::NONE && !core.ownsOutputs());
	}
};

// Models RCUpdate's unchanged-input publication interval.
// ManualControl republishes the selected sample while preserving its sample time.
struct CadenceRig : Rig {
	uint64_t raw_period_us;
	uint64_t raw_stamp{now};
	uint64_t sample_stamp{now};

	explicit CadenceRig(uint64_t period_us) : raw_period_us(period_us) {}

	Frame cadenceTick()
	{
		now += 1000;

		if (now - raw_stamp >= raw_period_us) {
			raw_stamp = now;

			if (now - sample_stamp > 300000) {
				sample_stamp = now;
			}
		}

		input.rc_ok = dragonfly_turtle_rc::healthy(now, raw_stamp, 0, false)
			      && manualInputFresh(now, now, sample_stamp);
		return core.step(now, input);
	}

	void cadenceUntil(Phase phase)
	{
		for (unsigned n = 0; n < 10000 && core.phase != phase; ++n) {
			cadenceTick();
		}

		assert(core.phase == phase);
	}
};

int main()
{
	unsigned scenarios = 0;

	// A programming transaction delays startup without acquiring the output lock.
	{
		Rig r;
		r.input.commands_idle = false;

		for (unsigned n = 0; n < 2000; ++n) {
			assert(r.tick().action == Action::NORMAL);
		}

		assert(!r.core.synced() && r.lock.owner() == Owner::NONE);
		r.input.commands_idle = true;
		r.boot();
		++scenarios;
	}

	// Periodic telemetry cannot consume a qualified request while it is pending.
	{
		Rig r;
		r.boot();
		r.input.commands_idle = false;
		r.on();

		for (unsigned n = 0; n < 1000; ++n) {
			r.tick();
		}

		assert(r.core.phase == Phase::IDLE && r.lock.owner() == Owner::NONE);
		r.input.commands_idle = true;
		r.tick();
		assert(r.core.phase == Phase::HOLD);
		++scenarios;
	}

	// Disabling stops drive but retains ownership until restoration and OFF release.
	{
		Rig r;
		r.boot();
		r.drive();
		r.input.recovery_enabled = false;
		assert(r.tick().action == Action::STOP);
		assert(r.core.reason == Reason::DISABLED);
		assert(!r.lock.acquireFlight(false));
		r.release();
		assert(r.core.restore_sequences_sent == 2);
		r.on();

		for (unsigned n = 0; n < 1000; ++n) {
			assert(r.tick().action == Action::NORMAL);
		}

		assert(r.core.attempts == 1);
		++scenarios;
	}

	// Same-boot recreation must retain an interrupted owner's fault, even disabled.
	{
		Interlock lock;
		Config cfg;
		cfg.enabled = cfg.esc = 1;
		cfg.outputs = 15;
		Input input;
		input.disarmed = input.hardware_ok = input.mapping_ok = true;

		{
			Core first{lock};
			assert(first.configure(cfg));
			first.step(1000000, input);
			assert(first.ownsOutputs());
		}

		assert(lock.owner() == Owner::FAULT);
		assert(!lock.acquireFlight(false));
		cfg.enabled = 0;
		Core recreated{lock};
		assert(recreated.configure(cfg));
		assert(recreated.step(1001000, input).action == Action::STOP);
		assert(recreated.phase == Phase::FAULT && recreated.reason == Reason::RESTART);
		++scenarios;
	}

	// A clean stopped instance invalidates readiness, requiring fresh normalisation.
	{
		Interlock lock;
		Config cfg;
		cfg.enabled = cfg.esc = 1;
		cfg.outputs = 15;
		Input input;
		input.disarmed = input.hardware_ok = input.mapping_ok = input.rc_ok = true;
		input.throttle_low = input.switch_off = true;

		{
			Core first{lock};
			assert(first.configure(cfg));

			for (uint64_t now = 1000000; now < 10000000 && !first.synced(); now += 1000) {
				first.step(now, input);
			}

			assert(first.synced() && lock.ready());
		}

		assert(lock.owner() == Owner::NONE && !lock.ready());
		Core recreated{lock};
		assert(recreated.configure(cfg));
		assert(recreated.step(11000000, input).action == Action::STOP);
		assert(recreated.ownsOutputs() && !recreated.synced());
		++scenarios;
	}

	// Missing or malformed switch data must discard prior OFF qualification.
	for (unsigned variant = 0; variant < 6; ++variant) {
		Rig r;
		r.boot();
		float channels[9] {};
		channels[8] = -1.f;
		uint8_t count = 9;

		switch (variant) {
		case 0:
			count = 8;
			break;

		case 1:
			channels[8] = std::numeric_limits<float>::quiet_NaN();
			break;

		case 2:
			channels[8] = std::numeric_limits<float>::infinity();
			break;

		case 3:
			channels[8] = -std::numeric_limits<float>::infinity();
			break;

		case 4:
			channels[8] = 1.1f;
			break;

		case 5:
			channels[8] = -1.1f;
			break;
		}

		const float selected = dragonfly_turtle_rc::select(channels, count, 9);
		assert(!std::isfinite(selected));
		r.input.rc_ok = std::isfinite(selected);
		r.tick();
		r.input.rc_ok = true;
		r.on();

		for (unsigned n = 0; n < 1000; ++n) {
			r.tick();
		}

		assert(r.core.phase == Phase::IDLE && r.core.attempts == 0);
		r.off();

		for (unsigned n = 0; n < 310; ++n) {
			r.tick();
		}

		r.drive();
		++scenarios;
	}

	// Neither previously-qualified nor unqualified IDLE may count a callback gap.
	for (bool was_qualified : {false, true}) {
		Rig r;
		r.boot();

		if (!was_qualified) {
			r.input.rc_ok = false;
			r.tick();
			r.input.rc_ok = true;
			r.tick();
		}

		r.tick(2000000);
		r.on();
		r.tick();
		assert(r.core.phase == Phase::IDLE && r.lock.owner() == Owner::NONE);
		++scenarios;
	}

	// WAIT_OFF retains ownership across a gap until a complete new OFF dwell.
	{
		Rig r;
		r.boot();
		r.drive();
		r.input.switch_on = false;
		r.input.switch_off = false;
		r.tick();
		r.until(Phase::WAIT_OFF);
		r.off();
		r.tick();
		r.tick(2000000);
		assert(r.core.phase == Phase::WAIT_OFF && !r.lock.acquireFlight(false));

		for (unsigned n = 0; n < 299; ++n) {
			r.tick();
		}

		assert(r.core.phase == Phase::WAIT_OFF);
		r.tick();
		assert(r.core.phase == Phase::IDLE && r.lock.owner() == Owner::NONE);
		++scenarios;
	}

	// Held inputs at different raw RC cadences remain valid through recovery.
	for (uint64_t period_us : {2000u, 10000u, 20000u, 50000u, 280000u}) {
		CadenceRig r{period_us};

		for (unsigned n = 0; n < 10000 && !r.core.synced(); ++n) {
			r.cadenceTick();
		}

		assert(r.core.synced() && r.lock.ready());

		for (unsigned n = 0; n < 301; ++n) {
			r.cadenceTick();
		}

		r.on();
		r.cadenceTick();
		r.cadenceUntil(Phase::DRIVE);

		for (unsigned n = 0; n < 700; ++n) {
			assert(r.cadenceTick().action == Action::THROTTLE);
		}

		r.input.vertical = 1.f;
		assert(r.cadenceTick().action == Action::STOP);
		assert(r.core.reason == Reason::UPRIGHT);
		r.off();
		r.cadenceUntil(Phase::IDLE);
		assert(r.lock.ready() && r.lock.owner() == Owner::NONE);
		++scenarios;
	}

	// A gap in restoration cannot count as continuous stopped/command time.
	for (Phase interrupted : {Phase::POSTSTOP, Phase::RESTORE, Phase::COOLDOWN}) {
		Rig r;
		r.boot();
		r.drive();
		r.input.switch_on = false;
		r.input.switch_off = false;
		r.tick();
		r.until(interrupted);

		if (interrupted == Phase::RESTORE) {
			// The first frame was sent when POSTSTOP transitioned to RESTORE.
			for (unsigned n = 0; n < 22; ++n) {
				r.tick();
			}
		}

		const uint32_t before_frames = r.core.restore_frames;
		const uint32_t before_sequences = r.core.restore_sequences_sent;
		assert(r.tick(200000).action == Action::STOP);
		assert(r.core.phase == Phase::POSTSTOP);
		assert(r.core.reason == Reason::CALLBACK_GAP);
		assert(r.core.restore_frames == before_frames);
		assert(r.core.restore_sequences_sent == before_sequences);
		assert(r.core.ownsOutputs() && !r.lock.acquireFlight(false));

		for (int n = 0; n < r.cfg.post_ms - 1; ++n) {
			assert(r.tick().action == Action::STOP);
			assert(r.core.phase == Phase::POSTSTOP);
		}

		assert(r.tick().action == Action::RESTORE);
		const uint64_t restart_command = r.now;
		r.until(Phase::COOLDOWN);
		assert(r.core.restore_frames - before_frames >= 24);
		assert(r.now - restart_command >= 50000);
		assert(r.core.restore_sequences_sent == before_sequences + 1);
		r.release();
		++scenarios;
	}

	// Repeated interruptions retain the original transaction budget.
	{
		Rig r;
		r.boot();
		r.drive();
		r.input.switch_on = false;
		r.input.switch_off = false;
		r.tick();

		for (unsigned n = 0; n < 80 && r.core.phase != Phase::FAULT; ++n) {
			assert(r.tick(200000).action == Action::STOP);
			assert(r.core.ownsOutputs() && !r.lock.acquireFlight(false));
		}

		assert(r.core.phase == Phase::FAULT && r.core.reason == Reason::COMMAND_TIMEOUT);
		assert(r.lock.owner() == Owner::FAULT);
		++scenarios;
	}

	// A capable installation restores at boot even with EN=0 and no RC at all.
	// Enabling recovery later cannot reuse an unobserved OFF edge or cause a
	// second normalisation on an otherwise-ready output set.
	{
		Rig r;
		r.cfg.enabled = 0;
		assert(r.core.configure(r.cfg));
		r.input.recovery_enabled = false;
		r.input.rc_ok = r.input.throttle_low = r.input.switch_off = false;

		for (unsigned n = 0; n < 10000 && !r.core.synced(); ++n) {
			const auto frame = r.tick();
			assert(frame.action != Action::THROTTLE && frame.action != Action::REVERSE);
		}

		assert(r.core.capable() && !r.core.enabled());
		assert(r.core.synced() && r.lock.ready() && r.lock.owner() == Owner::NONE);
		assert(r.core.phase == Phase::IDLE && r.core.restore_sequences_sent == 1);
		assert(r.core.restore_frames >= 24 && r.core.attempts == 0);
		r.cfg.enabled = 1;
		assert(r.core.configure(r.cfg));
		assert(r.core.synced());
		r.input.recovery_enabled = true;
		r.input.rc_ok = r.input.throttle_low = true;
		r.on();

		for (unsigned n = 0; n < 1000; ++n) {
			assert(r.tick().action == Action::NORMAL);
		}

		assert(r.core.attempts == 0 && r.core.restore_sequences_sent == 1);
		r.off();

		for (unsigned n = 0; n < 301; ++n) {
			r.tick();
		}

		r.drive();
		assert(r.core.attempts == 1);
		++scenarios;
	}

	// Ordinary successful arm/disarm cycles do not send direction commands or
	// introduce stopped dwell. EN changes also leave completed restoration valid.
	{
		Rig r;
		r.boot();
		const uint32_t sent = r.core.restore_sequences_sent;
		const uint32_t frames = r.core.restore_frames;

		for (unsigned cycle = 0; cycle < 100; ++cycle) {
			assert(r.lock.acquireFlight(true));
			r.input.disarmed = false;
			assert(r.tick().action == Action::NORMAL);
			r.lock.releaseFlight();
			r.input.disarmed = true;
			assert(r.tick().action == Action::NORMAL);
			r.cfg.enabled = int32_t(cycle % 2);
			assert(r.core.configure(r.cfg));
			r.input.recovery_enabled = r.cfg.enabled != 0;
			assert(r.tick().action == Action::NORMAL);
			assert(r.core.synced() && r.lock.ready());
		}

		assert(r.core.restore_sequences_sent == sent && r.core.restore_frames == frames);
		++scenarios;
	}

	// The reset model discards FC RAM while retaining the last ESC command.
	// Verify startup restoration order after the controller state is cleared.
	{
		bool esc_reversed = false;

		{
			Rig before;
			before.boot();
			before.on();

			for (unsigned n = 0; n < 10000 && before.core.phase != Phase::DRIVE; ++n) {
				const auto frame = before.tick();

				if (frame.action == Action::REVERSE) {
					esc_reversed = true;
				}
			}

			assert(before.core.phase == Phase::DRIVE && esc_reversed);
		}

		Rig after;
		after.cfg.enabled = 0;
		assert(after.core.configure(after.cfg));
		after.input.recovery_enabled = false;
		after.input.rc_ok = after.input.throttle_low = after.input.switch_off = false;

		for (unsigned n = 0; n < 10000 && !after.core.synced(); ++n) {
			const auto frame = after.tick();
			assert(frame.action != Action::THROTTLE && frame.action != Action::REVERSE);

			if (frame.action == Action::RESTORE) {
				assert(frame.outputs == 15);
				esc_reversed = false;
			}
		}

		assert(!esc_reversed && after.core.synced() && after.lock.ready());
		assert(after.core.attempts == 0 && after.core.restore_sequences_sent == 1);
		++scenarios;
	}

	// Disable requests never bypass restoration or switch-OFF release after a
	// real recovery request, regardless of the phase that receives the request.
	for (Phase disabled_at : {Phase::HOLD, Phase::PRESTOP, Phase::REVERSE, Phase::SETTLE, Phase::DRIVE}) {
		Rig r;
		r.boot();
		r.on();
		r.tick();
		r.until(disabled_at);
		r.input.recovery_enabled = false;
		assert(r.tick().action == Action::STOP);
		assert(r.core.reason == Reason::DISABLED);
		assert(!r.lock.acquireFlight(false));
		r.until(Phase::WAIT_OFF);

		for (unsigned n = 0; n < 1000; ++n) {
			assert(r.tick().action == Action::STOP);
			assert(!r.lock.acquireFlight(false));
		}

		assert(r.core.restore_sequences_sent == 2);
		r.release();
		assert(r.lock.ready());
		++scenarios;
	}

	// Callback gaps also restart the disabled-startup STOP dwell, and repeated
	// interruptions cannot extend the original restoration budget indefinitely.
	{
		Rig r;
		r.cfg.enabled = 0;
		assert(r.core.configure(r.cfg));
		r.input.recovery_enabled = false;
		r.input.rc_ok = r.input.throttle_low = r.input.switch_off = false;
		r.until(Phase::RESTORE);
		const uint32_t frames = r.core.restore_frames;
		assert(r.tick(200000).action == Action::STOP);
		assert(r.core.phase == Phase::POSTSTOP && r.core.restore_frames == frames);

		for (unsigned n = 0; n < 80 && r.core.phase != Phase::FAULT; ++n) {
			assert(r.tick(200000).action == Action::STOP);
		}

		assert(r.core.phase == Phase::FAULT && r.core.reason == Reason::COMMAND_TIMEOUT);
		assert(!r.lock.ready() && r.lock.owner() == Owner::FAULT);
		++scenarios;
	}

	// Stopping a ready driver still invalidates its readiness when EN is zero.
	{
		Interlock lock;
		Config cfg;
		cfg.enabled = 0;
		cfg.esc = 1;
		cfg.outputs = 15;
		Input input;
		input.disarmed = input.hardware_ok = input.mapping_ok = true;
		input.recovery_enabled = false;

		{
			Core first{lock};
			assert(first.configure(cfg));

			for (uint64_t stamp = 1000000; stamp < 10000000 && !first.synced(); stamp += 1000) {
				first.step(stamp, input);
			}

			assert(first.synced() && lock.ready());
		}

		assert(lock.owner() == Owner::NONE && !lock.ready());
		Core restarted{lock};
		assert(restarted.configure(cfg));
		assert(restarted.step(11000000, input).action == Action::STOP);
		assert(restarted.ownsOutputs() && !restarted.synced());
		++scenarios;
	}

	const uint64_t now = 2000000;
	assert(manualInputFresh(now, now, now));
	assert(manualInputFresh(now, now - 299999, now - 749999));
	assert(!manualInputFresh(now, now - 300000, now));
	assert(!manualInputFresh(now, now, now - 750000));
	assert(!manualInputFresh(now, 0, now));
	assert(!manualInputFresh(now, now, 0));
	assert(!manualInputFresh(now, now + 1, now));
	assert(!manualInputFresh(now, now, now + 1));
	assert(!manualInputFresh(0, 0, 0));
	assert(!dragonfly_turtle_rc::healthy(now, now - 300000, 0, false));
	assert(dragonfly_turtle_rc::healthy(now, now - 299999, 0, false));
	std::printf("PASS: %u lifecycle/request/cadence scenarios and 11 timestamp boundaries\n", scenarios);
	return 0;
}
