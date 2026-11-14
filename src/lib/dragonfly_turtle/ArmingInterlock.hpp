// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <stdint.h>

// Shared by Commander and DShot within one PX4 process.
// The reservation does not persist across a flight-controller reset.
namespace dragonfly_trtl
{

static_assert(__atomic_always_lock_free(sizeof(uint32_t), nullptr), "Turtle recovery requires lock-free 32-bit atomics");
static_assert(__atomic_always_lock_free(sizeof(bool), nullptr), "Turtle recovery requires lock-free boolean atomics");

enum class Owner : uint32_t { NONE, FLIGHT, RECOVERY, FAULT };

class Interlock
{
public:
	Interlock() = default;
	Interlock(const Interlock &) = delete;
	Interlock &operator=(const Interlock &) = delete;

	bool acquireRecovery()
	{
		uint32_t expected = uint32_t(Owner::NONE);
		return __atomic_compare_exchange_n(&_owner, &expected, uint32_t(Owner::RECOVERY), false,
						   __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST);
	}

	bool acquireFlight(bool require_ready)
	{
		if ((require_ready || directionRequired()) && !ready()) {
			return false;
		}

		uint32_t expected = uint32_t(Owner::NONE);

		if (!__atomic_compare_exchange_n(&_owner, &expected, uint32_t(Owner::FLIGHT), false,
						 __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST)) {
			return false;
		}

		// Recheck after claiming ownership: recovery may have invalidated readiness.
		if ((require_ready || directionRequired()) && !ready()) {
			releaseFlight();
			return false;
		}

		return true;
	}

	void releaseFlight()
	{
		release(Owner::FLIGHT);
	}
	void releaseRecovery()
	{
		release(Owner::RECOVERY);
	}
	void setReady(bool ready)
	{
		__atomic_store_n(&_ready, ready, __ATOMIC_SEQ_CST);
	}
	bool ready() const
	{
		return __atomic_load_n(&_ready, __ATOMIC_SEQ_CST);
	}
	// Capability is latched for this FC boot. Disabling recovery cannot erase a
	// direction obligation, including across driver destruction/recreation.
	void requireDirection()
	{
		__atomic_store_n(&_direction_required, true, __ATOMIC_SEQ_CST);
	}
	bool directionRequired() const
	{
		return __atomic_load_n(&_direction_required, __ATOMIC_SEQ_CST);
	}
	// Physical output gating must remain effective even if flight ownership was
	// acquired before the driver discovered a pending direction obligation.
	bool directionPending(bool runtime_requested, bool configured, bool synced) const
	{
		return (runtime_requested || directionRequired()) && (!configured || !synced);
	}
	Owner owner() const
	{
		return Owner(__atomic_load_n(&_owner, __ATOMIC_SEQ_CST));
	}

	bool recoveryOwned() const
	{
		const Owner current = owner();
		return current == Owner::RECOVERY || current == Owner::FAULT;
	}

	// Only the recovery owner may invoke this. A fault deliberately survives driver
	// destruction/recreation within this boot; release operations cannot clear it.
	void latchFault()
	{
		setReady(false);
		__atomic_store_n(&_owner, uint32_t(Owner::FAULT), __ATOMIC_SEQ_CST);
	}

private:
	void release(Owner expected_owner)
	{
		uint32_t expected = uint32_t(expected_owner);
		__atomic_compare_exchange_n(&_owner, &expected, uint32_t(Owner::NONE), false,
					    __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST);
	}

	alignas(4) uint32_t _owner{uint32_t(Owner::NONE)};
	bool _ready{false};
	bool _direction_required{false};
};

// Inline function-local storage provides one object across translation units.
inline Interlock &interlock()
{
	static Interlock value;
	return value;
}

// Parameter validation must not invalidate readiness or latch a mapping fault
// after Commander has claimed FLIGHT. No waiting or hardware I/O is permitted
// inside this short reservation. A latched FAULT cannot be released by it.
class ConfigurationGuard
{
public:
	explicit ConfigurationGuard(Interlock &lock) : _lock(lock), _reserved(lock.acquireRecovery()) {}
	~ConfigurationGuard()
	{
		if (_reserved) {
			_lock.releaseRecovery();
		}
	}
	bool allowed() const
	{
		return _reserved;
	}
	ConfigurationGuard(const ConfigurationGuard &) = delete;
	ConfigurationGuard &operator=(const ConfigurationGuard &) = delete;

private:
	Interlock &_lock;
	const bool _reserved;
};

// Keep the reservation on successful arming. Release it on every early return
// that leaves the vehicle disarmed, including a failed preflight check.
class ArmGuard
{
public:
	ArmGuard(const uint8_t &state, uint8_t armed_value, bool require_ready) :
		_state(state), _armed(armed_value), _already(state == armed_value),
		_reserved(!_already && interlock().acquireFlight(require_ready)) {}

	~ArmGuard()
	{
		if (_reserved && _state != _armed) {
			interlock().releaseFlight();
		}
	}

	bool allowed() const
	{
		return _already || _reserved;
	}
	ArmGuard(const ArmGuard &) = delete;
	ArmGuard &operator=(const ArmGuard &) = delete;

private:
	const uint8_t &_state;
	const uint8_t _armed;
	const bool _already;
	const bool _reserved;
};

class DisarmGuard
{
public:
	DisarmGuard(const uint8_t &state, uint8_t armed_value) : _state(state), _armed(armed_value) {}

	~DisarmGuard()
	{
		if (_state != _armed) {
			interlock().releaseFlight();
		}
	}

	DisarmGuard(const DisarmGuard &) = delete;
	DisarmGuard &operator=(const DisarmGuard &) = delete;

private:
	const uint8_t &_state;
	const uint8_t _armed;
};

} // namespace dragonfly_trtl
