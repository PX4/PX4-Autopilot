/****************************************************************************
 * SE3002 Assignment 02 - Component C (manual_control) - student-authored tests
 *
 * Test level : GTest FUNCTIONAL (needs parameters + uORB, no flight-controller
 *              context). Same mechanism as upstream ManualControlTest.cpp, but
 *              this file is written from the production logic of
 *              ManualControl.cpp (v1.17.0) and its baseline coverage gaps.
 *
 * Design rules used in every test
 *  - fresh ManualControl object per test (start()), parameters re-initialised
 *    in SetUp(), so there is no dependence on test order
 *  - expected values come from message constants (action_request_s,
 *    vehicle_status_s, vehicle_command_s ...), never from the code under test
 *  - switch tests need TWO samples: the first one only initialises
 *    _previous_switches, the second one is compared against it
 *
 * Test IDs map to the workbook (Sheet 1):
 *  TC-C-01/02 navStateFromParam        TC-C-03/04 print_usage/custom_command/print_status
 *  TC-C-05/06/26 mode slots            TC-C-07/08 switch gating
 *  TC-C-09 arm switch                  TC-C-10 arm button (hysteresis)
 *  TC-C-11 return/loiter/offboard      TC-C-12 kill/termination
 *  TC-C-13 landing gear                TC-C-14 VTOL transition
 *  TC-C-15 photo/video                 TC-C-16/17 arm/disarm gesture (+ boundaries)
 *  TC-C-18 kill gesture                TC-C-19 sticks_moving
 *  TC-C-21 invalid published once      TC-C-23/24 updateParams branches
 *  TC-C-27 gesture hysteresis reset
 *
 * Deliberately NOT in this file (analysed separately as coverage gaps):
 *  TC-C-20 input-instance switching, TC-C-25 init()/Run()/task_spawn()/main
 ****************************************************************************/

#include <gtest/gtest.h>

#include <climits>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/landing_gear.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_status.h>

#include "ManualControl.hpp"

using namespace time_literals;

#define EXPECT_EQ_INT(a, b) EXPECT_EQ(static_cast<int>(a), static_cast<int>(b))
#define ASSERT_EQ_INT(a, b) ASSERT_EQ(static_cast<int>(a), static_cast<int>(b))

static constexpr uint64_t SOME_TIME = 12345678;

// Expose the protected members that upstream also exposes for testing
class SQEManualControlImpl : public ManualControl
{
public:
	void processInput(hrt_abstime now) { ManualControl::processInput(now); }
	static int8_t navStateFromParam(int32_t v) { return ManualControl::navStateFromParam(v); }
};

struct Sticks {
	float roll{0.f};
	float pitch{0.f};
	float yaw{0.f};
	float throttle{0.f};
};

static Sticks S(float roll, float pitch, float yaw, float throttle)
{
	Sticks s;
	s.roll = roll;
	s.pitch = pitch;
	s.yaw = yaw;
	s.throttle = throttle;
	return s;
}

using SwField = uint8_t manual_control_switches_s::*;

class SQEManualControl : public ::testing::Test
{
public:
	void SetUp() override
	{
		// Avoid busy loop in param_set() (same as upstream fixture)
		param_control_autosave(false);

		// Known parameter state for EVERY test (parameters are process-global)
		setFloat("COM_RC_LOSS_T", 0.5f);   // stick input timeout 0.5 s
		setInt("COM_RC_IN_MODE", 0);       // RC only
		setFloat("COM_RC_STICK_OV", 30.f); // stick override threshold -> 0.3 /s
		setInt("COM_ARM_SWISBTN", 0);      // arm SWITCH, not button
		setInt("COM_RC_ARM_HYST", 100);    // arm hysteresis 100 ms
		setInt("MAN_ARM_GESTURE", 1);      // arm/disarm stick gesture on
		setFloat("MAN_KILL_GEST_T", 0.1f); // kill gesture hold time 100 ms
		setIntIfExists("RC_MAP_ARM_SW", 0);
		setIntIfExists("MC_AIRMODE", 0);

		// Slot k -> parameter code k-1 -> expected navigation state (see module.yaml COM_FLTMODE)
		for (int i = 0; i < 6; i++) {
			setInt(("COM_FLTMODE" + std::to_string(i + 1)).c_str(), i);
		}

		_t = SOME_TIME;
		_source = manual_control_setpoint_s::SOURCE_RC;

		// vehicle_command is a queued topic: the topic must exist and our subscription must be attached
		// BEFORE the module publishes, otherwise a late subscriber only sees the newest sample.
		_command_adv.advertise();
		(void)commands();
	}

	void TearDown() override { _mc.reset(); }

	// ---------------------------------------------------------------- params
	static void setInt(const char *name, int32_t v)
	{
		param_t h = param_find(name);
		ASSERT_NE(h, PARAM_INVALID) << "parameter not found: " << name;
		param_set(h, &v);
	}

	static void setFloat(const char *name, float v)
	{
		param_t h = param_find(name);
		ASSERT_NE(h, PARAM_INVALID) << "parameter not found: " << name;
		param_set(h, &v);
	}

	static void setIntIfExists(const char *name, int32_t v)
	{
		param_t h = param_find(name);

		if (h != PARAM_INVALID) {
			param_set(h, &v);
		}
	}

	static bool paramExists(const char *name) { return param_find(name) != PARAM_INVALID; }

	static int32_t getInt(const char *name)
	{
		int32_t v = INT32_MIN;
		param_get(param_find(name), &v);
		return v;
	}

	// ---------------------------------------------------------------- object
	// Construct AFTER the parameters of the scenario are set: the constructor calls updateParams()
	void start()
	{
		_mc.reset();
		_mc = std::make_unique<SQEManualControlImpl>();
		publishStatus(vehicle_status_s::ARMING_STATE_DISARMED, vehicle_status_s::VEHICLE_TYPE_FIXED_WING, false, 1);
	}

	void publishStatus(uint8_t arming_state, uint8_t vehicle_type, bool is_vtol, uint8_t system_id)
	{
		vehicle_status_s vs{};
		vs.timestamp = _t;
		vs.arming_state = arming_state;
		vs.vehicle_type = vehicle_type;
		vs.is_vtol = is_vtol;
		vs.system_id = system_id;
		_status_pub.publish(vs);
	}

	void publishInput(const Sticks &st)
	{
		manual_control_setpoint_s in{};
		in.timestamp = _t;
		in.timestamp_sample = _t;
		in.valid = true;
		in.data_source = _source;
		in.roll = st.roll;
		in.pitch = st.pitch;
		in.yaw = st.yaw;
		in.throttle = st.throttle;
		_input_pub.publish(in);
	}

	// Publish a sample on manual_control_input INSTANCE 1 (the plain publication above is instance 0)
	void publishInput2(const Sticks &st, uint8_t source)
	{
		manual_control_setpoint_s in{};
		in.timestamp = _t;
		in.timestamp_sample = _t;
		in.valid = true;
		in.data_source = source;
		in.roll = st.roll;
		in.pitch = st.pitch;
		in.yaw = st.yaw;
		in.throttle = st.throttle;
		_input_pub2.publish(in);
	}

	// One control-loop iteration: advance time, publish a fresh stick sample, run processInput
	void loop(hrt_abstime dt, const Sticks &st = Sticks{})
	{
		_t += dt;
		publishInput(st);
		_mc->processInput(_t);
	}

	// Same, but also publish a (new) switch sample
	void loopSwitches(const manual_control_switches_s &sw, hrt_abstime dt = 10_ms, const Sticks &st = Sticks{})
	{
		_t += dt;
		publishInput(st);
		manual_control_switches_s s = sw;
		s.timestamp = _t;
		s.timestamp_sample = _t;
		_switches_pub.publish(s);
		_mc->processInput(_t);
	}

	// ---------------------------------------------------------------- observation
	std::vector<action_request_s> actions()
	{
		std::vector<action_request_s> v;

		while (_action_sub.update()) {
			v.push_back(_action_sub.get());
		}

		return v;
	}

	std::vector<vehicle_command_s> commands()
	{
		std::vector<vehicle_command_s> v;

		while (_command_sub.update()) {
			v.push_back(_command_sub.get());
		}

		return v;
	}

	std::vector<landing_gear_s> gearEvents()
	{
		std::vector<landing_gear_s> v;

		while (_gear_sub.update()) {
			v.push_back(_gear_sub.get());
		}

		return v;
	}

	static int countAction(const std::vector<action_request_s> &v, uint8_t action)
	{
		int n = 0;

		for (const auto &a : v) {
			if (a.action == action) { n++; }
		}

		return n;
	}

	// ---------------------------------------------------------------- switch scenarios
	// start(), publish a baseline switch sample (initialisation only), discard its side effects
	void beginSwitches(SwField f, uint8_t from, uint8_t mode_slot = manual_control_switches_s::MODE_SLOT_NONE)
	{
		start();
		_sw = manual_control_switches_s{};
		_sw.mode_slot = mode_slot;
		_sw.*f = from;
		loopSwitches(_sw);
		(void)actions();
		(void)commands();
		(void)gearEvents();
	}

	void moveSwitch(SwField f, uint8_t to, hrt_abstime dt = 10_ms)
	{
		_sw.*f = to;
		loopSwitches(_sw, dt);
	}

	// baseline sample with <from>, then one sample with <to>; returns the action requests of the 2nd one
	std::vector<action_request_s> change(SwField f, uint8_t from, uint8_t to,
					     uint8_t mode_slot = manual_control_switches_s::MODE_SLOT_NONE)
	{
		beginSwitches(f, from, mode_slot);
		moveSwitch(f, to);
		return actions();
	}

	// ---------------------------------------------------------------- gesture scenarios
	// hold the sticks for 8 x 60 ms, count action requests of <action> coming from the stick gesture
	int gestureCount(const Sticks &sticks, uint8_t action, int steps = 8, hrt_abstime dt = 60_ms)
	{
		start();
		int n = 0;

		for (int i = 0; i < steps; i++) {
			loop(dt, sticks);

			for (const auto &a : actions()) {
				if (a.action == action && a.source == action_request_s::SOURCE_STICK_GESTURE) { n++; }
			}
		}

		return n;
	}

	// ---------------------------------------------------------------- members
	std::unique_ptr<SQEManualControlImpl> _mc;
	hrt_abstime _t{SOME_TIME};
	uint8_t _source{manual_control_setpoint_s::SOURCE_RC};
	manual_control_switches_s _sw{};

	uORB::Publication<manual_control_setpoint_s> _input_pub{ORB_ID(manual_control_input)};
	uORB::PublicationMulti<manual_control_setpoint_s> _input_pub2{ORB_ID(manual_control_input)};
	uORB::Publication<manual_control_switches_s> _switches_pub{ORB_ID(manual_control_switches)};
	uORB::Publication<vehicle_status_s> _status_pub{ORB_ID(vehicle_status)};
	uORB::Publication<vehicle_command_s> _command_adv{ORB_ID(vehicle_command)}; // only used to advertise the topic
	uORB::SubscriptionData<manual_control_setpoint_s> _setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::SubscriptionData<action_request_s> _action_sub{ORB_ID(action_request)};
	uORB::SubscriptionData<vehicle_command_s> _command_sub{ORB_ID(vehicle_command)};
	uORB::SubscriptionData<landing_gear_s> _gear_sub{ORB_ID(landing_gear)};
};

// ===========================================================================
// Group 1: navStateFromParam (static, no uORB)
// ===========================================================================

// TC-C-01: every documented COM_FLTMODE value maps to its navigation state
TEST_F(SQEManualControl, TC_C_01_NavStateFromParamMapping)
{
	struct Case {
		int32_t param;
		uint8_t nav;
	};
	const Case cases[] = {
		{0, vehicle_status_s::NAVIGATION_STATE_MANUAL},
		{1, vehicle_status_s::NAVIGATION_STATE_ALTCTL},
		{2, vehicle_status_s::NAVIGATION_STATE_POSCTL},
		{3, vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION},
		{4, vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER},
		{5, vehicle_status_s::NAVIGATION_STATE_AUTO_RTL},
		{6, vehicle_status_s::NAVIGATION_STATE_ACRO},
		{7, vehicle_status_s::NAVIGATION_STATE_OFFBOARD},
		{8, vehicle_status_s::NAVIGATION_STATE_STAB},
		{9, vehicle_status_s::NAVIGATION_STATE_POSITION_SLOW},
		{10, vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF},
		{11, vehicle_status_s::NAVIGATION_STATE_AUTO_LAND},
		{12, vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET},
		{13, vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND},
		{14, vehicle_status_s::NAVIGATION_STATE_ORBIT},
		{15, vehicle_status_s::NAVIGATION_STATE_AUTO_VTOL_TAKEOFF},
		{16, vehicle_status_s::NAVIGATION_STATE_ALTITUDE_CRUISE},
		{100, vehicle_status_s::NAVIGATION_STATE_EXTERNAL1},
		{101, vehicle_status_s::NAVIGATION_STATE_EXTERNAL2},
		{102, vehicle_status_s::NAVIGATION_STATE_EXTERNAL3},
		{103, vehicle_status_s::NAVIGATION_STATE_EXTERNAL4},
		{104, vehicle_status_s::NAVIGATION_STATE_EXTERNAL5},
		{105, vehicle_status_s::NAVIGATION_STATE_EXTERNAL6},
		{106, vehicle_status_s::NAVIGATION_STATE_EXTERNAL7},
		{107, vehicle_status_s::NAVIGATION_STATE_EXTERNAL8},
	};

	for (const auto &c : cases) {
		EXPECT_EQ_INT(SQEManualControlImpl::navStateFromParam(c.param), c.nav) << "param " << c.param;
	}
}

// TC-C-02: values outside the table (incl. both sides of the 16/100 and 107/108 boundaries) -> -1
TEST_F(SQEManualControl, TC_C_02_NavStateFromParamInvalid)
{
	const int32_t invalid[] = {-1, 17, 18, 50, 99, 108, 109, INT32_MAX, INT32_MIN};

	for (int32_t p : invalid) {
		EXPECT_EQ_INT(SQEManualControlImpl::navStateFromParam(p), -1) << "param " << p;
	}
}

// ===========================================================================
// Group 2: module plumbing that is reachable without a running work queue
// ===========================================================================

// TC-C-03: print_usage() with and without a reason (both outcomes of `if (reason)`)
TEST_F(SQEManualControl, TC_C_03_PrintUsage)
{
	EXPECT_EQ(ManualControl::print_usage(), 0);
	EXPECT_EQ(ManualControl::print_usage("sqe test reason"), 0);
}

// TC-C-04: custom_command() and print_status()
TEST_F(SQEManualControl, TC_C_04_CustomCommandAndPrintStatus)
{
	EXPECT_EQ(ManualControl::custom_command(0, nullptr), 0);
	start();
	EXPECT_EQ(_mc->print_status(), 0);
}

// ===========================================================================
// Group 3: mode slots (evaluateModeSlot / sendActionRequest)
// ===========================================================================

// TC-C-26: every mode slot requests the mode configured in its COM_FLTMODE parameter
TEST_F(SQEManualControl, TC_C_26_ModeSlotsMapToConfiguredModes)
{
	const uint8_t expected[6] = {
		vehicle_status_s::NAVIGATION_STATE_MANUAL,       // COM_FLTMODE1 = 0
		vehicle_status_s::NAVIGATION_STATE_ALTCTL,       // 1
		vehicle_status_s::NAVIGATION_STATE_POSCTL,       // 2
		vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION, // 3
		vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER,  // 4
		vehicle_status_s::NAVIGATION_STATE_AUTO_RTL,     // 5
	};

	for (uint8_t slot = manual_control_switches_s::MODE_SLOT_1; slot <= manual_control_switches_s::MODE_SLOT_6; slot++) {
		start();
		manual_control_switches_s s{};
		s.mode_slot = slot;
		loopSwitches(s); // first sample, vehicle disarmed -> mode is initialised from the slot
		auto a = actions();
		ASSERT_EQ(a.size(), 1u) << "slot " << static_cast<int>(slot);
		EXPECT_EQ_INT(a[0].action, action_request_s::ACTION_SWITCH_MODE);
		EXPECT_EQ_INT(a[0].source, action_request_s::SOURCE_RC_MODE_SLOT);
		EXPECT_EQ_INT(a[0].mode, expected[slot - 1]) << "slot " << static_cast<int>(slot);
	}
}

// TC-C-05: slot whose COM_FLTMODE is unassigned (-1) -> sendActionRequest drops the request
TEST_F(SQEManualControl, TC_C_05_UnassignedModeSlotIgnored)
{
	setInt("COM_FLTMODE1", -1);
	start();
	manual_control_switches_s s{};
	s.mode_slot = manual_control_switches_s::MODE_SLOT_1;
	loopSwitches(s);
	EXPECT_TRUE(actions().empty());
}

// TC-C-06: MODE_SLOT_NONE and an out-of-range slot never request a mode
TEST_F(SQEManualControl, TC_C_06_ModeSlotNoneAndOverflow)
{
	const uint8_t slots[] = {manual_control_switches_s::MODE_SLOT_NONE,
				 static_cast<uint8_t>(manual_control_switches_s::MODE_SLOT_NUM + 1), 255};

	for (uint8_t slot : slots) {
		start();
		manual_control_switches_s s{};
		s.mode_slot = slot;
		loopSwitches(s);
		EXPECT_TRUE(actions().empty()) << "slot " << static_cast<int>(slot);
	}
}

// ===========================================================================
// Group 4: processSwitches gating
// ===========================================================================

// TC-C-07: no new switch sample -> nothing is evaluated
TEST_F(SQEManualControl, TC_C_07_NoSwitchUpdateNoAction)
{
	beginSwitches(&manual_control_switches_s::arm_switch, manual_control_switches_s::SWITCH_POS_OFF);
	// valid RC sticks, but the switches topic is not republished
	loop(10_ms);
	loop(10_ms);
	EXPECT_TRUE(actions().empty());
}

// TC-C-08: switches are ignored while the chosen input is not RC
TEST_F(SQEManualControl, TC_C_08_SwitchesIgnoredForNonRcSource)
{
	setInt("COM_RC_IN_MODE", 1); // assumption: 1 = MAVLink only. Verified by the precondition below.
	_source = manual_control_setpoint_s::SOURCE_MAVLINK_0;
	beginSwitches(&manual_control_switches_s::arm_switch, manual_control_switches_s::SWITCH_POS_OFF);

	// precondition: the setpoint is valid AND comes from MAVLink (otherwise this test proves nothing)
	ASSERT_TRUE(_setpoint_sub.update()) << "no setpoint published; check COM_RC_IN_MODE meaning";
	ASSERT_TRUE(_setpoint_sub.get().valid);
	ASSERT_EQ_INT(_setpoint_sub.get().data_source, manual_control_setpoint_s::SOURCE_MAVLINK_0);

	moveSwitch(&manual_control_switches_s::arm_switch, manual_control_switches_s::SWITCH_POS_ON);
	EXPECT_TRUE(actions().empty()) << "arm switch must not act when the source is MAVLink";
}

// ===========================================================================
// Group 5: individual switches
// ===========================================================================

// TC-C-09: arm switch (COM_ARM_SWISBTN = 0)
TEST_F(SQEManualControl, TC_C_09_ArmSwitch)
{
	using M = manual_control_switches_s;
	SwField f = &M::arm_switch;

	auto on = change(f, M::SWITCH_POS_NONE, M::SWITCH_POS_ON);
	ASSERT_EQ(on.size(), 1u);
	EXPECT_EQ_INT(on[0].action, action_request_s::ACTION_ARM);
	EXPECT_EQ_INT(on[0].source, action_request_s::SOURCE_RC_SWITCH);

	auto off = change(f, M::SWITCH_POS_ON, M::SWITCH_POS_OFF);
	ASSERT_EQ(off.size(), 1u);
	EXPECT_EQ_INT(off[0].action, action_request_s::ACTION_DISARM);
	EXPECT_EQ_INT(off[0].source, action_request_s::SOURCE_RC_SWITCH);

	// middle position is neither ON nor OFF -> no request
	EXPECT_TRUE(change(f, M::SWITCH_POS_ON, M::SWITCH_POS_MIDDLE).empty());
	// no change at all -> no request
	EXPECT_TRUE(change(f, M::SWITCH_POS_ON, M::SWITCH_POS_ON).empty());
}

// TC-C-10: arm BUTTON (COM_ARM_SWISBTN = 1) with hysteresis
TEST_F(SQEManualControl, TC_C_10_ArmButtonHoldToggleOnce)
{
	using M = manual_control_switches_s;
	setInt("COM_ARM_SWISBTN", 1);
	SwField f = &M::arm_switch;
	beginSwitches(f, M::SWITCH_POS_OFF);

	int toggles = 0;

	// hold the button 8 x 60 ms: hysteresis is 100 ms, so nothing in the first two samples, then ONE toggle
	for (int i = 0; i < 8; i++) {
		moveSwitch(f, M::SWITCH_POS_ON, 60_ms);
		auto a = actions();

		if (i < 2) { EXPECT_TRUE(a.empty()) << "sample " << i; }

		for (const auto &r : a) {
			if (r.action == action_request_s::ACTION_TOGGLE_ARMING) {
				toggles++;
				EXPECT_EQ_INT(r.source, action_request_s::SOURCE_RC_BUTTON);
			}
		}
	}

	EXPECT_EQ(toggles, 1);

	// release and press again -> a second toggle
	moveSwitch(f, M::SWITCH_POS_OFF, 60_ms);
	EXPECT_TRUE(actions().empty());

	for (int i = 0; i < 8; i++) {
		moveSwitch(f, M::SWITCH_POS_ON, 60_ms);
		toggles += countAction(actions(), action_request_s::ACTION_TOGGLE_ARMING);
	}

	EXPECT_EQ(toggles, 2);
}

// TC-C-10b: a press shorter than the hysteresis time does nothing
TEST_F(SQEManualControl, TC_C_10b_ArmButtonShortPressIgnored)
{
	using M = manual_control_switches_s;
	setInt("COM_ARM_SWISBTN", 1);
	SwField f = &M::arm_switch;
	beginSwitches(f, M::SWITCH_POS_OFF);

	moveSwitch(f, M::SWITCH_POS_ON, 60_ms);
	moveSwitch(f, M::SWITCH_POS_OFF, 60_ms);
	moveSwitch(f, M::SWITCH_POS_OFF, 60_ms);
	moveSwitch(f, M::SWITCH_POS_OFF, 60_ms);
	EXPECT_TRUE(actions().empty());
}

// TC-C-11: return / loiter / offboard switches
TEST_F(SQEManualControl, TC_C_11_ReturnLoiterOffboardSwitches)
{
	using M = manual_control_switches_s;
	struct Case {
		SwField field;
		uint8_t nav_on;
	};
	const Case cases[] = {
		{&M::return_switch, vehicle_status_s::NAVIGATION_STATE_AUTO_RTL},
		{&M::loiter_switch, vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER},
		{&M::offboard_switch, vehicle_status_s::NAVIGATION_STATE_OFFBOARD},
	};

	for (const auto &c : cases) {
		// ON: direct mode switch
		auto on = change(c.field, M::SWITCH_POS_NONE, M::SWITCH_POS_ON);
		ASSERT_EQ(on.size(), 1u);
		EXPECT_EQ_INT(on[0].action, action_request_s::ACTION_SWITCH_MODE);
		EXPECT_EQ_INT(on[0].source, action_request_s::SOURCE_RC_SWITCH);
		EXPECT_EQ_INT(on[0].mode, c.nav_on);

		// OFF: fall back to the mode slot (slot 3 -> COM_FLTMODE3 = 2 -> POSCTL)
		auto off = change(c.field, M::SWITCH_POS_ON, M::SWITCH_POS_OFF, M::MODE_SLOT_3);
		ASSERT_EQ(off.size(), 1u);
		EXPECT_EQ_INT(off[0].action, action_request_s::ACTION_SWITCH_MODE);
		EXPECT_EQ_INT(off[0].source, action_request_s::SOURCE_RC_MODE_SLOT);
		EXPECT_EQ_INT(off[0].mode, vehicle_status_s::NAVIGATION_STATE_POSCTL);

		// MIDDLE: neither ON nor OFF
		EXPECT_TRUE(change(c.field, M::SWITCH_POS_ON, M::SWITCH_POS_MIDDLE, M::MODE_SLOT_3).empty());
	}
}

// TC-C-12: kill and termination switches
TEST_F(SQEManualControl, TC_C_12_KillAndTerminationSwitches)
{
	using M = manual_control_switches_s;

	auto kill = change(&M::kill_switch, M::SWITCH_POS_NONE, M::SWITCH_POS_ON);
	ASSERT_EQ(kill.size(), 1u);
	EXPECT_EQ_INT(kill[0].action, action_request_s::ACTION_KILL);
	EXPECT_EQ_INT(kill[0].source, action_request_s::SOURCE_RC_SWITCH);

	auto unkill = change(&M::kill_switch, M::SWITCH_POS_ON, M::SWITCH_POS_OFF);
	ASSERT_EQ(unkill.size(), 1u);
	EXPECT_EQ_INT(unkill[0].action, action_request_s::ACTION_UNKILL);

	EXPECT_TRUE(change(&M::kill_switch, M::SWITCH_POS_ON, M::SWITCH_POS_MIDDLE).empty());

	// termination only reacts to a change INTO the ON position (compound condition)
	auto term = change(&M::termination_switch, M::SWITCH_POS_NONE, M::SWITCH_POS_ON);
	ASSERT_EQ(term.size(), 1u);
	EXPECT_EQ_INT(term[0].action, action_request_s::ACTION_TERMINATION);
	EXPECT_EQ_INT(term[0].source, action_request_s::SOURCE_RC_SWITCH);

	EXPECT_TRUE(change(&M::termination_switch, M::SWITCH_POS_ON, M::SWITCH_POS_OFF).empty());
	EXPECT_TRUE(change(&M::termination_switch, M::SWITCH_POS_ON, M::SWITCH_POS_ON).empty());
}

// TC-C-13: landing gear switch
TEST_F(SQEManualControl, TC_C_13_LandingGearSwitch)
{
	using M = manual_control_switches_s;
	SwField f = &M::gear_switch;

	// OFF -> ON : gear up
	beginSwitches(f, M::SWITCH_POS_OFF);
	moveSwitch(f, M::SWITCH_POS_ON);
	auto up = gearEvents();
	ASSERT_EQ(up.size(), 1u);
	EXPECT_EQ_INT(up[0].landing_gear, landing_gear_s::GEAR_UP);

	// ON -> OFF : gear down
	beginSwitches(f, M::SWITCH_POS_ON);
	moveSwitch(f, M::SWITCH_POS_OFF);
	auto down = gearEvents();
	ASSERT_EQ(down.size(), 1u);
	EXPECT_EQ_INT(down[0].landing_gear, landing_gear_s::GEAR_DOWN);

	// previous position NONE (switch not mapped yet) -> ignored, even though the value changed
	beginSwitches(f, M::SWITCH_POS_NONE);
	moveSwitch(f, M::SWITCH_POS_ON);
	EXPECT_TRUE(gearEvents().empty());

	// MIDDLE is neither ON nor OFF -> nothing published
	beginSwitches(f, M::SWITCH_POS_ON);
	moveSwitch(f, M::SWITCH_POS_MIDDLE);
	EXPECT_TRUE(gearEvents().empty());

	// unchanged -> nothing published
	beginSwitches(f, M::SWITCH_POS_ON);
	moveSwitch(f, M::SWITCH_POS_ON);
	EXPECT_TRUE(gearEvents().empty());
}

// TC-C-14: VTOL transition switch
TEST_F(SQEManualControl, TC_C_14_TransitionSwitch)
{
	using M = manual_control_switches_s;
	SwField f = &M::transition_switch;

	auto fw = change(f, M::SWITCH_POS_NONE, M::SWITCH_POS_ON);
	ASSERT_EQ(fw.size(), 1u);
	EXPECT_EQ_INT(fw[0].action, action_request_s::ACTION_VTOL_TRANSITION_TO_FIXEDWING);
	EXPECT_EQ_INT(fw[0].source, action_request_s::SOURCE_RC_SWITCH);

	auto mc = change(f, M::SWITCH_POS_ON, M::SWITCH_POS_OFF);
	ASSERT_EQ(mc.size(), 1u);
	EXPECT_EQ_INT(mc[0].action, action_request_s::ACTION_VTOL_TRANSITION_TO_MULTICOPTER);

	EXPECT_TRUE(change(f, M::SWITCH_POS_ON, M::SWITCH_POS_MIDDLE).empty());
}

// TC-C-15a: photo switch publishes a camera-mode command then a capture command, sequence increments
TEST_F(SQEManualControl, TC_C_15a_PhotoSwitch)
{
	using M = manual_control_switches_s;
	SwField f = &M::photo_switch;
	beginSwitches(f, M::SWITCH_POS_OFF);
	publishStatus(vehicle_status_s::ARMING_STATE_DISARMED, vehicle_status_s::VEHICLE_TYPE_FIXED_WING, false, 9);

	moveSwitch(f, M::SWITCH_POS_ON);
	auto c1 = commands();
	ASSERT_EQ(c1.size(), 2u);
	EXPECT_EQ(c1[0].command, static_cast<uint32_t>(vehicle_command_s::VEHICLE_CMD_SET_CAMERA_MODE));
	EXPECT_FLOAT_EQ(c1[0].param2, 0.f); // CameraMode::Image
	EXPECT_EQ(c1[1].command, static_cast<uint32_t>(vehicle_command_s::VEHICLE_CMD_IMAGE_START_CAPTURE));
	EXPECT_FLOAT_EQ(c1[1].param3, 1.f); // one picture
	EXPECT_FLOAT_EQ(c1[1].param4, 0.f); // first sequence number
	EXPECT_EQ_INT(c1[1].target_system, 9);
	EXPECT_EQ_INT(c1[1].target_component, 100);

	// switching back OFF does nothing
	moveSwitch(f, M::SWITCH_POS_OFF);
	EXPECT_TRUE(commands().empty());

	// second photo -> sequence number 1
	moveSwitch(f, M::SWITCH_POS_ON);
	auto c2 = commands();
	ASSERT_EQ(c2.size(), 2u);
	EXPECT_FLOAT_EQ(c2[1].param4, 1.f);
}

// TC-C-15b: video switch toggles start / stop recording
TEST_F(SQEManualControl, TC_C_15b_VideoSwitchStartStop)
{
	using M = manual_control_switches_s;
	SwField f = &M::video_switch;
	beginSwitches(f, M::SWITCH_POS_OFF);
	publishStatus(vehicle_status_s::ARMING_STATE_DISARMED, vehicle_status_s::VEHICLE_TYPE_FIXED_WING, false, 9);

	// 1st ON: camera mode video + start capture
	moveSwitch(f, M::SWITCH_POS_ON);
	auto start_cmds = commands();
	ASSERT_EQ(start_cmds.size(), 2u);
	EXPECT_EQ(start_cmds[0].command, static_cast<uint32_t>(vehicle_command_s::VEHICLE_CMD_SET_CAMERA_MODE));
	EXPECT_FLOAT_EQ(start_cmds[0].param2, 1.f); // CameraMode::Video
	EXPECT_EQ(start_cmds[1].command, static_cast<uint32_t>(vehicle_command_s::VEHICLE_CMD_VIDEO_START_CAPTURE));
	EXPECT_EQ_INT(start_cmds[1].target_system, 9);

	moveSwitch(f, M::SWITCH_POS_OFF);
	EXPECT_TRUE(commands().empty());

	// 2nd ON: recording is running -> stop capture (status at 1 Hz)
	moveSwitch(f, M::SWITCH_POS_ON);
	auto stop_cmds = commands();
	ASSERT_EQ(stop_cmds.size(), 2u);
	EXPECT_EQ(stop_cmds[1].command, static_cast<uint32_t>(vehicle_command_s::VEHICLE_CMD_VIDEO_STOP_CAPTURE));
	EXPECT_FLOAT_EQ(stop_cmds[1].param2, 1.f);

	// 3rd ON: toggles back to start
	moveSwitch(f, M::SWITCH_POS_OFF);
	(void)commands();
	moveSwitch(f, M::SWITCH_POS_ON);
	auto again = commands();
	ASSERT_EQ(again.size(), 2u);
	EXPECT_EQ(again[1].command, static_cast<uint32_t>(vehicle_command_s::VEHICLE_CMD_VIDEO_START_CAPTURE));
}

// ===========================================================================
// Group 6: stick gestures (processStickArming)
// ===========================================================================

// TC-C-16: arm gesture = throttle low + yaw right, right stick centred, held longer than COM_RC_ARM_HYST
TEST_F(SQEManualControl, TC_C_16a_ArmGestureRequestsArmOnce)
{
	EXPECT_EQ(gestureCount(S(0.f, 0.f, 1.f, -1.f), action_request_s::ACTION_ARM), 1);
}

// TC-C-16b: disarm gesture = throttle low + yaw left
TEST_F(SQEManualControl, TC_C_16b_DisarmGestureRequestsDisarmOnce)
{
	EXPECT_EQ(gestureCount(S(0.f, 0.f, -1.f, -1.f), action_request_s::ACTION_DISARM), 1);
	EXPECT_EQ(gestureCount(S(0.f, 0.f, -1.f, -1.f), action_request_s::ACTION_ARM), 0);
}

// TC-C-16c: MAN_ARM_GESTURE = 0 disables both gestures
TEST_F(SQEManualControl, TC_C_16c_GestureDisabledByParameter)
{
	setInt("MAN_ARM_GESTURE", 0);
	EXPECT_EQ(gestureCount(S(0.f, 0.f, 1.f, -1.f), action_request_s::ACTION_ARM), 0);
	EXPECT_EQ(gestureCount(S(0.f, 0.f, -1.f, -1.f), action_request_s::ACTION_DISARM), 0);
}

// TC-C-16d: gesture released before the hysteresis time elapsed -> no request
TEST_F(SQEManualControl, TC_C_16d_GestureReleasedEarly)
{
	start();
	loop(60_ms, S(0.f, 0.f, 1.f, -1.f));
	loop(60_ms, S(0.f, 0.f, 0.f, 0.f)); // sticks back to neutral
	loop(60_ms, S(0.f, 0.f, 0.f, 0.f));
	loop(60_ms, S(0.f, 0.f, 0.f, 0.f));
	EXPECT_EQ(countAction(actions(), action_request_s::ACTION_ARM), 0);
}

// TC-C-17: boundaries of the strict comparisons (<, >) in the gesture conditions
TEST_F(SQEManualControl, TC_C_17_GestureBoundaries)
{
	const uint8_t ARM = action_request_s::ACTION_ARM;
	const uint8_t DISARM = action_request_s::ACTION_DISARM;

	// yaw must be strictly > 0.9
	EXPECT_EQ(gestureCount(S(0.f, 0.f, 0.9f, -1.f), ARM), 0);
	EXPECT_EQ(gestureCount(S(0.f, 0.f, 0.91f, -1.f), ARM), 1);
	// throttle must be strictly < -0.8
	EXPECT_EQ(gestureCount(S(0.f, 0.f, 1.f, -0.8f), ARM), 0);
	EXPECT_EQ(gestureCount(S(0.f, 0.f, 1.f, -0.81f), ARM), 1);
	// right stick centred means |pitch| < 0.1 and |roll| < 0.1 (strict)
	EXPECT_EQ(gestureCount(S(0.f, 0.1f, 1.f, -1.f), ARM), 0);
	EXPECT_EQ(gestureCount(S(0.f, 0.09f, 1.f, -1.f), ARM), 1);
	EXPECT_EQ(gestureCount(S(0.1f, 0.f, 1.f, -1.f), ARM), 0);
	EXPECT_EQ(gestureCount(S(-0.09f, 0.f, 1.f, -1.f), ARM), 1);
	EXPECT_EQ(gestureCount(S(-0.1f, 0.f, 1.f, -1.f), ARM), 0);
	// disarm side: yaw strictly < -0.9
	EXPECT_EQ(gestureCount(S(0.f, 0.f, -0.9f, -1.f), DISARM), 0);
	EXPECT_EQ(gestureCount(S(0.f, 0.f, -0.91f, -1.f), DISARM), 1);
}

// TC-C-27: hysteresis state is reset while the input is invalid -> the gesture must be held again
TEST_F(SQEManualControl, TC_C_27_GestureHysteresisResetOnInputLoss)
{
	const Sticks arm = S(0.f, 0.f, 1.f, -1.f);
	start();

	loop(60_ms, arm);
	EXPECT_EQ(countAction(actions(), action_request_s::ACTION_ARM), 0);

	// RC silent for longer than COM_RC_LOSS_T -> input invalid -> hysteresis reset
	_t += 600_ms;
	_mc->processInput(_t);
	(void)actions();

	// gesture resumes: first two samples must not request anything (hysteresis restarts)
	loop(60_ms, arm);
	EXPECT_EQ(countAction(actions(), action_request_s::ACTION_ARM), 0);
	loop(60_ms, arm);
	EXPECT_EQ(countAction(actions(), action_request_s::ACTION_ARM), 0);

	// ... but holding it long enough requests ARM exactly once
	int arms = 0;

	for (int i = 0; i < 6; i++) {
		loop(60_ms, arm);
		arms += countAction(actions(), action_request_s::ACTION_ARM);
	}

	EXPECT_EQ(arms, 1);
}

// TC-C-18a: kill gesture = left stick lower-left AND right stick lower-right
TEST_F(SQEManualControl, TC_C_18a_KillGesture)
{
	const Sticks kill = S(1.f, -1.f, -1.f, -1.f); // roll right, pitch back, yaw left, throttle low
	EXPECT_EQ(gestureCount(kill, action_request_s::ACTION_KILL), 1);
	// right stick is not centred, so the same sticks must NOT also trigger disarm/arm
	EXPECT_EQ(gestureCount(kill, action_request_s::ACTION_DISARM), 0);
	EXPECT_EQ(gestureCount(kill, action_request_s::ACTION_ARM), 0);
}

// TC-C-18b: MAN_KILL_GEST_T = 0 disables the kill gesture (block skipped)
TEST_F(SQEManualControl, TC_C_18b_KillGestureDisabled)
{
	setFloat("MAN_KILL_GEST_T", 0.f);
	EXPECT_EQ(gestureCount(S(1.f, -1.f, -1.f, -1.f), action_request_s::ACTION_KILL), 0);
}

// TC-C-18c: partial kill gestures and strict boundaries
TEST_F(SQEManualControl, TC_C_18c_KillGesturePartialAndBoundaries)
{
	const uint8_t KILL = action_request_s::ACTION_KILL;
	// only the right stick in the corner
	EXPECT_EQ(gestureCount(S(1.f, -1.f, 0.f, 0.f), KILL), 0);
	// only the left stick in the corner
	EXPECT_EQ(gestureCount(S(0.f, 0.f, -1.f, -1.f), KILL), 0);
	// pitch must be strictly < -0.9, roll strictly > 0.9
	EXPECT_EQ(gestureCount(S(1.f, -0.9f, -1.f, -1.f), KILL), 0);
	EXPECT_EQ(gestureCount(S(1.f, -0.91f, -1.f, -1.f), KILL), 1);
	EXPECT_EQ(gestureCount(S(0.9f, -1.f, -1.f, -1.f), KILL), 0);
	EXPECT_EQ(gestureCount(S(0.91f, -1.f, -1.f, -1.f), KILL), 1);
}

// ===========================================================================
// Group 7: processInput - stick override and input validity
// ===========================================================================

// TC-C-19: sticks_moving is true when ANY single axis changes fast enough, false otherwise
TEST_F(SQEManualControl, TC_C_19_SticksMovingPerAxis)
{
	struct Case {
		const char *name;
		Sticks after;
		bool moving;
	};
	const Case cases[] = {
		{"roll only", S(1.f, 0.f, 0.f, 0.f), true},
		{"pitch only", S(0.f, 1.f, 0.f, 0.f), true},
		{"yaw only", S(0.f, 0.f, 1.f, 0.f), true},
		{"throttle only", S(0.f, 0.f, 0.f, 1.f), true},
		{"no movement", S(0.f, 0.f, 0.f, 0.f), false},
	};

	for (const auto &c : cases) {
		start();
		loop(10_ms, S(0.f, 0.f, 0.f, 0.f)); // first sample: no previous value yet
		(void)_setpoint_sub.update();
		loop(10_ms, c.after);               // step change within 10 ms -> large (filtered) rate
		ASSERT_TRUE(_setpoint_sub.update()) << c.name;
		EXPECT_EQ(_setpoint_sub.get().sticks_moving, c.moving) << c.name;
	}
}

// TC-C-21: after the input times out an invalid setpoint is published ONCE, not on every loop
TEST_F(SQEManualControl, TC_C_21_InvalidSetpointPublishedOnce)
{
	start();
	loop(10_ms, S(0.f, 0.f, 0.f, 0.f));
	ASSERT_TRUE(_setpoint_sub.update());
	EXPECT_TRUE(_setpoint_sub.get().valid);

	// silent for longer than COM_RC_LOSS_T (0.5 s)
	_t += 600_ms;
	_mc->processInput(_t);
	ASSERT_TRUE(_setpoint_sub.update());
	EXPECT_FALSE(_setpoint_sub.get().valid);

	_t += 10_ms;
	_mc->processInput(_t);
	EXPECT_FALSE(_setpoint_sub.update()) << "invalid setpoint must not be re-published every loop";
}

// ===========================================================================
// Group 8: updateParams branches
// ===========================================================================

// TC-C-23a: a mapped arm switch forces MAN_ARM_GESTURE to 0
TEST_F(SQEManualControl, TC_C_23a_ArmGestureDisabledWhenArmSwitchMapped)
{
	if (!paramExists("RC_MAP_ARM_SW")) {
		GTEST_SKIP() << "RC_MAP_ARM_SW is not part of the SITL test parameter set (lines unreachable here)";
	}

	start();
	setInt("RC_MAP_ARM_SW", 5); // publishes parameter_update -> picked up by the next processInput
	loop(10_ms);
	EXPECT_EQ(getInt("MAN_ARM_GESTURE"), 0);
}

// TC-C-23b: no arm switch mapped -> gesture stays enabled
TEST_F(SQEManualControl, TC_C_23b_ArmGestureKeptWhenNoArmSwitch)
{
	if (!paramExists("RC_MAP_ARM_SW")) {
		GTEST_SKIP() << "RC_MAP_ARM_SW is not part of the SITL test parameter set (lines unreachable here)";
	}

	start();
	setInt("RC_MAP_ARM_SW", 0);
	loop(10_ms);
	EXPECT_EQ(getInt("MAN_ARM_GESTURE"), 1);
}

// TC-C-24: yaw airmode is downgraded only if gesture==1 && (rotary wing || VTOL) && airmode==2
TEST_F(SQEManualControl, TC_C_24_YawAirmodeVersusArmGesture)
{
	if (!paramExists("MC_AIRMODE")) {
		GTEST_SKIP() << "MC_AIRMODE is not part of the SITL test parameter set (lines unreachable here)";
	}

	struct Case {
		const char *name;
		uint8_t vehicle_type;
		bool vtol;
		int32_t gesture;
		int32_t airmode_in;
		int32_t airmode_out;
	};
	const uint8_t ROT = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
	const uint8_t FW = vehicle_status_s::VEHICLE_TYPE_FIXED_WING;
	const Case cases[] = {
		{"rotary, gesture on, yaw airmode", ROT, false, 1, 2, 1},
		{"vtol only, gesture on, yaw airmode", FW, true, 1, 2, 1},
		{"fixed wing, gesture on, yaw airmode", FW, false, 1, 2, 2},
		{"rotary, gesture on, roll/pitch airmode", ROT, false, 1, 1, 1},
		{"rotary, gesture on, airmode off", ROT, false, 1, 0, 0},
		{"rotary, gesture off, yaw airmode", ROT, false, 0, 2, 2},
	};

	for (const auto &c : cases) {
		setInt("MAN_ARM_GESTURE", c.gesture);
		setInt("MC_AIRMODE", c.airmode_in);
		start();                                                        // constructor sees vehicle type unknown
		publishStatus(vehicle_status_s::ARMING_STATE_DISARMED, c.vehicle_type, c.vtol, 1);
		setInt("MC_AIRMODE", c.airmode_in);                             // triggers parameter_update
		loop(10_ms);                                                    // status first, then updateParams()
		EXPECT_EQ(getInt("MC_AIRMODE"), c.airmode_out) << c.name;
	}
}

// TC-C-20: a higher-priority input on ANOTHER instance replaces the chosen instance
//          (unregisters the callback of the old instance, registers the new one)
TEST_F(SQEManualControl, TC_C_20_InputInstanceSwitch)
{
	setInt("COM_RC_IN_MODE", 5); // RC first, then MAVLink ascending: lower source id wins
	_input_pub.advertise();      // instance 0 first ...
	_input_pub2.advertise();     // ... so this one becomes instance 1
	start();

	// instance 0 carries MAVLink 1
	_source = manual_control_setpoint_s::SOURCE_MAVLINK_1;
	loop(10_ms);
	ASSERT_TRUE(_setpoint_sub.update());
	ASSERT_TRUE(_setpoint_sub.get().valid);
	ASSERT_EQ_INT(_setpoint_sub.get().data_source, manual_control_setpoint_s::SOURCE_MAVLINK_1);

	// instance 1 carries RC (id 1 <= 3) -> the selector switches to instance 1
	_t += 10_ms;
	publishInput2(S(0.f, 0.f, 0.f, 0.f), manual_control_setpoint_s::SOURCE_RC);
	_mc->processInput(_t);
	ASSERT_TRUE(_setpoint_sub.update());
	EXPECT_EQ_INT(_setpoint_sub.get().data_source, manual_control_setpoint_s::SOURCE_RC);
}

// TC-C-28: the mode slot changes after initialisation -> the new slot's mode is requested
TEST_F(SQEManualControl, TC_C_28_ModeSlotChange)
{
	using M = manual_control_switches_s;
	beginSwitches(&M::arm_switch, M::SWITCH_POS_OFF, M::MODE_SLOT_1);
	_sw.mode_slot = M::MODE_SLOT_3;
	loopSwitches(_sw);
	auto a = actions();
	ASSERT_EQ(a.size(), 1u);
	EXPECT_EQ_INT(a[0].action, action_request_s::ACTION_SWITCH_MODE);
	EXPECT_EQ_INT(a[0].source, action_request_s::SOURCE_RC_MODE_SLOT);
	EXPECT_EQ_INT(a[0].mode, vehicle_status_s::NAVIGATION_STATE_POSCTL); // COM_FLTMODE3 = 2

	// same slot again -> nothing
	loopSwitches(_sw);
	EXPECT_TRUE(actions().empty());
}
