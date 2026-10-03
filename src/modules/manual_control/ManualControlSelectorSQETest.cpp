/****************************************************************************
 * SE3002 Assignment 02 - Component C (manual_control) - student-authored tests
 * Target : ManualControlSelector.cpp (isInputValid + updateValidityOfChosenInput +
 *          updateWithNewInputSample), PX4 v1.17.0
 *
 * Test level : GTest UNIT. The selector is pure logic (no parameters, no uORB
 *              publication, no time source: "now" is passed in), so a unit test
 *              is the cheapest level that still reaches every decision.
 *
 * Derivation
 *  isInputValid() returns  sample_newer_than_timeout && input.valid && match
 *  where match depends on COM_RC_IN_MODE (switch with 9 cases + default).
 *  For every compound decision the tables below list the atomic conditions,
 *  the "current chosen input" state needed to reach them, and the expected
 *  result worked out by hand from the boolean expression in the source.
 *  Rows are chosen so that each atomic condition has a row pair where only that
 *  condition changes and the decision flips (see the comments per test).
 *
 * Source ids (manual_control_setpoint_s): UNKNOWN=0, RC=1, MAVLINK_0..5 = 2..7
 *
 * Test IDs: TC-C-30 .. TC-C-43 (see workbook Sheet 1)
 ****************************************************************************/

#include <gtest/gtest.h>

#include <cstdint>

#include "ManualControlSelector.hpp"

namespace
{

constexpr uint64_t TIMEOUT_US = 500000; // 0.5 s
constexpr uint64_t T0 = 1000000;

constexpr uint8_t SRC_UNKNOWN = manual_control_setpoint_s::SOURCE_UNKNOWN;
constexpr uint8_t SRC_RC = manual_control_setpoint_s::SOURCE_RC;
constexpr uint8_t SRC_M0 = manual_control_setpoint_s::SOURCE_MAVLINK_0;
constexpr uint8_t SRC_M1 = manual_control_setpoint_s::SOURCE_MAVLINK_1;
constexpr uint8_t SRC_M2 = manual_control_setpoint_s::SOURCE_MAVLINK_2;
constexpr uint8_t SRC_M3 = manual_control_setpoint_s::SOURCE_MAVLINK_3;
constexpr uint8_t SRC_M5 = manual_control_setpoint_s::SOURCE_MAVLINK_5;

// COM_RC_IN_MODE parameter values (documented parameter meaning)
enum Mode : int32_t {
	RC_ONLY = 0,
	MAVLINK_ONLY = 1,
	WITH_FALLBACK = 2,
	KEEP_FIRST = 3,
	DISABLED = 4,
	PRIO_RC_THEN_MAV_ASC = 5,
	PRIO_MAV_ASC_THEN_RC = 6,
	PRIO_RC_THEN_MAV_DESC = 7,
	PRIO_MAV_DESC_THEN_RC = 8
};

class Harness
{
public:
	explicit Harness(int32_t mode)
	{
		_sel.setRcInMode(mode);
		_sel.setTimeout(TIMEOUT_US);
	}

	// Offer one sample <dt_us> after the previous one. True if the selector chose it.
	bool feed(uint8_t source, uint64_t dt_us = 10000, bool valid = true)
	{
		_now += dt_us;
		manual_control_setpoint_s in{};
		in.timestamp_sample = _now;
		in.valid = valid;
		in.data_source = source;
		const int id = ++_next_id;
		_sel.updateWithNewInputSample(_now, in, id);
		return _sel.instance() == id;
	}

	void advance(uint64_t dt_us) { _now += dt_us; }
	uint64_t now() const { return _now; }
	ManualControlSelector &sel() { return _sel; }

private:
	ManualControlSelector _sel;
	uint64_t _now{T0};
	int _next_id{-1};
};

// Build the state "current chosen input comes from <current>" (-1 = nothing chosen yet),
// then offer <candidate>. Returns whether the candidate replaced / became the chosen input.
bool offered(int32_t mode, int current, uint8_t candidate)
{
	Harness h(mode);

	if (current >= 0) {
		if (!h.feed(static_cast<uint8_t>(current))) {
			ADD_FAILURE() << "precondition failed: current source " << current << " not accepted in mode " << mode;
		}
	}

	return h.feed(candidate);
}

} // namespace

// TC-C-30: RcOnly -> only the RC source is accepted
TEST(SQEManualControlSelector, TC_C_30_RcOnly)
{
	EXPECT_TRUE(offered(RC_ONLY, -1, SRC_RC));
	EXPECT_FALSE(offered(RC_ONLY, -1, SRC_M0));
	EXPECT_FALSE(offered(RC_ONLY, -1, SRC_UNKNOWN));
	EXPECT_TRUE(offered(RC_ONLY, SRC_RC, SRC_RC));
	EXPECT_FALSE(offered(RC_ONLY, SRC_RC, SRC_M0));
}

// TC-C-31: MavLinkOnly: match = A && (B || C), A=isMavlink(in) B=(in==current source) C=!current.valid
//   A: (none,M0)=T [A=T,B=F,C=T]  vs (none,RC)=F [A=F,B=F,C=T]
//   B: (cur M0,M0)=T [T,T,F]      vs (cur M0,M1)=F [T,F,F]
//   C: (none,M0)=T [T,F,T]        vs (cur M0,M1)=F [T,F,F]
TEST(SQEManualControlSelector, TC_C_31_MavlinkOnlyIndependencePairs)
{
	EXPECT_TRUE(offered(MAVLINK_ONLY, -1, SRC_M0));
	EXPECT_FALSE(offered(MAVLINK_ONLY, -1, SRC_RC));
	EXPECT_TRUE(offered(MAVLINK_ONLY, SRC_M0, SRC_M0));
	EXPECT_FALSE(offered(MAVLINK_ONLY, SRC_M0, SRC_M1));
}

// TC-C-32: isMavlink() accepts exactly sources 2..7; neighbours 0, 1 and 8 are rejected
TEST(SQEManualControlSelector, TC_C_32_IsMavlinkSourceRange)
{
	for (int src = 0; src <= SRC_M5 + 1; src++) {
		const bool expected = (src >= SRC_M0) && (src <= SRC_M5);
		EXPECT_EQ(offered(MAVLINK_ONLY, -1, static_cast<uint8_t>(src)), expected) << "source " << src;
	}
}

// TC-C-33: RcOrMavlinkWithFallback: match = (in==current source) || !current.valid
TEST(SQEManualControlSelector, TC_C_33_WithFallback)
{
	EXPECT_TRUE(offered(WITH_FALLBACK, -1, SRC_RC));
	EXPECT_TRUE(offered(WITH_FALLBACK, -1, SRC_M3));
	EXPECT_TRUE(offered(WITH_FALLBACK, SRC_RC, SRC_RC));
	EXPECT_FALSE(offered(WITH_FALLBACK, SRC_RC, SRC_M0));
	EXPECT_TRUE(offered(WITH_FALLBACK, SRC_M2, SRC_M2));
	EXPECT_FALSE(offered(WITH_FALLBACK, SRC_M2, SRC_RC));

	// once the chosen input timed out, any other source may take over
	Harness h(WITH_FALLBACK);
	ASSERT_TRUE(h.feed(SRC_RC));
	h.advance(TIMEOUT_US + 1);
	EXPECT_TRUE(h.feed(SRC_M0));
}

// TC-C-34: RcOrMavlinkKeepFirst: match = (in==first valid source) || (first==UNKNOWN)
TEST(SQEManualControlSelector, TC_C_34_KeepFirst)
{
	Harness h(KEEP_FIRST);
	EXPECT_TRUE(h.feed(SRC_M0));  // first == UNKNOWN -> accepted, M0 becomes the first source
	EXPECT_TRUE(h.feed(SRC_M0));  // same as first
	EXPECT_FALSE(h.feed(SRC_RC)); // different from first

	// the first source is remembered even after the chosen input expired
	h.advance(TIMEOUT_US + 1);
	EXPECT_FALSE(h.feed(SRC_RC));
	EXPECT_TRUE(h.feed(SRC_M0));

	// other way round: RC first, MAVLink rejected
	Harness h2(KEEP_FIRST);
	EXPECT_TRUE(h2.feed(SRC_RC));
	EXPECT_FALSE(h2.feed(SRC_M0));
}

// TC-C-35: DisableManualControl and out-of-range modes (default branch) accept nothing
TEST(SQEManualControlSelector, TC_C_35_DisabledAndUnknownMode)
{
	const int32_t modes[] = {DISABLED, 9, 99, -1};

	for (int32_t m : modes) {
		EXPECT_FALSE(offered(m, -1, SRC_RC)) << "mode " << m;
		EXPECT_FALSE(offered(m, -1, SRC_M0)) << "mode " << m;
	}
}

// TC-C-36: PriorityRcThenMavlinkAscending: match = !current.valid || (in <= current source), boundary at equality
TEST(SQEManualControlSelector, TC_C_36_PriorityRcThenMavlinkAscending)
{
	EXPECT_TRUE(offered(PRIO_RC_THEN_MAV_ASC, -1, SRC_M3));
	EXPECT_TRUE(offered(PRIO_RC_THEN_MAV_ASC, SRC_M1, SRC_M0));  // lower id wins
	EXPECT_TRUE(offered(PRIO_RC_THEN_MAV_ASC, SRC_M1, SRC_M1));  // equal id still accepted
	EXPECT_FALSE(offered(PRIO_RC_THEN_MAV_ASC, SRC_M1, SRC_M2)); // higher id rejected
	EXPECT_TRUE(offered(PRIO_RC_THEN_MAV_ASC, SRC_M1, SRC_RC));
	EXPECT_FALSE(offered(PRIO_RC_THEN_MAV_ASC, SRC_RC, SRC_M0));

	Harness h(PRIO_RC_THEN_MAV_ASC);
	ASSERT_TRUE(h.feed(SRC_M0));
	h.advance(TIMEOUT_US + 1);
	EXPECT_TRUE(h.feed(SRC_M5)); // current expired -> !valid -> accepted despite larger id
}

// TC-C-37: PriorityMavlinkAscendingThenRc: match = P || (Q && R) || (S && (R || T))
//   P=!current.valid  Q=isRc(in)  R=isRc(current)  S=isMavlink(in)  T=(in <= current source)
//   P : (none,RC)=T            vs (cur M1,RC)=F
//   Q : (cur RC,RC)=T          vs (cur RC,UNKNOWN)=F   [only Q/S differ, in no longer RC or MAVLink]
//   R : (cur RC,M0)=T [S&&R]   vs (cur M1,RC)=F        (R changes with Q fixed false/true respectively)
//   S : (cur M1,M0)=T          vs (cur M1,UNKNOWN)=F
//   T : (cur M1,M1)=T          vs (cur M1,M2)=F
TEST(SQEManualControlSelector, TC_C_37_PriorityMavlinkAscendingThenRc)
{
	EXPECT_TRUE(offered(PRIO_MAV_ASC_THEN_RC, -1, SRC_RC));
	EXPECT_TRUE(offered(PRIO_MAV_ASC_THEN_RC, SRC_RC, SRC_RC));
	EXPECT_TRUE(offered(PRIO_MAV_ASC_THEN_RC, SRC_RC, SRC_M0)); // MAVLink overrides RC
	EXPECT_FALSE(offered(PRIO_MAV_ASC_THEN_RC, SRC_M1, SRC_RC)); // RC does not override MAVLink
	EXPECT_TRUE(offered(PRIO_MAV_ASC_THEN_RC, SRC_M1, SRC_M0));
	EXPECT_TRUE(offered(PRIO_MAV_ASC_THEN_RC, SRC_M1, SRC_M1));
	EXPECT_FALSE(offered(PRIO_MAV_ASC_THEN_RC, SRC_M1, SRC_M2));
	EXPECT_FALSE(offered(PRIO_MAV_ASC_THEN_RC, SRC_M1, SRC_UNKNOWN));
}

// TC-C-38: PriorityRcThenMavlinkDescending: match = P || isRc(in) || (isMavlink(in) && isMavlink(cur) && in >= cur)
TEST(SQEManualControlSelector, TC_C_38_PriorityRcThenMavlinkDescending)
{
	EXPECT_TRUE(offered(PRIO_RC_THEN_MAV_DESC, -1, SRC_M0));
	EXPECT_TRUE(offered(PRIO_RC_THEN_MAV_DESC, SRC_M2, SRC_RC));      // RC always wins
	EXPECT_TRUE(offered(PRIO_RC_THEN_MAV_DESC, SRC_M2, SRC_M2));      // equal
	EXPECT_TRUE(offered(PRIO_RC_THEN_MAV_DESC, SRC_M2, SRC_M3));      // higher id wins
	EXPECT_FALSE(offered(PRIO_RC_THEN_MAV_DESC, SRC_M2, SRC_M1));     // lower id loses
	EXPECT_FALSE(offered(PRIO_RC_THEN_MAV_DESC, SRC_RC, SRC_M0));     // current is not MAVLink
	EXPECT_TRUE(offered(PRIO_RC_THEN_MAV_DESC, SRC_RC, SRC_RC));
	EXPECT_FALSE(offered(PRIO_RC_THEN_MAV_DESC, SRC_M2, SRC_UNKNOWN)); // neither RC nor MAVLink
}

// TC-C-39: PriorityMavlinkDescendingThenRc: match = !current.valid || (in >= current source)
TEST(SQEManualControlSelector, TC_C_39_PriorityMavlinkDescendingThenRc)
{
	EXPECT_TRUE(offered(PRIO_MAV_DESC_THEN_RC, -1, SRC_RC));
	EXPECT_TRUE(offered(PRIO_MAV_DESC_THEN_RC, SRC_M2, SRC_M3));
	EXPECT_TRUE(offered(PRIO_MAV_DESC_THEN_RC, SRC_M2, SRC_M2));
	EXPECT_FALSE(offered(PRIO_MAV_DESC_THEN_RC, SRC_M2, SRC_M1));
	EXPECT_FALSE(offered(PRIO_MAV_DESC_THEN_RC, SRC_M2, SRC_RC));
	EXPECT_TRUE(offered(PRIO_MAV_DESC_THEN_RC, SRC_RC, SRC_M0));
	EXPECT_TRUE(offered(PRIO_MAV_DESC_THEN_RC, SRC_RC, SRC_RC));

	Harness h(PRIO_MAV_DESC_THEN_RC);
	ASSERT_TRUE(h.feed(SRC_M5));
	h.advance(TIMEOUT_US + 1);
	EXPECT_TRUE(h.feed(SRC_RC)); // expired -> RC accepted although numerically lower
}

// TC-C-40: timeout boundary: sample is acceptable only while now < timestamp_sample + timeout
TEST(SQEManualControlSelector, TC_C_40_TimeoutBoundary)
{
	manual_control_setpoint_s in{};
	in.timestamp_sample = T0;
	in.valid = true;
	in.data_source = SRC_RC;

	ManualControlSelector just_in_time;
	just_in_time.setRcInMode(RC_ONLY);
	just_in_time.setTimeout(TIMEOUT_US);
	just_in_time.updateWithNewInputSample(T0 + TIMEOUT_US - 1, in, 7);
	EXPECT_EQ(just_in_time.instance(), 7);

	ManualControlSelector exactly_timeout;
	exactly_timeout.setRcInMode(RC_ONLY);
	exactly_timeout.setTimeout(TIMEOUT_US);
	exactly_timeout.updateWithNewInputSample(T0 + TIMEOUT_US, in, 7);
	EXPECT_EQ(exactly_timeout.instance(), -1);

	ManualControlSelector too_late;
	too_late.setRcInMode(RC_ONLY);
	too_late.setTimeout(TIMEOUT_US);
	too_late.updateWithNewInputSample(T0 + TIMEOUT_US + 1, in, 7);
	EXPECT_EQ(too_late.instance(), -1);
}

// TC-C-41: a sample flagged invalid is never chosen, even from an accepted source
TEST(SQEManualControlSelector, TC_C_41_InvalidSampleRejected)
{
	Harness h(RC_ONLY);
	EXPECT_FALSE(h.feed(SRC_RC, 10000, false));
	EXPECT_FALSE(h.sel().setpoint().valid);
	EXPECT_EQ(h.sel().instance(), -1);
	EXPECT_TRUE(h.feed(SRC_RC, 10000, true));
}

// TC-C-42: updateValidityOfChosenInput(): chosen input expires, or stops matching the configuration
TEST(SQEManualControlSelector, TC_C_42_ChosenInputExpiry)
{
	Harness h(RC_ONLY);
	ASSERT_TRUE(h.feed(SRC_RC));
	const uint64_t ts = h.now();
	const int chosen = h.sel().instance();

	h.sel().updateValidityOfChosenInput(ts + TIMEOUT_US - 1);
	EXPECT_TRUE(h.sel().setpoint().valid);
	EXPECT_EQ(h.sel().instance(), chosen);

	h.sel().updateValidityOfChosenInput(ts + TIMEOUT_US);
	EXPECT_FALSE(h.sel().setpoint().valid);
	EXPECT_EQ(h.sel().instance(), -1);

	// configuration change: chosen RC input is no longer allowed once manual control is disabled
	Harness h2(RC_ONLY);
	ASSERT_TRUE(h2.feed(SRC_RC));
	h2.sel().setRcInMode(DISABLED);
	h2.sel().updateValidityOfChosenInput(h2.now() + 1);
	EXPECT_FALSE(h2.sel().setpoint().valid);
	EXPECT_EQ(h2.sel().instance(), -1);
}

// TC-C-43: the chosen sample is copied; timestamp is overwritten with "now", timestamp_sample is preserved
TEST(SQEManualControlSelector, TC_C_43_SampleCopiedWithTimestampRules)
{
	manual_control_setpoint_s in{};
	in.timestamp_sample = T0;
	in.timestamp = 12345;
	in.valid = true;
	in.data_source = SRC_RC;
	in.roll = 0.25f;
	in.throttle = -0.5f;

	ManualControlSelector sel;
	sel.setRcInMode(RC_ONLY);
	sel.setTimeout(TIMEOUT_US);
	const uint64_t now = T0 + 100000;
	sel.updateWithNewInputSample(now, in, 2);

	EXPECT_EQ(sel.instance(), 2);
	EXPECT_TRUE(sel.setpoint().valid);
	EXPECT_EQ(sel.setpoint().timestamp, now);
	EXPECT_EQ(sel.setpoint().timestamp_sample, T0);
	EXPECT_FLOAT_EQ(sel.setpoint().roll, 0.25f);
	EXPECT_FLOAT_EQ(sel.setpoint().throttle, -0.5f);
	EXPECT_EQ(static_cast<int>(sel.setpoint().data_source), static_cast<int>(SRC_RC));
}
