#include <px4_config.h>

#include "mc_sequencer.h"

// TODO: make sure this gets stored in codespace ROM to avoid wasting RAM
static const struct seq_entry_s hover[] {
	{Seq_state::ATTITUDE, 0.5f, 0.0f, 0.0f, 0.0f, {0.0f, 0.0f, 0.0f}, 0.0f}
};

static const struct seq_entry_s coord_turn[] {
	{Seq_state::ATTITUDE, 0.8f, 0.0f, 0.0f, 0.0f, {0.5f, -0.25f, 0.0f}, 0.0f},
	{Seq_state::RATE, 0.8f, 0.0f, 0.0f, 0.785f, { -0.707f, 0.0f, 0.0f}, 26.0f},
	{Seq_state::ATTITUDE, 0.5f, 0.0f, 0.0f, 0.0f, {0.0f, 0.0f, 0.0f}, 0.0f}
};
static const struct sequence coord_turn_seq {
	sizeof(coord_turn) / sizeof(seq_entry_s), coord_turn
};

static const struct seq_entry_s pitch_flip[] {
	{Seq_state::ATTITUDE, 0.8f, 0.0f, 0.0f, 0.0f, {0.0f, 0.0f, 0.0f}, 0.5f},
	{Seq_state::RATE, 0.2f, 0.0f, M_TWOPI_F, 0.0f, {0.0f, 0.0f, 0.0f}, 1.0f},
	{Seq_state::ATTITUDE, 0.8f, 0.0f, 0.0f, 0.0f, {0.0f, 0.0f, 0.0f}, 0.25f},
	{Seq_state::ATTITUDE, 0.5f, 0.0f, 0.0f, 0.0f, {0.0f, 0.0f, 0.0f}, 0.0f}
};
static const struct sequence pitch_flip_seq {
	sizeof(pitch_flip) / sizeof(seq_entry_s), pitch_flip
};

static const struct seq_entry_s roll_flip[] {
	{Seq_state::ATTITUDE, 0.8f, 0.0f, 0.0f, 0.0f, {0.0f, 0.0f, 0.0f}, 0.5f},
	{Seq_state::RATE, 0.2f, M_TWOPI_F, 0.0f, 0.0f, {0.0f, 0.0f, 0.0f}, 1.0f},
	{Seq_state::ATTITUDE, 0.8f, 0.0f, 0.0f, 0.0f, {0.0f, 0.0f, 0.0f}, 0.25f},
	{Seq_state::ATTITUDE, 0.5f, 0.0f, 0.0f, 0.0f, {0.0f, 0.0f, 0.0f}, 0.0f}
};
static const struct sequence roll_flip_seq {
	sizeof(roll_flip) / sizeof(seq_entry_s), roll_flip
};

static const struct seq_entry_s two_point_roll[] {
	{Seq_state::ATTITUDE, 0.8f, 0.0f, 0.0f, 0.0f, {0.0f, 0.0f, 0.0f}, 0.5f},

	{Seq_state::RATE, 0.2f, M_TWOPI_F, 0.0f, 0.0f, {0.0f, 0.0f, 0.0f}, 0.5f},

	{Seq_state::RATE, 0.2f, 0.0F, 0.0f, 0.0f, {0.0f, 0.0f, 0.0f}, 0.5f},

	{Seq_state::RATE, 0.2f, M_TWOPI_F, 0.0f, 0.0f, {0.0f, 0.0f, 0.0f}, 0.5f},

	{Seq_state::ATTITUDE, 0.8f, 0.0f, 0.0f, 0.0f, {0.0f, 0.0f, 0.0f}, 0.25f},
	{Seq_state::ATTITUDE, 0.5f, 0.0f, 0.0f, 0.0f, {0.0f, 0.0f, 0.0f}, 0.0f}
};
static const struct sequence two_point_roll_seq {
	sizeof(two_point_roll) / sizeof(seq_entry_s), two_point_roll
};

static const struct seq_entry_s tilt_lr[] {
	{Seq_state::RATE, 0.4f, 1.0f, 0.0f, 0.0f, {0.0f, 0.0f, 0.0f}, 0.5f},
	{Seq_state::RATE, 0.4f, -1.0f, 0.0f, 0.0f, {0.0f, 0.0f, 0.0f}, 1.0f},
	{Seq_state::RATE, 0.4f, 1.0f, 0.0f, 0.0f, {0.0f, 0.0f, 0.0f}, 0.5f}
};
static const struct sequence tilt_lr_seq {
	sizeof(tilt_lr) / sizeof(seq_entry_s), tilt_lr
};
static const struct sequence *cur_sequence = &two_point_roll_seq;

/*
 * Execute a sequence of commands: each command is a seq_entry_s struct specifying
 * 	type: attitude, rate, or delay
 * 	roll/pitch/yaw rates
 * 	target attitude (fixed xyz Euler angles)
 * 	delay
 *
 * Inputs:
 * ctrl_state: for current attitude quaternion
 * manual: sequence trigger switch
 *
 * Outputs:
 * att_sp: thrust and rotation angle setpoints
 * R_sp: attitude setpoint
 * rollRate, pitchRate, yawRate: rotation rate commands
 */
void prog_sequence(
	struct control_state_s &ctrl_state,
	struct vehicle_attitude_setpoint_s &att_sp,
	struct manual_control_setpoint_s &manual,
	matrix::Quatf &R_sp,
	float &rollRate, float &pitchRate, float &yawRate)
{

	static Seq_state cur_state = IDLE;
	static Seq_state last_state = IDLE;
	static seq_entry_s seq_entry;
	static int seq_index = 0;

	static math::Quaternion q_end;
	static math::Quaternion q_cur;
	static math::Vector<3> euler_end;

	float cur_time = (double)hrt_absolute_time() / 1e6;
	static float start_sequence = -1.0f;
	static float start_time = cur_time;

	static uint8_t seq_switch_last = manual_control_setpoint_s::SWITCH_POS_OFF;

#if !defined CONFIG_ARCH_BOARD_SITL

	/* if seq_switch is on, begin substituting sequencer
	 * controls for manual controls.
	 */
	uint8_t seq_switch = manual.seq_switch;

	// force IDLE if switch transitions to off during a sequence
	if (seq_switch == manual_control_setpoint_s::SWITCH_POS_OFF &&
	    seq_switch != seq_switch_last) {

		cur_state = IDLE;
	}

#else
	// for SITL, simulate seq_switch activation
#define TRIG_INTVL 10.0F
	static uint8_t seq_switch = manual_control_setpoint_s::SWITCH_POS_OFF;

	if (seq_switch == manual_control_setpoint_s::SWITCH_POS_OFF) {
		if ((cur_time - start_sequence) > TRIG_INTVL) {
			seq_switch = manual_control_setpoint_s::SWITCH_POS_ON;
			PX4_DEBUG("seq_switch on: at %f", (double) cur_time);

		} else if (fmodf((cur_time - start_sequence), 2.0f) < 0.015f) {
			PX4_DEBUG("seq countdown: %5.3f", (double)(TRIG_INTVL - (cur_time - start_sequence)));
		}
	}

#endif

	// perform state transitions
	switch (cur_state) {

	case IDLE: {

			// only the rising edge of seq_switch triggers a sequence
			if (seq_switch == manual_control_setpoint_s::SWITCH_POS_ON &&
			    seq_switch != seq_switch_last) {

				seq_index = -1;
				start_sequence = cur_time;
				cur_state = NEXT_ENTRY;
			}

			seq_switch_last = seq_switch;

			break;
		}

	case RATE: {
			// immediate transition to delay state
			cur_state = DELAY;
			break;
		}

	case ATTITUDE: {
			// wait for target attitude
			att_sp.thrust = seq_entry.thrust;	// this does not persist across calls

			q_cur.set(ctrl_state.q);
			math::Quaternion q_err = q_cur * q_end.conjugated();
			float error = acosf(fabsf(q_err.data[0]));

			// once attitude is reached (or timeout), transition to DELAY
			if (error < 0.005f || (cur_time - start_time) > 2.0f) {
				cur_state = DELAY;

				if (error > .005f) { PX4_DEBUG("ATTITUDE state timeout"); }

				PX4_DEBUG("q_err: "); q_err.print();
				PX4_DEBUG("error %6.3f", (double) error);
				PX4_DEBUG("final Euler angles: "); q_cur.to_euler().print();
			}

			break;
		}

	case DELAY: {
			// set rates
			att_sp.thrust = seq_entry.thrust;	// this does not persist across calls
			rollRate = seq_entry.rollRate;
			pitchRate = seq_entry.pitchRate;
			yawRate = seq_entry.yawRate;

			// delay
			if (cur_time >= (start_time + seq_entry.delay)) {
				// then perform next sequence entry
				cur_state = NEXT_ENTRY;
			}

			break;
		}

	case NEXT_ENTRY:
		PX4_DEBUG("NEXT_ENTRY seq_index: %d at %6.3f, thrust: %6.3f", seq_index, (double) cur_time, (double)att_sp.thrust);

		if (seq_index < cur_sequence->N_entries) {
			seq_entry = cur_sequence->entries[seq_index];

			att_sp.thrust = seq_entry.thrust;	// this does not persist across calls
			cur_state = seq_entry.type;

			PX4_DEBUG("seq_entry: type: %d, \nrates: (%6.3f, %6.3f, %6.3f), \ntarget_euler: (%6.3f, %6.3f, %6.3f), \nthrust: %6.3f delay: %6.3f",
				  seq_entry.type,
				  (double)seq_entry.rollRate, (double)seq_entry.pitchRate, (double)seq_entry.yawRate,
				  (double)seq_entry.target_euler[0], (double)seq_entry.target_euler[1], (double)seq_entry.target_euler[2],
				  (double)seq_entry.thrust, (double)seq_entry.delay);

		} else {
			cur_state = IDLE;
		}

		break;
	}

	// perform state entry actions
	if (cur_state != last_state) {
		start_time = cur_time;

		switch (cur_state) {
		case IDLE:
			// initialize sequencer
			seq_index = -1;
			seq_switch = manual_control_setpoint_s::SWITCH_POS_OFF;
			PX4_DEBUG("sequence end at %6.3f, duration: %6.3f",
				  (double) cur_time,
				  (double)(cur_time - start_sequence));
			PX4_DEBUG("enter IDLE state: %d at %6.3f, thrust: %6.3f", cur_state, (double) cur_time, (double)att_sp.thrust);
			break;

		case RATE:
			// set rates
			rollRate = seq_entry.rollRate;
			pitchRate = seq_entry.pitchRate;
			yawRate = seq_entry.yawRate;
			PX4_DEBUG("enter RATE state: %d at %6.3f, thrust: %6.3f", cur_state, (double) cur_time, (double)att_sp.thrust);
			break;

		case ATTITUDE:
			att_sp.thrust = seq_entry.thrust;	// this does not persist across calls

			// set target attitude
			R_sp = matrix::Quatf(matrix::Eulerf(seq_entry.target_euler[0], seq_entry.target_euler[1],
							    att_sp.yaw_body)); //seq_entry.target_euler[2]));

			q_end.from_euler(seq_entry.target_euler[0], seq_entry.target_euler[1],
					 seq_entry.target_euler[2]);

			PX4_DEBUG("enter ATTITUDE state: %d at %6.3f, thrust: %6.3f", cur_state, (double) cur_time, (double)att_sp.thrust);
//			printf("R_sp:\n"); R_sp.print();
//			printf("q_end: "); q_end.print();
//			printf("target Euler angles: "); q_end.to_euler().print();
			break;

		case DELAY:
			PX4_DEBUG("enter DELAY state, duration: %6.3f at %6.3f, thrust: %6.3f", (double)seq_entry.delay, (double) cur_time,
				  (double)att_sp.thrust);
			break;

		case NEXT_ENTRY:
			seq_index++;
			break;

		default:
			break;
		}

		last_state = cur_state;
	}
}
