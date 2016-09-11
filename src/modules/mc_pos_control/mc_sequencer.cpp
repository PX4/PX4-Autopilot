#include "mc_sequencer.h"


#ifdef JUST_FLIPS
/*
 * Perform a 360 degree roll or pitch flip starting and ending at current attitude
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
void flip_sequence(
	struct control_state_s &ctrl_state,
	struct vehicle_attitude_setpoint_s &att_sp,
	struct manual_control_setpoint_s &manual,
	math::Matrix<3, 3> &R_sp,
	float &rollRate, float &pitchRate, float &yawRate)
{

	enum Flip_state {
		IDLE, CLIMB, ROLL, PITCH, FINISH
	};
	static Flip_state cur_state = IDLE;
	static Flip_state last_state = FINISH;

	static math::Quaternion q_end;
	static math::Quaternion q_cur;
	static math::Vector<3> euler_end;
	static uint64_t start_finish = 0;
	const uint64_t climb_dur = 1.0f;

	float cur_time = hrt_absolute_time() / 1e6f;
	static float start_time = cur_time;

	/* if seq_switch just transitioned from off to on, begin substituting sequencer
	 * controls for manual controls. The sequencer could be a separate module publishing
	 * manual_control_setpoint messages, or a smaller message containing only
	 * x, y, z, r
	 */
//						uint8_t seq_switch = _manual.seq_switch;

	// for SITL, simulate seq_switch activation every 5 seconds
	static uint8_t seq_switch = manual_control_setpoint_s::SWITCH_POS_OFF;

	if ((cur_time - start_time) > 10.0f) {
		if (seq_switch == manual_control_setpoint_s::SWITCH_POS_OFF) {
			seq_switch = manual_control_setpoint_s::SWITCH_POS_ON;
			PX4_INFO("seq_switch on: at %f", (double) cur_time);
		}

		start_time = cur_time;
	}

	// reduce thrust when inverted (Earth z points down in body frame)
	// TODO: use ctrl_state instead
	bool inversion = (att_sp.R_body[8] > 0.0f);
	static bool inverted = false;

	if (inversion != inverted) {
		inverted = inversion;

		PX4_INFO("inversion at %6.3f, roll: %6.3f, pitch: %6.3f",
			 (double) cur_time, (double) att_sp.roll_body,
			 (double) att_sp.pitch_body);
	}

	/* substitute attitude sequence for _manual_control_setpoint */
	if (cur_state != last_state) {
		PX4_INFO("state: %d at %6.3f", cur_state, (double) cur_time);
		last_state = cur_state;
	}

	switch (cur_state) {

	case CLIMB: {

			rollRate = 0.0f;
			pitchRate = 0.0f;
			yawRate = 0.0f;
			att_sp.thrust = 0.9f;

			if ((cur_time - start_time) > climb_dur) {
//				cur_state = ROLL;
				cur_state = PITCH;
			}

			break;
		}

	case ROLL: {
			rollRate = M_TWOPI_F;
			pitchRate = 0.0f;
			yawRate = 0.0f;

			if (att_sp.R_body[8] > 0.0f) {
				att_sp.thrust = 0.5f;

			} else {
				att_sp.thrust = 0.2f;
			}

			q_cur.set(ctrl_state.q);
			math::Quaternion q_err = q_cur * q_end.conjugated();
			float error = acosf(fabsf(q_err.data[0]));

			if ((cur_time - start_time) > (climb_dur + 0.25f) && (error < 0.24f)) {
				rollRate = 0.0f;
				R_sp.from_euler(euler_end.data[0], euler_end.data[1],
						euler_end.data[2]);
				memcpy(&att_sp.R_body[0], R_sp.data, sizeof(att_sp.R_body));
				cur_state = FINISH;
				start_finish = cur_time;

				printf("finish roll: q_cur: "); q_cur.print();
				printf("q_end: "); q_end.print();
				printf("q_err: "); q_err.print();
				PX4_INFO("error %6.3f", (double) error);
			}

			break;
		}

	case PITCH: {
			rollRate = 0.0f;
			pitchRate = M_TWOPI_F;
			yawRate = 0.0f;

			if (att_sp.R_body[8] > 0.0f) {
				att_sp.thrust = 0.5f;

			} else {
				att_sp.thrust = 0.2f;
			}

			q_cur.set(ctrl_state.q);
			math::Quaternion q_err = q_cur * q_end.conjugated();
			float error = acosf(fabsf(q_err.data[0]));

			if ((cur_time - start_time) > (climb_dur + 0.5f) && (error < 0.24f)) {
				pitchRate = 0.0f;
				R_sp.from_euler(euler_end.data[0], euler_end.data[1],
						euler_end.data[2]);
				memcpy(&att_sp.R_body[0], R_sp.data, sizeof(att_sp.R_body));
				cur_state = FINISH;
				start_finish = cur_time;

				printf("finish pitch: q_cur: ");
				q_cur.print(); printf("q_end: ");
				q_end.print(); printf("q_err: ");
				q_err.print(); PX4_INFO("error %6.3f", (double) error);
			}

			break;
		}

	case FINISH: {
			// return to starting orientation
			q_cur.set(ctrl_state.q);
			math::Quaternion q_err = q_cur * q_end.conjugated();
			float error = acosf(fabsf(q_err.data[0]));

			// exit once error is small, or timeout
			if (error < 0.005f || (cur_time - start_finish) > 5.0f) {
				cur_state = IDLE;
				seq_switch = manual_control_setpoint_s::SWITCH_POS_OFF;
				PX4_INFO("sequence end at %6.3f, duration: %6.3f",
					 (double) cur_time,
					 (double)(cur_time - start_time));
				printf("q_cur: "); q_cur.print();
				printf("q_end: "); q_end.print();
				printf("q_err: "); q_err.print();
				PX4_INFO("error %6.3f", (double) error);
				printf("final Euler angles: "); q_cur.to_euler().print();
			}

			break;
		}

	case IDLE: {

			if (seq_switch == manual_control_setpoint_s::SWITCH_POS_ON) {
				cur_state = CLIMB;

				// initialize sequencer
				q_end.set(ctrl_state.q);
				euler_end = q_end.to_euler();
				// set roll and pitch to zero
				euler_end.data[0] = 0.0f;
				euler_end.data[1] = 0.0f;
				q_end.from_euler(euler_end.data[0], euler_end.data[1],
						 euler_end.data[2]);
				printf("starting Euler angles: "); q_end.to_euler().print();
				printf("q_end: "); q_end.print();
			}

			break;
		}
	}

}
#else

// TODO: make sure this gets stored in codespace ROM to avoid wasting RAM
static const struct seq_entry_s tilt_lr[3] {
	{Seq_state::ATTITUDE, 0.6f, 0.0f, 0.0f, 0.0f, {0.707f, 0.0f, 0.0f}, 1.0f},
	{Seq_state::ATTITUDE, 0.6f, 0.0f, 0.0f, 0.0f, { -0.707f, 0.0f, 0.0f}, 1.0f},
	{Seq_state::ATTITUDE, 0.5f, 0.0f, 0.0f, 0.0f, {0.0f, 0.0f, 0.0f}, 0.0f}
};
static const struct seq_entry_s pitch_flip[] {
	{Seq_state::ATTITUDE, 0.8f, 0.0f, 0.0f, 0.0f, {0.0f, 0.0f, 0.0f}, 0.5f},
	{Seq_state::RATE, 0.2f, 0.0f, M_TWOPI_F, 0.0f, {0.0f, 0.0f, 0.0f}, 1.0f},
	{Seq_state::ATTITUDE, 0.8f, 0.0f, 0.0f, 0.0f, {0.0f, 0.0f, 0.0f}, 0.25f},
	{Seq_state::ATTITUDE, 0.5f, 0.0f, 0.0f, 0.0f, {0.0f, 0.0f, 0.0f}, 0.0f}
};
static const struct seq_entry_s roll_flip[] {
	{Seq_state::ATTITUDE, 0.8f, 0.0f, 0.0f, 0.0f, {0.0f, 0.0f, 0.0f}, 0.5f},
	{Seq_state::RATE, 0.2f, M_TWOPI_F, 0.0f, 0.0f, {0.0f, 0.0f, 0.0f}, 1.0f},
	{Seq_state::ATTITUDE, 0.8f, 0.0f, 0.0f, 0.0f, {0.0f, 0.0f, 0.0f}, 0.25f},
	{Seq_state::ATTITUDE, 0.5f, 0.0f, 0.0f, 0.0f, {0.0f, 0.0f, 0.0f}, 0.0f}
};
static const struct seq_entry_s two_point_roll[] {
	{Seq_state::ATTITUDE, 0.8f, 0.0f, 0.0f, 0.0f, {0.0f, 0.0f, 0.0f}, 0.5f},

	{Seq_state::RATE, 0.2f, M_TWOPI_F, 0.0f, 0.0f, {0.0f, 0.0f, 0.0f}, 0.5f},

	{Seq_state::RATE, 0.2f, 0.0F, 0.0f, 0.0f, {0.0f, 0.0f, 0.0f}, 0.5f},

	{Seq_state::RATE, 0.2f, M_TWOPI_F, 0.0f, 0.0f, {0.0f, 0.0f, 0.0f}, 0.5f},

	{Seq_state::ATTITUDE, 0.8f, 0.0f, 0.0f, 0.0f, {0.0f, 0.0f, 0.0f}, 0.25f},
	{Seq_state::ATTITUDE, 0.5f, 0.0f, 0.0f, 0.0f, {0.0f, 0.0f, 0.0f}, 0.0f}
};
static const struct sequence tilt_lr_seq {
	sizeof(tilt_lr) / sizeof(seq_entry_s), tilt_lr
};
static const struct sequence pitch_flip_seq {
	sizeof(pitch_flip) / sizeof(seq_entry_s), pitch_flip
};
static const struct sequence roll_flip_seq {
	sizeof(roll_flip) / sizeof(seq_entry_s), roll_flip
};
static const struct sequence two_point_roll_seq {
	sizeof(two_point_roll) / sizeof(seq_entry_s), two_point_roll
};
static const struct sequence *cur_sequence = &pitch_flip_seq;

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
	math::Matrix<3, 3> &R_sp,
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
	static float start_sequence = cur_time;
	static float start_time = cur_time;

	static uint8_t seq_switch = manual_control_setpoint_s::SWITCH_POS_OFF;

	/* if seq_switch is on, begin substituting sequencer
	 * controls for manual controls.
	 */
//						uint8_t seq_switch = _manual.seq_switch;

	// for SITL, simulate seq_switch activation

	if ((cur_time - start_sequence) > 10.0f) {
		seq_switch = manual_control_setpoint_s::SWITCH_POS_ON;
		PX4_INFO("seq_switch on: at %f", (double) cur_time);

		start_sequence = cur_time;
	}

	// perform state transitions
	switch (cur_state) {

	case IDLE: {

			if (seq_switch == manual_control_setpoint_s::SWITCH_POS_ON) {
				seq_switch = manual_control_setpoint_s::SWITCH_POS_OFF;
				seq_index = -1;
				cur_state = NEXT_ENTRY;
			}

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
			if (error < 0.005f || (cur_time - start_time) > 5.0f) {
				cur_state = DELAY;
				printf("q_err: "); q_err.print();
				PX4_INFO("error %6.3f", (double) error);
				printf("final Euler angles: "); q_cur.to_euler().print();
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
		att_sp.thrust = seq_entry.thrust;	// this does not persist across calls
		cur_state = seq_entry.type;
		break;
	}

	// perform state entry actions
	if (cur_state != last_state) {
		start_time = cur_time;

		switch (cur_state) {
		case IDLE:
			// initialize sequencer
			seq_index = 0;
			seq_switch = manual_control_setpoint_s::SWITCH_POS_OFF;
			PX4_INFO("sequence end at %6.3f, duration: %6.3f",
				 (double) cur_time,
				 (double)(cur_time - start_sequence));
			PX4_INFO("IDLE state: %d at %6.3f, thrust: %6.3f", cur_state, (double) cur_time, (double)att_sp.thrust);
			break;

		case RATE:
			// set rates
			rollRate = seq_entry.rollRate;
			pitchRate = seq_entry.pitchRate;
			yawRate = seq_entry.yawRate;
			PX4_INFO("RATE state: %d at %6.3f, thrust: %6.3f", cur_state, (double) cur_time, (double)att_sp.thrust);
			break;

		case ATTITUDE:
			att_sp.thrust = seq_entry.thrust;	// this does not persist across calls
			// set target attitude
			R_sp.from_euler(seq_entry.target_euler[0], seq_entry.target_euler[1],
					seq_entry.target_euler[2]);
			memcpy(&att_sp.R_body[0], R_sp.data, sizeof(att_sp.R_body));
			q_end.from_euler(seq_entry.target_euler[0], seq_entry.target_euler[1],
					 seq_entry.target_euler[2]);
			PX4_INFO("ATTITUDE state: %d at %6.3f, thrust: %6.3f", cur_state, (double) cur_time, (double)att_sp.thrust);
//			printf("R_sp:\n"); R_sp.print();
//			printf("q_end: "); q_end.print();
//			printf("target Euler angles: "); q_end.to_euler().print();
			break;

		case DELAY:
			PX4_INFO("DELAY duration: %6.3f at %6.3f, thrust: %6.3f", (double)seq_entry.delay, (double) cur_time,
				 (double)att_sp.thrust);
			break;

		case NEXT_ENTRY:
			seq_index++;
			PX4_INFO("NEXT_ENTRY seq_index: %d at %6.3f, thrust: %6.3f", seq_index, (double) cur_time, (double)att_sp.thrust);

			if (seq_index < cur_sequence->N_entries) {
				seq_entry = cur_sequence->entries[seq_index];
				PX4_INFO("seq_entry: type: %d, \nrates: (%6.3f, %6.3f, %6.3f), \ntarget_euler: (%6.3f, %6.3f, %6.3f), \nthrust: %6.3f delay: %6.3f",
					 seq_entry.type,
					 (double)seq_entry.rollRate, (double)seq_entry.pitchRate, (double)seq_entry.yawRate,
					 (double)seq_entry.target_euler[0], (double)seq_entry.target_euler[1], (double)seq_entry.target_euler[2],
					 (double)seq_entry.thrust, (double)seq_entry.delay);

			} else {
				cur_state = IDLE;
			}

		default:
			break;
		}

		last_state = cur_state;
	}
}
#endif
