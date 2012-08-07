#include "pid.h"

#include <px4/attitude_estimator_bm/matrix.h> //TODO: move matrix.h to somewhere else?

void pid_init(PID_t *pid, float kp, float ki, float kd, float intmax,
	      uint8_t mode, uint8_t plot_i)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->intmax = intmax;
	pid->mode = mode;
	pid->plot_i = plot_i;
	pid->count = 0;
	pid->saturated = 0;

	pid->sp = 0;
	pid->error_previous = 0;
	pid->integral = 0;
}
void pid_set_parameters(PID_t *pid, float kp, float ki, float kd, float intmax)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->intmax = intmax;
	//	pid->mode = mode;

	//	pid->sp = 0;
	//	pid->error_previous = 0;
	//	pid->integral = 0;
}

//void pid_set(PID_t *pid, float sp)
//{
//	pid->sp = sp;
//	pid->error_previous = 0;
//	pid->integral = 0;
//}

/**
 *
 * @param pid
 * @param val
 * @param dt
 * @return
 */
float pid_calculate(PID_t *pid, float sp, float val, float val_dot, float dt)
{
	/*  error = setpoint - actual_position
	 integral = integral + (error*dt)
	 derivative = (error - previous_error)/dt
	 output = (Kp*error) + (Ki*integral) + (Kd*derivative)
	 previous_error = error
	 wait(dt)
	 goto start
	 */

	float i, d;
	pid->sp = sp;
	float error = pid->sp - val;

	if (pid->saturated && (pid->integral * error > 0)) {
		//Output is saturated and the integral would get bigger (positive or negative)
		i = pid->integral;

		//Reset saturation. If we are still saturated this will be set again at output limit check.
		pid->saturated = 0;

	} else {
		i = pid->integral + (error * dt);
	}

	// Anti-Windup. Needed if we don't use the saturation above.
	if (pid->intmax != 0.0) {
		if (i > pid->intmax) {
			pid->integral = pid->intmax;

		} else if (i < -pid->intmax) {

			pid->integral = -pid->intmax;

		} else {
			pid->integral = i;
		}

		//Send Controller integrals
		//		Disabled because of new possibilities with debug_vect.
		//		Now sent in Main Loop at 5 Hz. 26.06.2010 Laurens
		//		if (pid->plot_i && (pid->count++ % 16 == 0)&&(global_data.param[PARAM_SEND_SLOT_DEBUG_2] == 1))
		//		{
		//			mavlink_msg_debug_send(MAVLINK_COMM_1, pid->plot_i, pid->integral);
		//		}
	}

	if (pid->mode == PID_MODE_DERIVATIV_CALC) {
		d = (error - pid->error_previous) / dt;

	} else if (pid->mode == PID_MODE_DERIVATIV_SET) {
		d = -val_dot;

	} else {
		d = 0;
	}

	pid->error_previous = error;

	return (error * pid->kp) + (i * pid->ki) + (d * pid->kd);
}
