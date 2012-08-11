/*
 * pid.h
 *
 *  Created on: May 29, 2012
 *      Author: thomasgubler
 */

#ifndef PID_H_
#define PID_H_

#include <stdint.h>

/* PID_MODE_DERIVATIV_CALC calculates discrete derivative from previous error
 * val_dot in pid_calculate() will be ignored */
#define PID_MODE_DERIVATIV_CALC	0
/* Use PID_MODE_DERIVATIV_SET if you have the derivative already (Gyros, Kalman) */
#define PID_MODE_DERIVATIV_SET	1

typedef struct {
	float kp;
	float ki;
	float kd;
	float intmax;
	float sp;
	float integral;
	float error_previous;
	uint8_t mode;
	uint8_t plot_i;
	uint8_t count;
	uint8_t saturated;
} PID_t;

void pid_init(PID_t *pid, float kp, float ki, float kd, float intmax, uint8_t mode, uint8_t plot_i);
void pid_set_parameters(PID_t *pid, float kp, float ki, float kd, float intmax);
//void pid_set(PID_t *pid, float sp);
float pid_calculate(PID_t *pid, float sp, float val, float val_dot, float dt);



#endif /* PID_H_ */
