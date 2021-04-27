/****************************************************************************
 *
 *   Copyright (c) 2013-2017 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file main.c
 *
 * Example implementation of a fixed wing attitude controller. This file is a complete
 * fixed wing controller for manual attitude control or auto waypoint control.
 * There is no need to touch any other system components to extend / modify the
 * complete control architecture.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch> change
 */

#include "params.h"
#include "controllers.h"
//#include <math.h>
#include <cmath>
#include <algorithm>
#include <iostream>
//#include <lib/matrix/matrix/math.hpp>


#include <poll.h>

#include <stdlib.h>
#include <mathlib/mathlib.h>
#include <drivers/drv_hrt.h>
#include <lib/ecl/geo/geo.h>
#include <matrix/math.hpp>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <systemlib/err.h>
#include <parameters/param.h>
#include <perf/perf_counter.h>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>

#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/estimator_states.h>
#include <uORB/topics/airspeed_validated.h>

/*
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
*/

using namespace time_literals;

/* Prototypes */

/**
 * Initialize all parameter handles and values
 *
 */
extern "C" int parameters_init(struct param_handles *h);

/**
 * Update all parameters
 *
 */
extern "C" int parameters_update(const struct param_handles *h, struct params *p);

/**
 * Daemon management function.
 *
 * This function allows to start / stop the background task (daemon).
 * The purpose of it is to be able to start the controller on the
 * command line, query its status and stop it, without giving up
 * the command line to one particular process or the need for bg/fg
 * ^Z support by the shell.
 */
extern "C" __EXPORT int ex_fixedwing_control_main(int argc, char *argv[]);

/**
 * Mainloop of daemon.
 */
int fixedwing_control_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);


/**
 * Control roll and pitch angle.
 *
 * This very simple roll and pitch controller takes the current roll angle
 * of the system and compares it to a reference. Pitch is controlled to zero and yaw remains
 * uncontrolled (tutorial code, not intended for flight).
 *
 * @param att_sp The current attitude setpoint - the values the system would like to reach.
 * @param att The current attitude. The controller should make the attitude match the setpoint
 * @param rates_sp The angular rate setpoint. This is the output of the controller.
 */
void control_attitude(const struct vehicle_attitude_setpoint_s *att_sp, const struct vehicle_attitude_s *att,
		      struct vehicle_rates_setpoint_s *rates_sp,
		      struct actuator_controls_s *actuators);

/**
 * Control heading.
 *
 * This very simple heading to roll angle controller outputs the desired roll angle based on
 * the current position of the system, the desired position (the setpoint) and the current
 * heading.
 *
 * @param pos The current position of the system
 * @param sp The current position setpoint
 * @param att The current attitude
 * @param att_sp The attitude setpoint. This is the output of the controller
 */
void control_heading(const struct vehicle_global_position_s *pos, const struct position_setpoint_s *sp,
		     const struct vehicle_attitude_s *att, struct vehicle_attitude_setpoint_s *att_sp);

/**
 *Matrix Multiplication
 *
 * Dampens the pitch rate(q) of the aircraft.
 *
 * @param rn Row of matrix n
 * @param cn column of matix n
 * @param matn matrix n
 */
float matrix_mult(int r1, int c1, int r2, int c2, float mat1[10][10], float mat2[10][10]);


/* Variables */
static bool thread_should_exit = false;		/**< Daemon exit flag */
static bool thread_running = false;		/**< Daemon status flag */
static int deamon_task;				/**< Handle of deamon task / thread */
static struct params p;
static struct param_handles ph;

void control_attitude(const struct vehicle_attitude_setpoint_s *att_sp, const struct vehicle_attitude_s *att,
		      struct vehicle_rates_setpoint_s *rates_sp,
		      struct actuator_controls_s *actuators)
{

	/*
	 * The PX4 architecture provides a mixer outside of the controller.
	 * The mixer is fed with a default vector of actuator controls, representing
	 * moments applied to the vehicle frame. This vector
	 * is structured as:
	 *
	 * Control Group 0 (attitude):
	 *
	 *    0  -  roll   (-1..+1)
	 *    1  -  pitch  (-1..+1)
	 *    2  -  yaw    (-1..+1)
	 *    3  -  thrust ( 0..+1)
	 *    4  -  flaps  (-1..+1)
	 *    ...
	 *
	 * Control Group 1 (payloads / special):
	 *
	 *    ...
	 */

	/*
	 * Calculate roll error and apply P gain
	 */

	matrix::Eulerf att_euler = matrix::Quatf(att->q);
	matrix::Eulerf att_sp_euler = matrix::Quatf(att_sp->q_d);

	float roll_err = att_euler.phi() - att_sp_euler.phi();
	actuators->control[0] = roll_err * p.roll_p;

	/*
	 * Calculate pitch error and apply P gain
	 */
	float pitch_err = att_euler.theta() - att_sp_euler.theta();
	actuators->control[1] = pitch_err * p.pitch_p;
}

void control_heading(const struct vehicle_global_position_s *pos, const struct position_setpoint_s *sp,
		     const struct vehicle_attitude_s *att, struct vehicle_attitude_setpoint_s *att_sp)
{

	/*
	 * Calculate heading error of current position to desired position
	 */

	float bearing = get_bearing_to_next_waypoint(pos->lat, pos->lon, sp->lat, sp->lon);

	matrix::Eulerf att_euler = matrix::Quatf(att->q);

	/* calculate heading error */
	float yaw_err = att_euler.psi() - bearing;
	/* apply control gain */
	float roll_body = yaw_err * p.hdng_p;

	/* limit output, this commonly is a tuning parameter, too */
	if (roll_body < -0.6f) {
		roll_body = -0.6f;

	} else if (att_sp->roll_body > 0.6f) {
		roll_body = 0.6f;
	}

	matrix::Eulerf att_spe(roll_body, 0, bearing);
	matrix::Quatf(att_spe).copyTo(att_sp->q_d);
}


void matrix_mult(float mat1[][10], float mat2[][10],float mat_mult[][10], int r1, int c1, int r2, int c2)
{
	if(c1!=r2){ // incorrect dimentions
		printf("Incorrect dimenstion size");
		thread_should_exit=true;
	}else{
		// Initializing elements of matrix mult to 0.
   		for (int i = 0; i < r1; ++i) {
      			for (int j = 0; j < c2; ++j) {
         			mat_mult[i][j] = 0;
      			}
   		}

		// Multiplying first and second matrices and storing it in result
   		for (int i = 0; i < r1; ++i) {
      			for (int j = 0; j < c2; ++j) {
         			for (int k = 0; k < c1; ++k) {
            				mat_mult[i][j] += mat1[i][k] * mat2[k][j];
         			}
      			}
   		}
	}

}


// Two funtions below from: https://www.geeksforgeeks.org/program-dot-product-cross-product-two-vector/
// Function that return
// dot product of two vector array.
float dotProduct(float vect_A[], float vect_B[])
{

    float product = 0;

    // Loop for calculate dot product
    for (int i = 0; i < 3; i++){
        product = product + vect_A[i] * vect_B[i];
    }
    return product;
}

// Function to find
// cross product of two vector array.
void crossProduct(float vect_A[], float vect_B[], float cross_P[])

{

    cross_P[0] = vect_A[1] * vect_B[2] - vect_A[2] * vect_B[1];
    cross_P[1] = vect_A[2] * vect_B[0] - vect_A[0] * vect_B[2];
    cross_P[2] = vect_A[0] * vect_B[1] - vect_A[1] * vect_B[0];
}

void Controllers::init_ECL_variables(){
	_hdot_max=2;//altitude controller saturation block max limit
	_hdot_min=-2;//altitude controller saturation block min limit
	_phi_max=30*M_PI/180;//heading controller roll angle saturation block max limit
	_phi_min=-30*M_PI/180;;//heading controller roll angle saturation block max limit
	_integrator_I[0][0]=0;_integrator_I[1][0]=0;//Airspeed and Climb Rate integrator
	_integrator_RA=0;//Roll Angle Controller intergrator
	_x_guide=0;//intial value must be 0
	_loc_guide=1;//initial value must be 1(location of first destination)
	K_q=-0.034739;//pitch rate damper gain
	float temp[2][6]={{-0.0106,24.6502,-0.3462,-38.3493,0.0285,-12.5189},{34.2713,13.9895,-0.0457,-13.7433,41.4445,6.3751}};
	std::copy(&temp[0][0],&temp[0][0]+2*6,&K_ASCR[0][0]);//airspeed and climbrate controller gain
	K_h=1.75;//altitude controller gain
	K_P_RAC=0.03679;//roll angle controller proportional control gain
	K_I_RAC=0.0147;//roll angle controller intergrator control gain
	K_psi=1.9;//heading controller gain
	K_y=0.01;//guidance controller gain
	vbar_trim=30;//trim airspeed
	alpha_trim=0.0355;//trim angle of attack(and pitch)
}

void Controllers::initialise_NSADLC_HPF()
{
	float f_cut=9.5;//1/T_c from MATLAB code !!!CHECK!!!
	NSADLC_HPF.setfCut(f_cut);
	NSADLC_HPF.setY(0);
	NSADLC_HPF.setU(0);
}

//Actual Controllers
float Controllers::airspeed_controller(const Control_Data &state_data,float vbar_ref,const float dt){
	float edot_a=(state_data.airspeed-vbar_trim)-vbar_ref;
	_intergrator_AS = _intergrator_AS + edot_a*dt;
	float dT = -Ki_AS*_intergrator_AS - Kp_AS*edot_a;
	return dT;
}

void Controllers::NSADLC_controller(const Control_Data &state_data,float Cw_ref,const float dt,float *defl)
{
	//DLC
	float ew=-Cw_ref+state_data.Cw;
	float edot_f=NSADLC_HPF.update(ew);
	_intergrator_DLC = _intergrator_DLC + edot_f*dt;
	float df = -Ki_f*_intergrator_DLC;

	//NSA
	float edot_c=-Cw_ref+state_data.Cw;
	_intergrator_NSA=_intergrator_NSA+edot_c*dt;
	float de = Cw_ref*Nc - _intergrator_NSA * Ki_e - state_data.Cw * Kc - state_data.q * Kq + Km * df;

	//copy to defl array
	*defl=de;
	defl++;
	*defl=df;
}

float Controllers::climb_rate_controller(const Control_Data &state_data,float hdot_ref,const float dt)
{
	float edot_r=-hdot_ref+state_data.h_dot;
	_intergrator_CR=_intergrator_CR+edot_r*dt;
	float Cw_ref=-Ki_CR*_intergrator_CR-Kp_CR*edot_r;
	return Cw_ref;
}

float Controllers::altitude_controller(const Control_Data &state_data,float h_ref,const float dt)
{
	float hdot_ref=-Kp_alt*(state_data.posz-h_ref); //!!!Check posz positive direction!!!
	return hdot_ref;
}

float Controllers::LSA(const Control_Data &state_data,float Bw_ref,const float dt)
{
	float edot_b=state_data.Bw-Bw_ref;
	_intergrator_LSA=_intergrator_LSA+edot_b*dt;
	float dr_r=-Ki_LSA*_intergrator_LSA;
	float dr=dr_r-state_data.Bw*K_B-state_data.r*K_r;
	return dr;
}

float Controllers::roll_rate_controller(const Control_Data &state_data,float p_ref,const float dt)
{
	float edot_p=state_data.p-p_ref;
	_intergrator_RR=_intergrator_RR+edot_p*dt;
	float da=-_intergrator_RR*Ki_RR-edot_p*Kp_RR;
	return da;
}

float Controllers::roll_angle_controller(const Control_Data &state_data,float phi_ref,const float dt)
{
	float p_ref=-Kp_RA*(state_data.phi-phi_ref);
	return p_ref;
}

float Controllers::guidance_controller_1(const Control_Data &state_data,float phibar_ref,float y_ref,const float dt)
{
	float phi_ref=phibar_ref-Kp_G*(state_data.y-y_ref)-state_data.ydot*Kd_G;
	return phi_ref;
}

float Controllers::yaw_controller(const Control_Data &state_data,float psi_ref,const float dt)
{
	float edot_psi=state_data.psi-psi_ref;
	_intergrator_yaw=_intergrator_yaw+edot_psi*dt;
	float Bw_ref=-Ki_yaw*_intergrator_yaw-Kp_yaw*edot_psi;
	return Bw_ref;
}

//Course Notes Controllers
// float Controllers::pitch_rate_damper_controller(const Control_Data &state_data,float dE)
// {
// 	float de=dE-state_data.q*K_q;
// 	return de;
// }

// void Controllers::airspeed_climb_rate_controller(const Control_Data &state_data,float vbar_ref,float hdot_ref,const float dt,float *defl)
// {
// 	float xdot_I[2][1]={{state_data.airspeed-vbar_trim-vbar_ref},{-1*state_data.h_dot-hdot_ref}};
// 	//matrix multiplication
// 	float res[10][10],m1[10][10],m2[10][10];//result,matrix1,matrix2
// 	int r1=2,c1=1,r2=1,c2=1;//row and column of matrix 1, row and column of matrix 2
// 	m1[0][0]=xdot_I[0][0];m1[1][0]=xdot_I[1][0];m2[0][0]=dt;
// 	matrix_mult(m1, m2,res,r1,c1,r2,c2);
// 	//Intergration calculation
// 	for(int i = 0; i < 2; i++){
// 		_integrator_I[i][0]=_integrator_I[i][0]+res[i][0];
// 	}
// 	//Intialise m1 and m2 to zero
// 	for (int i = 0; i < 10; ++i) {
//       		for (int j = 0; j < 10; ++j) {
//          		m1[i][j] = 0;
// 			m2[i][j] = 0;
//       		}
//    	}

// 	//LQR controller multiplication
// 	r1=2;c1=6;r2=6;c2=1;
// 	//Copy K_ASCR into m1
// 	for(int i=0;i<r1;i++){
// 		for(int j=0;j<c1;j++){
// 			m1[i][j]=K_ASCR[i][j];
// 		}
// 	}
// 	//Copy states into m2
// 	m2[0][0]=state_data.airspeed-vbar_trim;m2[1][0]=state_data.alpha-alpha_trim;m2[2][0]=state_data.q;m2[3][0]=state_data.theta-alpha_trim;m2[4][0]=_integrator_I[0][0];m2[5][0]=_integrator_I[1][0];
// 	//matrix multiplication
// 	matrix_mult(m1, m2,res,r1,c1,r2,c2);
// 	//copy to defl array
// 	for(int i=0;i<2;i++){
// 		*defl=res[i][0];
// 		defl++;
// 	}
// }

// float Controllers::altitude_controller(const Control_Data &state_data,float h_ref)
// {
// 	float hdot_ref=K_h*(h_ref-(-1*state_data.posz));
// 	//saturation block implmentation
// 	hdot_ref= math::constrain(hdot_ref, _hdot_max, _hdot_min);//limits define in ecl_controller.h file
// 	return hdot_ref;
// }

// // float BlockHighPass::update(float input)
// // {
// // 	float b = 2 * float(M_PI) * getFCut() * getdt();
// // 	float K_r=-0.072027;
// // 	float a = K_r*1 / (1 + b);
// // 	setY(a * (getY() + input - getU()));
// // 	setU(input);
// // 	return getY();
// // }

// float Controllers::dutch_roll_damper_controller(const Control_Data &state_data, float dt) //!!!Complete!!!
// {
// 	setdt(dt);
// 	float temp=update(state_data.r);
// 	return 0-temp;
// }

// float Controllers::roll_angle_controller(const Control_Data &state_data,float phi_ref,const float dt)
// {
// 	float phi_err=phi_ref-state_data.phi;
// 	_integrator_RA=_integrator_RA+phi_err*dt*K_I_RAC;
// 	float dA=-1*(_integrator_RA+phi_err*K_P_RAC);
// 	return dA;
// }

float Controllers::heading_controller(const Control_Data &state_data,float psi_ref)
{
	float psi_error,u_ref[3]={cos(psi_ref),sin(psi_ref),0},u[3]={cos(state_data.psi),sin(state_data.psi),0},u_D[3]={0,0,1};
	float p_mat[3];
	crossProduct(u,u_ref,p_mat);
	float x=dotProduct(p_mat,u_D);
	float sign=((x > 0) - (x < 0));
	psi_error= sign*acos(dotProduct(u,u_ref));
	float phi_ref= psi_error*K_psi;
	phi_ref = math::constrain(phi_ref, _phi_max, _phi_min);
	return phi_ref;
}

void Controllers:: guide_axis_alg(float S[2],float D[2],const Control_Data &state_data,float out[4])
{
	//track angle
	float psi_track=atan2((D[1]-S[1]),(D[2]-S[2]));
	//x and 	y calculation
	//conversion from inertial axis to track axis via matrix multiplication
	float res[10][10],m1[10][10],m2[10][10];//result,matrix1,matrix2
	int r1=2,c1=2,r2=2,c2=1;//row and column of matrix 1, row and column of matrix 2
	m1[0][0]=cos(psi_track);m1[0][1]=sin(psi_track);m1[1][0]=-sin(psi_track);m1[1][1]=cos(psi_track);m2[0][0]=(state_data.posx-S[1]);m2[1][0]=(state_data.posy-S[0]);
	matrix_mult(m1, m2,res,r1,c1,r2,c2);
	out[0]=res[0][0];out[1]=res[1][0];out[2]=psi_track;
	//ydot calculation
	for(int i=0;i<10;i++){
		for(int j=0;j<10;j++){
			res[i][j]=0;
			m1[i][j]=0;
			m2[i][j]=0;
		}
	}
	int r1=2,c1=2,r2=2,c2=1;//row and column of matrix 1, row and column of matrix 2
	m1[0][0]=cos(psi_track);m1[0][1]=sin(psi_track);m1[1][0]=-sin(psi_track);m1[1][1]=cos(psi_track);m2[0][0]=(state_data.velx_I);m2[1][0]=(state_data.vely_I);
	matrix_mult(m1,m2,res,r1,c1,r2,c2);
	out[3]=res[1][0];
}

void Controllers:: waypoint_scheduler(float Destinations[][2],int Destination_size,const Control_Data &state_data,float out[2][2]) //!!!COMPLETE!!!
{
	float L_track=sqrt(pow((Destinations[_loc_guide][1]-Destinations[_loc_guide-1][1]),2)+pow((Destinations[_loc_guide][0]-Destinations[_loc_guide-1][0]),2));//Track length
	if(_x_guide>L_track){
        	if(_loc_guide!=Destination_size)//If not final destination then update waypoints
		{
	    	std::copy(&Destinations[_loc_guide][0],&Destinations[_loc_guide][0]+1*2,&out[0][0]);//copy starting point coordinates into out array
		std::copy(&Destinations[_loc_guide+1][0],&Destinations[_loc_guide+1][0]+1*2,&out[1][0]);//copy destination point coordinates into out array
            	_loc_guide=_loc_guide+1;
		}else
		{
		std::copy(&Destinations[_loc_guide-1][0],&Destinations[_loc_guide-1][0]+1*2,&out[0][0]);//keep start point the same
		std::copy(&Destinations[_loc_guide][0],&Destinations[_loc_guide][0]+1*2,&out[1][0]);//keep desitination the same
 	        thread_should_exit=true;//End simulation (!!!CHECK!!!)
        	}
    	}else
    	{ //Did not reach destination point yet
       	std::copy(&Destinations[_loc_guide-1][0],&Destinations[_loc_guide-1][0]+1*2,&out[0][0]);//keep start point the same
	std::copy(&Destinations[_loc_guide][0],&Destinations[_loc_guide][0]+1*2,&out[1][0]);//keep destination the same
	}

}

float Controllers:: guidance_controller(float D[][2],int Destination_size,const Control_Data &state_data)
{
	//add waypoint_scheduler and guide_axis_alg function call outs
	float points[2][2],guide_val[3];
	waypoint_scheduler(D,Destination_size,state_data,points);
	guide_axis_alg(points[0],points[1],state_data,guide_val);
	_x_guide=guide_val[0];//update x in memory block
	// workout and return psi_ref
	float psi_ref=K_y*(0-guide_val[1])+guide_val[2];
	return psi_ref;
}

int parameters_init(struct param_handles *handles)
{
	/* PID parameters */
	handles->hdng_p 	=	param_find("EXFW_HDNG_P");
	handles->roll_p 	=	param_find("EXFW_ROLL_P");
	handles->pitch_p 	=	param_find("EXFW_PITCH_P");

	return 0;
}

int parameters_update(const struct param_handles *handles, struct params *parameters)
{
	param_get(handles->hdng_p, &(parameters->hdng_p));
	param_get(handles->roll_p, &(parameters->roll_p));
	param_get(handles->pitch_p, &(parameters->pitch_p));

	return 0;
}


/* Main Thread */
int fixedwing_control_thread_main(int argc, char *argv[])
{
	/* read arguments */
	bool verbose = false;

	for (int i = 1; i < argc; i++) {
		if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "--verbose") == 0) {
			verbose = true;
		}
	}

	/* welcome user (warnx prints a line, including an appended\n, with variable arguments */
	warnx("[example fixedwing control] started");

	/* initialize parameters, first the handles, then the values */
	parameters_init(&ph);
	parameters_update(&ph, &p);


	/*
	 * PX4 uses a publish/subscribe design pattern to enable
	 * multi-threaded communication.
	 *
	 * The most elegant aspect of this is that controllers and
	 * other processes can either 'react' to new data, or run
	 * at their own pace.
	 *
	 * PX4 developer guide:
	 * https://pixhawk.ethz.ch/px4/dev/shared_object_communication
	 *
	 * Wikipedia description:
	 * http://en.wikipedia.org/wiki/Publish–subscribe_pattern
	 *
	 */




	/*
	 * Declare and safely initialize all structs to zero.
	 *
	 * These structs contain the system state and things
	 * like attitude, position, the current waypoint, etc.
	 */
	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));
	struct vehicle_attitude_setpoint_s att_sp;
	memset(&att_sp, 0, sizeof(att_sp));
	struct vehicle_rates_setpoint_s rates_sp;
	memset(&rates_sp, 0, sizeof(rates_sp));
	struct vehicle_global_position_s global_pos;
	memset(&global_pos, 0, sizeof(global_pos));
	struct manual_control_setpoint_s manual_control_setpoint;
	memset(&manual_control_setpoint, 0, sizeof(manual_control_setpoint));
	struct vehicle_status_s vstatus;
	memset(&vstatus, 0, sizeof(vstatus));
	struct position_setpoint_s global_sp;
	memset(&global_sp, 0, sizeof(global_sp));
	struct estimator_states_s estim;
	memset(&estim, 0, sizeof(estim));
	struct vehicle_angular_velocity_s ang_vel;
	memset(&ang_vel, 0, sizeof(ang_vel));
	struct airspeed_validated_s airspeed;
	memset(&airspeed, 0, sizeof(airspeed));

	/* output structs - this is what is sent to the mixer */
	struct actuator_controls_s actuators;
	memset(&actuators, 0, sizeof(actuators));

	/*create controllers object to store states for the controllers*/
	Controllers _fw_cust_ctrls;
	//initalise ECL_control class variables and dutch roll damper HPF
	_fw_cust_ctrls.init_ECL_variables();
	//_fw_cust_ctrls.initialise_DRD_HPF();



	/* publish actuator controls with zero values */
	for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROLS; i++) {
		actuators.control[i] = 0.0f;
	}

	/*
	 * Advertise that this controller will publish actuator
	 * control values and the rate setpoint
	 */
	orb_advert_t actuator_pub = orb_advertise(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, &actuators);
	orb_advert_t rates_pub = orb_advertise(ORB_ID(vehicle_rates_setpoint), &rates_sp);

	/* subscribe to topics. */
	int att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	int global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	int manual_control_setpoint_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	int vstatus_sub = orb_subscribe(ORB_ID(vehicle_status));
	int global_sp_sub = orb_subscribe(ORB_ID(position_setpoint_triplet));
	int estim_sub = orb_subscribe(ORB_ID(estimator_states));
	int ang_vel_sub = orb_subscribe(ORB_ID(vehicle_angular_velocity));
	int airspeed_sub= orb_subscribe(ORB_ID(airspeed_validated));


	uORB::SubscriptionInterval parameter_update_sub{ORB_ID(parameter_update), 1_s};

	/* Setup of loop */

	struct pollfd fds[1] {};
	fds[0].fd = att_sub;
	fds[0].events = POLLIN;

	while (!thread_should_exit) {

		/*
		 * Wait for a sensor or param update, check for exit condition every 500 ms.
		 * This means that the execution will block here without consuming any resources,
		 * but will continue to execute the very moment a new attitude measurement or
		 * a param update is published. So no latency in contrast to the polling
		 * design pattern (do not confuse the poll() system call with polling).
		 *
		 * This design pattern makes the controller also agnostic of the attitude
		 * update speed - it runs as fast as the attitude updates with minimal latency.
		 */
		int ret = poll(fds, 1, 500);

		if (ret < 0) {
			/*
			 * Poll error, this will not really happen in practice,
			 * but its good design practice to make output an error message.
			 */
			warnx("poll error");

		} else if (ret == 0) {
			/* no return value = nothing changed for 500 ms, ignore */
		} else {

			// check for parameter updates
			if (parameter_update_sub.updated()) {
				// clear update
				parameter_update_s pupdate;
				parameter_update_sub.copy(&pupdate);

				// if a param update occured, re-read our parameters
				parameters_update(&ph, &p);
			}

			/* only run controller if attitude changed */
			if (fds[0].revents & POLLIN) {


				/* Check if there is a new position measurement or position setpoint */
				bool pos_updated;
				orb_check(global_pos_sub, &pos_updated);
				bool global_sp_updated;
				orb_check(global_sp_sub, &global_sp_updated);
				bool manual_control_setpoint_updated;
				orb_check(manual_control_setpoint_sub, &manual_control_setpoint_updated);

				/* get a local copy of attitude */
				orb_copy(ORB_ID(vehicle_attitude), att_sub, &att);

				/* time step(dt) calculation*/
				const float dt = math::constrain((att.timestamp - _last_run) * 1e-6f, 0.002f, 0.04f);
				_last_run = att.timestamp;

				/*calculate rotation matix(DCM) and euler angles(CHECK IF CORRECT AS IT IS DIRECTLY FROM FixedwingAttitudeControl.cpp)*/
				matrix::Dcmf R = matrix::Quatf(att.q);
				const matrix::Eulerf euler_angles(R);

				/*get local copy of estimator states*/
				orb_copy(ORB_ID(estimator_states), estim_sub, &estim);

				/*get local copy of angular velocity states*/
				orb_copy(ORB_ID(vehicle_angular_velocity), ang_vel_sub, &ang_vel);

				/*get local copy of airspeed state*/
				orb_copy(ORB_ID(airspeed_validated), airspeed_sub, &airspeed);

				/*get alpha and beta*/
				//get body velocity
				const matrix::Vector3f vel_inertia=matrix::Vector3f(estim.states[4],estim.states[5],estim.states[6]);
				const matrix::Vector3f vel_b=R.operator*(vel_inertia);
				//work out polar coordinates
				float vbar=sqrt(pow(vel_b(0),2)+pow(vel_b(1),2)+pow(vel_b(2),2));
				float alpha=atan2(vel_b(2),vel_b(0));
				float beta=asin(vel_b(1)/vbar);

				/*Define Control Data*/
				Control_Data control_input{};
				control_input.phi=euler_angles.phi();
				control_input.theta=euler_angles.theta();
				control_input.psi=euler_angles.psi();
				control_input.p=ang_vel.xyz[0];
				control_input.q=ang_vel.xyz[1];
				control_input.r=ang_vel.xyz[2];
				control_input.airspeed=airspeed.calibrated_airspeed_m_s;//check
				control_input.alpha=alpha;
				control_input.beta=beta;
				control_input.posx=estim.states[7];
				control_input.posy=estim.states[8];
				control_input.posz=estim.states[9];
				control_input.h_dot=estim.states[6];


				if (global_sp_updated) {
					struct position_setpoint_triplet_s triplet;
					orb_copy(ORB_ID(position_setpoint_triplet), global_sp_sub, &triplet);
					memcpy(&global_sp, &triplet.current, sizeof(global_sp));
				}

				if (manual_control_setpoint_updated)
					/* get the RC (or otherwise user based) input */
				{
					orb_copy(ORB_ID(manual_control_setpoint), manual_control_setpoint_sub, &manual_control_setpoint);
				}

				/* check if the throttle was ever more than 50% - go later only to failsafe if yes */
				if (PX4_ISFINITE(manual_control_setpoint.z) &&
				    (manual_control_setpoint.z >= 0.6f) &&
				    (manual_control_setpoint.z <= 1.0f)) {
				}

				/* get the system status and the flight mode we're in */
				orb_copy(ORB_ID(vehicle_status), vstatus_sub, &vstatus);


				/*Run Controllers*/
				//Longitudanal Controllers
				//Altitude controller
				float href=250;//altitude (Make a way to trigger )
				float hdot_ref=_fw_cust_ctrls.altitude_controller(control_input,href);
				float vbar_ref=0;
				//Airspeed and Climbrate controller
				float defl[2];
				_fw_cust_ctrls.airspeed_climb_rate_controller(control_input,vbar_ref,hdot_ref,dt,&defl[0]);
				//Pitch Rate Damper
				float dE=_fw_cust_ctrls.pitch_rate_damper_controller(control_input,defl[0]);
				float dT=defl[1];

				//Lateral Controllers
				//Guidance Controller
				float destinations[][2]={{0,0},{0,1000},{4000,1000},{4000,0},{0,0}};int destination_size=sizeof(destinations)/sizeof(destinations[0])-1;
				float psi_ref=_fw_cust_ctrls.guidance_controller(destinations,destination_size,control_input);
				//Heading controller
				float phi_ref=_fw_cust_ctrls.heading_controller(control_input,psi_ref);
				//Roll Angle controller
				float dA=_fw_cust_ctrls.roll_angle_controller(control_input,phi_ref,dt);
				//Dutch Roll Damper
				float dR=_fw_cust_ctrls.dutch_roll_damper_controller(control_input, dt);

				/*Assign actuators*/
				actuators.control[0]=dA;actuators.control[1]=dE;actuators.control[2]=dR;actuators.control[3]=dT;

				/*set setpoint rates*/
				//Check!!!
				rates_sp.roll=control_input.phi;
				rates_sp.pitch=control_input.theta;
				rates_sp.yaw=control_input.psi;

				/* publish rates */
				orb_publish(ORB_ID(vehicle_rates_setpoint), rates_pub, &rates_sp);

				/* sanity check and publish actuator outputs */
				if (PX4_ISFINITE(actuators.control[0]) &&
				    PX4_ISFINITE(actuators.control[1]) &&
				    PX4_ISFINITE(actuators.control[2]) &&
				    PX4_ISFINITE(actuators.control[3])) {
					orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);

					if (verbose) {
						warnx("published");
					}
				}
			}
		}
	}

	printf("[ex_fixedwing_control] exiting, stopping all motors.\n");
	thread_running = false;

	/* kill all outputs */
	for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROLS; i++) {
		actuators.control[i] = 0.0f;
	}

	orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);

	fflush(stdout);

	return 0;
}

/* Startup Functions */

static void
usage(const char *reason)
{
	if (reason) {
		fprintf(stderr, "%s\n", reason);
	}

	fprintf(stderr, "usage: ex_fixedwing_control {start|stop|status}\n\n");
}

/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to px4_task_spawn_cmd().
 */
int ex_fixedwing_control_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("ex_fixedwing_control already running\n");
			/* this is not an error */
			return 0;
		}

		thread_should_exit = false;
		deamon_task = px4_task_spawn_cmd("ex_fixedwing_custom_control",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_MAX - 20,
						 2048,
						 fixedwing_control_thread_main,
						 (argv) ? (char *const *)&argv[2] : (char *const *)nullptr);
		thread_running = true;
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			printf("\tex_fixedwing_control is running\n");

		} else {
			printf("\tex_fixedwing_control not started\n");
		}

		return 0;
	}

	usage("unrecognized command");
	return 0;
}
