/**
 * @file av_estimator_a.cpp
 * Filter a class method implementation
 *
 * @author frits.kuipers <f.p.kuipers@student.utwente.nl>
 * @author Moses Bangura <moses.bangura@anu.edu.au, dnovichman@hotmail.com>
 */

#include "../include/av_estimator_a.h"

void av_estimator_a::update(Vector3f &a, Vector3f &w, Matrix3f &Rhat_b, vehicle_velocity_meas_inertial_s &rawVelocity, vehicle_gps_position_s &rawGPS, av_estimator_params  attitude_params, Vector3f &vhat_body, Vector3f &beta_a_filterb, float &dt)
{
	/* Copy the parameters */	
	k2vc    = attitude_params.att_vel_k2vc; 
	k2r 	= attitude_params.att_vel_k2r;  
	k2rc    = attitude_params.att_vel_k2rc; 
	k2u 	= attitude_params.att_vel_k2u;  
	k2w 	= attitude_params.att_vel_k2w;  
	k2ba 	= attitude_params.att_vel_k2ba; 
	k2bac   = attitude_params.att_vel_k2bac;	

	/* TODO remove this when we use other sensors */
	if (rawGPS.fix_type >= 3)
	{
		k2v 	= attitude_params.att_vel_k2v * 0.6f;
	} else
	{
		k2v 	= attitude_params.att_vel_k2v;
	}

	/* Moses Hack to test yaw rotation */
	Matrix3f Rhot;
	
	/* The angle used on px4 is stuffed up see above*/
	float roll, pitch, yaw;
	float ctheta, stheta, cphi, sphi, cpsi, spsi;
	roll 	= atan2f(Rhat_b(2,1), Rhat_b(2,2));	
	pitch 	= -asinf(Rhat_b(2,0));	
	yaw 	= atan2f(Rhat_b(1,0), Rhat_b(0,0));	

	ctheta = cos(pitch);
	stheta = sin(pitch);
	cphi = cos(-roll);
	sphi = sin(-roll);
	cpsi = cos(-yaw);
	spsi = sin(-yaw);
	Rhot(0,0) = ctheta*cpsi; 
	Rhot(0,1) = sphi*stheta*cpsi - cphi*spsi; 
	Rhot(0,2) = cphi*stheta*cpsi + sphi*spsi;     

	Rhot(1,0) = ctheta*spsi; 
	Rhot(1,1) = sphi*stheta*spsi + cphi*cpsi; 
	Rhot(1,2) = cphi*stheta*spsi - sphi*cpsi;		    

	Rhot(2,0) = -stheta; 
	Rhot(2,1) = ctheta*sphi; 
	Rhot(2,2) = ctheta*cphi; 
	//printf("vaht %3.3f %3.3f %3.3f\n",double(vhat(0)), double(vhat(1)),double(vhat(2)));

	Rhot = Rhat_b;

	/* End Moses Hack */

	/*  Input of velocities in {A}, either VICON or GPS */
	if(rawVelocity.inertial_valid)
	{
		valid = true;

		vbar_a(0) = rawVelocity.inertial_vx;
		vbar_a(1) = rawVelocity.inertial_vy;
		vbar_a(2) = rawVelocity.inertial_vz;

		if(rawVelocity.timestamp != prev_vel_timestamp)
		{
			if(prev_vel_timestamp != 0.0f)
				dt2 = (rawVelocity.timestamp - prev_vel_timestamp) / 1000000.0f;

			prev_vel_timestamp = rawVelocity.timestamp;

			verror 				= vhat - vbar_a;
			Rhat_dot_2 			= - k2r * skew(g/u * verror.cross(e3)) * Rhat;
			udot 				= -k2u * g * verror.transpose() * e3;
			u 					= udot * dt2 + u_prev;
			u_prev = u;
		}
		else
		{
			dt2 = 0.0f;
		}
	}	
	else
	{
		valid = false;
	}

	/* Calculate intgration update */
	vhat_dot 	= Rhot * (a - beta_a) + g * e3;// - k2vc * ((vhat_a_prev-what_a_prev)-Rhat*vhat_prev);					
	Rhat_dot 	= Rhat * skew(w);
	what_dot 	= - k2w * ((what_prev - vhat_prev) + Rhot * vhat_body); 
	//beta_a_dot 	= k2ba * k2v * verror;// - k2bac*(beta_a_2_prev-beta_a_prev); //comment Jan16
	Matrix3f beta_gains = Matrix3f::Zero();
	Matrix3f beta_gains_coup = Matrix3f::Zero();
	beta_gains(0,0) = k2ba * k2v;
	beta_gains(1,1) = k2ba * k2v;
	beta_gains(2,2) = -k2ba * k2v;

	beta_gains_coup(0,0) = k2bac;
	beta_gains_coup(1,1) = k2bac;
	beta_gains_coup(2,2) = k2bac * 0.3f;

	beta_a_dot 	= -beta_gains * verror - beta_gains_coup * (beta_a_prev - beta_a_filterb);

	/* if there is no measurement of v, we want to keep the same vhat */
	if (!valid)
	{
		vhat_dot = Vector3f::Zero();

		what_dot =  Vector3f::Zero();
		what_prev = Vector3f::Zero();
	
		dt2 = 0.0f;

		if (vbar_a.norm() < 5.0f)
		{
			vhat_prev(0) = vbar_a(0);
			vhat_prev(1) = vbar_a(1);
			vhat_prev(2) = vbar_a(2);
		} else
		{
			vhat_prev = Vector3f::Zero();
		}
	}
	
	vhat_dot_2 = - k2v * (vhat - vbar_a) -k2vc * ((vhat - what) - Rhot * vhat_body); //Rhat_b was here

	/* Integrate */
	X 		= u * Rhat;
	vhat 	= vhat_dot * dt   + vhat_dot_2 * dt + vhat_prev;
	Rhat 	= Rhat_dot * dt   + Rhat_dot_2 * dt2 + Rhat_prev;
	what 	= what_dot * dt   + what_prev;
	beta_a 	= beta_a_dot * dt + beta_a_prev;

	/* During restart of velocity component and 5m/s is the max velocity the vehicle can experience */
	if (abs(vhat.norm() - vbar_a.norm()) > 5.0f)
		vhat = vbar_a;
	
	/* Set previous values */
	Rhat_prev 		= Rhat;
	vhat_prev 		= vhat;
	beta_a_prev 	= beta_a;
	what_prev 		= what;
}


