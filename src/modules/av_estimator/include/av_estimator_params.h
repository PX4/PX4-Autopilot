/**
* @file av_estimator_params.h
* Filter params header file
*
* @author frits.kuipers <f.p.kuipers@student.utwente.nl>
*/

#ifndef AV_ESTIMATOR_PARAMS_H
#define AV_ESTIMATOR_PARAMS_H

#include <systemlib/param/param.h>

struct av_estimator_params {
	float att_vel_k1;
	float att_vel_k2;
	float att_vel_k3;
	float att_vel_k4;
	float att_vel_k5;
	float att_vel_k6;
	float att_vel_k7;
	float att_vel_k8;
	float att_vel_k9;
	float att_vel_k1vicon;
	float att_vel_k10;
	float att_vel_k2v;
	float att_vel_k2vc; 
	float att_vel_k2r;  
	float att_vel_k2rc; 
	float att_vel_k2u;  
	float att_vel_k2w;  
	float att_vel_k2ba; 
	float att_vel_k2bac;
	float cbar_x;
	float cbar_y;
	float cbar_x_offset;
	float cbar_y_offset;
	int curr_calib_en;
	float magcalib_k0_0;
	float magcalib_k0_1;
	float magcalib_k0_2;
	float magcalib_k1_0;
	float magcalib_k1_1;
	float magcalib_k1_2;
	float magcalib_k2_0;
	float magcalib_k2_1;
	float magcalib_k2_2;
	float magcalib_k3_0;
	float magcalib_k3_1;
	float magcalib_k3_2;
	float magcalib_c0_0;
	float magcalib_c0_1;
	float magcalib_c0_2;
	float magcalib_c1_0;
	float magcalib_c1_1;
	float magcalib_c1_2;
	float magcalib_c2_0;
	float magcalib_c2_1;
	float magcalib_c2_2;
	float magcalib_c3_0;
	float magcalib_c3_1;
	float magcalib_c3_2;
	float baro_k1;
	float baro_k2;
	float baro_k3;
};

struct av_estimator_param_handles {
	param_t att_vel_k1;
	param_t att_vel_k2;
	param_t att_vel_k3;
	param_t att_vel_k4;
	param_t att_vel_k5;
	param_t att_vel_k6;
	param_t att_vel_k7;
	param_t att_vel_k8;
	param_t att_vel_k9;
	param_t att_vel_k10;
	param_t att_vel_k1vicon;
	param_t att_vel_k2v;
	param_t att_vel_k2vc; 
	param_t att_vel_k2r;  
	param_t att_vel_k2rc; 
	param_t att_vel_k2u;  
	param_t att_vel_k2w;  
	param_t att_vel_k2ba; 
	param_t att_vel_k2bac;
	param_t cbar_x;
	param_t cbar_y;
	param_t cbar_x_offset;
	param_t cbar_y_offset;
	param_t curr_calib_en;
	param_t magcalib_k0_0;
	param_t magcalib_k0_1;
	param_t magcalib_k0_2;
	param_t magcalib_k1_0;
	param_t magcalib_k1_1;
	param_t magcalib_k1_2;
	param_t magcalib_k2_0;
	param_t magcalib_k2_1;
	param_t magcalib_k2_2;
	param_t magcalib_k3_0;
	param_t magcalib_k3_1;
	param_t magcalib_k3_2;
	param_t magcalib_c0_0;
	param_t magcalib_c0_1;
	param_t magcalib_c0_2;
	param_t magcalib_c1_0;
	param_t magcalib_c1_1;
	param_t magcalib_c1_2;
	param_t magcalib_c2_0;
	param_t magcalib_c2_1;
	param_t magcalib_c2_2;
	param_t magcalib_c3_0;
	param_t magcalib_c3_1;
	param_t magcalib_c3_2;
	param_t baro_k1;
	param_t baro_k2;
	param_t baro_k3;
};

/**
 * Initialize all parameter handles and values
 *
 */
int parameters_init(struct av_estimator_param_handles *h);

/**
 * Update all parameters
 *
 */
int parameters_update(const struct av_estimator_param_handles *h, struct av_estimator_params *p);

#endif
