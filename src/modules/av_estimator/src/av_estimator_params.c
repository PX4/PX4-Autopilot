/**
 * @file av_estimator_params.cpp
 * Filter params file, initialisation and update
 *
 * @author frits.kuipers <f.p.kuipers@student.utwente.nl>
 * @author Moses Bangura <moses.bangura@anu.edu.au, dnovichman@hotmail.com>
 */

#include "../include/av_estimator_params.h"

 /* There is no need to follow px4 style as they will show
  * up in Default Group 
 */

/* Filter 1 */
PARAM_DEFINE_FLOAT(ATT_VEL_K1_V, 4.0f); 
PARAM_DEFINE_FLOAT(ATT_VEL_K1_VC, 0.0f);
PARAM_DEFINE_FLOAT(ATT_VEL_K1_R, 0.003f);
PARAM_DEFINE_FLOAT(ATT_VEL_K1_RC, 0.0f);
PARAM_DEFINE_FLOAT(ATT_VEL_K1_U, 0.1f);
PARAM_DEFINE_FLOAT(ATT_VEL_K1_MU, 0.01f);
PARAM_DEFINE_FLOAT(ATT_VEL_K1_BA, 0.01f);
PARAM_DEFINE_FLOAT(ATT_VEL_K1_BW, 0.01f);
PARAM_DEFINE_FLOAT(ATT_VEL_K1_BAC, 0.0f);
PARAM_DEFINE_FLOAT(ATT_VEL_K1_BWC, 0.0f);
PARAM_DEFINE_FLOAT(ATT_VEL_K1_VICON, 4.0f);


/* Filter 2 */
PARAM_DEFINE_FLOAT(ATT_VEL_K2_V, 12.0f); 
PARAM_DEFINE_FLOAT(ATT_VEL_K2_VC, 0.0f);
PARAM_DEFINE_FLOAT(ATT_VEL_K2_R, 0.003f);
PARAM_DEFINE_FLOAT(ATT_VEL_K2_RC, 0.0f);
PARAM_DEFINE_FLOAT(ATT_VEL_K2_U, 0.1f);
PARAM_DEFINE_FLOAT(ATT_VEL_K2_W, 0.01f);
PARAM_DEFINE_FLOAT(ATT_VEL_K2_BA, 0.01f);
PARAM_DEFINE_FLOAT(ATT_VEL_K2_BAC, 0.0f);

PARAM_DEFINE_FLOAT(ATT_CBAR_X, 0.2f);
PARAM_DEFINE_FLOAT(ATT_CBAR_Y, 0.2f);

PARAM_DEFINE_FLOAT(ATT_BARO_K1, 0.90f);
PARAM_DEFINE_FLOAT(ATT_BARO_K2, 0.5f);
PARAM_DEFINE_FLOAT(ATT_BARO_K3, 0.0905f);

PARAM_DEFINE_INT32(ATT_C_CAL_EN, 1);

PARAM_DEFINE_FLOAT(ATT_CBAR_XOFF, -0.03);
PARAM_DEFINE_FLOAT(ATT_CBAR_YOFF, -0.03);

int parameters_init(struct av_estimator_param_handles *h)
{	
	/* Filter gain parameters */
	h->att_vel_k1 = 	param_find("ATT_VEL_K1_V");
	h->att_vel_k2 = 	param_find("ATT_VEL_K1_VC");
	h->att_vel_k3 = 	param_find("ATT_VEL_K1_R");
	h->att_vel_k4 = 	param_find("ATT_VEL_K1_RC");
	h->att_vel_k5 = 	param_find("ATT_VEL_K1_U");
	h->att_vel_k6 = 	param_find("ATT_VEL_K1_MU");
	h->att_vel_k7 = 	param_find("ATT_VEL_K1_BA");
	h->att_vel_k8 = 	param_find("ATT_VEL_K1_BW");
	h->att_vel_k9 = 	param_find("ATT_VEL_K1_BAC");
	h->att_vel_k10 = 	param_find("ATT_VEL_K1_BWC");
	h->att_vel_k1vicon =	param_find("ATT_VEL_K1_VICON");


	h->att_vel_k2v   =      param_find("ATT_VEL_K2_V");
	h->att_vel_k2vc  =      param_find("ATT_VEL_K2_VC");
	h->att_vel_k2r   =      param_find("ATT_VEL_K2_R");
	h->att_vel_k2rc  =      param_find("ATT_VEL_K2_RC");
	h->att_vel_k2u   =      param_find("ATT_VEL_K2_U"); 
	h->att_vel_k2w   =      param_find("ATT_VEL_K2_W"); 
	h->att_vel_k2ba  =      param_find("ATT_VEL_K2_BA");
	h->att_vel_k2bac =      param_find("ATT_VEL_K2_BAC");

	h->cbar_x =		param_find("ATT_CBAR_X");
	h->cbar_y =		param_find("ATT_CBAR_Y");

	h->cbar_x_offset = param_find("ATT_CBAR_XOFF");
	h->cbar_y_offset = param_find("ATT_CBAR_YOFF");


	h->curr_calib_en =	param_find("ATT_C_CAL_EN");

	/* Find params for magnetometer current calibration */
	h->magcalib_k0_0 = param_find("MAGCALIB_K0_0");
	h->magcalib_k0_1 = param_find("MAGCALIB_K0_1");
	h->magcalib_k0_2 = param_find("MAGCALIB_K0_2");
	h->magcalib_k1_0 = param_find("MAGCALIB_K1_0");
	h->magcalib_k1_1 = param_find("MAGCALIB_K1_1");
	h->magcalib_k1_2 = param_find("MAGCALIB_K1_2");
	h->magcalib_k2_0 = param_find("MAGCALIB_K2_0");
	h->magcalib_k2_1 = param_find("MAGCALIB_K2_1");
	h->magcalib_k2_2 = param_find("MAGCALIB_K2_2");
	h->magcalib_k3_0 = param_find("MAGCALIB_K3_0");
	h->magcalib_k3_1 = param_find("MAGCALIB_K3_1");
	h->magcalib_k3_2 = param_find("MAGCALIB_K3_2");

	h->magcalib_c0_0 = param_find("MAGCALIB_C0_0");
	h->magcalib_c0_1 = param_find("MAGCALIB_C0_1");
	h->magcalib_c0_2 = param_find("MAGCALIB_C0_2");
	h->magcalib_c1_0 = param_find("MAGCALIB_C1_0");
	h->magcalib_c1_1 = param_find("MAGCALIB_C1_1");
	h->magcalib_c1_2 = param_find("MAGCALIB_C1_2");
	h->magcalib_c2_0 = param_find("MAGCALIB_C2_0");
	h->magcalib_c2_1 = param_find("MAGCALIB_C2_1");
	h->magcalib_c2_2 = param_find("MAGCALIB_C2_2");
	h->magcalib_c3_0 = param_find("MAGCALIB_C3_0");
	h->magcalib_c3_1 = param_find("MAGCALIB_C3_1");
	h->magcalib_c3_2 = param_find("MAGCALIB_C3_2");

	h->baro_k1 = param_find("ATT_BARO_K1");
	h->baro_k2 = param_find("ATT_BARO_K2");
	h->baro_k3 = param_find("ATT_BARO_K3");

	return OK;
}

int parameters_update(const struct av_estimator_param_handles *h, struct av_estimator_params *p)
{
	/* Update filter gain */
	param_get(h->att_vel_k1, &(p->att_vel_k1));
	param_get(h->att_vel_k2, &(p->att_vel_k2));
	param_get(h->att_vel_k3, &(p->att_vel_k3));
	param_get(h->att_vel_k4, &(p->att_vel_k4));
	param_get(h->att_vel_k5, &(p->att_vel_k5));
	param_get(h->att_vel_k6, &(p->att_vel_k6));
	param_get(h->att_vel_k7, &(p->att_vel_k7));
	param_get(h->att_vel_k8, &(p->att_vel_k8));
	param_get(h->att_vel_k9, &(p->att_vel_k9));
	param_get(h->att_vel_k10, &(p->att_vel_k10));
	param_get(h->att_vel_k1vicon, &(p->att_vel_k1vicon));


	param_get(h->att_vel_k2v  ,	&(p->att_vel_k2v  ));
	param_get(h->att_vel_k2vc ,	&(p->att_vel_k2vc ));
	param_get(h->att_vel_k2r  ,	&(p->att_vel_k2r  ));
	param_get(h->att_vel_k2rc ,	&(p->att_vel_k2rc ));
	param_get(h->att_vel_k2u  ,	&(p->att_vel_k2u  ));
	param_get(h->att_vel_k2w  ,	&(p->att_vel_k2w  ));
	param_get(h->att_vel_k2ba ,	&(p->att_vel_k2ba ));
	param_get(h->att_vel_k2bac,	&(p->att_vel_k2bac));

	param_get(h->cbar_x, &(p->cbar_x));
	param_get(h->cbar_y, &(p->cbar_y));

	param_get(h->cbar_x_offset, &(p->cbar_x_offset));
	param_get(h->cbar_y_offset, &(p->cbar_y_offset));

	param_get(h->baro_k1, &(p->baro_k1));
	param_get(h->baro_k2, &(p->baro_k2));
	param_get(h->baro_k3, &(p->baro_k3));

	param_get(h->curr_calib_en, &(p->curr_calib_en));

	/* Update params for current calibration*/
	param_get(h->magcalib_k0_0, &(p->magcalib_k0_0));
	param_get(h->magcalib_k0_1, &(p->magcalib_k0_1));
	param_get(h->magcalib_k0_2, &(p->magcalib_k0_2));
	param_get(h->magcalib_k1_0, &(p->magcalib_k1_0));
	param_get(h->magcalib_k1_1, &(p->magcalib_k1_1));
	param_get(h->magcalib_k1_2, &(p->magcalib_k1_2));
	param_get(h->magcalib_k2_0, &(p->magcalib_k2_0));
	param_get(h->magcalib_k2_1, &(p->magcalib_k2_1));
	param_get(h->magcalib_k2_2, &(p->magcalib_k2_2));
	param_get(h->magcalib_k3_0, &(p->magcalib_k3_0));
	param_get(h->magcalib_k3_1, &(p->magcalib_k3_1));
	param_get(h->magcalib_k3_2, &(p->magcalib_k3_2));

	param_get(h->magcalib_c0_0, &(p->magcalib_c0_0));
	param_get(h->magcalib_c0_1, &(p->magcalib_c0_1));
	param_get(h->magcalib_c0_2, &(p->magcalib_c0_2));
	param_get(h->magcalib_c1_0, &(p->magcalib_c1_0));
	param_get(h->magcalib_c1_1, &(p->magcalib_c1_1));
	param_get(h->magcalib_c1_2, &(p->magcalib_c1_2));
	param_get(h->magcalib_c2_0, &(p->magcalib_c2_0));
	param_get(h->magcalib_c2_1, &(p->magcalib_c2_1));
	param_get(h->magcalib_c2_2, &(p->magcalib_c2_2));
	param_get(h->magcalib_c3_0, &(p->magcalib_c3_0));
	param_get(h->magcalib_c3_1, &(p->magcalib_c3_1));
	param_get(h->magcalib_c3_2, &(p->magcalib_c3_2));

	return OK;
}
