/*
 * kalman.c
 *
 *  Created on: 01.12.2010
 *      Author: Laurens Mackay
 */
#include "kalman.h"
//#include "mavlink_debug.h"

void kalman_init(kalman_t *kalman, int states, int measurements, m_elem a[],
		 m_elem c[], m_elem gain_start[], m_elem gain[], m_elem x_apriori[],
		 m_elem x_aposteriori[], int gainfactorsteps)
{
	kalman->states = states;
	kalman->measurements = measurements;
	kalman->gainfactorsteps = gainfactorsteps;
	kalman->gainfactor = 0;

	//Create all matrices that are persistent
	kalman->a = matrix_create(states, states, a);
	kalman->c = matrix_create(measurements, states, c);
	kalman->gain_start = matrix_create(states, measurements, gain_start);
	kalman->gain = matrix_create(states, measurements, gain);
	kalman->x_apriori = matrix_create(states, 1, x_apriori);
	kalman->x_aposteriori = matrix_create(states, 1, x_aposteriori);
}

void kalman_predict(kalman_t *kalman)
{
	matrix_mult(kalman->a, kalman->x_aposteriori, kalman->x_apriori);
}

void kalman_correct(kalman_t *kalman, m_elem measurement_a[], m_elem mask_a[])
{
	//create matrices from inputs
	matrix_t measurement =
		matrix_create(kalman->measurements, 1, measurement_a);
	matrix_t mask = matrix_create(kalman->measurements, 1, mask_a);

	//create temporary matrices
	m_elem gain_start_part_a[KALMAN_MAX_STATES * KALMAN_MAX_MEASUREMENTS] =
		{ };
	matrix_t gain_start_part = matrix_create(kalman->states,
				   kalman->measurements, gain_start_part_a);

	m_elem gain_part_a[KALMAN_MAX_STATES * KALMAN_MAX_MEASUREMENTS] =
		{ };
	matrix_t gain_part = matrix_create(kalman->states, kalman->measurements,
					   gain_part_a);

	m_elem gain_sum_a[KALMAN_MAX_STATES * KALMAN_MAX_MEASUREMENTS] =
		{ };
	matrix_t gain_sum = matrix_create(kalman->states, kalman->measurements,
					  gain_sum_a);

	m_elem error_a[KALMAN_MAX_MEASUREMENTS * 1] =
		{ };
	matrix_t error = matrix_create(kalman->measurements, 1, error_a);

	m_elem measurement_estimate_a[KALMAN_MAX_MEASUREMENTS * 1] =
		{ };
	matrix_t measurement_estimate = matrix_create(kalman->measurements, 1,
					measurement_estimate_a);

	m_elem x_update_a[KALMAN_MAX_STATES * 1] =
		{ };
	matrix_t x_update = matrix_create(kalman->states, 1, x_update_a);



	//x(:,i+1)=xapriori+(gainfactor*[M_50(:,1) M(:,2)]+(1-gainfactor)*M_start)*(z-C*xapriori);


	//est=C*xapriori;
	matrix_mult(kalman->c, kalman->x_apriori, measurement_estimate);
	//error=(z-C*xapriori) = measurement-estimate
	matrix_sub(measurement, measurement_estimate, error);
	matrix_mult_element(error, mask, error);

	kalman->gainfactor = kalman->gainfactor * (1.0f - 1.0f
			     / kalman->gainfactorsteps) + 1.0f * 1.0f / kalman->gainfactorsteps;



	matrix_mult_scalar(kalman->gainfactor, kalman->gain, gain_part);

	matrix_mult_scalar(1.0f - kalman->gainfactor, kalman->gain_start,
			   gain_start_part);

	matrix_add(gain_start_part, gain_part, gain_sum);

	//gain*(z-C*xapriori)
	matrix_mult(gain_sum, error, x_update);

	//xaposteriori = xapriori + update

	matrix_add(kalman->x_apriori, x_update, kalman->x_aposteriori);


//	static int i=0;
//	if(i++==4){
//		i=0;
//	float_vect3 out_kal;
//	out_kal.x = M(gain_sum,0,1);
////	out_kal_z.x = z_measurement[1];
//	out_kal.y = M(gain_sum,1,1);
//	out_kal.z = M(gain_sum,2,1);
//	debug_vect("out_kal", out_kal);
//	}
}

m_elem kalman_get_state(kalman_t *kalman, int state)
{
	return M(kalman->x_aposteriori, state, 0);
}
