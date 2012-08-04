/*
 * kalman.h
 *
 *  Created on: 01.12.2010
 *      Author: Laurens Mackay
 */

#ifndef KALMAN_H_
#define KALMAN_H_

#include "matrix.h"

#define KALMAN_MAX_STATES 12
#define KALMAN_MAX_MEASUREMENTS 9
typedef struct {
	int states;
	int measurements;
	matrix_t a;
	matrix_t c;
	matrix_t gain_start;
	matrix_t gain;
	matrix_t x_apriori;
	matrix_t x_aposteriori;
	float gainfactor;
	int gainfactorsteps;
} kalman_t;

void kalman_init(kalman_t *kalman, int states, int measurements, m_elem a[],
		 m_elem c[], m_elem gain_start[], m_elem gain[], m_elem x_apriori[],
		 m_elem x_aposteriori[], int gainfactorsteps);
void kalman_predict(kalman_t *kalman);
void kalman_correct(kalman_t *kalman, m_elem measurement_a[], m_elem mask_a[]);
m_elem kalman_get_state(kalman_t *kalman, int state);

#endif /* KALMAN_H_ */
