#include "kalman_filter_inertial.h"

void kalman_filter_inertial_predict(float dt, float x[3]) {
	x[0] += x[1] * dt + x[2] * dt * dt / 2.0f;
	x[1] += x[2] * dt;
}

void kalman_filter_inertial_update(float x[3], float z[2], float k[3][2], bool use[2]) {
	float y[2];
	// y = z - x
	for (int i = 0; i < 2; i++) {
		y[i] = z[i] - x[i];
	}
	// x = x + K * y
	for (int i = 0; i < 3; i++) {		// Row
		for (int j = 0; j < 2; j++) {	// Column
			if (use[j])
				x[i] += k[i][j] * y[j];
		}
	}
}
