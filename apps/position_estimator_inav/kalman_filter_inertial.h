#include <stdbool.h>

void kalman_filter_inertial_predict(float dt, float x[3]);

void kalman_filter_inertial_update(float x[3], float z[2], float k[3][2], bool use[2]);
