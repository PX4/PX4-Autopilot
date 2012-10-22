
/**
 * Least-squares fit of a sphere to a set of points.
 *
 * Fits a sphere to a set of points on the sphere surface.
 *
 * @param x point coordinates on the X axis
 * @param y point coordinates on the Y axis
 * @param z point coordinates on the Z axis
 * @param size number of points
 * @param max_iterations abort if maximum number of iterations have been reached. If unsure, set to 100.
 * @param delta abort if error is below delta. If unsure, set to 0 to run max_iterations times.
 * @param sphere_x coordinate of the sphere center on the X axis
 * @param sphere_y coordinate of the sphere center on the Y axis
 * @param sphere_z coordinate of the sphere center on the Z axis
 * @param sphere_radius sphere radius
 *
 * @return 0 on success, 1 on failure
 */
int sphere_fit_least_squares(const float x[], const float y[], const float z[],
    unsigned int size, unsigned int max_iterations, float delta, float *sphere_x, float *sphere_y, float *sphere_z, float *sphere_radius);