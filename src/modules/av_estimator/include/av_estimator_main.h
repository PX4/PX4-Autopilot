/**
 * @file av_estimator_main.h
 * Main loop header file
 *
 * @author frits.kuipers <f.p.kuipers@student.utwente.nl>
 */

/**
 * @brief Mainloop of av_estimator
 * 
 * @param argc number of arguments
 * @param argv arguments
 * 
 * @return succes
 */
int av_estimator_thread_main(int argc, char *argv[]);

/**
 * @brief prints usage
 * 
 * @param reason reason message is printed
 */
static void usage(const char *reason);

/**
 * @brief Retreives current calibration parameters
 * 
 * @param av_estimator_params params struct
 * @param current_k current calib k
 * @param current_c current calib h
 */
void getCurrentCalibParam(struct av_estimator_params  attitude_params, float * current_k, float * current_c);

/**
 * @brief Applies current compensation to the magnetometer data
 * 
 * @param mu input magnetometer data, current calibration is applied to this data
 * @param current current measured
 * @param current_k current calib k values
 * @param current_c current calib c values
 * @param armed_start_time time vehicle was armed, used to delay calibration
 * @param attitude_params params file
 * @param vehicle_status_raw vehicle status, calibration only applied when vehicle is armed.
 */
void applyCurrentCompensation(Vector3f &mu, float * current, float * current_k, float * current_c, uint64_t &armed_start_time, av_estimator_params  attitude_params, vehicle_status_s vehicle_status_raw);

/* Function to use barometer to determine vz */
void use_barometer(av_estimator_params  attitude_params, uint64_t cur_baro_time, float baro_alt, float * cur_att, Vector3f &acc, Vector2f &baro_out);

/* Function to do drag offset calibration */
void velocity_offset_calibration(Vector3f veh_vel, Vector3f acc);

/* Check for changes in RC */
void		vehicle_rc_poll();