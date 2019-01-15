from typing import Tuple

# matplotlib don't use Xwindows backend (must be before pyplot import)
import matplotlib
matplotlib.use('Agg')
import numpy as np

from plotting import get_max_arg_time_value, get_estimator_check_flags
from pdf_report import create_pdf_report


def analyse_ekf(estimator_status, ekf2_innovations, sensor_preflight, check_levels,
                plot=False, output_plot_filename=None, late_start_early_ending=True):

    control_mode, innov_flags, gps_fail_flags = get_estimator_check_flags(estimator_status)

    innov_time = 1e-6 * ekf2_innovations['timestamp']
    status_time = 1e-6 * estimator_status['timestamp']

    b_finishes_in_air, b_starts_in_air, in_air_duration, in_air_transition_time, \
    on_ground_transition_time = detect_airtime(control_mode, status_time)

    if plot:
        create_pdf_report(control_mode, ekf2_innovations, estimator_status, gps_fail_flags,
                          in_air_duration, in_air_transition_time, innov_flags,
                          on_ground_transition_time, output_plot_filename, sensor_preflight,
                          status_time)

    # generate metadata for the normalised innovation consistency test levels
    # a value > 1.0 means the measurement data for that test has been rejected by the EKF
    # airspeed data
    tas_test_max = np.amax(estimator_status['tas_test_ratio'])
    # height above ground data (rangefinder)
    hagl_test_max = np.amax(estimator_status['hagl_test_ratio'])

    # calculate alignment completion times
    tilt_align_time_arg, _, tilt_align_time = get_max_arg_time_value(
        np.diff(control_mode['tilt_aligned']), status_time)

    # Do some automated analysis of the status data
    # normal index range is defined by the flight duration
    start_index = np.amin(np.where(status_time > in_air_transition_time))
    end_index = np.amax(np.where(status_time <= on_ground_transition_time))
    num_valid_values = (end_index - start_index + 1)
    # find a late/early index range from 5 sec after in_air_transtion_time to 5 sec before on-ground transition time for mag and optical flow checks to avoid false positives
    # this can be used to prevent false positives for sensors adversely affected by close proximity to the ground
    # don't do this if the log starts or finishes in air or if it is shut off by flag
    late_start_index = np.amin(np.where(status_time > (in_air_transition_time + 5.0)))\
        if (late_start_early_ending and not b_starts_in_air) else start_index
    early_end_index = np.amax(np.where(status_time <= (on_ground_transition_time - 5.0))) \
        if (late_start_early_ending and not b_finishes_in_air) else end_index
    num_valid_values_trimmed = (early_end_index - late_start_index + 1)
    # also find the start and finish indexes for the innovation data
    innov_start_index = np.amin(np.where(innov_time > in_air_transition_time))
    innov_end_index = np.amax(np.where(innov_time <= on_ground_transition_time))
    innov_num_valid_values = (innov_end_index - innov_start_index + 1)
    innov_late_start_index = np.amin(np.where(innov_time > (in_air_transition_time + 5.0))) \
        if (late_start_early_ending and not b_starts_in_air) else innov_start_index
    innov_early_end_index = np.amax(np.where(innov_time <= (on_ground_transition_time - 5.0))) \
        if (late_start_early_ending and not b_finishes_in_air) else innov_end_index
    innov_num_valid_values_trimmed = (innov_early_end_index - innov_late_start_index + 1)
    # define dictionary of test results and descriptions
    test_results = {
        'master_status': ['Pass',
                          'Master check status which can be either Pass Warning or Fail. A Fail result indicates a significant error that caused a significant reduction in vehicle navigation performance was detected. A Warning result indicates that error levels higher than normal were detected but these errors did not significantly impact navigation performance. A Pass result indicates that no amonalies were detected and no further investigation is required'],
        'mag_sensor_status': ['Pass',
                              'Magnetometer sensor check summary. A Fail result indicates a significant error that caused a significant reduction in vehicle navigation performance was detected. A Warning result indicates that error levels higher than normal were detected but these errors did not significantly impact navigation performance. A Pass result indicates that no amonalies were detected and no further investigation is required'],
        'yaw_sensor_status': ['Pass',
                              'Yaw sensor check summary. This sensor data can be sourced from the magnetometer or an external vision system. A Fail result indicates a significant error that caused a significant reduction in vehicle navigation performance was detected. A Warning result indicates that error levels higher than normal were detected but these errors did not significantly impact navigation performance. A Pass result indicates that no amonalies were detected and no further investigation is required'],
        'vel_sensor_status': ['Pass',
                              'Velocity sensor check summary. A Fail result indicates a significant error that caused a significant reduction in vehicle navigation performance was detected. A Warning result indicates that error levels higher than normal were detected but these errors did not significantly impact navigation performance. A Pass result indicates that no amonalies were detected and no further investigation is required'],
        'pos_sensor_status': ['Pass',
                              'Position sensor check summary. A Fail result indicates a significant error that caused a significant reduction in vehicle navigation performance was detected. A Warning result indicates that error levels higher than normal were detected but these errors did not significantly impact navigation performance. A Pass result indicates that no amonalies were detected and no further investigation is required'],
        'hgt_sensor_status': ['Pass',
                              'Height sensor check summary. This sensor data can be sourced from either Baro or GPS or range finder or external vision system. A Fail result indicates a significant error that caused a significant reduction in vehicle navigation performance was detected. A Warning result indicates that error levels higher than normal were detected but these errors did not significantly impact navigation performance. A Pass result indicates that no anomalies were detected and no further investigation is required'],
        'hagl_sensor_status': ['Pass',
                               'Height above ground sensor check summary. This sensor data is normally sourced from a rangefinder sensor. A Fail result indicates a significant error that caused a significant reduction in vehicle navigation performance was detected. A Warning result indicates that error levels higher than normal were detected but these errors did not significantly impact navigation performance. A Pass result indicates that no amonalies were detected and no further investigation is required'],
        'tas_sensor_status': ['Pass',
                              'Airspeed sensor check summary. A Fail result indicates a significant error that caused a significant reduction in vehicle navigation performance was detected. A Warning result indicates that error levels higher than normal were detected but these errors did not significantly impact navigation performance. A Pass result indicates that no amonalies were detected and no further investigation is required'],
        'imu_sensor_status': ['Pass',
                              'IMU sensor check summary. A Fail result indicates a significant error that caused a significant reduction in vehicle navigation performance was detected. A Warning result indicates that error levels higher than normal were detected but these errors did not significantly impact navigation performance. A Pass result indicates that no amonalies were detected and no further investigation is required'],
        'imu_vibration_check': ['Pass',
                              'IMU vibration check summary. A Fail result indicates a significant error that caused a significant reduction in vehicle navigation performance was detected. A Warning result indicates that error levels higher than normal were detected but these errors did not significantly impact navigation performance. A Pass result indicates that no amonalies were detected and no further investigation is required'],
        'imu_bias_check': ['Pass',
                              'IMU bias check summary. A Fail result indicates a significant error that caused a significant reduction in vehicle navigation performance was detected. A Warning result indicates that error levels higher than normal were detected but these errors did not significantly impact navigation performance. A Pass result indicates that no amonalies were detected and no further investigation is required'],
        'imu_output_predictor_check': ['Pass',
                              'IMU output predictor check summary. A Fail result indicates a significant error that caused a significant reduction in vehicle navigation performance was detected. A Warning result indicates that error levels higher than normal were detected but these errors did not significantly impact navigation performance. A Pass result indicates that no amonalies were detected and no further investigation is required'],
        'flow_sensor_status': ['Pass',
                               'Optical Flow sensor check summary. A Fail result indicates a significant error that caused a significant reduction in vehicle navigation performance was detected. A Warning result indicates that error levels higher than normal were detected but these errors did not significantly impact navigation performance. A Pass result indicates that no amonalies were detected and no further investigation is required'],
        'filter_fault_status': ['Pass',
                               'Internal Filter check summary. A Fail result indicates a significant error that caused a significant reduction in vehicle navigation performance was detected. A Warning result indicates that error levels higher than normal were detected but these errors did not significantly impact navigation performance. A Pass result indicates that no amonalies were detected and no further investigation is required'],
        'mag_percentage_red': [float('NaN'),
                               'The percentage of in-flight consolidated magnetic field sensor innovation consistency test values > 1.0.'],
        'mag_percentage_amber': [float('NaN'),
                                 'The percentage of in-flight consolidated magnetic field sensor innovation consistency test values > 0.5.'],
        'magx_fail_percentage': [float('NaN'),
                                 'The percentage of in-flight recorded failure events for the X-axis magnetic field sensor innovation consistency test.'],
        'magy_fail_percentage': [float('NaN'),
                                 'The percentage of in-flight recorded failure events for the Y-axis magnetic field sensor innovation consistency test.'],
        'magz_fail_percentage': [float('NaN'),
                                 'The percentage of in-flight recorded failure events for the Z-axis magnetic field sensor innovation consistency test.'],
        'yaw_fail_percentage': [float('NaN'),
                                'The percentage of in-flight recorded failure events for the yaw sensor innovation consistency test.'],
        'mag_test_max': [float('NaN'),
                         'The maximum in-flight value of the magnetic field sensor innovation consistency test ratio.'],
        'mag_test_mean': [float('NaN'),
                          'The mean in-flight value of the magnetic field sensor innovation consistency test ratio.'],
        'vel_percentage_red': [float('NaN'),
                               'The percentage of in-flight velocity sensor consolidated innovation consistency test values > 1.0.'],
        'vel_percentage_amber': [float('NaN'),
                                 'The percentage of in-flight velocity sensor consolidated innovation consistency test values > 0.5.'],
        'vel_fail_percentage': [float('NaN'),
                                'The percentage of in-flight recorded failure events for the velocity sensor consolidated innovation consistency test.'],
        'vel_test_max': [float('NaN'),
                         'The maximum in-flight value of the velocity sensor consolidated innovation consistency test ratio.'],
        'vel_test_mean': [float('NaN'),
                          'The mean in-flight value of the velocity sensor consolidated innovation consistency test ratio.'],
        'pos_percentage_red': [float('NaN'),
                               'The percentage of in-flight position sensor consolidated innovation consistency test values > 1.0.'],
        'pos_percentage_amber': [float('NaN'),
                                 'The percentage of in-flight position sensor consolidated innovation consistency test values > 0.5.'],
        'pos_fail_percentage': [float('NaN'),
                                'The percentage of in-flight recorded failure events for the velocity sensor consolidated innovation consistency test.'],
        'pos_test_max': [float('NaN'),
                         'The maximum in-flight value of the position sensor consolidated innovation consistency test ratio.'],
        'pos_test_mean': [float('NaN'),
                          'The mean in-flight value of the position sensor consolidated innovation consistency test ratio.'],
        'hgt_percentage_red': [float('NaN'),
                               'The percentage of in-flight height sensor innovation consistency test values > 1.0.'],
        'hgt_percentage_amber': [float('NaN'),
                                 'The percentage of in-flight height sensor innovation consistency test values > 0.5.'],
        'hgt_fail_percentage': [float('NaN'),
                                'The percentage of in-flight recorded failure events for the height sensor innovation consistency test.'],
        'hgt_test_max': [float('NaN'),
                         'The maximum in-flight value of the height sensor innovation consistency test ratio.'],
        'hgt_test_mean': [float('NaN'),
                          'The mean in-flight value of the height sensor innovation consistency test ratio.'],
        'tas_percentage_red': [float('NaN'),
                               'The percentage of in-flight airspeed sensor innovation consistency test values > 1.0.'],
        'tas_percentage_amber': [float('NaN'),
                                 'The percentage of in-flight airspeed sensor innovation consistency test values > 0.5.'],
        'tas_fail_percentage': [float('NaN'),
                                'The percentage of in-flight recorded failure events for the airspeed sensor innovation consistency test.'],
        'tas_test_max': [float('NaN'),
                         'The maximum in-flight value of the airspeed sensor innovation consistency test ratio.'],
        'tas_test_mean': [float('NaN'),
                          'The mean in-flight value of the airspeed sensor innovation consistency test ratio.'],
        'hagl_percentage_red': [float('NaN'),
                                'The percentage of in-flight height above ground sensor innovation consistency test values > 1.0.'],
        'hagl_percentage_amber': [float('NaN'),
                                  'The percentage of in-flight height above ground sensor innovation consistency test values > 0.5.'],
        'hagl_fail_percentage': [float('NaN'),
                                 'The percentage of in-flight recorded failure events for the height above ground sensor innovation consistency test.'],
        'hagl_test_max': [float('NaN'),
                          'The maximum in-flight value of the height above ground sensor innovation consistency test ratio.'],
        'hagl_test_mean': [float('NaN'),
                           'The mean in-flight value of the height above ground sensor innovation consistency test ratio.'],
        'ofx_fail_percentage': [float('NaN'),
                                'The percentage of in-flight recorded failure events for the optical flow sensor X-axis innovation consistency test.'],
        'ofy_fail_percentage': [float('NaN'),
                                'The percentage of in-flight recorded failure events for the optical flow sensor Y-axis innovation consistency test.'],
        'filter_faults_max': [float('NaN'),
                              'Largest recorded value of the filter internal fault bitmask. Should always be zero.'],
        'imu_coning_peak': [float('NaN'), 'Peak in-flight value of the IMU delta angle coning vibration metric (rad)'],
        'imu_coning_mean': [float('NaN'), 'Mean in-flight value of the IMU delta angle coning vibration metric (rad)'],
        'imu_hfdang_peak': [float('NaN'),
                            'Peak in-flight value of the IMU delta angle high frequency vibration metric (rad)'],
        'imu_hfdang_mean': [float('NaN'),
                            'Mean in-flight value of the IMU delta angle high frequency vibration metric (rad)'],
        'imu_hfdvel_peak': [float('NaN'),
                            'Peak in-flight value of the IMU delta velocity high frequency vibration metric (m/s)'],
        'imu_hfdvel_mean': [float('NaN'),
                            'Mean in-flight value of the IMU delta velocity high frequency vibration metric (m/s)'],
        'output_obs_ang_err_median': [float('NaN'),
                                      'Median in-flight value of the output observer angular error (rad)'],
        'output_obs_vel_err_median': [float('NaN'),
                                      'Median in-flight value of the output observer velocity error (m/s)'],
        'output_obs_pos_err_median': [float('NaN'), 'Median in-flight value of the output observer position error (m)'],
        'imu_dang_bias_median': [float('NaN'), 'Median in-flight value of the delta angle bias vector length (rad)'],
        'imu_dvel_bias_median': [float('NaN'), 'Median in-flight value of the delta velocity bias vector length (m/s)'],
        'tilt_align_time': [float('NaN'),
                            'The time in seconds measured from startup that the EKF completed the tilt alignment. A nan value indicates that the alignment had completed before logging started or alignment did not complete.'],
        'yaw_align_time': [float('NaN'),
                           'The time in seconds measured from startup that the EKF completed the yaw alignment.'],
        'in_air_transition_time': [round(in_air_transition_time, 1),
                                   'The time in seconds measured from startup that the EKF transtioned into in-air mode. Set to a nan if a transition event is not detected.'],
        'on_ground_transition_time': [round(on_ground_transition_time, 1),
                                      'The time in seconds measured from startup  that the EKF transitioned out of in-air mode. Set to a nan if a transition event is not detected.'],
    }
    # generate test metadata
    # reduction of innovation message data
    if (innov_early_end_index > (innov_late_start_index + 50)):
        # Output Observer Tracking Errors
        test_results['output_obs_ang_err_median'][0] = np.median(
            ekf2_innovations['output_tracking_error[0]'][innov_late_start_index:innov_early_end_index + 1])
        test_results['output_obs_vel_err_median'][0] = np.median(
            ekf2_innovations['output_tracking_error[1]'][innov_late_start_index:innov_early_end_index + 1])
        test_results['output_obs_pos_err_median'][0] = np.median(
            ekf2_innovations['output_tracking_error[2]'][innov_late_start_index:innov_early_end_index + 1])
    # reduction of status message data
    if (early_end_index > (late_start_index + 50)):
        # IMU vibration checks
        temp = np.amax(estimator_status['vibe[0]'][late_start_index:early_end_index])
        if (temp > 0.0):
            test_results['imu_coning_peak'][0] = temp
            test_results['imu_coning_mean'][0] = np.mean(estimator_status['vibe[0]'][late_start_index:early_end_index + 1])
        temp = np.amax(estimator_status['vibe[1]'][late_start_index:early_end_index])
        if (temp > 0.0):
            test_results['imu_hfdang_peak'][0] = temp
            test_results['imu_hfdang_mean'][0] = np.mean(estimator_status['vibe[1]'][late_start_index:early_end_index + 1])
        temp = np.amax(estimator_status['vibe[2]'][late_start_index:early_end_index])
        if (temp > 0.0):
            test_results['imu_hfdvel_peak'][0] = temp
            test_results['imu_hfdvel_mean'][0] = np.mean(estimator_status['vibe[2]'][late_start_index:early_end_index + 1])

        # Magnetometer Sensor Checks
        if (np.amax(control_mode['yaw_aligned']) > 0.5):
            mag_num_red = (estimator_status['mag_test_ratio'][start_index:end_index + 1] > 1.0).sum()
            mag_num_amber = (estimator_status['mag_test_ratio'][start_index:end_index + 1] > 0.5).sum() - mag_num_red
            test_results['mag_percentage_red'][0] = 100.0 * mag_num_red / num_valid_values_trimmed
            test_results['mag_percentage_amber'][0] = 100.0 * mag_num_amber / num_valid_values_trimmed
            test_results['mag_test_max'][0] = np.amax(
                estimator_status['mag_test_ratio'][late_start_index:early_end_index + 1])
            test_results['mag_test_mean'][0] = np.mean(estimator_status['mag_test_ratio'][start_index:end_index])
            test_results['magx_fail_percentage'][0] = 100.0 * (
                    magx_innov_fail[late_start_index:early_end_index + 1] > 0.5).sum() / num_valid_values_trimmed
            test_results['magy_fail_percentage'][0] = 100.0 * (
                    magy_innov_fail[late_start_index:early_end_index + 1] > 0.5).sum() / num_valid_values_trimmed
            test_results['magz_fail_percentage'][0] = 100.0 * (
                    magz_innov_fail[late_start_index:early_end_index + 1] > 0.5).sum() / num_valid_values_trimmed
            test_results['yaw_fail_percentage'][0] = 100.0 * (
                    yaw_innov_fail[late_start_index:early_end_index + 1] > 0.5).sum() / num_valid_values_trimmed

        # Velocity Sensor Checks
        if (np.amax(control_mode['using_gps']) > 0.5):
            vel_num_red = (estimator_status['vel_test_ratio'][start_index:end_index + 1] > 1.0).sum()
            vel_num_amber = (estimator_status['vel_test_ratio'][start_index:end_index + 1] > 0.5).sum() - vel_num_red
            test_results['vel_percentage_red'][0] = 100.0 * vel_num_red / num_valid_values
            test_results['vel_percentage_amber'][0] = 100.0 * vel_num_amber / num_valid_values
            test_results['vel_test_max'][0] = np.amax(estimator_status['vel_test_ratio'][start_index:end_index + 1])
            test_results['vel_test_mean'][0] = np.mean(estimator_status['vel_test_ratio'][start_index:end_index + 1])
            test_results['vel_fail_percentage'][0] = 100.0 * (
                    vel_innov_fail[start_index:end_index + 1] > 0.5).sum() / num_valid_values

        # Position Sensor Checks
        if ((np.amax(control_mode['using_gps']) > 0.5) or (np.amax(using_evpos) > 0.5)):
            pos_num_red = (estimator_status['pos_test_ratio'][start_index:end_index + 1] > 1.0).sum()
            pos_num_amber = (estimator_status['pos_test_ratio'][start_index:end_index + 1] > 0.5).sum() - pos_num_red
            test_results['pos_percentage_red'][0] = 100.0 * pos_num_red / num_valid_values
            test_results['pos_percentage_amber'][0] = 100.0 * pos_num_amber / num_valid_values
            test_results['pos_test_max'][0] = np.amax(estimator_status['pos_test_ratio'][start_index:end_index + 1])
            test_results['pos_test_mean'][0] = np.mean(estimator_status['pos_test_ratio'][start_index:end_index + 1])
            test_results['pos_fail_percentage'][0] = 100.0 * (
                    posh_innov_fail[start_index:end_index + 1] > 0.5).sum() / num_valid_values

        # Height Sensor Checks
        hgt_num_red = (estimator_status['hgt_test_ratio'][late_start_index:early_end_index + 1] > 1.0).sum()
        hgt_num_amber = (estimator_status['hgt_test_ratio'][late_start_index:early_end_index + 1] > 0.5).sum() - hgt_num_red
        test_results['hgt_percentage_red'][0] = 100.0 * hgt_num_red / num_valid_values_trimmed
        test_results['hgt_percentage_amber'][0] = 100.0 * hgt_num_amber / num_valid_values_trimmed
        test_results['hgt_test_max'][0] = np.amax(estimator_status['hgt_test_ratio'][late_start_index:early_end_index + 1])
        test_results['hgt_test_mean'][0] = np.mean(estimator_status['hgt_test_ratio'][late_start_index:early_end_index + 1])
        test_results['hgt_fail_percentage'][0] = 100.0 * (
                posv_innov_fail[late_start_index:early_end_index + 1] > 0.5).sum() / num_valid_values_trimmed

        # Airspeed Sensor Checks
        if (tas_test_max > 0.0):
            tas_num_red = (estimator_status['tas_test_ratio'][start_index:end_index + 1] > 1.0).sum()
            tas_num_amber = (estimator_status['tas_test_ratio'][start_index:end_index + 1] > 0.5).sum() - tas_num_red
            test_results['tas_percentage_red'][0] = 100.0 * tas_num_red / num_valid_values
            test_results['tas_percentage_amber'][0] = 100.0 * tas_num_amber / num_valid_values
            test_results['tas_test_max'][0] = np.amax(estimator_status['tas_test_ratio'][start_index:end_index + 1])
            test_results['tas_test_mean'][0] = np.mean(estimator_status['tas_test_ratio'][start_index:end_index + 1])
            test_results['tas_fail_percentage'][0] = 100.0 * (
                    tas_innov_fail[start_index:end_index + 1] > 0.5).sum() / num_valid_values

        # HAGL Sensor Checks
        if (hagl_test_max > 0.0):
            hagl_num_red = (estimator_status['hagl_test_ratio'][start_index:end_index + 1] > 1.0).sum()
            hagl_num_amber = (estimator_status['hagl_test_ratio'][start_index:end_index + 1] > 0.5).sum() - hagl_num_red
            test_results['hagl_percentage_red'][0] = 100.0 * hagl_num_red / num_valid_values
            test_results['hagl_percentage_amber'][0] = 100.0 * hagl_num_amber / num_valid_values
            test_results['hagl_test_max'][0] = np.amax(estimator_status['hagl_test_ratio'][start_index:end_index + 1])
            test_results['hagl_test_mean'][0] = np.mean(estimator_status['hagl_test_ratio'][start_index:end_index + 1])
            test_results['hagl_fail_percentage'][0] = 100.0 * (
                    hagl_innov_fail[start_index:end_index + 1] > 0.5).sum() / num_valid_values

        # optical flow sensor checks
        if (np.amax(control_mode['using_optflow']) > 0.5):
            test_results['ofx_fail_percentage'][0] = 100.0 * (
                    ofx_innov_fail[late_start_index:early_end_index + 1] > 0.5).sum() / num_valid_values_trimmed
            test_results['ofy_fail_percentage'][0] = 100.0 * (
                    ofy_innov_fail[late_start_index:early_end_index + 1] > 0.5).sum() / num_valid_values_trimmed

        # IMU bias checks
        test_results['imu_dang_bias_median'][0] = (np.median(estimator_status['states[10]']) ** 2 + np.median(
            estimator_status['states[11]']) ** 2 + np.median(estimator_status['states[12]']) ** 2) ** 0.5
        test_results['imu_dvel_bias_median'][0] = (np.median(estimator_status['states[13]']) ** 2 + np.median(
            estimator_status['states[14]']) ** 2 + np.median(estimator_status['states[15]']) ** 2) ** 0.5
    # Check for internal filter nummerical faults
    test_results['filter_faults_max'][0] = np.amax(estimator_status['filter_fault_flags'])
    # TODO - process the following bitmask's when they have been properly documented in the uORB topic
    # estimator_status['health_flags']
    # estimator_status['timeout_flags']
    # calculate a master status - Fail, Warning, Pass

    # check test results against levels to provide a master status
    # check for warnings
    if (test_results.get('mag_percentage_amber')[0] > check_levels.get('mag_amber_warn_pct')):
        test_results['master_status'][0] = 'Warning'
        test_results['mag_sensor_status'][0] = 'Warning'
    if (test_results.get('vel_percentage_amber')[0] > check_levels.get('vel_amber_warn_pct')):
        test_results['master_status'][0] = 'Warning'
        test_results['vel_sensor_status'][0] = 'Warning'
    if (test_results.get('pos_percentage_amber')[0] > check_levels.get('pos_amber_warn_pct')):
        test_results['master_status'][0] = 'Warning'
        test_results['pos_sensor_status'][0] = 'Warning'
    if (test_results.get('hgt_percentage_amber')[0] > check_levels.get('hgt_amber_warn_pct')):
        test_results['master_status'][0] = 'Warning'
        test_results['hgt_sensor_status'][0] = 'Warning'
    if (test_results.get('hagl_percentage_amber')[0] > check_levels.get('hagl_amber_warn_pct')):
        test_results['master_status'][0] = 'Warning'
        test_results['hagl_sensor_status'][0] = 'Warning'
    if (test_results.get('tas_percentage_amber')[0] > check_levels.get('tas_amber_warn_pct')):
        test_results['master_status'][0] = 'Warning'
        test_results['tas_sensor_status'][0] = 'Warning'
    # check for IMU sensor warnings
    if ((test_results.get('imu_coning_peak')[0] > check_levels.get('imu_coning_peak_warn')) or
            (test_results.get('imu_coning_mean')[0] > check_levels.get('imu_coning_mean_warn'))):
        test_results['master_status'][0] = 'Warning'
        test_results['imu_sensor_status'][0] = 'Warning'
        test_results['imu_vibration_check'][0] = 'Warning'
        print('IMU gyro coning check warning.')
    if ((test_results.get('imu_hfdang_peak')[0] > check_levels.get('imu_hfdang_peak_warn')) or
            (test_results.get('imu_hfdang_mean')[0] > check_levels.get('imu_hfdang_mean_warn'))):
        test_results['master_status'][0] = 'Warning'
        test_results['imu_sensor_status'][0] = 'Warning'
        test_results['imu_vibration_check'][0] = 'Warning'
        print('IMU gyro vibration check warning.')
    if ((test_results.get('imu_hfdvel_peak')[0] > check_levels.get('imu_hfdvel_peak_warn')) or
            (test_results.get('imu_hfdvel_mean')[0] > check_levels.get('imu_hfdvel_mean_warn'))):
        test_results['master_status'][0] = 'Warning'
        test_results['imu_sensor_status'][0] = 'Warning'
        test_results['imu_vibration_check'][0] = 'Warning'
        print('IMU accel vibration check warning.')
    if ((test_results.get('imu_dang_bias_median')[0] > check_levels.get('imu_dang_bias_median_warn')) or
            (test_results.get('imu_dvel_bias_median')[0] > check_levels.get('imu_dvel_bias_median_warn'))):
        test_results['master_status'][0] = 'Warning'
        test_results['imu_sensor_status'][0] = 'Warning'
        test_results['imu_bias_check'][0] = 'Warning'
        print('IMU bias check warning.')
    if ((test_results.get('output_obs_ang_err_median')[0] > check_levels.get('obs_ang_err_median_warn')) or
            (test_results.get('output_obs_vel_err_median')[0] > check_levels.get('obs_vel_err_median_warn')) or
            (test_results.get('output_obs_pos_err_median')[0] > check_levels.get('obs_pos_err_median_warn'))):
        test_results['master_status'][0] = 'Warning'
        test_results['imu_sensor_status'][0] = 'Warning'
        test_results['imu_output_predictor_check'][0] = 'Warning'
        print('IMU output predictor check warning.')
    # check for failures
    if ((test_results.get('magx_fail_percentage')[0] > check_levels.get('mag_fail_pct')) or
            (test_results.get('magy_fail_percentage')[0] > check_levels.get('mag_fail_pct')) or
            (test_results.get('magz_fail_percentage')[0] > check_levels.get('mag_fail_pct')) or
            (test_results.get('mag_percentage_amber')[0] > check_levels.get('mag_amber_fail_pct'))):
        test_results['master_status'][0] = 'Fail'
        test_results['mag_sensor_status'][0] = 'Fail'
        print('Magnetometer sensor check failure.')
    if (test_results.get('yaw_fail_percentage')[0] > check_levels.get('yaw_fail_pct')):
        test_results['master_status'][0] = 'Fail'
        test_results['yaw_sensor_status'][0] = 'Fail'
        print('Yaw sensor check failure.')
    if ((test_results.get('vel_fail_percentage')[0] > check_levels.get('vel_fail_pct')) or
            (test_results.get('vel_percentage_amber')[0] > check_levels.get('vel_amber_fail_pct'))):
        test_results['master_status'][0] = 'Fail'
        test_results['vel_sensor_status'][0] = 'Fail'
        print('Velocity sensor check failure.')
    if ((test_results.get('pos_fail_percentage')[0] > check_levels.get('pos_fail_pct')) or
            (test_results.get('pos_percentage_amber')[0] > check_levels.get('pos_amber_fail_pct'))):
        test_results['master_status'][0] = 'Fail'
        test_results['pos_sensor_status'][0] = 'Fail'
        print('Position sensor check failure.')
    if ((test_results.get('hgt_fail_percentage')[0] > check_levels.get('hgt_fail_pct')) or
            (test_results.get('hgt_percentage_amber')[0] > check_levels.get('hgt_amber_fail_pct'))):
        test_results['master_status'][0] = 'Fail'
        test_results['hgt_sensor_status'][0] = 'Fail'
        print('Height sensor check failure.')
    if ((test_results.get('tas_fail_percentage')[0] > check_levels.get('tas_fail_pct')) or
            (test_results.get('tas_percentage_amber')[0] > check_levels.get('tas_amber_fail_pct'))):
        test_results['master_status'][0] = 'Fail'
        test_results['tas_sensor_status'][0] = 'Fail'
        print('Airspeed sensor check failure.')
    if ((test_results.get('hagl_fail_percentage')[0] > check_levels.get('hagl_fail_pct')) or
            (test_results.get('hagl_percentage_amber')[0] > check_levels.get('hagl_amber_fail_pct'))):
        test_results['master_status'][0] = 'Fail'
        test_results['hagl_sensor_status'][0] = 'Fail'
        print('Height above ground sensor check failure.')
    if ((test_results.get('ofx_fail_percentage')[0] > check_levels.get('flow_fail_pct')) or
            (test_results.get('ofy_fail_percentage')[0] > check_levels.get('flow_fail_pct'))):
        test_results['master_status'][0] = 'Fail'
        test_results['flow_sensor_status'][0] = 'Fail'
        print('Optical flow sensor check failure.')
    if (test_results.get('filter_faults_max')[0] > 0):
        test_results['master_status'][0] = 'Fail'
        test_results['filter_fault_status'][0] = 'Fail'

    return test_results



def detect_airtime(control_mode, status_time):
    # define flags for starting and finishing in air
    b_starts_in_air = False
    b_finishes_in_air = False
    # calculate in-air transition time
    if (np.amin(control_mode['airborne']) < 0.5) and (np.amax(control_mode['airborne']) > 0.5):
        in_air_transtion_time_arg = np.argmax(np.diff(control_mode['airborne']))
        in_air_transition_time = status_time[in_air_transtion_time_arg]
    elif (np.amax(control_mode['airborne']) > 0.5):
        in_air_transition_time = np.amin(status_time)
        print('log starts while in-air at ' + str(round(in_air_transition_time, 1)) + ' sec')
        b_starts_in_air = True
    else:
        in_air_transition_time = float('NaN')
        print('always on ground')
    # calculate on-ground transition time
    if (np.amin(np.diff(control_mode['airborne'])) < 0.0):
        on_ground_transition_time_arg = np.argmin(np.diff(control_mode['airborne']))
        on_ground_transition_time = status_time[on_ground_transition_time_arg]
    elif (np.amax(control_mode['airborne']) > 0.5):
        on_ground_transition_time = np.amax(status_time)
        print('log finishes while in-air at ' + str(round(on_ground_transition_time, 1)) + ' sec')
        b_finishes_in_air = True
    else:
        on_ground_transition_time = float('NaN')
        print('always on ground')
    if (np.amax(np.diff(control_mode['airborne'])) > 0.5) and (np.amin(np.diff(control_mode['airborne'])) < -0.5):
        if ((on_ground_transition_time - in_air_transition_time) > 0.0):
            in_air_duration = on_ground_transition_time - in_air_transition_time
        else:
            in_air_duration = float('NaN')
    else:
        in_air_duration = float('NaN')
    return b_finishes_in_air, b_starts_in_air, in_air_duration, in_air_transition_time, on_ground_transition_time




