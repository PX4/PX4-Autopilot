#! /usr/bin/env python3
"""
function collection for calculation ecl ekf metrics.
"""

from typing import Dict, List, Tuple, Callable

from pyulog import ULog
import numpy as np

from analysis.detectors import InAirDetector

def calculate_ecl_ekf_metrics(
        ulog: ULog, estimator_status_flags: Dict[str, float], innov_fail_checks: List[str],
        sensor_checks: List[str], in_air: InAirDetector, in_air_no_ground_effects: InAirDetector,
        multi_instance: int = 0, red_thresh: float = 1.0, amb_thresh: float = 0.5) -> Tuple[dict, dict, dict, dict]:

    sensor_metrics = calculate_sensor_metrics(
        ulog, sensor_checks, in_air, in_air_no_ground_effects,
        red_thresh=red_thresh, amb_thresh=amb_thresh)

    innov_fail_metrics = calculate_innov_fail_metrics(
        estimator_status_flags, innov_fail_checks, in_air, in_air_no_ground_effects)

    imu_metrics = calculate_imu_metrics(ulog, multi_instance, in_air_no_ground_effects)

    estimator_status_data = ulog.get_dataset('estimator_status', multi_instance).data

    # Check for internal filter nummerical faults
    ekf_metrics = {'filter_faults_max': np.amax(estimator_status_data['filter_fault_flags'])}
    # TODO - process these bitmask's when they have been properly documented in the uORB topic
    # estimator_status['health_flags']
    # estimator_status['timeout_flags']

    # combine the metrics
    combined_metrics = dict()
    combined_metrics.update(imu_metrics)
    combined_metrics.update(sensor_metrics)
    combined_metrics.update(innov_fail_metrics)
    combined_metrics.update(ekf_metrics)

    return combined_metrics


def calculate_sensor_metrics(
        ulog: ULog, sensor_checks: List[str], in_air: InAirDetector,
        in_air_no_ground_effects: InAirDetector, multi_instance: int = 0,
        red_thresh: float = 1.0, amb_thresh: float = 0.5) -> Dict[str, float]:

    estimator_status_data = ulog.get_dataset('estimator_status', multi_instance).data

    sensor_metrics = dict()

    # calculates peak, mean, percentage above 0.5 std, and percentage above std metrics for
    # estimator status variables
    for signal, result_id in [('hgt_test_ratio', 'hgt'),
                              ('mag_test_ratio', 'mag'),
                              ('vel_test_ratio', 'vel'),
                              ('pos_test_ratio', 'pos'),
                              ('tas_test_ratio', 'tas'),
                              ('hagl_test_ratio', 'hagl')]:

        # only run sensor checks, if they apply.
        if result_id in sensor_checks:

            if result_id == 'mag' or result_id == 'hgt':
                in_air_detector = in_air_no_ground_effects
            else:
                in_air_detector = in_air

            # the percentage of samples above / below std dev
            sensor_metrics['{:s}_percentage_red'.format(result_id)] = calculate_stat_from_signal(
                estimator_status_data, 'estimator_status', signal, in_air_detector,
                lambda x: 100.0 * np.mean(x > red_thresh))
            sensor_metrics['{:s}_percentage_amber'.format(result_id)] = calculate_stat_from_signal(
                estimator_status_data, 'estimator_status', signal, in_air_detector,
                lambda x: 100.0 * np.mean(x > amb_thresh)) - \
                    sensor_metrics['{:s}_percentage_red'.format(result_id)]

            # the peak and mean ratio of samples above / below std dev
            peak = calculate_stat_from_signal(
                estimator_status_data, 'estimator_status', signal, in_air_detector, np.amax)
            if peak > 0.0:
                sensor_metrics['{:s}_test_max'.format(result_id)] = peak
                sensor_metrics['{:s}_test_mean'.format(result_id)] = calculate_stat_from_signal(
                    estimator_status_data, 'estimator_status', signal,
                    in_air_detector, np.mean)

    return sensor_metrics


def calculate_innov_fail_metrics(
        estimator_status_flags: dict, innov_fail_checks: List[str], in_air: InAirDetector,
        in_air_no_ground_effects: InAirDetector) -> dict:
    """
    :param estimator_status_flags:
    :param innov_fail_checks:
    :param in_air:
    :param in_air_no_ground_effects:
    :return:
    """

    innov_fail_metrics = dict()

    # calculate innovation check fail metrics
    for signal_id, signal, result in [('posv', 'reject_ver_pos', 'hgt_fail_percentage'),
                                      ('magx', 'reject_mag_x', 'magx_fail_percentage'),
                                      ('magy', 'reject_mag_y', 'magy_fail_percentage'),
                                      ('magz', 'reject_mag_z', 'magz_fail_percentage'),
                                      ('yaw', 'reject_yaw', 'yaw_fail_percentage'),
                                      ('velh', 'reject_hor_vel', 'vel_fail_percentage'),
                                      ('velv', 'reject_ver_vel', 'vel_fail_percentage'),
                                      ('posh', 'reject_hor_pos', 'pos_fail_percentage'),
                                      ('tas', 'reject_airspeed', 'tas_fail_percentage'),
                                      ('hagl', 'reject_hagl', 'hagl_fail_percentage'),
                                      ('ofx', 'reject_optflow_x', 'ofx_fail_percentage'),
                                      ('ofy', 'reject_optflow_y', 'ofy_fail_percentage')]:

        # only run innov fail checks, if they apply.
        if signal_id in innov_fail_checks:

            if signal_id.startswith('mag') or signal_id == 'yaw' or signal_id == 'posv' or \
                signal_id.startswith('of'):
                in_air_detector = in_air_no_ground_effects
            else:
                in_air_detector = in_air

            innov_fail_metrics[result] = calculate_stat_from_signal(
                estimator_status_flags, 'estimator_status_flags', signal, in_air_detector,
                lambda x: 100.0 * np.mean(x > 0.5))

    return innov_fail_metrics


def calculate_imu_metrics(ulog: ULog, multi_instance, in_air_no_ground_effects: InAirDetector) -> dict:

    estimator_status_data = ulog.get_dataset('estimator_status', multi_instance).data

    imu_metrics = dict()

    # calculates the median of the output tracking error ekf innovations
    for signal, result in [('output_tracking_error[0]', 'output_obs_ang_err_median'),
                           ('output_tracking_error[1]', 'output_obs_vel_err_median'),
                           ('output_tracking_error[2]', 'output_obs_pos_err_median')]:
        imu_metrics[result] = calculate_stat_from_signal(
            estimator_status_data, 'estimator_status', signal, in_air_no_ground_effects, np.median)


    # calculates peak and mean for IMU vibration checks
    for imu_status_instance in range(4):
        try:
            vehicle_imu_status_data = ulog.get_dataset('vehicle_imu_status', imu_status_instance).data

            if vehicle_imu_status_data['accel_device_id'][0] == estimator_status_data['accel_device_id'][0]:

                for signal, result in [('gyro_coning_vibration', 'imu_coning'),
                                       ('gyro_vibration_metric', 'imu_hfgyro'),
                                       ('accel_vibration_metric', 'imu_hfaccel')]:

                    peak = calculate_stat_from_signal(vehicle_imu_status_data, 'vehicle_imu_status', signal, in_air_no_ground_effects, np.amax)

                    if peak > 0.0:
                        imu_metrics['{:s}_peak'.format(result)] = peak
                        imu_metrics['{:s}_mean'.format(result)] = calculate_stat_from_signal(vehicle_imu_status_data, 'vehicle_imu_status', signal, in_air_no_ground_effects, np.mean)

        except:
            pass


    # IMU bias checks
    estimator_states_data = ulog.get_dataset('estimator_states', multi_instance).data

    imu_metrics['imu_dang_bias_median'] = np.sqrt(np.sum([np.square(calculate_stat_from_signal(
        estimator_states_data, 'estimator_states', signal, in_air_no_ground_effects, np.median))
        for signal in ['states[10]', 'states[11]', 'states[12]']]))
    imu_metrics['imu_dvel_bias_median'] = np.sqrt(np.sum([np.square(calculate_stat_from_signal(
        estimator_states_data, 'estimator_states', signal, in_air_no_ground_effects, np.median))
        for signal in ['states[13]', 'states[14]', 'states[15]']]))

    return imu_metrics


def calculate_stat_from_signal(
        data: Dict[str, np.ndarray], dataset: str, variable: str,
        in_air_det: InAirDetector, stat_function: Callable) -> float:
    """
    :param data:
    :param variable:
    :param in_air_detector:
    :return:
    """

    return stat_function(data[variable][in_air_det.get_airtime(dataset)])
