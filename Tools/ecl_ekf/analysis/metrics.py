#! /usr/bin/env python3
"""
function collection for calculation ecl ekf metrics.
"""

from typing import Dict, List, Tuple, Callable

from pyulog import ULog
import numpy as np

from analysis.detectors import InAirDetector

def calculate_ecl_ekf_metrics(
        ulog: ULog, innov_flags: Dict[str, float], innov_fail_checks: List[str],
        sensor_checks: List[str], in_air: InAirDetector, in_air_no_ground_effects: InAirDetector,
        red_thresh: float = 1.0, amb_thresh: float = 0.5) -> Tuple[dict, dict, dict, dict]:

    sensor_metrics = calculate_sensor_metrics(
        ulog, sensor_checks, in_air, in_air_no_ground_effects,
        red_thresh=red_thresh, amb_thresh=amb_thresh)

    innov_fail_metrics = calculate_innov_fail_metrics(
        innov_flags, innov_fail_checks, in_air, in_air_no_ground_effects)

    imu_metrics = calculate_imu_metrics(ulog, in_air_no_ground_effects)

    estimator_status_data = ulog.get_dataset('estimator_status').data

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
        in_air_no_ground_effects: InAirDetector, red_thresh: float = 1.0,
        amb_thresh: float = 0.5) -> dict:

    estimator_status_data = ulog.get_dataset('estimator_status').data

    sensor_metrics = dict()

    # calculates peak, mean, percentage above 0.5 std, and percentage above std metrics for
    # estimator status variables
    for signal, result in [('{:s}_test_ratio'.format(check), check) for check in sensor_checks]:

        if signal.startswith('mag') or signal.startswith('hgt'):
            in_air_detector = in_air_no_ground_effects
        else:
            in_air_detector = in_air

        # the percentage of samples above / below std dev
        sensor_metrics['{:s}_percentage_red'.format(result)] = calculate_stat_from_signal(
            estimator_status_data, 'estimator_status', signal, in_air_detector,
            lambda x: 100.0 * np.mean(x > red_thresh))
        sensor_metrics['{:s}_percentage_amber'.format(result)] = calculate_stat_from_signal(
            estimator_status_data, 'estimator_status', signal, in_air_detector,
            lambda x: 100.0 * np.mean(x > amb_thresh))

        # the percentage of samples above / below std dev
        peak = calculate_stat_from_signal(
            estimator_status_data, 'estimator_status', signal, in_air, np.amax)
        if peak > 0.0:
            sensor_metrics['{:s}_test_max'.format(result)] = peak
            sensor_metrics['{:s}_test_mean'.format(result)] = calculate_stat_from_signal(
                estimator_status_data, 'estimator_status', signal,
                in_air_no_ground_effects, np.mean)

    return sensor_metrics


def calculate_innov_fail_metrics(
        innov_flags: dict, innov_fail_checks: List[str], in_air: InAirDetector,
        in_air_no_ground_effects: InAirDetector) -> dict:

    innov_fail_metrics = dict()

    # calculate innovation check fail metrics
    for signal, result in [('{:s}_innov_fail'.format(check), '{:s}_fail_percentage'.format(check))
                           for check in innov_fail_checks]:

        if signal.startswith('posv'):
            result = 'hgt{:s}'.format(result[4:])
        elif signal.startswith('posh'):
            result = 'pos{:s}'.format(result[4:])

        if signal.startswith('mag') or signal.startswith('posv') or signal.startswith('ox'):
            in_air_detector = in_air_no_ground_effects
        else:
            in_air_detector = in_air

        innov_fail_metrics[result] = calculate_stat_from_signal(
            innov_flags, 'estimator_status', signal, in_air_detector,
            lambda x: 100.0 * np.mean(x > 0.5))

    return innov_fail_metrics


def calculate_imu_metrics(
        ulog: ULog, in_air_no_ground_effects: InAirDetector) -> dict:

    ekf2_innovation_data = ulog.get_dataset('ekf2_innovations').data

    estimator_status_data = ulog.get_dataset('estimator_status').data

    imu_metrics = dict()

    # calculates the median of the output tracking error ekf innovations
    for signal, result in [('output_tracking_error[0]', 'output_obs_ang_err_median'),
                           ('output_tracking_error[1]', 'output_obs_vel_err_median'),
                           ('output_tracking_error[2]', 'output_obs_pos_err_median')]:
        imu_metrics[result] = calculate_stat_from_signal(
            ekf2_innovation_data, 'ekf2_innovations', signal, in_air_no_ground_effects, np.median)

    # calculates peak and mean for IMU vibration checks
    for signal, result in [('vibe[0]', 'imu_coning'),
                           ('vibe[1]', 'imu_hfdang'),
                           ('vibe[2]', 'imu_hfdvel')]:
        peak = calculate_stat_from_signal(
            estimator_status_data, 'estimator_status', signal, in_air_no_ground_effects, np.amax)
        if peak > 0.0:
            imu_metrics['{:s}_peak'.format(result)] = peak
            imu_metrics['{:s}_mean'.format(result)] = calculate_stat_from_signal(
                estimator_status_data, 'estimator_status', signal,
                in_air_no_ground_effects, np.mean)

    # IMU bias checks
    imu_metrics['imu_dang_bias_median'] = np.sqrt(np.sum([np.square(calculate_stat_from_signal(
        estimator_status_data, 'estimator_status', signal, in_air_no_ground_effects, np.median))
        for signal in ['states[10]', 'states[11]', 'states[12]']]))
    imu_metrics['imu_dvel_bias_median'] = np.sqrt(np.sum([np.square(calculate_stat_from_signal(
        estimator_status_data, 'estimator_status', signal, in_air_no_ground_effects, np.median))
        for signal in ['states[10]', 'states[11]', 'states[12]']]))

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