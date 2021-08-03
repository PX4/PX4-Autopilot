#! /usr/bin/env python3
"""
function collection for performing ecl ekf checks
"""

from typing import Tuple, List, Dict


def perform_ecl_ekf_checks(
        metrics: Dict[str, float], sensor_checks: List[str], innov_fail_checks: List[str],
        check_levels: Dict[str, float]) -> Tuple[Dict[str, str], str]:
    """
    # performs the imu, sensor, amd ekf checks and calculates a master status.
    :param metrics:
    :param sensor_checks:
    :param innov_fail_checks:
    :param check_levels:
    :return:
    """

    imu_status = perform_imu_checks(metrics, check_levels)

    sensor_status = perform_sensor_innov_checks(
        metrics, sensor_checks, innov_fail_checks, check_levels)

    ekf_status = dict()
    ekf_status['filter_fault_status'] = 'Fail' if metrics['filter_faults_max'] > 0 else 'Pass'

    # combine the status from the checks
    combined_status = dict()
    combined_status.update(imu_status)
    combined_status.update(sensor_status)
    combined_status.update(ekf_status)
    if any(val == 'Fail' for val in combined_status.values()):
        master_status = 'Fail'
    elif any(val == 'Warning' for val in combined_status.values()):
        master_status = 'Warning'
    else:
        master_status = 'Pass'

    return combined_status, master_status


def perform_imu_checks(
        imu_metrics: Dict[str, float], check_levels: Dict[str, float]) -> Dict[str, str]:
    """
    performs the imu checks.
    :param imu_metrics:
    :param check_levels:
    :return:
    """

    # check for IMU sensor warnings
    imu_status = dict()

    # perform the vibration check
    imu_status['imu_vibration_check'] = 'Pass'
    for imu_vibr_metric in ['imu_coning', 'imu_hfdang', 'imu_hfdvel']:
        mean_metric = '{:s}_mean'.format(imu_vibr_metric)
        peak_metric = '{:s}_peak'.format(imu_vibr_metric)
        if imu_metrics[mean_metric] > check_levels['{:s}_warn'.format(mean_metric)] \
                or imu_metrics[peak_metric] > check_levels['{:s}_warn'.format(peak_metric)]:
            imu_status['imu_vibration_check'] = 'Warning'

    if imu_status['imu_vibration_check'] == 'Warning':
        print('IMU vibration check warning.')

    # perform the imu bias check
    if imu_metrics['imu_dang_bias_median'] > check_levels['imu_dang_bias_median_warn'] \
            or imu_metrics['imu_dvel_bias_median'] > check_levels['imu_dvel_bias_median_warn']:
        imu_status['imu_bias_check'] = 'Warning'
        print('IMU bias check warning.')
    else:
        imu_status['imu_bias_check'] = 'Pass'

    # perform output predictor
    if imu_metrics['output_obs_ang_err_median'] > check_levels['obs_ang_err_median_warn'] \
            or imu_metrics['output_obs_vel_err_median'] > check_levels['obs_vel_err_median_warn'] \
            or imu_metrics['output_obs_pos_err_median'] > check_levels['obs_pos_err_median_warn']:
        imu_status['imu_output_predictor_check'] = 'Warning'
        print('IMU output predictor check warning.')
    else:
        imu_status['imu_output_predictor_check'] = 'Pass'

    imu_status['imu_sensor_status'] = 'Warning' if any(
        val == 'Warning' for val in imu_status.values()) else 'Pass'

    return imu_status


def perform_sensor_innov_checks(
        metrics: Dict[str, float], sensor_checks: List[str], innov_fail_checks: List[str],
        check_levels: Dict[str, float]) -> Dict[str, str]:
    """
    performs the sensor checks.
    :param metrics:
    :param sensor_checks:
    :param innov_fail_checks:
    :param check_levels:
    :return:
    """

    sensor_status = dict()

    for result_id in ['hgt', 'mag', 'vel', 'pos', 'tas', 'hagl']:

        # only run sensor checks, if they apply.
        if result_id in sensor_checks:
            if metrics['{:s}_percentage_amber'.format(result_id)] > check_levels[
                '{:s}_amber_fail_pct'.format(result_id)]:
                sensor_status['{:s}_sensor_status'.format(result_id)] = 'Fail'
                print('{:s} sensor check failure.'.format(result_id))
            elif metrics['{:s}_percentage_amber'.format(result_id)] > check_levels[
                '{:s}_amber_warn_pct'.format(result_id)]:
                sensor_status['{:s}_sensor_status'.format(result_id)] = 'Warning'
                print('{:s} sensor check warning.'.format(result_id))
            else:
                sensor_status['{:s}_sensor_status'.format(result_id)] = 'Pass'

    # perform innovation checks.
    for signal_id, metric_name, result_id in [('posv', 'hgt_fail_percentage', 'hgt'),
                                              ('magx', 'magx_fail_percentage', 'mag'),
                                              ('magy', 'magy_fail_percentage', 'mag'),
                                              ('magz', 'magz_fail_percentage', 'mag'),
                                              ('yaw', 'yaw_fail_percentage', 'yaw'),
                                              ('vel', 'vel_fail_percentage', 'vel'),
                                              ('posh', 'pos_fail_percentage', 'pos'),
                                              ('tas', 'tas_fail_percentage', 'tas'),
                                              ('hagl', 'hagl_fail_percentage', 'hagl'),
                                              ('ofx', 'ofx_fail_percentage', 'flow'),
                                              ('ofy', 'ofy_fail_percentage', 'flow')]:

        # only run innov fail checks, if they apply.
        if signal_id in innov_fail_checks:

            if metrics[metric_name] > check_levels['{:s}_fail_pct'.format(result_id)]:
                sensor_status['{:s}_sensor_status'.format(result_id)] = 'Fail'
                print('{:s} sensor check failure.'.format(result_id))
            else:
                if not ('{:s}_sensor_status'.format(result_id) in sensor_status):
                    sensor_status['{:s}_sensor_status'.format(result_id)] = 'Pass'

    return sensor_status
