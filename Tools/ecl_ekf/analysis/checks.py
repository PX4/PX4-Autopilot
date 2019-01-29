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

    sensor_status = perform_sensor_checks(
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


def perform_sensor_checks(
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
    for sensor_check in sensor_checks:
        if metrics['{:s}_percentage_amber'.format(sensor_check)] > check_levels[
            '{:s}_amber_fail_pct'.format(sensor_check)]:
            sensor_status['{:s}_sensor_status'.format(sensor_check)] = 'Fail'
        elif metrics['{:s}_percentage_amber'.format(sensor_check)] > check_levels[
            '{:s}_amber_warn_pct'.format(sensor_check)]:
            sensor_status['{:s}_sensor_status'.format(sensor_check)] = 'Warning'
        else:
            sensor_status['{:s}_sensor_status'.format(sensor_check)] = 'Pass'


    for innov_check in innov_fail_checks:
        # the normal case
        metric_name = '{:s}_fail_percentage'.format(innov_check)
        result_name = innov_check

        # special cases
        if innov_check.startswith('posv'):
            metric_name = 'hgt_fail_percentage'
            result_name = 'hgt'
        elif innov_check.startswith('posh'):
            metric_name = 'pos_fail_percentage'
            result_name = 'pos'
        elif innov_check.startswith('of'):
            result_name = 'flow'
        elif innov_check.startswith('mag'):
            result_name = 'mag'

        if metrics[metric_name] > check_levels['{:s}_fail_pct'.format(result_name)]:
            sensor_status['{:s}_sensor_status'.format(result_name)] = 'Fail'

    return sensor_status