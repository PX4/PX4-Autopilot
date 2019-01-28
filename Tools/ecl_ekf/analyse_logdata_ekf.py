from typing import Tuple, List, Dict, Optional, Callable

# matplotlib don't use Xwindows backend (must be before pyplot import)
import matplotlib
matplotlib.use('Agg')
import numpy as np

from pyulog import ULog

from post_processing import get_estimator_check_flags
from plotting import get_max_arg_time_value
from pdf_report import create_pdf_report
from detectors import InAirDetector, PreconditionError
from metrics import calculate_ecl_ekf_metrics


def analyse_ekf(
        ulog: ULog, check_levels: Dict[str, float], test_results_table: Dict[str, list],
        red_thresh: float = 1.0, amb_thresh: float = 0.5) -> list:

    try:
        estimator_status = ulog.get_dataset('estimator_status').data
        print('found estimator_status data')
    except:
        raise PreconditionError('could not find estimator_status data')

    try:
        ekf2_innovations = ulog.get_dataset('ekf2_innovations').data
        print('found ekf2_innovation data')
    except:
        raise PreconditionError('could not find ekf2_innovation data')

    try:
        sensor_preflight = ulog.get_dataset('sensor_preflight').data
        print('found sensor_preflight data')
    except:
        raise PreconditionError('could not find sensor_preflight data')

    try:
        in_air = InAirDetector(
            ulog, min_flight_time_seconds=5.0, in_air_margin_seconds=0.0)
        in_air_no_ground_effects = InAirDetector(
            ulog, min_flight_time_seconds=5.0, in_air_margin_seconds=5.0)
    except Exception as e:
        raise PreconditionError(str(e))

    if in_air_no_ground_effects.take_off is None:
        raise PreconditionError('no airtime detected.')

    control_mode, innov_flags, gps_fail_flags = get_estimator_check_flags(estimator_status)

    sensor_checks, innov_fail_checks = find_checks_that_apply(control_mode, estimator_status)

    ekf_metrics, imu_metrics, innov_fail_metrics, sensor_metrics = calculate_ecl_ekf_metrics(
        ulog, innov_flags, innov_fail_checks, sensor_checks, in_air, in_air_no_ground_effects,
        red_thresh=red_thresh, amb_thresh=amb_thresh)

    # combine the metrics
    combined_metrics = dict()
    combined_metrics.update(imu_metrics)
    combined_metrics.update(sensor_metrics)
    combined_metrics.update(innov_fail_metrics)
    combined_metrics.update(ekf_metrics)

    # perform the checks
    imu_status = perform_imu_checks(imu_metrics, check_levels)

    sensor_status = perform_sensor_checks(
        sensor_checks, sensor_metrics, innov_fail_metrics, check_levels)

    ekf_status = dict()
    ekf_status['filter_fault_status'] = 'Fail' if ekf_metrics['filter_faults_max'] > 0 else 'Pass'

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

    test_results = create_results_table(
        test_results_table, in_air, combined_metrics, combined_status, master_status)

    return test_results


def create_results_table(
        test_results: Dict[str, list], in_air: InAirDetector, combined_metrics: Dict[str, float],
        combined_status: Dict[str, str], master_status: str) -> Dict[str, list]:

    # store metrics
    for key, value in combined_metrics.items():
        test_results[key][0] = value

    # store check results
    for key, value in combined_status.items():
        test_results[key][0] = value

    # store master status
    test_results['master_status'][0] = master_status

    # store take_off and landing information
    test_results['in_air_transition_time'][0] = [round(in_air.take_off, 1)]
    test_results['on_ground_transition_time'][0] = [round(in_air.landing, 1)]

    return test_results


def perform_imu_checks(
        imu_metrics: Dict[str, float], check_levels: Dict[str, float]) -> Dict[str, str]:

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
        sensor_checks: List[str], sensor_metrics: Dict[str, float],
        innov_fail_metrics: Dict[str, float], check_levels: Dict[str, float]) -> Dict[str, str]:


    sensor_status = dict()
    for sensor_check in sensor_checks:
        if sensor_metrics['{:s}_percentage_amber'.format(sensor_check)] > check_levels[
            '{:s}_amber_fail_pct'.format(sensor_check)]:
            sensor_status['{:s}_sensor_status'.format(sensor_check)] = 'Fail'
        elif sensor_metrics['{:s}_percentage_amber'.format(sensor_check)] > check_levels[
            '{:s}_amber_warn_pct'.format(sensor_check)]:
            sensor_status['{:s}_sensor_status'.format(sensor_check)] = 'Warning'
        else:
            sensor_status['{:s}_sensor_status'.format(sensor_check)] = 'Pass'


    for innov_fail_metric, metric_value in innov_fail_metrics.items():
        if innov_fail_metric.startswith('hagl'):
            sensor_id = 'hagl'
        elif innov_fail_metric.startswith('of'):
            sensor_id = 'flow'
        else:
            sensor_id = innov_fail_metric[0:3]

        if metric_value > check_levels['{:s}_fail_pct'.format(sensor_id)]:
            sensor_status['{:s}_sensor_status'.format(sensor_id)] = 'Fail'

    return sensor_status


def find_checks_that_apply(control_mode: dict, estimator_status: dict) -> \
        Tuple[List[str], List[str]]:
    """
    finds the checks that apply and stores them in lists for the std checks and the innovation
    fail checks.
    :param control_mode:
    :param estimator_status:
    :return: a tuple of two lists that contain strings for the std checks and for the innovation
    fail checks.
    """
    sensor_checks = list()
    innov_fail_checks = list()

    # Height Sensor Checks
    sensor_checks.append('hgt')
    innov_fail_checks.append('posv')

    # Magnetometer Sensor Checks
    if (np.amax(control_mode['yaw_aligned']) > 0.5):
        sensor_checks.append('mag')

        innov_fail_checks.append('magx')
        innov_fail_checks.append('magy')
        innov_fail_checks.append('magz')
        innov_fail_checks.append('yaw')

    # Velocity Sensor Checks
    if (np.amax(control_mode['using_gps']) > 0.5):
        sensor_checks.append('vel')
        innov_fail_checks.append('vel')

    # Position Sensor Checks
    if ((np.amax(control_mode['using_gps']) > 0.5) or (np.amax(control_mode['using_evpos']) > 0.5)):
        sensor_checks.append('pos')
        innov_fail_checks.append('posh')

    # Airspeed Sensor Checks
    # a value > 1.0 means the measurement data for that test has been rejected by the EKF
    if (np.amax(estimator_status['tas_test_ratio']) > 0.0):
        sensor_checks.append('tas')
        innov_fail_checks.append('tas')

    # Height above ground (rangefinder) sensor checks
    if (np.amax(estimator_status['hagl_test_ratio']) > 0.0):
        sensor_checks.append('hagl')
        innov_fail_checks.append('hagl')

    # optical flow sensor checks
    if (np.amax(control_mode['using_optflow']) > 0.5):
        innov_fail_checks.append('ofx')
        innov_fail_checks.append('ofy')

    return sensor_checks, innov_fail_checks







