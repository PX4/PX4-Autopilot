from typing import Tuple, List, Dict, Optional, Callable

# matplotlib don't use Xwindows backend (must be before pyplot import)
import matplotlib
matplotlib.use('Agg')
import numpy as np

from post_processing import get_estimator_check_flags
from plotting import get_max_arg_time_value
from pdf_report import create_pdf_report
from detectors import InAirDetector


def analyse_ekf(
        estimator_status: dict, ekf2_innovations: dict, sensor_preflight: dict,
        check_levels: Dict[str, float], test_results_table: Dict[str, list], in_air: InAirDetector,
        in_air_no_ground_effects: InAirDetector, plot_report: bool = False,
        output_plot_filename: Optional[str] = None) -> list:

    control_mode, innov_flags, gps_fail_flags = get_estimator_check_flags(estimator_status)

    status_time = 1e-6 * estimator_status['timestamp']

    b_finishes_in_air, b_starts_in_air, in_air_duration, in_air_transition_time, \
    on_ground_transition_time = detect_airtime(control_mode, status_time)

    if plot_report:
        create_pdf_report(
            control_mode, ekf2_innovations, estimator_status, gps_fail_flags, in_air_duration,
            in_air_transition_time, innov_flags, on_ground_transition_time, output_plot_filename,
            sensor_preflight, status_time)

    red_thresh = 1.0
    amb_thresh = 0.5

    sensor_checks, innov_fail_checks = find_sensor_checks_that_apply(control_mode, estimator_status)

    sensor_metrics = calculate_sensor_metrics(
        estimator_status, sensor_checks, in_air, in_air_no_ground_effects,
        red_thresh=red_thresh, amb_thresh=amb_thresh)

    innov_fail_metrics = calculate_innov_fail_metrics(
        innov_flags, innov_fail_checks, in_air, in_air_no_ground_effects)

    imu_metrics = calculate_imu_metrics(
        ekf2_innovations, estimator_status, in_air_no_ground_effects)

    # Check for internal filter nummerical faults
    ekf_metrics = {'filter_faults_max': np.amax(estimator_status['filter_fault_flags'])}
    # TODO - process these bitmask's when they have been properly documented in the uORB topic
    # estimator_status['health_flags']
    # estimator_status['timeout_flags']

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


def perform_imu_checks(imu_metrics: Dict[str, float], check_levels: Dict[str, float])\
        -> Dict[str, str]:

    # check for IMU sensor warnings
    imu_status = dict()
    # check for vibrations
    imu_status['imu_vibration_check'] = 'Pass'
    for imu_vibr_metric in ['imu_coning', 'imu_hfdang', 'imu_hfdvel']:
        mean_metric = '{:s}_mean'.format(imu_vibr_metric)
        peak_metric = '{:s}_peak'.format(imu_vibr_metric)
        if imu_metrics[mean_metric] > check_levels['{:s}_warn'.format(mean_metric)] \
                or imu_metrics[peak_metric] > check_levels['{:s}_warn'.format(peak_metric)]:
            imu_status['imu_vibration_check'] = 'Warning'
    if imu_status['imu_vibration_check'] == 'Warning':
        print('IMU vibration check warning.')
    if imu_metrics['imu_dang_bias_median'] > check_levels['imu_dang_bias_median_warn'] \
            or imu_metrics['imu_dvel_bias_median'] > check_levels['imu_dvel_bias_median_warn']:
        imu_status['imu_bias_check'] = 'Warning'
        print('IMU bias check warning.')
    else:
        imu_status['imu_bias_check'] = 'Pass'
    if imu_metrics['output_obs_ang_err_median'] > check_levels['obs_ang_err_median_warn'] \
            or imu_metrics['output_obs_vel_err_median'] > check_levels['obs_vel_err_median_warn'] \
            or imu_metrics['output_obs_pos_err_median'] > check_levels['obs_pos_err_median_warn']:
        imu_status['imu_output_predictor_check'] = 'Warning'
        print('IMU output predictor check warning.')
    else:
        imu_status['imu_output_predictor_check'] = 'Pass'
    imu_status['imu_sensor_status'] = 'Warning' if any(
        val == 'Warning' for val in imu_status.values()) else 'Pass'

    return  imu_status


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


def calculate_sensor_metrics(
        estimator_status: dict, sensor_checks: List[str], in_air: InAirDetector,
        in_air_no_ground_effects: InAirDetector, red_thresh: float = 1.0,
        amb_thresh: float = 0.5) -> dict:

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
            estimator_status, 'estimator_status', signal, in_air_detector,
            lambda x: 100.0 * np.mean(x > red_thresh))
        sensor_metrics['{:s}_percentage_amber'.format(result)] = calculate_stat_from_signal(
            estimator_status, 'estimator_status', signal, in_air_detector,
            lambda x: 100.0 * np.mean(x > amb_thresh))

        # the percentage of samples above / below std dev
        peak = calculate_stat_from_signal(
            estimator_status, 'estimator_status', signal, in_air, np.amax)
        if peak > 0.0:
            sensor_metrics['{:s}_test_max'.format(result)] = peak
            sensor_metrics['{:s}_test_mean'.format(result)] = calculate_stat_from_signal(
                estimator_status, 'estimator_status', signal, in_air_no_ground_effects, np.mean)

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
        ekf2_innovations: dict, estimator_status: dict, in_air_no_ground_effects: dict) -> dict:

    imu_metrics = dict()

    # calculates the median of the output tracking error ekf innovations
    for signal, result in [('output_tracking_error[0]', 'output_obs_ang_err_median'),
                           ('output_tracking_error[1]', 'output_obs_vel_err_median'),
                           ('output_tracking_error[2]', 'output_obs_pos_err_median')]:
        imu_metrics[result] = calculate_stat_from_signal(
            ekf2_innovations, 'ekf2_innovations', signal, in_air_no_ground_effects, np.median)
    # calculates peak and mean for IMU vibration checks
    for signal, result in [('vibe[0]', 'imu_coning'),
                           ('vibe[1]', 'imu_hfdang'),
                           ('vibe[2]', 'imu_hfdvel')]:
        peak = calculate_stat_from_signal(
            estimator_status, 'estimator_status', signal, in_air_no_ground_effects, np.amax)
        if peak > 0.0:
            imu_metrics['{:s}_peak'.format(result)] = peak
            imu_metrics['{:s}_mean'.format(result)] = calculate_stat_from_signal(
                estimator_status, 'estimator_status', signal, in_air_no_ground_effects, np.mean)
    # IMU bias checks
    imu_metrics['imu_dang_bias_median'] = np.sqrt(np.sum([np.square(calculate_stat_from_signal(
        estimator_status, 'estimator_status', signal, in_air_no_ground_effects, np.median))
        for signal in ['states[10]', 'states[11]', 'states[12]']]))
    imu_metrics['imu_dvel_bias_median'] = np.sqrt(np.sum([np.square(calculate_stat_from_signal(
        estimator_status, 'estimator_status', signal, in_air_no_ground_effects, np.median))
        for signal in ['states[10]', 'states[11]', 'states[12]']]))

    return imu_metrics


def find_sensor_checks_that_apply(control_mode: dict, estimator_status: dict) -> \
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


def calculate_stats_from_signals(
        data: Dict[str, np.ndarray], dataset: str, variables: List[str],
        in_air_det: InAirDetector, stat_function: Callable) -> List[float]:
    """
    :param data:
    :param variables:
    :param in_air_detector:
    :return:
    """

    return [stat_function(data[var][in_air_det.get_airtime(dataset)]) for var in variables]


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




