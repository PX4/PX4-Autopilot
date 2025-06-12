#! /usr/bin/env python3
"""
function collection for plotting
"""

# matplotlib don't use Xwindows backend (must be before pyplot import)
import matplotlib
matplotlib.use('Agg')

import numpy as np
from matplotlib.backends.backend_pdf import PdfPages
from pyulog import ULog

from analysis.post_processing import magnetic_field_estimates_from_states, get_gps_check_fail_flags
from plotting.data_plots import TimeSeriesPlot, InnovationPlot, ControlModeSummaryPlot, \
    CheckFlagsPlot
from analysis.detectors import PreconditionError
import analysis.data_version_handler as dvh

def create_pdf_report(ulog: ULog, multi_instance: int, output_plot_filename: str) -> None:
    """
    creates a pdf report of the ekf analysis.
    :param ulog:
    :param output_plot_filename:
    :return:
    """

    # create summary plots
    # save the plots to PDF

    try:
        estimator_status = ulog.get_dataset('estimator_status', multi_instance).data
    except:
        raise PreconditionError('could not find estimator_status instance', multi_instance)

    try:
        estimator_status_flags = ulog.get_dataset('estimator_status_flags', multi_instance).data
    except:
        raise PreconditionError('could not find estimator_status_flags instance', multi_instance)

    try:
        estimator_states = ulog.get_dataset('estimator_states', multi_instance).data
    except:
        raise PreconditionError('could not find estimator_states instance', multi_instance)

    try:
        estimator_sensor_bias = ulog.get_dataset('estimator_sensor_bias', multi_instance).data
    except:
        raise PreconditionError('could not find estimator_sensor_bias instance', multi_instance)

    try:
        estimator_innovations = ulog.get_dataset('estimator_innovations', multi_instance).data
        estimator_innovation_variances = ulog.get_dataset('estimator_innovation_variances', multi_instance).data
        innovation_data = estimator_innovations
        for key in estimator_innovation_variances:
            # append 'var' to the field name such that we can distingush between innov and innov_var
            innovation_data.update({str('var_'+key): estimator_innovation_variances[key]})

        innovations_min_length = float('inf')
        for key in estimator_innovations:
            if len(estimator_innovations[key]) < innovations_min_length:
                innovations_min_length = len(estimator_innovations[key])

        variances_min_length = float('inf')
        for key in estimator_innovation_variances:
            if len(estimator_innovation_variances[key]) < variances_min_length:
                variances_min_length = len(estimator_innovation_variances[key])

        # ensure consistent sizing for plots
        if (innovations_min_length != variances_min_length):
            print("estimator_innovations and estimator_innovation_variances are different sizes, adjusting")
            innovation_data_min_length = min(innovations_min_length, variances_min_length)
            for key in innovation_data:
                innovation_data[key] = innovation_data[key][0:innovation_data_min_length]

        print('found innovation data (merged estimator_innovations + estimator_innovation_variances) instance', multi_instance)

    except:
        raise PreconditionError('could not find innovation data')

    gps_fail_flags = get_gps_check_fail_flags(estimator_status)

    status_time = 1e-6 * estimator_status['timestamp']
    status_flags_time = 1e-6 * estimator_status_flags['timestamp']

    b_finishes_in_air, b_starts_in_air, in_air_duration, in_air_transition_time, \
    on_ground_transition_time = detect_airtime(estimator_status_flags, status_flags_time)

    with PdfPages(output_plot_filename) as pdf_pages:

        # vertical velocity and position innovations
        data_plot = InnovationPlot(
            innovation_data, [('gps_vpos', 'var_gps_vpos'),
                               ('gps_vvel', 'var_gps_vvel')],
            x_labels=['time (sec)', 'time (sec)'],
            y_labels=['Down Vel (m/s)', 'Down Pos (m)'], plot_title='Vertical Innovations',
            pdf_handle=pdf_pages)
        data_plot.save()
        data_plot.close()

        # horizontal velocity innovations
        data_plot = InnovationPlot(
            innovation_data, [('gps_hvel[0]', 'var_gps_hvel[0]'),
                              ('gps_hvel[1]', 'var_gps_hvel[1]')],
            x_labels=['time (sec)', 'time (sec)'],
            y_labels=['North Vel (m/s)', 'East Vel (m/s)'],
            plot_title='Horizontal Velocity  Innovations', pdf_handle=pdf_pages)
        data_plot.save()
        data_plot.close()

        # horizontal position innovations
        data_plot = InnovationPlot(
            innovation_data, [('gps_hpos[0]', 'var_gps_hpos[0]'),
                              ('gps_hpos[1]', 'var_gps_hpos[1]')],
            x_labels=['time (sec)', 'time (sec)'],
            y_labels=['North Pos (m)', 'East Pos (m)'], plot_title='Horizontal Position Innovations',
            pdf_handle=pdf_pages)
        data_plot.save()
        data_plot.close()

        # magnetometer innovations
        data_plot = InnovationPlot(
            innovation_data, [('mag_field[0]', 'var_mag_field[0]'),
                              ('mag_field[1]', 'var_mag_field[1]'),
                              ('mag_field[2]', 'var_mag_field[2]')],
            x_labels=['time (sec)', 'time (sec)', 'time (sec)'],
            y_labels=['X (Gauss)', 'Y (Gauss)', 'Z (Gauss)'], plot_title='Magnetometer Innovations',
            pdf_handle=pdf_pages)
        data_plot.save()
        data_plot.close()

        # magnetic heading innovations
        data_plot = InnovationPlot(
            innovation_data, [('heading', 'var_heading')],
            x_labels=['time (sec)'], y_labels=['Heading (rad)'],
            plot_title='Magnetic Heading Innovations', pdf_handle=pdf_pages)
        data_plot.save()
        data_plot.close()

        # air data innovations
        data_plot = InnovationPlot(
            innovation_data, [('airspeed', 'var_airspeed'),
                              ('beta', 'var_beta')],
            x_labels=['time (sec)', 'time (sec)'],
            y_labels=['innovation (m/sec)', 'innovation (rad)'],
            sub_titles=['True Airspeed Innovations', 'Synthetic Sideslip Innovations'],
            pdf_handle=pdf_pages)
        data_plot.save()
        data_plot.close()

        # optical flow innovations
        data_plot = InnovationPlot(
            innovation_data, [('flow[0]', 'var_flow[0]'),
                              ('flow[1]', 'var_flow[1]')],
            x_labels=['time (sec)', 'time (sec)'],
            y_labels=['X (rad/sec)', 'Y (rad/sec)'],
            plot_title='Optical Flow Innovations', pdf_handle=pdf_pages)
        data_plot.save()
        data_plot.close()

        # plot normalised innovation test levels
        # define variables to plot
        variables = [['hdg_test_ratio'], ['vel_test_ratio', 'pos_test_ratio'], ['hgt_test_ratio']]
        y_labels = ['mag', 'vel, pos', 'hgt']
        legend = [['mag'], ['vel', 'pos'], ['hgt']]
        if np.amax(estimator_status['hagl_test_ratio']) > 0.0:  # plot hagl test ratio, if applicable
            variables[-1].append('hagl_test_ratio')
            y_labels[-1] += ', hagl'
            legend[-1].append('hagl')

        if np.amax(estimator_status[
                       'tas_test_ratio']) > 0.0:  # plot airspeed sensor test ratio, if applicable
            variables.append(['tas_test_ratio'])
            y_labels.append('TAS')
            legend.append(['airspeed'])

        data_plot = CheckFlagsPlot(
            status_time, estimator_status, variables, x_label='time (sec)', y_labels=y_labels,
            plot_title='Normalised Innovation Test Levels', pdf_handle=pdf_pages, annotate=True,
            legend=legend
        )
        data_plot.save()
        data_plot.close()

        # plot control mode summary A
        data_plot = ControlModeSummaryPlot(
            status_flags_time, estimator_status_flags, [['cs_tilt_align', 'cs_yaw_align'],
            ['cs_gnss_pos', 'cs_opt_flow', 'cs_ev_pos'], ['cs_baro_hgt', 'cs_gps_hgt',
             'cs_rng_hgt', 'cs_ev_hgt'], ['cs_mag_hdg', 'cs_mag_3d', 'cs_mag_dec']],
            x_label='time (sec)', y_labels=['aligned', 'pos aiding', 'hgt aiding', 'mag aiding'],
            annotation_text=[['tilt alignment', 'yaw alignment'], ['GPS aiding', 'optical flow aiding',
             'external vision aiding'], ['Baro aiding', 'GPS aiding', 'rangefinder aiding',
             'external vision aiding'], ['magnetic yaw aiding', '3D magnetoemter aiding',
             'magnetic declination aiding']], plot_title='EKF Control Status - Figure A',
            pdf_handle=pdf_pages)
        data_plot.save()
        data_plot.close()

        # plot control mode summary B
        # construct additional annotations for the airborne plot
        airborne_annotations = list()
        if np.amin(np.diff(estimator_status_flags['cs_in_air'])) > -0.5:
            airborne_annotations.append(
                (on_ground_transition_time, 'air to ground transition not detected'))
        else:
            airborne_annotations.append((on_ground_transition_time, 'on-ground at {:.1f} sec'.format(
                on_ground_transition_time)))
        if in_air_duration > 0.0:
            airborne_annotations.append(((in_air_transition_time + on_ground_transition_time) / 2,
                                         'duration = {:.1f} sec'.format(in_air_duration)))
        if np.amax(np.diff(estimator_status_flags['cs_in_air'])) < 0.5:
            airborne_annotations.append(
                (in_air_transition_time, 'ground to air transition not detected'))
        else:
            airborne_annotations.append(
                (in_air_transition_time, 'in-air at {:.1f} sec'.format(in_air_transition_time)))

        data_plot = ControlModeSummaryPlot(
            status_flags_time, estimator_status_flags, [['cs_in_air'], ['cs_wind']],
            x_label='time (sec)', y_labels=['airborne', 'estimating wind'], annotation_text=[[], []],
            additional_annotation=[airborne_annotations, []],
            plot_title='EKF Control Status - Figure B', pdf_handle=pdf_pages)
        data_plot.save()
        data_plot.close()

        # plot innovation flags summary
        data_plot = CheckFlagsPlot(
            status_flags_time, estimator_status_flags, [['reject_hor_vel', 'reject_hor_pos'], ['reject_ver_vel', 'reject_ver_pos',
                                                                               'reject_hagl'],
                                       ['fs_bad_mag_x', 'fs_bad_mag_y', 'fs_bad_mag_z',
                                        'reject_yaw'], ['reject_airspeed'], ['reject_sideslip'],
                                       ['reject_optflow_x',
                                        'reject_optflow_y']], x_label='time (sec)',
            y_labels=['failed', 'failed', 'failed', 'failed', 'failed', 'failed'],
            y_lim=(-0.1, 1.1),
            legend=[['vel NE', 'pos NE'], ['vel D', 'hgt absolute', 'hgt above ground'],
                    ['mag_x', 'mag_y', 'mag_z', 'yaw'], ['airspeed'], ['sideslip'],
                    ['flow X', 'flow Y']],
            plot_title='EKF Innovation Test Fails', annotate=False, pdf_handle=pdf_pages)
        data_plot.save()
        data_plot.close()

        # gps_check_fail_flags summary
        data_plot = CheckFlagsPlot(
            status_time, gps_fail_flags,
            [['nsat_fail', 'pdop_fail', 'herr_fail', 'verr_fail', 'gfix_fail', 'serr_fail'],
             ['hdrift_fail', 'vdrift_fail', 'hspd_fail', 'veld_diff_fail']],
            x_label='time (sec)', y_lim=(-0.1, 1.1), y_labels=['failed', 'failed'],
            sub_titles=['GPS Direct Output Check Failures', 'GPS Derived Output Check Failures'],
            legend=[['N sats', 'PDOP', 'horiz pos error', 'vert pos error', 'fix type',
                     'speed error'], ['horiz drift', 'vert drift', 'horiz speed',
                                      'vert vel inconsistent']], annotate=False, pdf_handle=pdf_pages)
        data_plot.save()
        data_plot.close()

        # filter reported accuracy
        data_plot = CheckFlagsPlot(
            status_time, estimator_status, [['pos_horiz_accuracy', 'pos_vert_accuracy']],
            x_label='time (sec)', y_labels=['accuracy (m)'], plot_title='Reported Accuracy',
            legend=[['horizontal', 'vertical']], annotate=False, pdf_handle=pdf_pages)
        data_plot.save()
        data_plot.close()


        # Plot the EKF IMU vibration metrics
        for imu_status_instance in range(4):
            try:
                vehicle_imu_status_data = ulog.get_dataset('vehicle_imu_status', imu_status_instance).data

                imu_status_time = 1e-6 * vehicle_imu_status_data['timestamp']

                if vehicle_imu_status_data['accel_device_id'][0] == estimator_status['accel_device_id'][0]:

                    scaled_estimator_status = {'delta_angle_coning_metric': 1000. * vehicle_imu_status_data['delta_angle_coning_metric'],
                                               'gyro_vibration_metric': vehicle_imu_status_data['gyro_vibration_metric'],
                                               'accel_vibration_metric': vehicle_imu_status_data['accel_vibration_metric']
                                              }
                    data_plot = CheckFlagsPlot(
                        imu_status_time, scaled_estimator_status, [['delta_angle_coning_metric'], ['gyro_vibration_metric'], ['accel_vibration_metric']],
                        x_label='time (sec)',
                        y_labels=['Del Ang Coning (mrad^2)', 'HF Gyro (rad/s)', 'HF accel (m/s/s)'],
                        plot_title='IMU Vibration Metrics',
                        pdf_handle=pdf_pages, annotate=True)
                    data_plot.save()
                    data_plot.close()

            except:
                pass


        # Plot the EKF output observer tracking errors
        scaled_innovations = {
            'output_tracking_error[0]': 1000. * estimator_status['output_tracking_error[0]'],
            'output_tracking_error[1]': estimator_status['output_tracking_error[1]'],
            'output_tracking_error[2]': estimator_status['output_tracking_error[2]']
            }
        data_plot = CheckFlagsPlot(
            1e-6 * estimator_status['timestamp'], scaled_innovations,
            [['output_tracking_error[0]'], ['output_tracking_error[1]'],
             ['output_tracking_error[2]']], x_label='time (sec)',
            y_labels=['angles (mrad)', 'velocity (m/s)', 'position (m)'],
            plot_title='Output Observer Tracking Error Magnitudes',
            pdf_handle=pdf_pages, annotate=True)
        data_plot.save()
        data_plot.close()

        # Plot the gyro bias estimates
        data_plot = CheckFlagsPlot(
            1e-6 * estimator_sensor_bias['timestamp'], estimator_sensor_bias,
            [['gyro_bias[0]'], ['gyro_bias[1]'], ['gyro_bias[2]']],
            x_label='time (sec)', y_labels=['X (rad/s)', 'Y (rad/s)', 'Z (rad/s)'],
            plot_title='Gyro Bias Estimates', annotate=False, pdf_handle=pdf_pages)
        data_plot.save()
        data_plot.close()

        # Plot the accel bias estimates
        data_plot = CheckFlagsPlot(
            1e-6 * estimator_sensor_bias['timestamp'], estimator_sensor_bias,
            [['accel_bias[0]'], ['accel_bias[1]'], ['accel_bias[2]']],
            x_label='time (sec)', y_labels=['X (m/s^2)', 'Y (m/s^2)', 'Z (m/s^2)'],
            plot_title='Accel Bias Estimates', annotate=False, pdf_handle=pdf_pages)
        data_plot.save()
        data_plot.close()

        # Plot the earth frame magnetic field estimates
        declination, field_strength, inclination = magnetic_field_estimates_from_states(
            estimator_states)
        data_plot = CheckFlagsPlot(
            1e-6 * estimator_states['timestamp'],
            {'strength': field_strength, 'declination': declination, 'inclination': inclination},
            [['declination'], ['inclination'], ['strength']],
            x_label='time (sec)', y_labels=['declination (deg)', 'inclination (deg)',
                                            'strength (Gauss)'],
            plot_title='Earth Magnetic Field Estimates', annotate=False,
            pdf_handle=pdf_pages)
        data_plot.save()
        data_plot.close()

        # Plot the body frame magnetic field estimates
        data_plot = CheckFlagsPlot(
            1e-6 * estimator_states['timestamp'], estimator_states,
            [['states[19]'], ['states[20]'], ['states[21]']],
            x_label='time (sec)', y_labels=['X (Gauss)', 'Y (Gauss)', 'Z (Gauss)'],
            plot_title='Magnetometer Bias Estimates', annotate=False, pdf_handle=pdf_pages)
        data_plot.save()
        data_plot.close()

        # Plot the EKF wind estimates
        data_plot = CheckFlagsPlot(
            1e-6 * estimator_states['timestamp'], estimator_states,
            [['states[22]'], ['states[23]']], x_label='time (sec)',
            y_labels=['North (m/s)', 'East (m/s)'], plot_title='Wind Velocity Estimates',
            annotate=False, pdf_handle=pdf_pages)
        data_plot.save()
        data_plot.close()


def detect_airtime(estimator_status_flags, status_flags_time):
    # define flags for starting and finishing in air
    b_starts_in_air = False
    b_finishes_in_air = False
    # calculate in-air transition time
    if (np.amin(estimator_status_flags['cs_in_air']) < 0.5) and (np.amax(estimator_status_flags['cs_in_air']) > 0.5):
        in_air_transtion_time_arg = np.argmax(np.diff(estimator_status_flags['cs_in_air']))
        in_air_transition_time = status_flags_time[in_air_transtion_time_arg]
    elif (np.amax(estimator_status_flags['cs_in_air']) > 0.5):
        in_air_transition_time = np.amin(status_flags_time)
        print('log starts while in-air at ' + str(round(in_air_transition_time, 1)) + ' sec')
        b_starts_in_air = True
    else:
        in_air_transition_time = float('NaN')
        print('always on ground')
    # calculate on-ground transition time
    if (np.amin(np.diff(estimator_status_flags['cs_in_air'])) < 0.0):
        on_ground_transition_time_arg = np.argmin(np.diff(estimator_status_flags['cs_in_air']))
        on_ground_transition_time = status_flags_time[on_ground_transition_time_arg]
    elif (np.amax(estimator_status_flags['cs_in_air']) > 0.5):
        on_ground_transition_time = np.amax(status_flags_time)
        print('log finishes while in-air at ' + str(round(on_ground_transition_time, 1)) + ' sec')
        b_finishes_in_air = True
    else:
        on_ground_transition_time = float('NaN')
        print('always on ground')
    if (np.amax(np.diff(estimator_status_flags['cs_in_air'])) > 0.5) and (np.amin(np.diff(estimator_status_flags['cs_in_air'])) < -0.5):
        if ((on_ground_transition_time - in_air_transition_time) > 0.0):
            in_air_duration = on_ground_transition_time - in_air_transition_time
        else:
            in_air_duration = float('NaN')
    else:
        in_air_duration = float('NaN')
    return b_finishes_in_air, b_starts_in_air, in_air_duration, in_air_transition_time, on_ground_transition_time
