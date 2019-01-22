#! /usr/bin/env python3
"""
function collection for plotting
"""

import numpy as np
from matplotlib.backends.backend_pdf import PdfPages

from post_processing import magnetic_field_estimates_from_status
from plotting import TimeSeriesPlot, InnovationPlot, ControlModeSummaryPlot, CheckFlagsPlot


def create_pdf_report(control_mode, ekf2_innovations, estimator_status, gps_fail_flags,
                      in_air_duration, in_air_transition_time, innov_flags,
                      on_ground_transition_time, output_plot_filename, sensor_preflight,
                      status_time):
    """
    creates a pdf report of the ekf analysis.
    :param control_mode:
    :param ekf2_innovations:
    :param estimator_status:
    :param gps_fail_flags:
    :param in_air_duration:
    :param in_air_transition_time:
    :param innov_flags:
    :param on_ground_transition_time:
    :param output_plot_filename:
    :param sensor_preflight:
    :param status_time:
    :return:
    """

    # create summary plots
    # save the plots to PDF
    with PdfPages(output_plot_filename) as pdf_pages:

        # plot IMU consistency data
        if ('accel_inconsistency_m_s_s' in sensor_preflight.keys()) and (
                'gyro_inconsistency_rad_s' in sensor_preflight.keys()):
            data_plot = TimeSeriesPlot(
                sensor_preflight, [['accel_inconsistency_m_s_s'], ['gyro_inconsistency_rad_s']],
                x_labels=['data index', 'data index'],
                y_labels=['acceleration (m/s/s)', 'angular rate (rad/s)'],
                plot_title='IMU Consistency Check Levels', pdf_handle=pdf_pages)
            data_plot.save()

        # vertical velocity and position innovations
        data_plot = InnovationPlot(
            ekf2_innovations, [('vel_pos_innov[2]', 'vel_pos_innov_var[2]'),
                               ('vel_pos_innov[5]', 'vel_pos_innov_var[5]')],
            x_labels=['time (sec)', 'time (sec)'],
            y_labels=['Down Vel (m/s)', 'Down Pos (m)'], plot_title='Vertical Innovations',
            pdf_handle=pdf_pages)
        data_plot.save()

        # horizontal velocity innovations
        data_plot = InnovationPlot(
            ekf2_innovations, [('vel_pos_innov[0]', 'vel_pos_innov_var[0]'),
                               ('vel_pos_innov[1]','vel_pos_innov_var[1]')],
            x_labels=['time (sec)', 'time (sec)'],
            y_labels=['North Vel (m/s)', 'East Vel (m/s)'],
            plot_title='Horizontal Velocity  Innovations', pdf_handle=pdf_pages)
        data_plot.save()

        # horizontal position innovations
        data_plot = InnovationPlot(
            ekf2_innovations, [('vel_pos_innov[3]', 'vel_pos_innov_var[3]'), ('vel_pos_innov[4]',
                                                                              'vel_pos_innov_var[4]')],
            x_labels=['time (sec)', 'time (sec)'],
            y_labels=['North Pos (m)', 'East Pos (m)'], plot_title='Horizontal Position Innovations',
            pdf_handle=pdf_pages)
        data_plot.save()

        # magnetometer innovations
        data_plot = InnovationPlot(
            ekf2_innovations, [('mag_innov[0]', 'mag_innov_var[0]'),
           ('mag_innov[1]', 'mag_innov_var[1]'), ('mag_innov[2]', 'mag_innov_var[2]')],
            x_labels=['time (sec)', 'time (sec)', 'time (sec)'],
            y_labels=['X (Gauss)', 'Y (Gauss)', 'Z (Gauss)'], plot_title='Magnetometer Innovations',
            pdf_handle=pdf_pages)
        data_plot.save()

        # magnetic heading innovations
        data_plot = InnovationPlot(
            ekf2_innovations, [('heading_innov', 'heading_innov_var')],
            x_labels=['time (sec)'], y_labels=['Heading (rad)'],
            plot_title='Magnetic Heading Innovations', pdf_handle=pdf_pages)
        data_plot.save()

        # air data innovations
        data_plot = InnovationPlot(
            ekf2_innovations,
            [('airspeed_innov', 'airspeed_innov_var'), ('beta_innov', 'beta_innov_var')],
            x_labels=['time (sec)', 'time (sec)'],
            y_labels=['innovation (m/sec)', 'innovation (rad)'],
            sub_titles=['True Airspeed Innovations', 'Synthetic Sideslip Innovations'],
            pdf_handle=pdf_pages)
        data_plot.save()

        # optical flow innovations
        data_plot = InnovationPlot(
            ekf2_innovations, [('flow_innov[0]', 'flow_innov_var[0]'), ('flow_innov[1]',
                                                                        'flow_innov_var[1]')],
            x_labels=['time (sec)', 'time (sec)'],
            y_labels=['X (rad/sec)', 'Y (rad/sec)'],
            plot_title='Optical Flow Innovations', pdf_handle=pdf_pages)
        data_plot.save()

        # plot normalised innovation test levels
        # define variables to plot
        variables = [['mag_test_ratio'], ['vel_test_ratio', 'pos_test_ratio'], ['hgt_test_ratio']]
        if np.amax(estimator_status['hagl_test_ratio']) > 0.0:  # plot hagl test ratio, if applicable
            variables[-1].append('hagl_test_ratio')
        y_labels = ['mag', 'vel, pos', 'hgt']

        if np.amax(estimator_status[
                       'tas_test_ratio']) > 0.0:  # plot airspeed sensor test ratio, if applicable
            variables.append(['tas_test_ratio'])
            y_labels.append('TAS')

        data_plot = CheckFlagsPlot(
            status_time, estimator_status, variables, x_label='time (sec)', y_labels=y_labels,
            plot_title='Normalised Innovation Test Levels', pdf_handle=pdf_pages)
        data_plot.save()

        # plot control mode summary A
        data_plot = ControlModeSummaryPlot(
            status_time, control_mode, [['tilt_aligned', 'yaw_aligned'],
            ['using_gps', 'using_optflow', 'using_evpos'], ['using_barohgt', 'using_gpshgt',
             'using_rnghgt', 'using_evhgt'], ['using_magyaw', 'using_mag3d', 'using_magdecl']],
            x_label='time (sec)', y_labels=['aligned', 'pos aiding', 'hgt aiding', 'mag aiding'],
            annotation_text=[['tilt alignment', 'yaw alignment'], ['GPS aiding', 'optical flow aiding',
             'external vision aiding'], ['Baro aiding', 'GPS aiding', 'rangefinder aiding',
             'external vision aiding'], ['magnetic yaw aiding', '3D magnetoemter aiding',
             'magnetic declination aiding']], plot_title='EKF Control Status - Figure A',
            pdf_handle=pdf_pages)
        data_plot.save()

        # plot control mode summary B
        # construct additional annotations for the airborne plot
        airborne_annotations = list()
        if np.amin(np.diff(control_mode['airborne'])) > -0.5:
            airborne_annotations.append(
                (on_ground_transition_time, 'air to ground transition not detected'))
        else:
            airborne_annotations.append((on_ground_transition_time, 'on-ground at {:.1f} sec'.format(
                on_ground_transition_time)))
        if in_air_duration > 0.0:
            airborne_annotations.append(((in_air_transition_time + on_ground_transition_time) / 2,
                                         'duration = {:.1f} sec'.format(in_air_duration)))
        if np.amax(np.diff(control_mode['airborne'])) < 0.5:
            airborne_annotations.append(
                (in_air_transition_time, 'ground to air transition not detected'))
        else:
            airborne_annotations.append(
                (in_air_transition_time, 'in-air at {:.1f} sec'.format(in_air_transition_time)))

        data_plot = ControlModeSummaryPlot(
            status_time, control_mode, [['airborne'], ['estimating_wind']],
            x_label='time (sec)', y_labels=['airborne', 'estimating wind'], annotation_text=[[], []],
            additional_annotation=[airborne_annotations, []],
            plot_title='EKF Control Status - Figure B', pdf_handle=pdf_pages)
        data_plot.save()

        # plot innovation_check_flags summary
        data_plot = CheckFlagsPlot(
            status_time, innov_flags, [['vel_innov_fail', 'posh_innov_fail'], ['posv_innov_fail',
                                                                               'hagl_innov_fail'],
                                       ['magx_innov_fail', 'magy_innov_fail', 'magz_innov_fail',
                                        'yaw_innov_fail'], ['tas_innov_fail'], ['sli_innov_fail'],
                                       ['ofx_innov_fail',
                                        'ofy_innov_fail']], x_label='time (sec)',
            y_labels=['failed', 'failed', 'failed', 'failed', 'failed', 'failed'],
            y_lim=(-0.1, 1.1),
            legend=[['vel NED', 'pos NE'], ['hgt absolute', 'hgt above ground'],
                    ['mag_x', 'mag_y', 'mag_z', 'yaw'], ['airspeed'], ['sideslip'],
                    ['flow X', 'flow Y']],
            plot_title='EKF Innovation Test Fails', annotate=False, pdf_handle=pdf_pages)
        data_plot.save()

        # gps_check_fail_flags summary
        data_plot = CheckFlagsPlot(
            status_time, gps_fail_flags,
            [['nsat_fail', 'gdop_fail', 'herr_fail', 'verr_fail', 'gfix_fail', 'serr_fail'],
             ['hdrift_fail', 'vdrift_fail', 'hspd_fail', 'veld_diff_fail']],
            x_label='time (sec)', y_lim=(-0.1, 1.1), y_labels=['failed', 'failed'],
            sub_titles=['GPS Direct Output Check Failures', 'GPS Derived Output Check Failures'],
            legend=[['N sats', 'GDOP', 'horiz pos error', 'vert pos error', 'fix type',
                     'speed error'], ['horiz drift', 'vert drift', 'horiz speed',
                                      'vert vel inconsistent']], annotate=False, pdf_handle=pdf_pages)
        data_plot.save()

        # filter reported accuracy
        data_plot = CheckFlagsPlot(
            status_time, estimator_status, [['pos_horiz_accuracy', 'pos_vert_accuracy']],
            x_label='time (sec)', y_labels=['accuracy (m)'], plot_title='Reported Accuracy',
            legend=[['horizontal', 'vertical']], annotate=False, pdf_handle=pdf_pages)
        data_plot.save()

        # Plot the EKF IMU vibration metrics
        scaled_estimator_status = {'vibe[0]': 1000. * estimator_status['vibe[0]'],
                                   'vibe[1]': 1000. * estimator_status['vibe[1]'],
                                   'vibe[2]': estimator_status['vibe[2]']
                                   }
        data_plot = CheckFlagsPlot(
            status_time, scaled_estimator_status, [['vibe[0]'], ['vibe[1]'], ['vibe[2]']],
            x_label='time (sec)', y_labels=['Del Ang Coning (mrad)', 'HF Del Ang (mrad)',
                                            'HF Del Vel (m/s)'], plot_title='IMU Vibration Metrics',
            pdf_handle=pdf_pages)
        data_plot.save()

        # Plot the EKF output observer tracking errors
        scaled_innovations = {
            'output_tracking_error[0]': 1000. * ekf2_innovations['output_tracking_error[0]'],
            'output_tracking_error[1]': ekf2_innovations['output_tracking_error[1]'],
            'output_tracking_error[2]': ekf2_innovations['output_tracking_error[2]']
            }
        data_plot = CheckFlagsPlot(
            1e-6 * ekf2_innovations['timestamp'], scaled_innovations,
            [['output_tracking_error[0]'], ['output_tracking_error[1]'], ['output_tracking_error[2]']],
            x_label='time (sec)', y_labels=['angles (mrad)', 'velocity (m/s)', 'position (m)'],
            plot_title='Output Observer Tracking Error Magnitudes', pdf_handle=pdf_pages)
        data_plot.plot()

        # Plot the delta angle bias estimates
        data_plot = CheckFlagsPlot(
            1e-6 * estimator_status['timestamp'], estimator_status,
            [['states[10]'], ['states[11]'], ['states[12]']],
            x_label='time (sec)', y_labels=['X (rad)', 'Y (rad)', 'Z (rad)'],
            plot_title='Delta Angle Bias Estimates', annotate=False, pdf_handle=pdf_pages)
        data_plot.save()

        # Plot the delta velocity bias estimates
        data_plot = CheckFlagsPlot(
            1e-6 * estimator_status['timestamp'], estimator_status,
            [['states[13]'], ['states[14]'], ['states[15]']],
            x_label='time (sec)', y_labels=['X (m/s)', 'Y (m/s)', 'Z (m/s)'],
            plot_title='Delta Velocity Bias Estimates', annotate=False, pdf_handle=pdf_pages)
        data_plot.save()

        # Plot the earth frame magnetic field estimates
        declination, field_strength, inclination = magnetic_field_estimates_from_status(
            estimator_status)
        data_plot = CheckFlagsPlot(
            1e-6 * estimator_status['timestamp'],
            {'strength': field_strength, 'declination': declination, 'inclination': inclination},
            [['declination'], ['inclination'], ['strength']],
            x_label='time (sec)', y_labels=['declination (deg)', 'inclination (deg)',
                                            'strength (Gauss)'],
            plot_title='Earth Magnetic Field Estimates', annotate=False,
            pdf_handle=pdf_pages)
        data_plot.save()

        # Plot the body frame magnetic field estimates
        data_plot = CheckFlagsPlot(
            1e-6 * estimator_status['timestamp'], estimator_status,
            [['states[19]'], ['states[20]'], ['states[21]']],
            x_label='time (sec)', y_labels=['X (Gauss)', 'Y (Gauss)', 'Z (Gauss)'],
            plot_title='Magnetometer Bias Estimates', annotate=False, pdf_handle=pdf_pages)
        data_plot.save()

        # Plot the EKF wind estimates
        data_plot = CheckFlagsPlot(
            1e-6 * estimator_status['timestamp'], estimator_status,
            [['states[22]'], ['states[23]']], x_label='time (sec)',
            y_labels=['North (m/s)', 'East (m/s)'], plot_title='Wind Velocity Estimates',
            annotate=False, pdf_handle=pdf_pages)
        data_plot.save()