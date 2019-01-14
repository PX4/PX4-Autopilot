import numpy as np

# matplotlib don't use Xwindows backend (must be before pyplot import)
import matplotlib
matplotlib.use('Agg')

import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

def plot_timeseries(pdf_pages, dataset, variables, title, xlabels, ylabels):
    plt.figure(figsize=(20, 13))
    plt.title(title)
    for i in range(len(variables)):
        plt.subplot(2, 1, i+1)
        for v in variables[i]:
            plt.plot(dataset[v], 'b')
        plt.xlabel(xlabels[i])
        plt.ylabel(ylabels[i])
    pdf_pages.savefig()
    plt.close()

def get_min_arg_time_value(time_series_data, data_time):
    min_arg = np.argmin(time_series_data)
    min_time = data_time[min_arg]
    min_value = np.amin(time_series_data)
    return min_arg, min_value, min_time

def get_max_arg_time_value(time_series_data, data_time):
    max_arg = np.argmax(time_series_data)
    max_time = data_time[max_arg]
    max_value = np.amax(time_series_data)
    return max_arg, max_value, max_time

def plot_innovations(pdf_pages, dataset, variables, variances, titles, xlabels, ylabels):

    plt.figure(figsize=(20, 13))
    # if there is one title per variable, every subplot gets its own title, otherwise a single title is set
    if len(titles) == 1:
        plt.suptitle(titles[0])
    for i in range(len(variables)):
        # create a subplot for every variable
        plt.subplot(len(variables), 1, i+1)
        if len(titles) > 1:
            plt.title(titles[i])

        # plot the value and the standard deviation as a variable
        plt.plot(1e-6 * dataset['timestamp'], dataset[variables[i]], 'b')
        plt.plot(1e-6 * dataset['timestamp'], np.sqrt(dataset[variances[i]]), 'r')
        plt.plot(1e-6 * dataset['timestamp'], -np.sqrt(dataset[variances[i]]), 'r')

        plt.xlabel(xlabels[i])
        plt.ylabel(ylabels[i])
        plt.grid()

        # add the maximum and minimum value as an annotation
        _, max_value, max_time = get_max_arg_time_value(dataset[variables[i]], 1e-6 * dataset['timestamp'])
        _, min_value, min_time = get_min_arg_time_value(dataset[variables[i]], 1e-6 * dataset['timestamp'])

        plt.text(
            max_time, max_value, 'max={:.2f}'.format(max_value), fontsize=12, horizontalalignment='left',
            verticalalignment='bottom')
        plt.text(
            min_time, min_value, 'min={:.2f}'.format(min_value), fontsize=12, horizontalalignment='left',
            verticalalignment='top')

    if pdf_pages is not None:
        pdf_pages.savefig()
        plt.close()

def plot_check_flags(
        pdf_pages, dataset, data_time, variables, titles, xlabel, ylabels, ylim=None, legend=None, annotate=True):

    colors = ['b', 'r', 'g', 'c', 'k', 'm']

    plt.figure(figsize=(20, 13))
    # if there is one title per variable, every subplot gets its own title, otherwise a single title is set
    if len(titles) == 1:
        plt.suptitle(titles[0])

    for i in range(len(variables)):
        # create a subplot for every variable
        plt.subplot(len(variables), 1, i + 1)
        if len(titles) > 1:
            plt.title(titles[i])

        for col, var in zip(colors[:len(variables[i])], variables[i]):
            plt.plot(data_time, dataset[var], col)

        plt.xlabel(xlabel)
        plt.ylabel(ylabels[i])
        plt.grid()
        if ylim is not None:
            plt.ylim(ylim)

        if legend is not None:
            plt.legend(legend[i], loc='upper left')

        if annotate:
            for var in variables[i]:
                # add the maximum and minimum value as an annotation
                _, max_value, max_time = get_max_arg_time_value(dataset[var], data_time)
                mean_value = np.mean(dataset[var])

                plt.text(
                    max_time, max_value, 'max={:.4f}, mean={:.4f}'.format(max_value, mean_value), fontsize=12,
                    horizontalalignment='left', verticalalignment='bottom')

    if pdf_pages is not None:
        pdf_pages.savefig()
        plt.close()

def control_mode_summary_plot(pdf_pages, dataset, data_time, variables, titles, xlabel, ylabels, annotation_text,
                              additional_annotation=None):

    colors = ['b', 'r', 'g', 'c']

    plt.figure(figsize=(20, 13))
    # if there is one title per variable, every subplot gets its own title, otherwise a single title is set
    if len(titles) == 1:
        plt.suptitle(titles[0])

    for i in range(len(variables)):
        # create a subplot for every variable
        plt.subplot(len(variables), 1, i + 1)
        if len(titles) > 1:
            plt.title(titles[i])

        for col, var in zip(colors[:len(variables[i])], variables[i]):
            plt.plot(data_time, dataset[var], col)

        plt.xlabel(xlabel)
        plt.ylabel(ylabels[i])
        plt.grid()
        plt.ylim(-0.1, 1.1)

        for t in range(len(annotation_text[i])):

            _, _, align_time = get_max_arg_time_value(np.diff(dataset[variables[i][t]]), data_time)
            v_annot_pos = (t+1.0)/(len(variables[i])+1) # vertical annotation position

            if np.amin(dataset[variables[i][t]]) > 0:
                plt.text(
                    align_time, v_annot_pos,
                    'no pre-arm data - cannot calculate {:s} start time'.format(annotation_text[i][t]),
                    fontsize=12, horizontalalignment='left', verticalalignment='center', color=colors[t])
            elif np.amax(dataset[variables[i][t]]) > 0:
                plt.text(
                    align_time, v_annot_pos, '{:s} at {:.1f} sec'.format(
                        annotation_text[i][t], align_time), fontsize=12, horizontalalignment='left',
                    verticalalignment='center', color=colors[t])

        if additional_annotation is not None:
            for a in range(len(additional_annotation[i])):
                v_annot_pos = (a + 1.0) / (len(additional_annotation[i]) + 1)  # vertical annotation position
                plt.text(
                    additional_annotation[i][a][0], v_annot_pos, additional_annotation[i][a][1], fontsize=12,
                    horizontalalignment='left', verticalalignment='center', color='b')

    if pdf_pages is not None:
        pdf_pages.savefig()
        plt.close()

def get_control_mode_flags(estimator_status):

    control_mode = dict()
    # extract control mode metadata from estimator_status.control_mode_flags
    # 0 - true if the filter tilt alignment is complete
    # 1 - true if the filter yaw alignment is complete
    # 2 - true if GPS measurements are being fused
    # 3 - true if optical flow measurements are being fused
    # 4 - true if a simple magnetic yaw heading is being fused
    # 5 - true if 3-axis magnetometer measurement are being fused
    # 6 - true if synthetic magnetic declination measurements are being fused
    # 7 - true when the vehicle is airborne
    # 8 - true when wind velocity is being estimated
    # 9 - true when baro height is being fused as a primary height reference
    # 10 - true when range finder height is being fused as a primary height reference
    # 11 - true when range finder height is being fused as a primary height reference
    # 12 - true when local position data from external vision is being fused
    # 13 - true when yaw data from external vision measurements is being fused
    # 14 - true when height data from external vision measurements is being fused
    control_mode['tilt_aligned'] = ((2 ** 0 & estimator_status['control_mode_flags']) > 0) * 1
    control_mode['yaw_aligned'] = ((2 ** 1 & estimator_status['control_mode_flags']) > 0) * 1
    control_mode['using_gps'] = ((2 ** 2 & estimator_status['control_mode_flags']) > 0) * 1
    control_mode['using_optflow'] = ((2 ** 3 & estimator_status['control_mode_flags']) > 0) * 1
    control_mode['using_magyaw'] = ((2 ** 4 & estimator_status['control_mode_flags']) > 0) * 1
    control_mode['using_mag3d'] = ((2 ** 5 & estimator_status['control_mode_flags']) > 0) * 1
    control_mode['using_magdecl'] = ((2 ** 6 & estimator_status['control_mode_flags']) > 0) * 1
    control_mode['airborne'] = ((2 ** 7 & estimator_status['control_mode_flags']) > 0) * 1
    control_mode['estimating_wind'] = ((2 ** 8 & estimator_status['control_mode_flags']) > 0) * 1
    control_mode['using_barohgt'] = ((2 ** 9 & estimator_status['control_mode_flags']) > 0) * 1
    control_mode['using_rnghgt'] = ((2 ** 10 & estimator_status['control_mode_flags']) > 0) * 1
    control_mode['using_gpshgt'] = ((2 ** 11 & estimator_status['control_mode_flags']) > 0) * 1
    control_mode['using_evpos'] = ((2 ** 12 & estimator_status['control_mode_flags']) > 0) * 1
    control_mode['using_evyaw'] = ((2 ** 13 & estimator_status['control_mode_flags']) > 0) * 1
    control_mode['using_evhgt'] = ((2 ** 14 & estimator_status['control_mode_flags']) > 0) * 1
    return control_mode

def get_estimator_check_flags(estimator_status):
    control_mode = get_control_mode_flags(estimator_status)
    innov_flags = get_innovation_check_flags(estimator_status)
    gps_fail_flags = get_gps_check_fail_flags(estimator_status)
    return control_mode, innov_flags, gps_fail_flags

def analyse_ekf(estimator_status, ekf2_innovations, sensor_preflight, check_levels,
                plot=False, output_plot_filename=None, late_start_early_ending=True):

    control_mode, innov_flags, gps_fail_flags = get_estimator_check_flags(estimator_status)

    innov_time = 1e-6 * ekf2_innovations['timestamp']
    status_time = 1e-6 * estimator_status['timestamp']

    b_finishes_in_air, b_starts_in_air, in_air_duration, in_air_transition_time, on_ground_transition_time = detect_airtime(
        control_mode, status_time)

    if plot:
        # create summary plots
        # save the plots to PDF
        pdf_pages = PdfPages(output_plot_filename)
        # plot IMU consistency data
        if ('accel_inconsistency_m_s_s' in sensor_preflight.keys()) and (
                'gyro_inconsistency_rad_s' in sensor_preflight.keys()):
            plot_timeseries(
                pdf_pages, sensor_preflight, [['accel_inconsistency_m_s_s'], ['gyro_inconsistency_rad_s']],
                title='IMU Consistency Check Levels', xlabels=['data index', 'data index'],
                ylabels=['acceleration (m/s/s)', 'angular rate (rad/s)'])

        # vertical velocity and position innovations
        plot_innovations(
            pdf_pages, ekf2_innovations, ['vel_pos_innov[2]', 'vel_pos_innov[5]'],
            ['vel_pos_innov_var[2]', 'vel_pos_innov_var[5]'], titles=['Vertical Innovations'],
            xlabels=['time (sec)', 'time (sec)'], ylabels=['Down Vel (m/s)', 'Down Pos (m)'])

        # horizontal velocity innovations
        plot_innovations(
            pdf_pages, ekf2_innovations, ['vel_pos_innov[0]', 'vel_pos_innov[1]'],
            ['vel_pos_innov_var[0]', 'vel_pos_innov_var[1]'], titles=['Horizontal Velocity  Innovations'],
            xlabels=['time (sec)', 'time (sec)'], ylabels=['North Vel (m/s)', 'East Vel (m/s)'])

        # horizontal position innovations
        plot_innovations(
            pdf_pages, ekf2_innovations, ['vel_pos_innov[3]', 'vel_pos_innov[4]'],
            ['vel_pos_innov_var[3]', 'vel_pos_innov_var[4]'], titles=['Horizontal Position Innovations'],
            xlabels=['time (sec)', 'time (sec)'], ylabels=['North Pos (m)', 'East Pos (m)'])

        # magnetometer innovations
        plot_innovations(
            pdf_pages, ekf2_innovations, ['mag_innov[0]', 'mag_innov[1]', 'mag_innov[2]'],
            ['mag_innov_var[0]', 'mag_innov_var[1]', 'mag_innov_var[2]'], titles=['Magnetometer Innovations'],
            xlabels=['time (sec)', 'time (sec)', 'time (sec)'], ylabels=['X (Gauss)', 'Y (Gauss)', 'Z (Gauss)'])

        # magnetic heading innovations
        plot_innovations(
            pdf_pages, ekf2_innovations, ['heading_innov'], ['heading_innov_var'],
            titles=['Magnetic Heading Innovations'], xlabels=['time (sec)'], ylabels=['Heading (rad)'])

        # air data innovations
        plot_innovations(
            pdf_pages, ekf2_innovations, ['airspeed_innov', 'beta_innov'], ['airspeed_innov_var', 'beta_innov_var'],
            titles=['True Airspeed Innovations', 'Synthetic Sideslip Innovations'],
            xlabels=['time (sec)', 'time (sec)'], ylabels=['innovation (m/sec)', 'innovation (rad)'])

        # optical flow innovations
        plot_innovations(
            pdf_pages, ekf2_innovations, ['flow_innov[0]', 'flow_innov[1]'],
            ['flow_innov_var[0]', 'flow_innov_var[1]'],
            titles=['Optical Flow Innovations'], xlabels=['time (sec)', 'time (sec)'],
            ylabels=['X (rad/sec)', 'Y (rad/sec)'])

        # plot normalised innovation test levels
        # define variables to plot
        variables = [['mag_test_ratio'], ['vel_test_ratio', 'pos_test_ratio'], ['hgt_test_ratio']]
        if np.amax(estimator_status['hagl_test_ratio']) > 0.0:  # plot hagl test ratio, if applicable
            variables[-1].append('hagl_test_ratio')
        ylabels = ['mag', 'vel, pos', 'hgt']

        if np.amax(estimator_status['tas_test_ratio']) > 0.0:  # plot airspeed sensor test ratio, if applicable
            variables.append(['tas_test_ratio'])
            ylabels.append('TAS')

        plot_check_flags(
            pdf_pages, estimator_status, status_time, variables, titles=['Normalised Innovation Test Levels'],
            xlabel='time (sec)', ylabels=ylabels)

        # plot control mode summary A
        control_mode_summary_plot(pdf_pages, control_mode, status_time, [['tilt_aligned', 'yaw_aligned'],
            ['using_gps', 'using_optflow', 'using_evpos'], ['using_barohgt', 'using_gpshgt', 'using_rnghgt', 'using_evhgt'],
             ['using_magyaw', 'using_mag3d', 'using_magdecl']], titles=['EKF Control Status - Figure A'],
            xlabel='time (sec)', ylabels=['aligned', 'pos aiding', 'hgt aiding', 'mag aiding'], annotation_text=[
                ['tilt alignment', 'yaw alignment'], ['GPS aiding', 'optical flow aiding', 'external vision aiding'],
                ['Baro aiding', 'GPS aiding', 'rangefinder aiding', 'external vision aiding'],
                ['magnetic yaw aiding', '3D magnetoemter aiding', 'magnetic declination aiding']])

        # plot control mode summary B
        # construct additional annotations for the airborne plot
        airborne_annotations = list()
        if np.amin(np.diff(control_mode['airborne'])) > -0.5:
            airborne_annotations.append((on_ground_transition_time, 'air to ground transition not detected'))
        else:
            airborne_annotations.append((on_ground_transition_time, 'on-ground at {:.1f} sec'.format(
                on_ground_transition_time)))

        if in_air_duration > 0.0:
            airborne_annotations.append(((in_air_transition_time + on_ground_transition_time) / 2,
                                         'duration = {:.1f} sec'.format(in_air_duration)))

        if np.amax(np.diff(control_mode['airborne'])) < 0.5:
            airborne_annotations.append((in_air_transition_time, 'ground to air transition not detected'))
        else:
            airborne_annotations.append((in_air_transition_time, 'in-air at {:.1f} sec'.format(in_air_transition_time)))

        control_mode_summary_plot(
            pdf_pages, control_mode, status_time, [['airborne'], ['estimating_wind']],
            titles=['EKF Control Status - Figure B'], xlabel='time (sec)',
            ylabels=['airborne', 'estimating wind'], annotation_text=[[], []],
            additional_annotation=[airborne_annotations, []])

    # generate metadata for the normalised innovation consistency test levels
    # a value > 1.0 means the measurement data for that test has been rejected by the EKF
    # magnetometer data
    mag_test_max_arg = np.argmax(estimator_status['mag_test_ratio'])
    mag_test_max_time = status_time[mag_test_max_arg]
    mag_test_max = np.amax(estimator_status['mag_test_ratio'])
    mag_test_mean = np.mean(estimator_status['mag_test_ratio'])
    # velocity data (GPS)
    vel_test_max_arg = np.argmax(estimator_status['vel_test_ratio'])
    vel_test_max_time = status_time[vel_test_max_arg]
    vel_test_max = np.amax(estimator_status['vel_test_ratio'])
    vel_test_mean = np.mean(estimator_status['vel_test_ratio'])
    # horizontal position data (GPS or external vision)
    pos_test_max_arg = np.argmax(estimator_status['pos_test_ratio'])
    pos_test_max_time = status_time[pos_test_max_arg]
    pos_test_max = np.amax(estimator_status['pos_test_ratio'])
    pos_test_mean = np.mean(estimator_status['pos_test_ratio'])
    # height data (Barometer, GPS or rangefinder)
    hgt_test_max_arg = np.argmax(estimator_status['hgt_test_ratio'])
    hgt_test_max_time = status_time[hgt_test_max_arg]
    hgt_test_max = np.amax(estimator_status['hgt_test_ratio'])
    hgt_test_mean = np.mean(estimator_status['hgt_test_ratio'])
    # airspeed data
    tas_test_max_arg = np.argmax(estimator_status['tas_test_ratio'])
    tas_test_max_time = status_time[tas_test_max_arg]
    tas_test_max = np.amax(estimator_status['tas_test_ratio'])
    tas_test_mean = np.mean(estimator_status['tas_test_ratio'])
    # height above ground data (rangefinder)
    hagl_test_max_arg = np.argmax(estimator_status['hagl_test_ratio'])
    hagl_test_max_time = status_time[hagl_test_max_arg]
    hagl_test_max = np.amax(estimator_status['hagl_test_ratio'])
    hagl_test_mean = np.mean(estimator_status['hagl_test_ratio'])

    # TODO: continue here

    # calculate alignment completion times
    tilt_align_time_arg, _, tilt_align_time = get_max_arg_time_value(np.diff(control_mode['tilt_aligned']), status_time)

    tilt_align_time_arg = np.argmax(np.diff(control_mode['tilt_aligned']))
    tilt_align_time = status_time[tilt_align_time_arg]
    yaw_align_time_arg = np.argmax(np.diff(control_mode['yaw_aligned']))
    yaw_align_time = status_time[yaw_align_time_arg]
    # calculate position aiding start times
    gps_aid_time_arg = np.argmax(np.diff(control_mode['using_gps']))
    gps_aid_time = status_time[gps_aid_time_arg]
    optflow_aid_time_arg = np.argmax(np.diff(control_mode['using_optflow']))
    optflow_aid_time = status_time[optflow_aid_time_arg]
    evpos_aid_time_arg = np.argmax(np.diff(control_mode['using_evpos']))
    evpos_aid_time = status_time[evpos_aid_time_arg]
    # calculate height aiding start times
    barohgt_aid_time_arg = np.argmax(np.diff(control_mode['using_barohgt']))
    barohgt_aid_time = status_time[barohgt_aid_time_arg]
    gpshgt_aid_time_arg = np.argmax(np.diff(control_mode['using_gpshgt']))
    gpshgt_aid_time = status_time[gpshgt_aid_time_arg]
    rnghgt_aid_time_arg = np.argmax(np.diff(control_mode['using_rnghgt']))
    rnghgt_aid_time = status_time[rnghgt_aid_time_arg]
    evhgt_aid_time_arg = np.argmax(np.diff(control_mode['using_evhgt']))
    evhgt_aid_time = status_time[evhgt_aid_time_arg]
    # calculate magnetometer aiding start times
    using_magyaw_time_arg = np.argmax(np.diff(control_mode['using_magyaw']))
    using_magyaw_time = status_time[using_magyaw_time_arg]
    using_mag3d_time_arg = np.argmax(np.diff(control_mode['using_mag3d']))
    using_mag3d_time = status_time[using_mag3d_time_arg]
    using_magdecl_time_arg = np.argmax(np.diff(control_mode['using_magdecl']))
    using_magdecl_time = status_time[using_magdecl_time_arg]

    if plot:

        # plot innovation_check_flags summary
        plot_check_flags(
            pdf_pages, innov_flags, status_time, [['vel_innov_fail', 'posh_innov_fail'], ['posv_innov_fail',
            'hagl_innov_fail'], ['magx_innov_fail', 'magy_innov_fail', 'magz_innov_fail', 'yaw_innov_fail'],
            ['tas_innov_fail'], ['sli_innov_fail'], ['ofx_innov_fail', 'ofy_innov_fail']],
            titles=['EKF Innovation Test Fails'], xlabel='time (sec)', ylim=(-0.1, 1.1),
            ylabels=['failed', 'failed', 'failed', 'failed', 'failed', 'failed'],
            legend=[['vel NED', 'pos NE'], ['hgt absolute', 'hgt above ground'], ['mag_x', 'mag_y', 'mag_z', 'yaw'],
                    ['airspeed'], ['sideslip'], ['flow X', 'flow Y']], annotate=False)

        # gps_check_fail_flags summary
        plot_check_flags(
            pdf_pages, gps_fail_flags, status_time,
            [['nsat_fail', 'gdop_fail', 'herr_fail', 'verr_fail', 'gfix_fail', 'serr_fail'],
             ['hdrift_fail', 'vdrift_fail', 'hspd_fail', 'veld_diff_fail']],
            titles=['GPS Direct Output Check Failures', 'GPS Derived Output Check Failures'], xlabel='time (sec)',
            ylim=(-0.1, 1.1), ylabels=['failed', 'failed'],
            legend=[['N sats', 'GDOP', 'horiz pos error', 'vert pos error', 'fix type', 'speed error'],
                    ['horiz drift', 'vert drift', 'horiz speed', 'vert vel inconsistent']], annotate=False)



        # filter reported accuracy
        plot_check_flags(
            pdf_pages, estimator_status, status_time, [['pos_horiz_accuracy', 'pos_vert_accuracy']],
            titles=['Reported Accuracy'], xlabel='time (sec)', ylabels=['accuracy (m)'],
            legend=[['horizontal', 'vertical']], annotate=False)

        # Plot the EKF IMU vibration metrics
        scaled_estimator_status = {'vibe[0]': 1000.* estimator_status['vibe[0]'],
                                   'vibe[1]': 1000.* estimator_status['vibe[1]'],
                                   'vibe[2]': estimator_status['vibe[2]']
                                    }

        plot_check_flags(
            pdf_pages, scaled_estimator_status, status_time, [['vibe[0]'], ['vibe[1]'], ['vibe[2]']],
            titles=['IMU Vibration Metrics'], xlabel='time (sec)',
            ylabels=['Del Ang Coning (mrad)', 'HF Del Ang (mrad)', 'HF Del Vel (m/s)'])

        # Plot the EKF output observer tracking errors

        scaled_innovations = {'output_tracking_error[0]': 1000.* ekf2_innovations['output_tracking_error[0]'],
                              'output_tracking_error[1]': ekf2_innovations['output_tracking_error[1]'],
                              'output_tracking_error[2]': ekf2_innovations['output_tracking_error[2]']
                              }

        plot_check_flags(
            pdf_pages, scaled_innovations, 1e-6 * ekf2_innovations['timestamp'],
            [['output_tracking_error[0]'], ['output_tracking_error[1]'], ['output_tracking_error[2]']],
            titles=['Output Observer Tracking Error Magnitudes'], xlabel='time (sec)',
            ylabels=['angles (mrad)', 'velocity (m/s)', 'position (m)'])

        # Plot the delta angle bias estimates
        plot_check_flags(
            pdf_pages, estimator_status, 1e-6 * estimator_status['timestamp'],
            [['states[10]'], ['states[11]'], ['states[12]']],
            titles=['Delta Angle Bias Estimates'], xlabel='time (sec)',
            ylabels=['X (rad)', 'Y (rad)', 'Z (rad)'], annotate=False)

        plt.figure(16, figsize=(20, 13))
        plt.subplot(3, 1, 1)
        plt.plot(1e-6 * estimator_status['timestamp'], estimator_status['states[10]'], 'b')
        plt.title('Delta Angle Bias Estimates')
        plt.ylabel('X (rad)')
        plt.xlabel('time (sec)')
        plt.grid()
        plt.subplot(3, 1, 2)
        plt.plot(1e-6 * estimator_status['timestamp'], estimator_status['states[11]'], 'b')
        plt.ylabel('Y (rad)')
        plt.xlabel('time (sec)')
        plt.grid()
        plt.subplot(3, 1, 3)
        plt.plot(1e-6 * estimator_status['timestamp'], estimator_status['states[12]'], 'b')
        plt.ylabel('Z (rad)')
        plt.xlabel('time (sec)')
        plt.grid()
        pdf_pages.savefig()
        plt.close(16)
        # Plot the delta velocity bias estimates
        plot_check_flags(
            pdf_pages, estimator_status, 1e-6 * estimator_status['timestamp'],
            [['states[13]'], ['states[14]'], ['states[15]']],
            titles=['Delta Velocity Bias Estimates'], xlabel='time (sec)',
            ylabels=['X (m/s)', 'Y (m/s)', 'Z (m/s)'], annotate=False)

        plt.figure(17, figsize=(20, 13))
        plt.subplot(3, 1, 1)
        plt.plot(1e-6 * estimator_status['timestamp'], estimator_status['states[13]'], 'b')
        plt.title('Delta Velocity Bias Estimates')
        plt.ylabel('X (m/s)')
        plt.xlabel('time (sec)')
        plt.grid()
        plt.subplot(3, 1, 2)
        plt.plot(1e-6 * estimator_status['timestamp'], estimator_status['states[14]'], 'b')
        plt.ylabel('Y (m/s)')
        plt.xlabel('time (sec)')
        plt.grid()
        plt.subplot(3, 1, 3)
        plt.plot(1e-6 * estimator_status['timestamp'], estimator_status['states[15]'], 'b')
        plt.ylabel('Z (m/s)')
        plt.xlabel('time (sec)')
        plt.grid()
        pdf_pages.savefig()
        plt.close(17)

        # Plot the earth frame magnetic field estimates
        rad2deg = 57.2958
        field_strength = np.sqrt(
            estimator_status['states[16]'] ** 2 + estimator_status['states[17]'] ** 2 +
            estimator_status['states[18]'] ** 2)
        declination = rad2deg * np.arctan2(estimator_status['states[17]'], estimator_status['states[16]'])
        inclination = rad2deg * np.arcsin(
            estimator_status['states[18]'] / np.maximum(field_strength, np.finfo(np.float32).eps))

        plot_check_flags(
            pdf_pages, {'strength': field_strength, 'declination': declination, 'inclination': inclination},
            1e-6 * estimator_status['timestamp'], [['declination'], ['inclination'], ['strength']],
            titles=['Earth Magnetic Field Estimates'], xlabel='time (sec)',
            ylabels=['declination (deg)', 'inclination (deg)', 'strength (Gauss)'], annotate=False)

        plt.figure(18, figsize=(20, 13))
        plt.subplot(3, 1, 3)

        plt.plot(1e-6 * estimator_status['timestamp'], strength, 'b')
        plt.ylabel('strength (Gauss)')
        plt.xlabel('time (sec)')
        plt.grid()
        plt.subplot(3, 1, 1)

        plt.plot(1e-6 * estimator_status['timestamp'], declination, 'b')
        plt.title('Earth Magnetic Field Estimates')
        plt.ylabel('declination (deg)')
        plt.xlabel('time (sec)')
        plt.grid()
        plt.subplot(3, 1, 2)
        plt.plot(1e-6 * estimator_status['timestamp'], inclination, 'b')
        plt.ylabel('inclination (deg)')
        plt.xlabel('time (sec)')
        plt.grid()
        pdf_pages.savefig()
        plt.close(18)

        # Plot the body frame magnetic field estimates
        plot_check_flags(
            pdf_pages, estimator_status, 1e-6 * estimator_status['timestamp'],
            [['states[19]'], ['states[20]'], ['states[21]']],
            titles=['Magnetomer Bias Estimates'], xlabel='time (sec)',
            ylabels=['X (Gauss)', 'Y (Gauss)', 'Z (Gauss)'], annotate=False)


        plt.figure(19, figsize=(20, 13))
        plt.subplot(3, 1, 1)
        plt.plot(1e-6 * estimator_status['timestamp'], estimator_status['states[19]'], 'b')
        plt.title('Magnetomer Bias Estimates')
        plt.ylabel('X (Gauss)')
        plt.xlabel('time (sec)')
        plt.grid()
        plt.subplot(3, 1, 2)
        plt.plot(1e-6 * estimator_status['timestamp'], estimator_status['states[20]'], 'b')
        plt.ylabel('Y (Gauss)')
        plt.xlabel('time (sec)')
        plt.grid()
        plt.subplot(3, 1, 3)
        plt.plot(1e-6 * estimator_status['timestamp'], estimator_status['states[21]'], 'b')
        plt.ylabel('Z (Gauss)')
        plt.xlabel('time (sec)')
        plt.grid()
        pdf_pages.savefig()
        plt.close(19)

        # Plot the EKF wind estimates
        plt.figure(20, figsize=(20, 13))
        plt.subplot(2, 1, 1)
        plt.plot(1e-6 * estimator_status['timestamp'], estimator_status['states[22]'], 'b')
        plt.title('Wind Velocity Estimates')
        plt.ylabel('North (m/s)')
        plt.xlabel('time (sec)')
        plt.grid()
        plt.subplot(2, 1, 2)
        plt.plot(1e-6 * estimator_status['timestamp'], estimator_status['states[23]'], 'b')
        plt.ylabel('East (m/s)')
        plt.xlabel('time (sec)')
        plt.grid()
        pdf_pages.savefig()
        plt.close(20)
        # close the pdf file
        pdf_pages.close()
        # don't display to screen
        # plt.show()
        # clase all figures
        plt.close("all")

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


def get_gps_check_fail_flags(estimator_status):
    gps_fail_flags = dict()

    # 0 : insufficient fix type (no 3D solution)
    # 1 : minimum required sat count fail
    # 2 : minimum required GDoP fail
    # 3 : maximum allowed horizontal position error fail
    # 4 : maximum allowed vertical position error fail
    # 5 : maximum allowed speed error fail
    # 6 : maximum allowed horizontal position drift fail
    # 7 : maximum allowed vertical position drift fail
    # 8 : maximum allowed horizontal speed fail
    # 9 : maximum allowed vertical velocity discrepancy fail
    gps_fail_flags['gfix_fail'] = ((2 ** 0 & estimator_status['gps_check_fail_flags']) > 0) * 1
    gps_fail_flags['nsat_fail'] = ((2 ** 1 & estimator_status['gps_check_fail_flags']) > 0) * 1
    gps_fail_flags['gdop_fail'] = ((2 ** 2 & estimator_status['gps_check_fail_flags']) > 0) * 1
    gps_fail_flags['herr_fail'] = ((2 ** 3 & estimator_status['gps_check_fail_flags']) > 0) * 1
    gps_fail_flags['verr_fail'] = ((2 ** 4 & estimator_status['gps_check_fail_flags']) > 0) * 1
    gps_fail_flags['serr_fail'] = ((2 ** 5 & estimator_status['gps_check_fail_flags']) > 0) * 1
    gps_fail_flags['hdrift_fail'] = ((2 ** 6 & estimator_status['gps_check_fail_flags']) > 0) * 1
    gps_fail_flags['vdrift_fail'] = ((2 ** 7 & estimator_status['gps_check_fail_flags']) > 0) * 1
    gps_fail_flags['hspd_fail'] = ((2 ** 8 & estimator_status['gps_check_fail_flags']) > 0) * 1
    gps_fail_flags['veld_diff_fail'] = ((2 ** 9 & estimator_status['gps_check_fail_flags']) > 0) * 1
    return gps_fail_flags


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
            in_air_duration = on_ground_transition_time - in_air_transition_time;
        else:
            in_air_duration = float('NaN')
    else:
        in_air_duration = float('NaN')
    return b_finishes_in_air, b_starts_in_air, in_air_duration, in_air_transition_time, on_ground_transition_time


def get_innovation_check_flags(estimator_status):

    innov_flags = dict()
    # innovation_check_flags summary
    # 0 - true if velocity observations have been rejected
    # 1 - true if horizontal position observations have been rejected
    # 2 - true if true if vertical position observations have been rejected
    # 3 - true if the X magnetometer observation has been rejected
    # 4 - true if the Y magnetometer observation has been rejected
    # 5 - true if the Z magnetometer observation has been rejected
    # 6 - true if the yaw observation has been rejected
    # 7 - true if the airspeed observation has been rejected
    # 8 - true if synthetic sideslip observation has been rejected
    # 9 - true if the height above ground observation has been rejected
    # 10 - true if the X optical flow observation has been rejected
    # 11 - true if the Y optical flow observation has been rejected
    innov_flags['vel_innov_fail'] = ((2 ** 0 & estimator_status['innovation_check_flags']) > 0) * 1
    innov_flags['posh_innov_fail'] = ((2 ** 1 & estimator_status['innovation_check_flags']) > 0) * 1
    innov_flags['posv_innov_fail'] = ((2 ** 2 & estimator_status['innovation_check_flags']) > 0) * 1
    innov_flags['magx_innov_fail'] = ((2 ** 3 & estimator_status['innovation_check_flags']) > 0) * 1
    innov_flags['magy_innov_fail'] = ((2 ** 4 & estimator_status['innovation_check_flags']) > 0) * 1
    innov_flags['magz_innov_fail'] = ((2 ** 5 & estimator_status['innovation_check_flags']) > 0) * 1
    innov_flags['yaw_innov_fail'] = ((2 ** 6 & estimator_status['innovation_check_flags']) > 0) * 1
    innov_flags['tas_innov_fail'] = ((2 ** 7 & estimator_status['innovation_check_flags']) > 0) * 1
    innov_flags['sli_innov_fail'] = ((2 ** 8 & estimator_status['innovation_check_flags']) > 0) * 1
    innov_flags['hagl_innov_fail'] = ((2 ** 9 & estimator_status['innovation_check_flags']) > 0) * 1
    innov_flags['ofx_innov_fail'] = ((2 ** 10 & estimator_status['innovation_check_flags']) > 0) * 1
    innov_flags['ofy_innov_fail'] = ((2 ** 11 & estimator_status['innovation_check_flags']) > 0) * 1
    return innov_flags

