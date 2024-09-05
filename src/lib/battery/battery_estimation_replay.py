# Test internal resistance and flight time estimator on flight logs
# run with:
# python3 battery_estimation_replay.py -f <(required)pathToLogFile>
#   -r <(optional)useLoggedRemainingForFlightTimeEstimation>
#   -c <(optional)#batteryCells>
#   -u <(optional)fullCellVoltage>
#   -e <(optional)emptyCellVoltage>
#   -l <(optional)forgettingFactor>
#   -s <(optional)averageSocConsumption>
#   -t <(optional)remainingFilterTimeConstant>
#   -k <(optional)flightTimeFilterTimeConstant>
# Note: Can lead to slightly different results than the online estimation due to the fact that
# the log frequency of the voltage and current are not the same as the online frequency.

from pyulog import ULog
import matplotlib.pyplot as plt
import numpy as np
import argparse

def getData(log, topic_name, variable_name, instance=0):
    for elem in log.data_list:
        if elem.name == topic_name and instance == elem.multi_id:
            return elem.data[variable_name]
    return np.array([])

def us2s(time_us):
    return time_us * 1e-6

def getParam(log, param_name):
    if param_name in log.initial_parameters:
        return log.initial_parameters[param_name]
    else:
        print(f"Parameter {param_name} not found in log.")
    return None

def alphaFilter(filter_state, alpha, sample):
    return filter_state + alpha * (sample - filter_state)

def updateRLS(theta, P, x, voltage, current, lam):
    gamma = P @ x / (lam + x.T @ P @ x)
    error = voltage - x.T @ theta
    data_cov = x.T @ P @ x
    theta_temp = theta + gamma * error
    P_temp = (P - gamma @ x.T @ P) / lam
    if (abs(np.linalg.norm(P)) < abs(np.linalg.norm(P_temp))):
        theta_corr = np.array([voltage + theta[1] * current, theta[1]]) # Correct OCV estimation
        P_corr = P
        return theta_corr, P_corr, error[0][0], data_cov[0][0], 0, 0
    return theta_temp, P_temp, error[0][0], data_cov[0][0], gamma[0][0], gamma[1][0]

def main(log_name, logged_remaining, n_cells, full_cell, empty_cell, lam, soc_consumption_avg, time_constant_ocv_derivative, time_constant_flight_time):
    log = ULog(log_name)

    log_n_cells    = getParam(log, 'BAT1_N_CELLS')
    log_full_cell  = getParam(log, 'BAT1_V_CHARGED')
    log_empty_cell = getParam(log, 'BAT1_V_EMPTY')
    log_capacity   = getParam(log, 'BAT1_CAPACITY')

    # Debug information
    print("\nParameters:")
    print(f"Extracted from log - BAT1_N_CELLS: {log_n_cells}, BAT1_V_CHARGED: {log_full_cell}, BAT1_V_EMPTY: {log_empty_cell}, BAT1_CAPACITY: {log_capacity}")

    # Use log parameters unless overridden
    if n_cells is None:
        n_cells = log_n_cells
    else:
        print(f"Using override for n_cells: {n_cells}")
    if full_cell is None:
        full_cell = log_full_cell
    else:
        print(f"Using override for full_cell: {full_cell}")
    if empty_cell is None:
        empty_cell = log_empty_cell
    else:
        print(f"Using override for empty_cell: {empty_cell}")

    # Debug information for final parameter values
    print(f"Using parameters - n_cells: {n_cells}, full_cell: {full_cell}, empty_cell: {empty_cell}")

    # Extract data from log
    timestamps            = us2s(getData(log, 'battery_status', 'timestamp'))
    current               =      getData(log, 'battery_status', 'current_a')
    current_average       =      getData(log, 'battery_status', 'current_average_a')
    voltage               =      getData(log, 'battery_status', 'voltage_v')
    remaining             =      getData(log, 'battery_status', 'remaining')
    remaining_flight_time =      getData(log, 'battery_status', 'time_remaining_s')

    if not timestamps.size or not current.size or not current_average.size or not voltage.size or not remaining.size:
        print("Error: Incomplete data.")
        return

    ## Internal resistance estimation

    # Containers for plotting
    ocv_est                    = np.zeros_like(timestamps)
    ocv_est_filtered           = np.zeros_like(timestamps)
    r_est                      = np.zeros_like(timestamps)
    error_hist                 = np.zeros_like(timestamps)
    v_est                      = np.zeros_like(timestamps)
    internal_resistance_stable = np.zeros_like(timestamps)
    cov_norm                   = np.zeros_like(timestamps)
    r_cov                      = np.zeros_like(timestamps)
    ocv_cov                    = np.zeros_like(timestamps)
    mixed_cov                  = np.zeros_like(timestamps)
    data_cov_hist              = np.zeros_like(timestamps)
    gamma_ocv_hist             = np.zeros_like(timestamps)
    gamma_r_hist               = np.zeros_like(timestamps)
    remaining_est              = np.zeros_like(timestamps)
    remaining_est_filtered     = np.zeros_like(timestamps)

    # Initializations
    theta = np.array([[voltage[0] + 0.005 * n_cells * current[0]], [0.005 * n_cells]])  # Initial VOC and R
    P = np.diag([1.2 * n_cells, 0.1 * n_cells]) # Initial covariance
    error = 0
    sample_interval = timestamps[1] - timestamps[0]
    alpha_ocv = sample_interval / (sample_interval + 1)
    internal_resistance_stable[-1] = 0.005

    for index in range(len(current)):
        # RLS algorithm
        x = np.array([[1.0], [-current[index]]]) # Input vector
        theta, P, error, data_cov, gamma_ocv_hist[index], gamma_r_hist[index] = updateRLS(theta, P, x, voltage[index], current[index], lam) # Run RLS

        # Save steps for plotting
        ocv_est[index] = theta[0][0]
        if (index == 0):
            ocv_est_filtered[index] = ocv_est[index]
        else:
            ocv_est_filtered[index] = alphaFilter(ocv_est_filtered[index - 1], alpha_ocv, ocv_est[index])
        r_est[index] = theta[1][0]
        error_hist[index] = error
        v_est[index] = (x.T @ theta)[0][0]
        cov_norm[index] = abs(np.linalg.norm(P))
        ocv_cov[index] = P[0][0]
        r_cov[index] = P[1][1]
        mixed_cov[index] = P[0][1]
        data_cov_hist[index] = data_cov
        internal_resistance_stable[index] = max(r_est[index]/n_cells, 0.001)
        remaining_est[index] = np.interp((voltage[index] + internal_resistance_stable[index] * n_cells * current[index]) / n_cells, [empty_cell, full_cell], [0, 1])
        remaining_est_filtered[index] = np.interp(ocv_est_filtered[index] / n_cells, [empty_cell, full_cell], [0, 1])

    ## Flight time estimation

    # Containers for plotting
    flight_time_estimated          = np.zeros_like(timestamps)
    flight_time_estimated_filtered = np.zeros_like(timestamps)
    remaining_consumption          = np.zeros_like(timestamps)
    remaining_consumption_average  = np.zeros_like(timestamps)

    # Initializations
    alpha_ocv_derivative = (sample_interval) / (sample_interval + time_constant_ocv_derivative)
    alpha_flight_time = (sample_interval) / (sample_interval + time_constant_flight_time)
    if (logged_remaining):
        state_of_charge = remaining
    else:
        state_of_charge = remaining_est_filtered
    flight_time_estimated[0] = remaining[0] / soc_consumption_avg
    flight_time_estimated_filtered[0] = flight_time_estimated[0]
    remaining_consumption_average[0] = soc_consumption_avg
    dt = 0

    for index in range(len(current)):
        if index == 0:
            continue
        dt = timestamps[index] - timestamps[index - 1]
        remaining_consumption[index] = (state_of_charge[index - 1] - state_of_charge[index]) / dt
        if state_of_charge[index] > 0.99:
            remaining_consumption_average[index] = soc_consumption_avg
            flight_time_estimated[index] = state_of_charge[index] / soc_consumption_avg
        else:
            remaining_consumption_average[index] = np.clip(alphaFilter(remaining_consumption_average[index - 1], alpha_ocv_derivative, remaining_consumption[index]), 0.0005, 0.1)
            flight_time_estimated[index] = state_of_charge[index] / remaining_consumption_average[index]
        flight_time_estimated_filtered[index] = alphaFilter(flight_time_estimated_filtered[index - 1], alpha_flight_time, flight_time_estimated[index])

    ### Plot data

    ## Internal resistance estimation plots
    print("\nInternal Resistance estimation:")
    print("Internal Resistance mean (per cell): ", np.mean(r_est) / n_cells)
    sumFig = plt.figure("Battery Estimation with RLS")

    volt = plt.subplot(2, 3, 1)
    volt.plot(timestamps, voltage, label='Measured voltage')
    volt.plot(timestamps, v_est, label='Estimated voltage')
    volt.plot(timestamps, np.array(voltage) + np.array(internal_resistance_stable) * np.array(current) * n_cells, label='OCV estimate')
    volt.plot(timestamps, ocv_est_filtered, label='OCV estimate smoothed')
    volt.plot(timestamps, np.full_like(current, full_cell * n_cells), label='100% SOC')
    volt.set_title("Measured Voltage vs Estimated voltage vs Estimated Open circuit voltage [voltage]")
    volt.legend()

    intR = plt.subplot(2, 3, 2)
    intR.plot(timestamps, np.array(r_est) * 1000 / n_cells, label='Internal resistance estimate')
    intR.set_title("Internal resistance estimate (per cell) [mOhm]")
    intR.legend()

    soc = plt.subplot(2, 3, 3)
    soc.plot(timestamps, remaining, label='SoC logged')
    soc.plot(timestamps, remaining_est, label='SoC with estimator')
    soc.plot(timestamps, remaining_est_filtered, label='SoC with estimator smoothed')
    soc.set_title("State of charge")
    soc.legend()

    curr = plt.subplot(2, 3, 4)
    curr.plot(timestamps, current, label='Measured current')
    curr.set_title("Measured Current [A]")
    curr.legend()

    err = plt.subplot(2, 3, 5)
    err.plot(timestamps, error_hist, label='$Error$')
    err.set_title("Voltage estimation error [voltage]")
    err.legend()

    cov = plt.subplot(2, 3, 6)
    cov.plot(timestamps, cov_norm, label = 'Covariance norm')
    cov.set_title("Covariance norm")
    cov.legend()

    # # SoC estimation plots
    # socFig = plt.figure("SoC estimation")
    # plt.plot(timestamps, np.interp((np.array(voltage) + np.array(current) * 0.005 * n_cells) / n_cells, [empty_cell, full_cell], [0, 1]), label='SoC with default $R_{int}$')
    # plt.plot(timestamps, remaining, label='SoC logged')
    # plt.plot(timestamps, np.interp((np.array(voltage) + np.array(internal_resistance_stable) * n_cells * np.array(current)) / n_cells, [empty_cell, full_cell], [0, 1]), label='SoC with estimator')
    # # plt.plot(timestamps, np.convolve(np.interp((np.array(voltage) + np.array(internal_resistance_stable) * n_cells * np.array(current)) / n_cells, [empty_cell, full_cell], [0, 1]), np.ones(500)/500, mode='full')[0:np.size(timestamps)], label='SoC with estimator smoothed')
    # # plt.plot(timestamps, np.interp((np.array(voltage) + np.array(current) * 0.0009 * n_cells) / n_cells, [empty_cell, full_cell], [0, 1]), label='SoC with $R_{int}$ measured beforehand')
    # # plt.plot(timestamps, np.interp(ocv_est/n_cells, [empty_cell, full_cell], [0, 1]), label='SoC with VOC estimate')
    # plt.legend()

    # # Covariance plots
    # covFig = plt.figure("Covariance plots")
    # covR = plt.subplot(2, 2, 1)
    # covR.plot(timestamps, r_cov, label = 'r_cov')
    # covR.set_title("Internal resistance covariance")
    # covR.legend()
    # covVOC = plt.subplot(2, 2, 2)
    # covVOC.plot(timestamps, ocv_cov, label = 'ocv_cov')
    # covVOC.set_title("Open circuit covariance")
    # covVOC.legend()
    # covM = plt.subplot(2, 2, 3)
    # covM.plot(timestamps, mixed_cov, label = 'mixed_cov')
    # covM.set_title("Mixed covariance")
    # covM.legend()
    # covM = plt.subplot(2, 2, 4)
    # covM.plot(timestamps, cov_norm, label = 'cov_norm')
    # covM.set_title("Covariance norm")
    # covM.legend()

    # # Gain plots
    # gainFig = plt.figure("Gain plots")
    # gainVoc = plt.subplot(1, 2, 1)
    # gainVoc.plot(timestamps, gamma_ocv_hist, label = 'gain_voc')
    # gainVoc.set_title("Gain VOC")
    # gainVoc.legend()
    # gainR = plt.subplot(1, 2, 2)
    # gainR.plot(timestamps, gamma_r_hist, label = 'gain_r')
    # gainR.set_title("Gain R")
    # gainR.legend()

    ## Flight time estimation plots
    print("\nFlight time estimation:")
    if (logged_remaining):
        print("Using logged remaining for flight time estimation")
    else:
        print("Using estimated remaining for flight time estimation")
    print(f"Alpha for remaining derivative: {round(alpha_ocv_derivative, 6)}, Alpha for flight time estimation: {round(alpha_flight_time, 6)}")
    print(f"In {round(timestamps[-1] - timestamps[0])} seconds, the remaining flight time estimate was reduced by {round(flight_time_estimated_filtered[0] - flight_time_estimated_filtered[-1])} seconds.")

    flightTimeEstFig = plt.figure("Flight Time Estimation")

    flightTime = plt.subplot(2, 2, 1)
    if (remaining_flight_time.size):
        flightTime.plot(timestamps, remaining_flight_time, label='Flight time remaining (logged)')
    flightTime.plot(timestamps, flight_time_estimated, label='Flight time remaining (estimated)')
    flightTime.plot(timestamps, flight_time_estimated_filtered, label='Flight time remaining (estimated, filtered)')
    flightTime.set_title("Flight time remaining [s]")
    flightTime.legend()

    remainingPlot = plt.subplot(2, 2, 2)
    remainingPlot.plot(timestamps, remaining, label='Remaining (logged)')
    remainingPlot.plot(timestamps, remaining_est, label='Remaining (estimated)')
    remainingPlot.plot(timestamps, remaining_est_filtered, label='Remaining (estimated, filtered)')
    remainingPlot.set_title("Remaining [0, 1]")
    remainingPlot.legend()

    currentPlot = plt.subplot(2, 2, 3)
    currentPlot.plot(timestamps, current, label='Current')
    currentPlot.plot(timestamps, current_average, label='Current average')
    currentPlot.set_title("Current [A]")
    currentPlot.legend()

    remainingPlot = plt.subplot(2, 2, 4)
    remainingPlot.plot(timestamps, remaining_consumption, label='Remaining consumption')
    remainingPlot.plot(timestamps, remaining_consumption_average, label='Remaining consumption average')
    remainingPlot.set_title("Remaining consumption [1/s]")
    remainingPlot.legend()

    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Estimate battery parameters from ulog file.')
    parser.add_argument('-f', type = str,   required = True,  help = 'Full path to ulog file')
    parser.add_argument('-r', type = bool,  required = False, help = 'Use logged remaining value for flight time estimation',              default = False)
    parser.add_argument('-c', type = float, required = False, help = 'Number of cells in battery',                                         default = None )
    parser.add_argument('-u', type = float, required = False, help = 'Full cell voltage',                                                  default = None )
    parser.add_argument('-e', type = float, required = False, help = 'Empty cell voltage',                                                 default = None )
    parser.add_argument('-l', type = float, required = False, help = 'Forgetting factor',                                                  default = 0.99 )
    parser.add_argument('-s', type = float, required = False, help = 'Average SoC consumption per second',                                 default = 0.001)
    parser.add_argument('-t', type = int,   required = False, help = 'Time constant for alpha filter of remaining derivative',             default = 50   )
    parser.add_argument('-k', type = int,   required = False, help = 'Time constant for alpha filter of remaining flight time estimation', default = 5    )
    args = parser.parse_args()
    main(args.f, args.r, args.c, args.u, args.e, args.l, args.s, args.t, args.k)
