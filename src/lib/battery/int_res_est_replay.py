# Test internal resistance estimator on flight logs
# run with:
# python3 int_res_est_replay.py -f <pathToLogFile> -c <#batteryCells>
#   -u <(optional)fullCellVoltage> -e <(optional)emptyCellVoltage> -l <(optional)forgettingFactor> -d <(optional)filterMeasurements>
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

def rls_update(theta, P, x, V, I, lam):
    gamma = P @ x / (lam + x.T @ P @ x)
    error = V - x.T @ theta
    data_cov = x.T @ P @ x
    theta_temp = theta + gamma * error
    P_temp = (P - gamma @ x.T @ P) / lam
    if (abs(np.linalg.norm(P)) < abs(np.linalg.norm(P_temp))):
        theta_corr = np.array([V + theta[1] * I, theta[1]]) # Correct OCV estimation
        P_corr = P
        return theta_corr, P_corr, error, data_cov, 0, 0
    return theta_temp, P_temp, error, data_cov, gamma[0], gamma[1]

def main(log_name, n_cells, full_cell, empty_cell, lam):
    log = ULog(log_name)

    log_n_cells = getParam(log, 'BAT1_N_CELLS')
    log_full_cell = getParam(log, 'BAT1_V_CHARGED')
    log_empty_cell = getParam(log, 'BAT1_V_EMPTY')

    # Debug information
    print(f"Extracted from log - BAT1_N_CELLS: {log_n_cells}, BAT1_V_CHARGED: {log_full_cell}, BAT1_V_EMPTY: {log_empty_cell}")

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

    timestamps = us2s(getData(log, 'battery_status', 'timestamp'))
    I = getData(log, 'battery_status', 'current_a')
    V = getData(log, 'battery_status', 'voltage_v')
    remaining = getData(log, 'battery_status', 'remaining')

    if not timestamps.size or not I.size or not V.size or not remaining.size:
        print("Error: Incomplete data.")
        return

    # Initializations
    theta = np.array([[V[0] + 0.005 * n_cells * I[0]], [0.005 * n_cells]])  # Initial VOC and R
    P = np.diag([1.2 * n_cells, 0.1 * n_cells]) # Initial covariance
    error = 0

    # For plotting
    VOC_est = np.zeros_like(I)
    R_est = np.zeros_like(I)
    error_hist = np.zeros_like(I)
    v_est = np.zeros_like(I)
    internal_resistance_stable = np.zeros_like(I)
    internal_resistance_stable[-1] = 0.005
    cov_norm = np.zeros_like(I)
    r_cov = np.zeros_like(I)
    ocv_cov = np.zeros_like(I)
    mixed_cov = np.zeros_like(I)
    data_cov_hist = np.zeros_like(I)
    gamma_voc_hist = np.zeros_like(I)
    gamma_r_hist = np.zeros_like(I)

    for index in range(len(I)):
        # RLS algorithm
        x = np.array([[1.0], [-I[index]]]) # Input vector
        theta, P, error, data_cov, gamma_voc_hist[index], gamma_r_hist[index] = rls_update(theta, P, x, V[index], I[index], lam) # Run RLS

        # For plotting
        VOC_est[index] = theta[0][0]
        R_est[index] = theta[1][0]
        error_hist[index] = error
        v_est[index] = x.T @ theta
        cov_norm[index] = abs(np.linalg.norm(P))
        ocv_cov[index] = P[0][0]
        r_cov[index] = P[1][1]
        mixed_cov[index] = P[0][1]
        data_cov_hist[index] = data_cov
        internal_resistance_stable[index] = max(R_est[index]/n_cells, 0.001)

    # Plot data
    print("Internal Resistance mean (per cell): ", np.mean(R_est) / n_cells)

    # Summary plot
    sumFig = plt.figure("Battery Estimation with RLS")

    volt = plt.subplot(2, 3, 1)
    volt.plot(timestamps, V, label='Measured voltage')
    volt.plot(timestamps, v_est, label='Estimated voltage')
    volt.plot(timestamps, np.array(V) + np.array(internal_resistance_stable) * np.array(I) * n_cells, label='OCV estimate')
    ocv_smoothed = np.convolve(np.array(V) + np.array(internal_resistance_stable) * np.array(I) * n_cells, np.ones(30)/30, mode='full')[0:np.size(timestamps)]
    ocv_smoothed[0:30] = ocv_smoothed[31]
    volt.plot(timestamps, ocv_smoothed, label='OCV estimate smoothed')
    volt.plot(timestamps, np.full_like(I, full_cell * n_cells), label='100% SOC')
    volt.set_title("Measured Voltage vs Estimated voltage vs Estimated Open circuit voltage [V]")
    volt.legend()

    intR = plt.subplot(2, 3, 2)
    intR.plot(timestamps, np.array(R_est) * 1000 / n_cells, label='Internal resistance estimate')
    intR.set_title("Internal resistance estimate (per cell) [mOhm]")
    intR.legend()

    soc = plt.subplot(2, 3, 3)
    soc.plot(timestamps, remaining, label='SoC logged')
    soc.plot(timestamps, np.interp((np.array(V) + np.array(internal_resistance_stable) * n_cells * np.array(I)) / n_cells, [empty_cell, full_cell], [0, 1]), label='SoC with estimator')
    soc.plot(timestamps, np.interp(ocv_smoothed / n_cells, [empty_cell, full_cell], [0, 1]), label='SoC with estimator smoothed')
    soc.set_title("State of charge")
    soc.legend()

    curr = plt.subplot(2, 3, 4)
    curr.plot(timestamps, I, label='Measured current')
    curr.set_title("Measured Current [A]")
    curr.legend()

    err = plt.subplot(2, 3, 5)
    err.plot(timestamps, error_hist, label='$Error$')
    err.set_title("Voltage estimation error [V]")
    err.legend()

    cov = plt.subplot(2, 3, 6)
    cov.plot(timestamps, cov_norm, label = 'Covariance norm')
    cov.set_title("Covariance norm")
    cov.legend()

    # # SoC estimation plots
    # socFig = plt.figure("SoC estimation")
    # plt.plot(timestamps, np.interp((np.array(V) + np.array(I) * 0.005 * n_cells) / n_cells, [empty_cell, full_cell], [0, 1]), label='SoC with default $R_{int}$')
    # plt.plot(timestamps, remaining, label='SoC logged')
    # plt.plot(timestamps, np.interp((np.array(V) + np.array(internal_resistance_stable) * n_cells * np.array(I)) / n_cells, [empty_cell, full_cell], [0, 1]), label='SoC with estimator')
    # # plt.plot(timestamps, np.convolve(np.interp((np.array(V) + np.array(internal_resistance_stable) * n_cells * np.array(I)) / n_cells, [empty_cell, full_cell], [0, 1]), np.ones(500)/500, mode='full')[0:np.size(timestamps)], label='SoC with estimator smoothed')
    # # plt.plot(timestamps, np.interp((np.array(V) + np.array(I) * 0.0009 * n_cells) / n_cells, [empty_cell, full_cell], [0, 1]), label='SoC with $R_{int}$ measured beforehand')
    # # plt.plot(timestamps, np.interp(VOC_est/n_cells, [empty_cell, full_cell], [0, 1]), label='SoC with VOC estimate')
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
    # gainVoc.plot(timestamps, gamma_voc_hist, label = 'gain_voc')
    # gainVoc.set_title("Gain VOC")
    # gainVoc.legend()
    # gainR = plt.subplot(1, 2, 2)
    # gainR.plot(timestamps, gamma_r_hist, label = 'gain_r')
    # gainR.set_title("Gain R")
    # gainR.legend()

    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Estimate battery parameters from ulog file.')
    parser.add_argument('-f', type = str,   required = True,  help = 'Full path to ulog file')
    parser.add_argument('-c', type = float, required = False, help = 'Number of cells in battery')
    parser.add_argument('-u', type = float, required = False, default = None, help = 'Full cell voltage')
    parser.add_argument('-e', type = float, required = False, default = None,  help = 'Empty cell voltage')
    parser.add_argument('-l', type = float, required = False, default = 0.99, help = 'Forgetting factor')
    args = parser.parse_args()
    main(args.f, args.c, args.u, args.e, args.l)
