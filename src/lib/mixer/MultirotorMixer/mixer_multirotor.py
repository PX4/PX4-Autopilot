#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Mixer multirotor test and prototyping script.

Author: Mathieu Bresciani <brescianimathieu@gmail.com>, Beat Kueng <beat-kueng@gmx.net>
Description: This script can be used to prototype new mixer algorithms and test
it against the C++ implementation.
"""


from __future__ import print_function

from argparse import ArgumentParser
import numpy as np
import numpy.matlib
import subprocess


# --------------------------------------------------
# mixing algorithms
# --------------------------------------------------

def compute_desaturation_gain(u, u_min, u_max, desaturation_vector):
    """
    Computes the gain k by which desaturation_vector has to be multiplied
    in order to unsaturate the output that has the greatest saturation
    """
    d_u_sat_plus = u_max - u
    d_u_sat_minus = u_min - u
    k = np.zeros(u.size*2)
    for i in range(u.size):
        if abs(desaturation_vector[i]) < 0.000001:
            # avoid division by zero
            continue

        if d_u_sat_minus[i] > 0.0:
            k[2*i] = d_u_sat_minus[i] / desaturation_vector[i]
        if d_u_sat_plus[i] < 0.0:
            k[2*i+1] = d_u_sat_plus[i] / desaturation_vector[i]

    k_min = min(k)
    k_max = max(k)

    # Reduce the saturation as much as possible
    k = k_min + k_max
    return k


def minimize_sat(u, u_min, u_max, desaturation_vector):
    """
    Minimize the saturation of the actuators by
    adding or substracting a fraction of desaturation_vector.
    desaturation_vector is the vector that added to the output u,
    modifies the thrust or angular acceleration on a
    specific axis.
    For example, if desaturation_vector is given
    to slide along the vertical thrust axis, the saturation will
    be minimized by shifting the vertical thrust setpoint,
    without changing the roll/pitch/yaw accelerations.
    """
    k_1 = compute_desaturation_gain(u, u_min, u_max, desaturation_vector)
    u_1 = u + k_1 * desaturation_vector # Try to unsaturate


    # Compute the desaturation gain again based on the updated outputs.
    # In most cases it will be zero. It won't be if max(outputs) - min(outputs)
    # > max_output - min_output.
    # In that case adding 0.5 of the gain will equilibrate saturations.
    k_2 = compute_desaturation_gain(u_1, u_min, u_max, desaturation_vector)

    k_opt = k_1 + 0.5 * k_2

    u_prime = u + k_opt * desaturation_vector
    return u_prime

def mix_yaw(m_sp, u, P, u_min, u_max):
    """
    Mix yaw by adding it to an existing output vector u

    Desaturation behavior: thrust is allowed to be decreased up to 15% in order to allow
    some yaw control on the upper end. On the lower end thrust will never be increased,
    but yaw is decreased as much as required.
    """
    m_sp_yaw_only = np.matlib.zeros(m_sp.size).T
    m_sp_yaw_only[2, 0] = m_sp[2, 0]
    u_p = u + P * m_sp_yaw_only

    # Change yaw acceleration to unsaturate the outputs if needed (do not change roll/pitch),
    # and allow some yaw response at maximum thrust
    u_r_dot = P[:,2]
    u_pp = minimize_sat(u_p, u_min, u_max+0.15, u_r_dot)
    u_T = P[:, 3]
    u_ppp = minimize_sat(u_pp, 0, u_max, u_T)
    # reduce thrust only
    if (u_ppp > (u_pp)).any():
        u_ppp = u_pp
    return u_ppp

def airmode_rp(m_sp, P, u_min, u_max):
    """
    Mix roll, pitch, yaw and thrust.

    Desaturation behavior: airmode for roll/pitch:
    thrust is increased/decreased as much as required to meet the demanded roll/pitch.
    Yaw is not allowed to increase the thrust, @see mix_yaw() for the exact behavior.
    """
    # Mix without yaw
    m_sp_no_yaw = m_sp.copy()
    m_sp_no_yaw[2, 0] = 0.0
    u = P * m_sp_no_yaw

    # Use thrust to unsaturate the outputs if needed
    u_T = P[:, 3]
    u_prime = minimize_sat(u, u_min, u_max, u_T)

    # Mix yaw axis independently
    u_final = mix_yaw(m_sp, u_prime, P, u_min, u_max)

    return (u, u_final)


def airmode_rpy(m_sp, P, u_min, u_max):
    """
    Mix roll, pitch, yaw and thrust.

    Desaturation behavior: full airmode for roll/pitch/yaw:
    thrust is increased/decreased as much as required to meet demanded the roll/pitch/yaw.
    """
    # Mix with yaw
    u = P * m_sp

    # Use thrust to unsaturate the outputs if needed
    u_T = P[:, 3]
    u_prime = minimize_sat(u, u_min, u_max, u_T)

    # Unsaturate yaw (in case upper and lower bounds are exceeded)
    # to prioritize roll/pitch over yaw.
    u_T = P[:, 2]
    u_prime_yaw = minimize_sat(u_prime, u_min, u_max, u_T)

    return (u, u_prime_yaw)


def normal_mode(m_sp, P, u_min, u_max):
    """
    Mix roll, pitch, yaw and thrust.

    Desaturation behavior: no airmode, thrust is NEVER increased to meet the demanded
    roll/pitch/yaw. Instead roll/pitch/yaw is reduced as much as needed.
    Thrust can be reduced to unsaturate the upper side.
    @see mix_yaw() for the exact yaw behavior.
    """
    # Mix without yaw
    m_sp_no_yaw = m_sp.copy()
    m_sp_no_yaw[2, 0] = 0.0
    u = P * m_sp_no_yaw

    # Use thrust to unsaturate the outputs if needed
    # by reducing the thrust only
    u_T = P[:, 3]
    u_prime = minimize_sat(u, u_min, u_max, u_T)
    if (u_prime > (u)).any():
        u_prime = u

    # Reduce roll/pitch acceleration if needed to unsaturate
    u_p_dot = P[:, 0]
    u_p2 = minimize_sat(u_prime, u_min, u_max, u_p_dot)
    u_q_dot = P[:, 1]
    u_p3 = minimize_sat(u_p2, u_min, u_max, u_q_dot)

    # Mix yaw axis independently
    u_final = mix_yaw(m_sp, u_p3, P, u_min, u_max)
    return (u, u_final)


# --------------------------------------------------
# test cases
# --------------------------------------------------

# normalized control allocation test matrices (B_px from px_generate_mixers.py)
# quad_x
P1 = np.matrix([
    [-0.71,  0.71,  1.,   1.  ],
    [ 0.71, -0.71,  1.,   1.  ],
    [ 0.71,  0.71, -1.,   1.  ],
    [-0.71, -0.71, -1.,   1.  ]])
# quad_wide
P2 = np.matrix([
    [-0.5,   0.71,  0.77,  1.  ],
    [ 0.5,  -0.71,  1.,    1.  ],
    [ 0.5,   0.71, -0.77,  1.  ],
    [-0.5,  -0.71, -1.,    1.  ]])
# hex_x
P3 = np.matrix([
    [-1.,    0.,   -1.,    1.  ],
    [ 1.,   -0.,    1.,    1.  ],
    [ 0.5,   0.87, -1.,    1.  ],
    [-0.5,  -0.87,  1.,    1.  ],
    [-0.5,   0.87,  1.,    1.  ],
    [ 0.5,  -0.87, -1.,    1.  ]])
# hex_cox
P4 = np.matrix([
    [-0.87,  0.5,  -1.,    1.  ],
    [-0.87,  0.5,   1.,    1.  ],
    [ 0.,   -1.,   -1.,    1.  ],
    [ 0.,   -1.,    1.,    1.  ],
    [ 0.87,  0.5,  -1.,    1.  ],
    [ 0.87,  0.5,   1.,    1.  ]])
# octa_plus
P5 = np.matrix([
    [-0.,    1.,   -1.,    1.  ],
    [ 0.,   -1.,   -1.,    1.  ],
    [-0.71,  0.71,  1.,    1.  ],
    [-0.71, -0.71,  1.,    1.  ],
    [ 0.71,  0.71,  1.,    1.  ],
    [ 0.71, -0.71,  1.,    1.  ],
    [ 1.,    0.,   -1.,    1.  ],
    [-1.,   -0.,   -1.,    1.  ]])

P_tests = [ P1, P2, P3, P4, P5 ]

test_cases_input = np.matrix([
    # desired accelerations (must be within [-1, 1]):
    #roll pitch yaw thrust
    [ 0.0,   0.0,  0.0,    0.0],
    [-0.05,  0.0,  0.0,    0.0],
    [ 0.05, -0.05, 0.0,    0.0],
    [ 0.05,  0.05, -0.025, 0.0],
    [ 0.0,   0.2,  -0.025, 0.0],
    [ 0.2,   0.05, 0.09,   0.0],
    [-0.125, 0.02, 0.04,   0.0],

    # extreme cases
    [ 1.0,  0.0,  0.0,  0.0],
    [ 0.0, -1.0,  0.0,  0.0],
    [ 0.0,  0.0,  1.0,  0.0],
    [ 1.0,  1.0, -1.0,  0.0],
    [-1.0,  0.9, -0.9,  0.0],
    [-1.0,  0.9,  0.0,  0.0],
    ])

# use the following thrust values for all test cases (must be within [0, 1])
thrust_values = [0, 0.1, 0.45, 0.9, 1.0]
test_cases = np.zeros((test_cases_input.shape[0] * len(thrust_values), 4))
for i in range(test_cases_input.shape[0]):
    for k in range(len(thrust_values)):
        test_case = test_cases_input[i]
        test_case[0, 3] = thrust_values[k]
        test_cases[i * len(thrust_values) + k, :] = test_case


def run_tests(mixer_cb, P, test_mixer_binary, test_index=None):
    """
    Run all (or a specific) tests for a certain mixer method an control
    allocation matrix P
    """
    B = np.linalg.pinv(P)
    proc = subprocess.Popen(
        test_mixer_binary,
        #'cat > /tmp/test_'+str(mode_idx), shell=True, # just to test the output
        stdout=subprocess.PIPE,
        stdin=subprocess.PIPE)
    proc.stdin.write("{:}\n".format(mode_idx).encode('utf-8')) # airmode
    motor_count = P.shape[0]
    proc.stdin.write("{:}\n".format(motor_count).encode('utf-8')) # motor count
    # control allocation matrix
    for row in P.getA():
        for col in row:
            proc.stdin.write("{:.8f} ".format(col).encode('utf-8'))
        proc.stdin.write("\n".encode('utf-8'))
    proc.stdin.write("\n".encode('utf-8'))

    failed = False
    try:
        if test_index is None:
            # go through all test cases
            test_indices = range(test_cases.shape[0])
        else:
            test_indices = [test_index]
        for i in test_indices:
            actuator_controls = test_cases[[i], :].T
            proc.stdin.write("{:.8f} {:.8f} {:.8f} {:.8f}\n"
                    .format(actuator_controls[0, 0], actuator_controls[1, 0],
                        actuator_controls[2, 0], actuator_controls[3, 0]).encode('utf-8'))

            (u, u_new) = mixer_cb(actuator_controls, P, 0.0, 1.0)
            # Saturate the outputs between 0 and 1
            u_new_sat = np.maximum(u_new, np.matlib.zeros(u.size).T)
            u_new_sat = np.minimum(u_new_sat, np.matlib.ones(u.size).T)
            # write expected outputs
            for j in range(motor_count):
                proc.stdin.write("{:.8f} ".format(u_new_sat[j, 0]).encode('utf-8'))
            proc.stdin.write("\n".encode('utf-8'))

        proc.stdin.close()
    except IOError as e:
        failed = True
    result = proc.stdout.read()
    proc.wait()
    if proc.returncode != 0: failed = True
    if failed:
        print("Error: test failed")
        print("B:\n{}".format(B))
        print("P:\n{}".format(P))
        print(result)
        raise Exception('Test failed')

parser = ArgumentParser(description=__doc__)
parser.add_argument('--test', action='store_true', default=False, help='Run tests')
parser.add_argument("--mixer-multirotor-binary",
                  help="select test_mixer_multirotor binary file name",
                  default='./test_mixer_multirotor')
parser.add_argument("--mode", "-m", dest="mode",
                help="mixer mode: none, rp, rpy", default=None)
parser.add_argument("-i", dest="index", type=int,
                  help="Select a single test to run (starting at 1)", default=None)

args = parser.parse_args()
mixer_mode = args.mode

if args.test:
    mixer_binary = args.mixer_multirotor_binary
    test_index = args.index
    if test_index is not None: test_index -= 1
    for mode_idx, (airmode, mixer_cb) in enumerate([
            ('none', normal_mode),
            ('rp', airmode_rp),
            ('rpy', airmode_rpy)]):
        if mixer_mode is not None and mixer_mode != airmode:
            continue
        print('Testing mode: '+airmode)
        for P in P_tests:
            run_tests(mixer_cb, P, mixer_binary, test_index)
    exit(0)

# --------------------------------------------------
# Prototyping and corner case testing playground
# --------------------------------------------------

# Compute the control allocation matrix
# u = P * m
P = P1 # normal quad
#P = P2 # wide quad

# Normalized actuator effectiveness matrix using the pseudo inverse of P
# m = B * u
B = np.linalg.pinv(P)

# Desired accelerations (actuator controls, in [-1, 1])
p_dot_sp = 0.0 # roll acceleration (p is the roll rate)
q_dot_sp = 0.1 # pitch acceleration
r_dot_sp = 0.1 # yaw acceleration
T_sp = 0.0 # vertical thrust
m_sp = np.matrix([p_dot_sp, q_dot_sp, r_dot_sp, T_sp]).T # Vector of desired "accelerations"

# Airmode type (none/rp/rpy)
airmode = mixer_mode
if airmode is None: airmode = "none"

# Actuators output saturations
u_max = 1.0
u_min = 0.0

if airmode == "none":
    (u, u_new) = normal_mode(m_sp, P, u_min, u_max)
elif airmode == "rp":
    (u, u_new) = airmode_rp(m_sp, P, u_min, u_max)
elif airmode == "rpy":
    (u, u_new) = airmode_rpy(m_sp, P, u_min, u_max)
else:
    u = 0.0
    u_new = 0.0

# Saturate the outputs between 0 and 1
u_new_sat = np.maximum(u_new, np.matlib.zeros(u.size).T)
u_new_sat = np.minimum(u_new_sat, np.matlib.ones(u.size).T)

np.set_printoptions(suppress=True)

# Display some results
print("u = \n{}\n".format(u))
print("u_new = \n{}\n".format(u_new))
print("u_new_sat = \n{}\n".format(u_new_sat))
print("Desired accelerations = \n{}\n".format(m_sp))
# Compute back the allocated accelerations
m_new = B * u_new_sat
print("Allocated accelerations = \n{}\n".format(m_new))
