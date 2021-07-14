"""

This script calculates the observation scalars (H matrix) for fusing optical flow
measurements for terrain estimation.

@author: roman
"""

from sympy import *

# q: quaternion describing rotation from frame 1 to frame 2
# returns a rotation matrix derived form q which describes the same
# rotation
def quat2Rot(q):
    q0 = q[0]
    q1 = q[1]
    q2 = q[2]
    q3 = q[3]

    Rot = Matrix([[q0**2 + q1**2 - q2**2 - q3**2, 2*(q1*q2 - q0*q3), 2*(q1*q3 + q0*q2)],
                  [2*(q1*q2 + q0*q3), q0**2 - q1**2 + q2**2 - q3**2, 2*(q2*q3 - q0*q1)],
                   [2*(q1*q3-q0*q2), 2*(q2*q3 + q0*q1), q0**2 - q1**2 - q2**2 + q3**2]])

    return Rot

# take an expression calculated by the cse() method and write the expression
# into a text file in C format
def write_simplified(P_touple, filename, out_name):
    subs = P_touple[0]
    P = Matrix(P_touple[1])
    fd = open(filename, 'a')

    is_vector = P.shape[0] == 1 or P.shape[1] == 1

    # write sub expressions
    for index, item in enumerate(subs):
        fd.write('float ' + str(item[0]) + ' = ' + str(item[1]) + ';\n')

    # write actual matrix values
    fd.write('\n')

    if not is_vector:
        iterator = range(0,sqrt(len(P)), 1)
        for row in iterator:
            for column in iterator:
                fd.write(out_name + '(' + str(row) + ',' + str(column) + ') = ' + str(P[row, column]) + ';\n')
    else:
        iterator = range(0, len(P), 1)

        for item in iterator:
            fd.write(out_name + '(' + str(item) + ') = ' + str(P[item]) + ';\n')

    fd.write('\n\n')
    fd.close()

########## Symbolic variable definition #######################################


# vehicle velocity
v_x = Symbol("v_x", real=True)  # vehicle body x velocity
v_y = Symbol("v_y", real=True)  # vehicle body y velocity

# unit quaternion describing vehicle attitude, qw is real part
qw = Symbol("q0", real=True)
qx = Symbol("q1", real=True)
qy = Symbol("q2", real=True)
qz = Symbol("q3", real=True)
q_att = Matrix([qw, qx, qy, qz])

# terrain vertial position in local NED frame
_terrain_vpos = Symbol("_terrain_vpos", real=True)

_terrain_var = Symbol("_terrain_var", real=True)

# vehicle vertical position in local NED frame
pos_z = Symbol("z", real=True)

R_body_to_earth = quat2Rot(q_att)

# Optical flow around x axis
flow_x = -v_y / (_terrain_vpos - pos_z) * R_body_to_earth[2,2]

# Calculate observation scalar
H_x = Matrix([flow_x]).jacobian(Matrix([_terrain_vpos]))

H_x_simple = cse(H_x, symbols('t0:30'))

# Optical flow around y axis
flow_y = v_x / (_terrain_vpos - pos_z) * R_body_to_earth[2,2]

# Calculate observation scalar
H_y = Matrix([flow_y]).jacobian(Matrix([_terrain_vpos]))

H_y_simple = cse(H_y, symbols('t0:30'))

write_simplified(H_x_simple, "flow_x_observation.txt", "Hx")
write_simplified(H_y_simple, "flow_y_observation.txt", "Hy")

