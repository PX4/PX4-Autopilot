from sympy import *
from code_gen import *

def create_cov_matrix(i, j):
    if j >= i:
        return Symbol("P" + str(i) + str(j), real=True)
    else:
        return 0

def create_symmetric_cov_matrix():
    # define a symbolic covariance matrix
    P = Matrix(3,3,create_cov_matrix)

    for index in range(3):
        for j in range(3):
            if index > j:
                P[index,j] = P[j,index]

    return P

print('Starting code generation:')

dt = symbols("dt", real=True)  # dt (sec)
psi = symbols("psi", real=True)  # yaw angle of body frame wrt earth frame
vn, ve = symbols("vn ve", real=True)  # velocity in world frame (north/east) - m/sec
daz = symbols("daz", real=True)  # IMU z axis delta angle measurement in body axes - rad
dazVar = symbols("dazVar", real=True) # IMU Z axis delta angle measurement variance (rad^2)
dvx, dvy = symbols("dvx dvy", real=True)  # IMU x and y axis delta velocity measurement in body axes - m/sec
dvxVar, dvyVar = symbols("dvxVar dvyVar", real=True)   # IMU x and y axis delta velocity measurement variance (m/s)^2

# derive the body to nav direction transformation matrix
Tbn = Matrix([[cos(psi) , -sin(psi)],
              [sin(psi) ,  cos(psi)]])

# attitude update equation
psiNew = psi + daz

# velocity update equations
velNew = Matrix([vn,ve]) + Tbn*Matrix([dvx,dvy])

# Define the state vectors
stateVector = Matrix([vn,ve,psi])

# Define vector of process equations
newStateVector = Matrix([velNew,psiNew])

# Calculate state transition matrix
print('Computing state propagation jacobian ...')
F = newStateVector.jacobian(stateVector)

# Derive the covariance prediction equations
# Error growth in the inertial solution is assumed to be driven by 'noise' in the delta angles and
# velocities, after bias effects have been removed.

# derive the control(disturbance) influence matrix from IMU noise to state noise
G = newStateVector.jacobian(Matrix([dvx,dvy,daz]))

# derive the state error matrix
distMatrix = Matrix([[dvxVar , 0 , 0],
                     [0 , dvyVar , 0],
                     [0 , 0 , dazVar]])

Q = G * distMatrix * G.T

# propagate covariance matrix
P = create_symmetric_cov_matrix()

print('Computing covariance propagation ...')
P_new = F * P * F.T + Q

print('Simplifying covariance propagation ...')
P_new_simple = cse(P_new, symbols("S0:1000"), optimizations='basic')

print('Writing covariance propagation to file ...')
cov_prediction_code_generator = CodeGenerator("./generated/covariance_prediction_generated.cpp")
cov_prediction_code_generator.print_string("Equations for covariance matrix prediction")
cov_prediction_code_generator.write_subexpressions(P_new_simple[0])
cov_prediction_code_generator.write_matrix(Matrix(P_new_simple[1]), "_ekf_gsf[model_index].P", True)
cov_prediction_code_generator.close()

# derive the covariance update equation for a NE velocity observation
print('Computing NE velocity observatio innovation variance code ...')
velObsVar = symbols("velObsVar", real=True) # velocity observation variance (m/s)^2
H = Matrix([[1,0,0],
            [0,1,0]])

R = Matrix([[velObsVar , 0],
            [0 , velObsVar]])

print('Computing NE velocity measurement update code ...')
S = H * P * H.T + R
S_det_inv = 1 / S.det()
S_inv = S.inv()
K = (P * H.T) * S_inv
P_new = P - K * S * K.T

# optimize code
t, [S_det_inv_s, S_inv_s, K_s, P_new_s] = cse([S_det_inv, S_inv, K, P_new], symbols("t0:1000"), optimizations='basic')

print('Writing NE velocity measurement update code to file ...')
measurement_update_code_generator = CodeGenerator("./generated/measurement_update_generated.cpp")
measurement_update_code_generator.print_string("Intermediate variables")
measurement_update_code_generator.write_subexpressions(t)
measurement_update_code_generator.print_string("Equations for NE velocity innovation variance's determinante inverse")
measurement_update_code_generator.write_matrix(Matrix([[S_det_inv_s]]), "_ekf_gsf[model_index].S_det_inverse", False)
measurement_update_code_generator.print_string("Equations for NE velocity innovation variance inverse")
measurement_update_code_generator.write_matrix(Matrix(S_inv_s), "_ekf_gsf[model_index].S_inverse", True)
measurement_update_code_generator.print_string("Equations for NE velocity Kalman gain")
measurement_update_code_generator.write_matrix(Matrix(K_s), "K", False)
measurement_update_code_generator.print_string("Equations for covariance matrix update")
measurement_update_code_generator.write_matrix(Matrix(P_new_s), "_ekf_gsf[model_index].P", True)
measurement_update_code_generator.close()
