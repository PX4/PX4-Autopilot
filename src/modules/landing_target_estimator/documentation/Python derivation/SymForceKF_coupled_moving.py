import symforce.symbolic as sf

def generate_px4_function(function_name, output_names):
    from symforce.codegen import Codegen, CppConfig
    import os
    import fileinput

    codegen = Codegen.function(
            function_name,
            output_names=output_names,
            config=CppConfig())
    metadata = codegen.generate_function(
            output_dir="generated coupled moving",
            skip_directory_nesting=True)

    print("Files generated in {}:\n".format(metadata.output_dir))
    for f in metadata.generated_files:
        print("  |- {}".format(os.path.relpath(f, metadata.output_dir)))

    # Replace cstdlib and Eigen functions by PX4 equivalents
    with fileinput.FileInput(os.path.abspath(metadata.generated_files[0]), inplace=True, backup='.bak') as file:
        for line in file:
            line = line.replace("std::max", "math::max")
            line = line.replace("std::min", "math::min")
            line = line.replace("Eigen", "matrix")
            line = line.replace("matrix/Dense", "matrix/math.hpp")
            print(line, end='')

#-------------------------------- COUPLED DYNAMICS, STATIC TARGET ------------------------------------------ #

class State:
    rx = 0
    ry = 1
    rz = 2
    rx_dot = 3
    ry_dot = 4
    rz_dot = 5
    bx = 6
    by = 7
    bz = 8
    atx = 9,
    aty = 10,
    atz = 11,
    n_states = 12

class Input:
    ax = 0
    ay = 1
    az = 2
    n_inputs = 3

class Directions:
    x = 0,
    y = 1,
    z = 2,
    nb_directions = 3

class VState(sf.Matrix):
    SHAPE = (State.n_states, 1)

class MState(sf.Matrix):
    SHAPE = (State.n_states, State.n_states)

class VInput(sf.Matrix):
    SHAPE = (Input.n_inputs, 1)

class MInput(sf.Matrix):
    SHAPE = (State.n_states, Input.n_inputs)

class VMeas(sf.Matrix):
    SHAPE = (1, State.n_states)

class VDirections(sf.Matrix):
    SHAPE = (Directions.nb_directions, 1)

class MDirections(sf.Matrix):
    SHAPE = (Directions.nb_directions, Directions.nb_directions)

def predictState(dt: sf.Scalar, _state: VState, acc: VInput) -> (VState):

    identity = sf.Matrix([  [1, 0, 0],
                            [0, 1, 0],
                            [0, 0, 1]])

    Phi = sf.Matrix.block_matrix([   [1 * identity, dt * identity, 0 * identity, 0.5*dt*dt * identity],
                        [0 * identity, 1 * identity,  0 * identity, dt * identity       ],
                        [0 * identity, 0 * identity,  1 * identity, 0  * identity       ],
                        [0 * identity, 0 * identity,  0 * identity, 1  * identity       ]])


    G = sf.Matrix.block_matrix([ [-0.5*dt*dt * identity], [-dt * identity], [0 * identity], [0 * identity]])

    return Phi*_state + G*acc

def syncState(dt: sf.Scalar, _state: VState, acc: VInput) -> (VState):

    identity = sf.Matrix([  [1, 0, 0],
                            [0, 1, 0],
                            [0, 0, 1]])

    Phi = sf.Matrix.block_matrix([   [1 * identity, dt * identity, 0 * identity, 0.5*dt*dt * identity],
                        [0 * identity, 1 * identity,  0 * identity, dt * identity       ],
                        [0 * identity, 0 * identity,  1 * identity, 0  * identity       ],
                        [0 * identity, 0 * identity,  0 * identity, 1  * identity       ]])


    G = sf.Matrix.block_matrix([ [-0.5*dt*dt * identity], [-dt * identity], [0 * identity], [0 * identity]])

    return (Phi.inv() * (_state - G*acc)).simplify()

def computeProcessNoise(dt: sf.Scalar, _input_var: MDirections, _bias_var: MDirections, _acc_var: MDirections) -> (MState):

    identity = sf.Matrix([  [1, 0, 0],
                            [0, 1, 0],
                            [0, 0, 1]])

    G = sf.Matrix.block_matrix([ [-0.5*dt*dt * identity], [-dt * identity], [0 * identity], [0 * identity]])

    mult_mat_acc = sf.Matrix([  [0, 0, 0, 0],
                                [0, 0, 0, 0],
                                [0, 0, 0, 0],
                                [0, 0, 0, 1]])

    Q_acc = sf.Matrix.block_matrix([   [_acc_var[0,0] * mult_mat_acc, 0 * mult_mat_acc, 0 * mult_mat_acc],
                                        [0 * mult_mat_acc, _acc_var[1,1] * mult_mat_acc, 0 * mult_mat_acc],
                                        [0 * mult_mat_acc, 0 * mult_mat_acc, _acc_var[2,2] * mult_mat_acc]])

    mult_mat = sf.Matrix([  [0, 0, 0, 0],
                            [0, 0, 0, 0],
                            [0, 0, 1, 0],
                            [0, 0, 0, 0]])

    Q_bias = sf.Matrix.block_matrix([   [_bias_var[0,0] * mult_mat, 0 * mult_mat, 0 * mult_mat],
                                        [0 * mult_mat, _bias_var[1,1] * mult_mat, 0 * mult_mat],
                                        [0 * mult_mat, 0 * mult_mat, _bias_var[2,2] * mult_mat]])

    return G*_input_var*G.T + Q_bias + Q_acc

def compute_phiPphi(dt: sf.Scalar, _covariance: MState) -> (MState):

    identity = sf.Matrix([  [1, 0, 0],
                            [0, 1, 0],
                            [0, 0, 1]])

    Phi = sf.Matrix.block_matrix([   [1 * identity, dt * identity, 0 * identity, 0.5*dt*dt * identity],
                        [0 * identity, 1 * identity,  0 * identity, dt * identity       ],
                        [0 * identity, 0 * identity,  1 * identity, 0  * identity       ],
                        [0 * identity, 0 * identity,  0 * identity, 1  * identity       ]])

    return Phi*_covariance*Phi.T

def predictCov(dt: sf.Scalar, _input_var: MDirections, _bias_var: MDirections, _acc_var: MDirections, _covariance: MState) -> (MState):
    identity = sf.Matrix([  [1, 0, 0],
                            [0, 1, 0],
                            [0, 0, 1]])

    Phi = sf.Matrix.block_matrix([   [1 * identity, dt * identity, 0 * identity, 0.5*dt*dt * identity],
                        [0 * identity, 1 * identity,  0 * identity, dt * identity       ],
                        [0 * identity, 0 * identity,  1 * identity, 0  * identity       ],
                        [0 * identity, 0 * identity,  0 * identity, 1  * identity       ]])

    G = sf.Matrix.block_matrix([ [-0.5*dt*dt * identity], [-dt * identity], [0 * identity], [0 * identity]])

    mult_mat_acc = sf.Matrix([  [0, 0, 0, 0],
                                [0, 0, 0, 0],
                                [0, 0, 0, 0],
                                [0, 0, 0, 1]])

    Q_acc = sf.Matrix.block_matrix([   [_acc_var[0,0] * mult_mat_acc, 0 * mult_mat_acc, 0 * mult_mat_acc],
                                        [0 * mult_mat_acc, _acc_var[1,1] * mult_mat_acc, 0 * mult_mat_acc],
                                        [0 * mult_mat_acc, 0 * mult_mat_acc, _acc_var[2,2] * mult_mat_acc]])

    mult_mat = sf.Matrix([  [0, 0, 0, 0],
                            [0, 0, 0, 0],
                            [0, 0, 1, 0],
                            [0, 0, 0, 0]])

    Q_bias = sf.Matrix.block_matrix([   [_bias_var[0,0] * mult_mat, 0 * mult_mat, 0 * mult_mat],
                                        [0 * mult_mat, _bias_var[1,1] * mult_mat, 0 * mult_mat],
                                        [0 * mult_mat, 0 * mult_mat, _bias_var[2,2] * mult_mat]])

    return G*_input_var*G.T + Q_bias + Q_acc + Phi*_covariance*Phi.T

def computeInnov(meas: sf.Scalar, _state: VState, _meas_matrix: VMeas) -> (sf.Scalar):
    return   meas - (_meas_matrix*_state)[0,0]

def computeInnovCov(meas_unc: sf.Scalar, _covariance: MState, _meas_matrix: VMeas) -> (sf.Scalar):
    return (_meas_matrix*_covariance*_meas_matrix.T)[0,0] + meas_unc

def computeKalmanGain(_innov_cov: sf.Scalar, _covariance: MState, _meas_matrix: VMeas) -> (VState):
    return _covariance*_meas_matrix.T / _innov_cov

generate_px4_function(predictState, output_names=["state"])
generate_px4_function(syncState, output_names=["sync_state"])
generate_px4_function(computeProcessNoise, output_names=["process_noise"])
generate_px4_function(compute_phiPphi, output_names=["phiPphi"])
generate_px4_function(predictCov, output_names=["covariance"])
generate_px4_function(computeInnov, output_names=["innov"])
generate_px4_function(computeInnovCov, output_names=["innov_cov"])
generate_px4_function(computeKalmanGain, output_names=["kalman_gain"])