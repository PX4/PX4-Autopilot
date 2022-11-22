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
            output_dir="generated decoupled static",
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



#-------------------------------- DECOUPLED DYNAMICS, STATIC TARGET ------------------------------------------ #

class State:
    r = 0
    r_dot = 1
    b = 2
    n_states = 3

class Input:
    a = 0
    n_inputs = 1

class Directions:
    x = 0,
    nb_directions = 1

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

def predictState(dt: sf.Scalar, _state: VState, acc: sf.Scalar) -> (VState):

    Phi = sf.Matrix([   [1, dt, 0],
                        [0, 1, 0],
                        [0, 0, 1]])

    G = sf.Matrix([-0.5*dt*dt, -dt, 0])

    return Phi*_state + G*acc

def syncState(dt: sf.Scalar, _state: VState, acc: sf.Scalar) -> (VState):

    Phi = sf.Matrix([   [1, dt, 0],
                        [0, 1, 0],
                        [0, 0, 1]])

    G = sf.Matrix([-0.5*dt*dt, -dt, 0])

    return (Phi.inv() * (_state - G*acc)).simplify()

def computeProcessNoise(dt: sf.Scalar, _input_var: sf.Scalar, _bias_var: sf.Scalar) -> (MState):

    G = sf.Matrix([-0.5*dt*dt, -dt, 0])

    Q_bias = sf.Matrix([    [0, 0, 0],
                            [0, 0, 0],
                            [0, 0, _bias_var]])

    return G*_input_var*G.T + Q_bias

def compute_phiPphi(dt: sf.Scalar, _covariance: MState) -> (MState):
    Phi = sf.Matrix([   [1, dt, 0],
                        [0, 1, 0],
                        [0, 0, 1]])

    return Phi*_covariance*Phi.T

def predictCov(dt: sf.Scalar, _input_var: sf.Scalar, _bias_var: sf.Scalar, _covariance: MState) -> (MState):
    Phi = sf.Matrix([   [1, dt, 0],
                        [0, 1, 0],
                        [0, 0, 1]])

    G = sf.Matrix([-0.5*dt*dt, -dt, 0])

    Q_bias = sf.Matrix([    [0, 0, 0],
                            [0, 0, 0],
                            [0, 0, _bias_var]])

    return G*_input_var*G.T + Q_bias + Phi*_covariance*Phi.T

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