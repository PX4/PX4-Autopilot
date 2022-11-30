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
            output_dir="src/modules/landing_target_estimator/python_derivation/generated/coupled_moving",
            skip_directory_nesting=True)

    print("Files generated in {}:\n".format(metadata.output_dir))
    for f in metadata.generated_files:
        print("  |- {}".format(os.path.relpath(f, metadata.output_dir)))

    # Replace cstdlib and Eigen functions by PX4 equivalents
    with fileinput.FileInput(os.path.abspath(metadata.generated_files[0]), inplace=True) as file:
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

class VMeas(sf.Matrix):
    SHAPE = (1, State.n_states)

class MDirections(sf.Matrix):
    SHAPE = (Directions.nb_directions, Directions.nb_directions)


def syncState(dt: sf.Scalar, state: VState, acc: VInput) -> (VState):

    identity = sf.Matrix([  [1, 0, 0],
                            [0, 1, 0],
                            [0, 0, 1]])

    Phi = sf.Matrix.block_matrix([   [1 * identity, dt * identity, 0 * identity, 0.5*dt*dt * identity],
                        [0 * identity, 1 * identity,  0 * identity, dt * identity       ],
                        [0 * identity, 0 * identity,  1 * identity, 0  * identity       ],
                        [0 * identity, 0 * identity,  0 * identity, 1  * identity       ]])


    G = sf.Matrix.block_matrix([ [-0.5*dt*dt * identity], [-dt * identity], [0 * identity], [0 * identity]])

    return (Phi.inv() * (state - G*acc)).simplify()

def predictCov(dt: sf.Scalar, input_var: MDirections, bias_var: MDirections, acc_var: MDirections, covariance: MState) -> (MState):
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

    Q_acc = sf.Matrix.block_matrix([   [acc_var[0,0] * mult_mat_acc, 0 * mult_mat_acc, 0 * mult_mat_acc],
                                        [0 * mult_mat_acc, acc_var[1,1] * mult_mat_acc, 0 * mult_mat_acc],
                                        [0 * mult_mat_acc, 0 * mult_mat_acc, acc_var[2,2] * mult_mat_acc]])

    mult_mat = sf.Matrix([  [0, 0, 0, 0],
                            [0, 0, 0, 0],
                            [0, 0, 1, 0],
                            [0, 0, 0, 0]])

    Q_bias = sf.Matrix.block_matrix([   [bias_var[0,0] * mult_mat, 0 * mult_mat, 0 * mult_mat],
                                        [0 * mult_mat, bias_var[1,1] * mult_mat, 0 * mult_mat],
                                        [0 * mult_mat, 0 * mult_mat, bias_var[2,2] * mult_mat]])

    return G*input_var*G.T + Q_bias + Q_acc + Phi*covariance*Phi.T


def computeInnovCov(meas_unc: sf.Scalar, covariance: MState, meas_matrix: VMeas) -> (sf.Scalar):
    return (meas_matrix*covariance*meas_matrix.T)[0,0] + meas_unc

generate_px4_function(syncState, output_names=["state_updated"])
generate_px4_function(predictCov, output_names=["cov_updated"])
generate_px4_function(computeInnovCov, output_names=["innov_cov_updated"])