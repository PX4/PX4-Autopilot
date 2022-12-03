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
            output_dir="src/modules/landing_target_estimator/python_derivation/generated/decoupled_moving_xyzb",
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



#-------------------------------- DECOUPLED DYNAMICS, MOVING TARGET ------------------------------------------ #

class State:
    r = 0,
    r_dot = 1,
    b = 2,
    at = 3,
    n_states = 4

class Input:
    a = 0
    n_inputs = 1

class Directions:
    x = 0,
    nb_directions = 1

class MState(sf.Matrix):
    SHAPE = (State.n_states, State.n_states)

class VMeas(sf.Matrix):
    SHAPE = (1, State.n_states)

class MDirections(sf.Matrix):
    SHAPE = (Directions.nb_directions, Directions.nb_directions)

def predictCov(dt: sf.Scalar, input_var: sf.Scalar, bias_var: sf.Scalar, acc_var: sf.Scalar, covariance: MState) -> (MState):
    Phi = sf.Matrix([  [1, dt, 0, 0.5*dt*dt],
                    [0, 1,  0, dt       ],
                    [0, 0,  1, 0        ],
                    [0, 0,  0, 1        ]])

    G = sf.Matrix([-0.5*dt*dt, -dt, 0, 0])

    Q_bias = sf.Matrix([   [0, 0, 0      , 0],
                    [0, 0, 0      , 0],
                    [0, 0, bias_var, 0],
                    [0, 0, 0      , 0]])

    # Target acc noise:
    Q_acc = sf.Matrix([ [0, 0, 0, 0],
                        [0, 0, 0, 0],
                        [0, 0, 0, 0],
                        [0, 0, 0, acc_var]])

    return (G*input_var*G.T + Q_bias + Q_acc + Phi*covariance*Phi.T)

def computeInnovCov(meas_unc: sf.Scalar, covariance: MState, meas_matrix: VMeas) -> (sf.Scalar):
    return ((meas_matrix*covariance*meas_matrix.T)[0,0] + meas_unc)

generate_px4_function(predictCov, output_names=["cov_updated"])
generate_px4_function(computeInnovCov, output_names=["innov_cov_updated"])