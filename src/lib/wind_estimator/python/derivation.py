import os
from symforce import symbolic as sm
from symforce import geo
from symforce import typing as T

def fuse_airspeed(
        v_local: geo.V3,
        state: geo.V3,
        P: geo.M33,
        airspeed: T.Scalar,
        R: T.Scalar,
        epsilon: T.Scalar
) -> (geo.V3, geo.V3, T.Scalar, T.Scalar):

    vel_rel = geo.V3(v_local[0] - state[0], v_local[1] - state[1], v_local[2])
    airspeed_pred = vel_rel.norm(epsilon=epsilon) * state[2]

    innov = airspeed - airspeed_pred

    H = geo.V1(airspeed_pred).jacobian(state)
    innov_var = (H * P * H.transpose() + R)[0,0]

    K = P * H.transpose() / sm.Max(innov_var, epsilon)

    return (geo.V3(H), K, innov_var, innov)

# q: quaternion describing rotation from frame 1 to frame 2
# returns a rotation matrix derived form q which describes the same
# rotation
def quat2Rot(q):
    q0 = q[0]
    q1 = q[1]
    q2 = q[2]
    q3 = q[3]

    Rot = geo.M33([[q0**2 + q1**2 - q2**2 - q3**2, 2*(q1*q2 - q0*q3), 2*(q1*q3 + q0*q2)],
                  [2*(q1*q2 + q0*q3), q0**2 - q1**2 + q2**2 - q3**2, 2*(q2*q3 - q0*q1)],
                   [2*(q1*q3-q0*q2), 2*(q2*q3 + q0*q1), q0**2 - q1**2 - q2**2 + q3**2]])

    return Rot

def sign_no_zero(x) -> T.Scalar:
    """
    Returns -1 if x is negative, 1 if x is positive, and 1 if x is zero
    """
    return 2 * sm.Min(sm.sign(x), 0) + 1

def add_epsilon_sign(expr, var, eps):
    return expr.subs(var, var + eps * sign_no_zero(var))

def fuse_beta(
        v_local: geo.V3,
        state: geo.V3,
        P: geo.M33,
        q_att: geo.V4,
        R: T.Scalar,
        epsilon: T.Scalar
) -> (geo.V3, geo.V3, T.Scalar, T.Scalar):

    vel_rel = geo.V3(v_local[0] - state[0], v_local[1] - state[1], v_local[2])
    relative_wind_body = quat2Rot(q_att).T * vel_rel

    # small angle approximation of side slip model
    # protect division by zero using epsilon
    beta_pred = add_epsilon_sign(relative_wind_body[1] / relative_wind_body[0], relative_wind_body[0], epsilon)

    innov = 0.0 - beta_pred

    H = geo.V1(beta_pred).jacobian(state)
    innov_var = (H * P * H.transpose() + R)[0,0]

    K = P * H.transpose() / sm.Max(innov_var, epsilon)

    return (geo.V3(H), K, innov_var, innov)


def generatePx4Function(function_name, output_names):
    from symforce.codegen import Codegen, CppConfig
    codegen = Codegen.function(
            function_name,
            output_names=output_names,
            config=CppConfig())
    metadata = codegen.generate_function(
            output_dir="generated",
            skip_directory_nesting=True)

    print("Files generated in {}:\n".format(metadata.output_dir))
    for f in metadata.generated_files:
        print("  |- {}".format(os.path.relpath(f, metadata.output_dir)))

    # Replace cstdlib and Eigen functions by PX4 equivalents
    import fileinput

    with fileinput.FileInput(os.path.abspath(metadata.generated_files[0]), inplace=True, backup='.bak') as file:
        for line in file:
            line = line.replace("std::max", "math::max")
            line = line.replace("std::min", "math::min")
            line = line.replace("Eigen", "matrix")
            line = line.replace("matrix/Dense", "matrix/math.hpp")
            print(line, end='')

def generatePythonFunction(function_name, output_names):
    from symforce.codegen import Codegen, PythonConfig
    codegen = Codegen.function(
            function_name,
            output_names=output_names,
            config=PythonConfig())

    metadata = codegen.generate_function(
            output_dir="generated",
            skip_directory_nesting=True)

generatePx4Function(fuse_airspeed, output_names=["H", "K", "innov_var", "innov"])
generatePythonFunction(fuse_airspeed, output_names=["H", "K", "innov_var", "innov"])
generatePx4Function(fuse_beta, output_names=["H", "K", "innov_var", "innov"])
