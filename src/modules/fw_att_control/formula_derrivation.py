#!/usr/bin/env python3
import symforce.symbolic as sf


def section(title: str) -> None:
    print("\n" + "=" * 80)
    print(title)
    print("=" * 80)


def derive_turn_coordination() -> None:
    section("Turn coordination")

    # Quaternion attitude of the aircraft.
    # This represents the rotation from body frame -> earth frame.
    R = sf.Rot3.symbolic("q")

    # Body angular velocity vector.
    # This contains the aircraft rotational rates:
    #   w_b[0] = roll rate (p)
    #   w_b[1] = pitch rate (q)
    #   w_b[2] = yaw rate (r)
    w_b = sf.V3.symbolic("w_b")

    # True airspeed magnitude of the aircraft.
    v = sf.Symbol("v")

    # Angle of attack (angle between body x-axis and velocity vector).
    alpha = sf.Symbol("alpha")
    q_w, q_x, q_y, q_z = sf.symbols("q_w q_x q_y q_z")

    # Velocity vector expressed in the body frame.
    #
    # We assume no sideslip, meaning the velocity lies in the body x-z plane.
    v_b = sf.V3(v * sf.cos(alpha), 0, v * sf.sin(alpha))

    # Gravity magnitude.
    g_z = sf.Symbol("g")
    g = sf.V3(0, 0, -g_z)

    # Centrifugal acceleration experienced during a turn.
    #
    # a = -ω × v
    #
    # This comes from the rotating reference frame of the aircraft.
    # If the aircraft is turning, the velocity vector produces a
    # centrifugal acceleration relative to the body frame.
    a_centrifugal = -w_b.cross(v_b)

    # Total body acceleration required to maintain the turn.
    #
    # We must cancel the centrifugal acceleration AND account for gravity.
    #
    # Gravity is defined in the earth frame, so we rotate it into the
    # body frame using R.inverse().
    a_b = -a_centrifugal + R.inverse() * g

    # Steady coordinated turn assumption:
    #
    # Roll rate p = 0 (bank angle is constant)
    #
    # This removes transient roll motion from the equations.
    a_b = a_b.subs({w_b[0]: 0})

    print("a_b =")
    print(a_b)

    # Solve for the body yaw rate r = w_b[2].
    #
    # We use the lateral acceleration component a_b[1].
    #
    # In a coordinated turn this must be zero (no lateral slip).
    # Solving this gives the yaw rate required to sustain the turn.
    yaw_rate_body = sf.solve(a_b[1], w_b[2])[0]

    print("\nyaw_rate_body =")
    print(yaw_rate_body)

    # Now compute the centrifugal acceleration expressed in the earth frame.
    #
    # This helps derive relationships between pitch rate and yaw rate
    # that maintain the circular flight path.
    a_centrifugal_e = R * a_centrifugal

    # For simplicity we assume zero angle of attack.
    #
    # This isolates the rotational dynamics of the turn itself.
    a_centrifugal_e = a_centrifugal_e.subs({alpha: 0})

    print("\na_centrifugal_e =")
    print(a_centrifugal_e)

    # Solve the vertical earth-frame centrifugal acceleration component
    # for pitch rate q = w_b[1].
    #
    # This determines the pitch rate required to maintain the circular
    # trajectory while the aircraft is yawing.
    pitch_rate_body = sf.solve(a_centrifugal_e[2], w_b[1])[0]

    print("\npitch_rate_body (raw) =")
    print(pitch_rate_body)

    # Substitute the yaw-rate expression into the pitch-rate equation.
    #
    # This removes w_b[2] and expresses the pitch rate purely in terms
    # of quaternion attitude, gravity, airspeed, and angle of attack.
    pitch_rate_body_subbed = pitch_rate_body.subs({w_b[2]: yaw_rate_body})

    print("\npitch_rate_body_with_yaw_subbed =")
    print(pitch_rate_body_subbed)


def derive_yaw_offset() -> None:
    section("Yaw offset")

    # Current aircraft attitude quaternion.
    q = sf.Quaternion.symbolic("q")

    # Desired attitude quaternion setpoint.
    q_sp = sf.Quaternion.symbolic("q_sp")

    # Small yaw correction we want to solve for.
    yaw_off = sf.Symbol("yaw_off")

    q_w, q_x, q_y, q_z = sf.symbols("q_w q_x q_y q_z")
    q_sp_w, q_sp_x, q_sp_y, q_sp_z = sf.symbols("q_sp_w q_sp_x q_sp_y q_sp_z")

    # Quaternion representing a small yaw rotation.
    #
    # For small angles:
    #   q ≈ [0, 0, yaw/2, 1]
    #
    # This approximation simplifies the algebra.
    q_yaw_off = sf.Quaternion(xyz=sf.V3(0.0, 0.0, yaw_off / 2), w=1)

    print("q_yaw_off =")
    print(q_yaw_off)

    # Compute attitude error quaternion.
    #
    # delta_q represents the rotation needed to move
    # from the current attitude q to the desired attitude q_sp
    # with an additional yaw offset applied.
    delta_q = q.conj() * q_yaw_off * q_sp

    print("\ndelta_q.z =")
    print(delta_q.z)

    # Solve the z component of the error quaternion for yaw_off.
    #
    # Setting delta_q.z = 0 corresponds to removing yaw error.
    yaw_off_solution = sf.solve(delta_q.z, yaw_off)[0]

    print("\nyaw_off =")
    print(yaw_off_solution)


if __name__ == "__main__":
    derive_turn_coordination()
    derive_yaw_offset()
