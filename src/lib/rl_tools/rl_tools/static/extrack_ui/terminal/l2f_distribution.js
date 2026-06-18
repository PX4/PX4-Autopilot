mass_thrust_no_crash = []
mass_thrust_crash = []
tw_ti_no_crash = []
tw_ti_crash = []
tw_ti_scale_no_crash = []
tw_ti_scale_crash = []
initial_position_orientation_no_crash = []
initial_position_orientation_crash = []
Object.keys(data).forEach(path => {
    data[path].forEach(trajectory =>{
        // console.log(trajectory.parameters.dynamics.rotors[0].thrust_curve)
        const single_rotor_thrust = trajectory.parameters.dynamics.rotors[0].thrust_curve.reduce((a, c) => a + c)
        mass_thrust = {
            x: trajectory.parameters.dynamics.mass,
            y: single_rotor_thrust * 4
        }
        const thrust_to_weight = single_rotor_thrust * 4/ (trajectory.parameters.dynamics.mass*9.81)
        const rotor_position = trajectory.parameters.dynamics.rotors[0].pose.position
        const rotor_lever = Math.sqrt(rotor_position[0] * rotor_position[0] + rotor_position[1] * rotor_position[1] + rotor_position[2] * rotor_position[2])
        const torque = rotor_lever * Math.sqrt(2) * single_rotor_thrust
        const inertia = trajectory.parameters.dynamics.J[0][0]
        const torque_to_inertia = torque / inertia
        tw_ti = {
            x: thrust_to_weight,
            y: torque_to_inertia
        }
        tw_ti_scale = {
            x: rotor_lever,
            y: thrust_to_weight / torque_to_inertia
        }
        const initial_position = trajectory.trajectory[0].state.position
        const initial_distance = Math.sqrt(initial_position[0] * initial_position[0] * initial_position[1] * initial_position[1] * initial_position[2] * initial_position[2])
        const initial_orientation = trajectory.trajectory[0].state.orientation
        const [w, x, y, z] = initial_orientation;
        const length = Math.sqrt(x * x + y * y + z * z + w * w);
        const angle = 2 * Math.acos(w / length);
        const angle_degrees = angle * (180 / Math.PI);
        let angle_degrees_normalized = angle_degrees % 360;
        if (angle_degrees_normalized < 0) angle_degrees_normalized += 360;
        angle_degrees_normalized = Math.min(angle_degrees_normalized, 360 - angle_degrees_normalized);
        const position_orientation = {
            x: initial_distance,
            y: angle_degrees_normalized
        }

        if(trajectory.trajectory.some(x => x.terminated === true) === true){
            mass_thrust_crash.push(mass_thrust)
            tw_ti_crash.push(tw_ti)
            tw_ti_scale_crash.push(tw_ti_scale)
            initial_position_orientation_crash.push(position_orientation)
        }
        else{
            mass_thrust_no_crash.push(mass_thrust)
            tw_ti_no_crash.push(tw_ti)
            tw_ti_scale_no_crash.push(tw_ti_scale)
            initial_position_orientation_no_crash.push(position_orientation)
        }
    })
})


output.scatter([{label: "no crash", data: mass_thrust_no_crash}, {label: "crash", data: mass_thrust_crash}], "Mass [kg]", "Thrust [N]", "Thrust to Mass")
output.scatter([{label: "no crash", data: tw_ti_no_crash}, {label: "crash", data: tw_ti_crash}], "Thrust to Weight Ratio", "Torque to Inertia Ratio", "Thrust to weight / Torque to Inertia")
output.scatter([{label: "no crash", data: tw_ti_scale_no_crash}, {label: "crash", data: tw_ti_scale_crash}], "Radius [m]", "Thrust to Weight to Torque to Inertia Ratio", "Thrust to weight / Torque to Inertia by Scale")
output.scatter([{label: "no crash", data: initial_position_orientation_no_crash}, {label: "crash", data: initial_position_orientation_crash}], "Initial Distance From Origin [m]", "Initial Orientation [degrees]", "Initial Position / Orientation")
