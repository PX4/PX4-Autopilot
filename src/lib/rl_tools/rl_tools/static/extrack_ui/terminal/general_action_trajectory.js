for (let trajectory_i = 0; trajectory_i < 10; trajectory_i++){
    var outputs = [
        {label: 'actor_0', data: data[Object.keys(data)[0]][trajectory_i].trajectory.map((x, i) => {return {"x": i, "y": x["action"][0]}})},
        {label: 'actor_1', data: data[Object.keys(data)[0]][trajectory_i].trajectory.map((x, i) => {return {"x": i, "y": x["action"][1]}})},
        {label: 'actor_2', data: data[Object.keys(data)[0]][trajectory_i].trajectory.map((x, i) => {return {"x": i, "y": x["action"][2]}})},
        {label: 'actor_3', data: data[Object.keys(data)[0]][trajectory_i].trajectory.map((x, i) => {return {"x": i, "y": x["action"][3]}})},
    ]
    output.scatter(outputs, "step", "value", `Trajectory ${trajectory_i}`)
}