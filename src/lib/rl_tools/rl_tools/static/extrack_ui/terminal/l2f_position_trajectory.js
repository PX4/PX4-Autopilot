for (let trajectory_i = 0; trajectory_i < 10; trajectory_i++){
    var outputs = [
        {label: 'x', data: data[Object.keys(data)[0]][trajectory_i].trajectory.map((x, i) => {return {"x": i, "y": x["state"]["position"][0]}})},
        {label: 'y', data: data[Object.keys(data)[0]][trajectory_i].trajectory.map((x, i) => {return {"x": i, "y": x["state"]["position"][1]}})},
        {label: 'z', data: data[Object.keys(data)[0]][trajectory_i].trajectory.map((x, i) => {return {"x": i, "y": x["state"]["position"][2]}})},
    ]
    output.scatter(outputs, "step", "value", `Trajectory ${trajectory_i}`)
}