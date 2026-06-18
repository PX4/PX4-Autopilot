

function R(alpha) {
    return [
        [Math.cos(alpha), -Math.sin(alpha)],
        [Math.sin(alpha), Math.cos(alpha)]
    ];
}

function globalPosition(x_offset, y_offset, x, y, alpha, s, scale) {
    const rotation = R(alpha);
    return [
        x_offset + s.x * scale + (rotation[0][0] * x + rotation[0][1] * y) * scale,
        y_offset - s.y * scale - (rotation[1][0] * x + rotation[1][1] * y) * scale
    ];
}

function drawWheel(ctx, x_offset, y_offset, x, y, size, width, delta, s, scale) {
    const start = [-size / 2 * Math.cos(delta), -size / 2 * Math.sin(delta)];
    const finish = [size / 2 * Math.cos(delta), size / 2 * Math.sin(delta)];

    const startPosition = globalPosition(x_offset, y_offset, x + start[0], y + start[1], s.mu, s, scale);
    const finishPosition = globalPosition(x_offset, y_offset, x + finish[0], y + finish[1], s.mu, s, scale);

    ctx.beginPath();
    ctx.moveTo(...startPosition);
    ctx.lineTo(...finishPosition);
    ctx.strokeStyle = 'red';
    ctx.lineWidth = width * scale;
    ctx.stroke();
}


export function drawCar(canvas, ctx, parameters, state, action, ratio, carScale) {
    let h = canvas.height / ratio;
    let w = canvas.width / ratio;
    let x_offset = w / 2;
    let y_offset = h / 2;

    let L = parameters.lr + parameters.lf;
    let W = L / 2.5;

    let position = [0, 0];
    position = globalPosition(x_offset, y_offset, -parameters.lr, 0, state.mu, state, carScale);
    ctx.beginPath();
    ctx.moveTo(position[0], position[1]);
    position = globalPosition(x_offset, y_offset, +parameters.lf, 0, state.mu, state, carScale);
    ctx.lineTo(position[0], position[1]);
    ctx.strokeStyle = 'blue';
    ctx.lineWidth = W * carScale;
    ctx.stroke();

    let wheel_size = L / 2;
    let wheel_width = L / 10;

    // Assuming draw_wheel is adapted to JavaScript and added here
    drawWheel(ctx, x_offset, y_offset, -parameters.lr, W / 2 + wheel_width / 2, wheel_size, wheel_width, 0, state, carScale);
    drawWheel(ctx, x_offset, y_offset, -parameters.lr, -W / 2 - wheel_width / 2, wheel_size, wheel_width, 0, state, carScale);
    drawWheel(ctx, x_offset, y_offset, parameters.lf, W / 2 + wheel_width / 2, wheel_size, wheel_width, action[1], state, carScale);
    drawWheel(ctx, x_offset, y_offset, parameters.lf, -W / 2 - wheel_width / 2, wheel_size, wheel_width, action[1], state, carScale);
}