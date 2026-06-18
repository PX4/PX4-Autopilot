const experiments_stub = "/experiments"
const run_stub = Object.keys(data)[0].split("/").slice(0, -3).join("/")
ui_path = `${experiments_stub}/${run_stub}/ui.esm.js`

const container = output.container
const training_step_data = data[Object.keys(data)[0]]
const old_execution_id = window.terminal_execution_id === undefined ? -1 : window.terminal_execution_id;
const execution_id = old_execution_id + 1;
window.terminal_execution_id = execution_id
async function main(){
    console.log(ui_path)

    const ui = await import(ui_path)

    container.style.display = "block"
    container.innerHTML = "hello"

    container.innerHTML = ""
    container.style.textAlign = "center"
    const canvas = document.createElement('canvas')
    canvas.width = 800
    canvas.height = 800
    container.appendChild(canvas);
    container.appendChild(document.createElement("br"))
    const button = document.createElement("button")
    button.innerText = "reset"
    container.appendChild(button)
    ui_state = await ui.init(canvas, {devicePixelRatio: window.devicePixelRatio})
    if(ui_state.cursor_grab){
        canvas.style.cursor = "grab"
    }
    const drones = await Promise.all(training_step_data.map(async trajectory => {
        const parameters = trajectory.parameters
        parameters.ui = {
            camera_distance: 20
        }
        const drone = new ui.Drone(parameters)
        ui_state.simulator.add(drone.get())
        const steps = trajectory.trajectory
        return {parameters, drone, steps}
    }))
    const normal_dt = drones[0].steps[0].dt
    const rollover_dt = 1
    let dt = rollover_dt
    let current_step = 0;
    let last_animation_frame = new Date()
    button.addEventListener('click', () => {
        current_step = 0;
        dt = rollover_dt;
        last_animation_frame = new Date()
    });
    async function loop(){
        const now = new Date()
        if((now - last_animation_frame)/1000 >= dt){
            current_step += 1
            if(current_step >= drones[0].steps.length){
                current_step = 0
                dt = rollover_dt
            }
            else{
                dt = normal_dt
            }
            last_animation_frame = now
        }
        drones.map(drone => {
            const state = drone.steps[current_step].state
            drone.drone.drone.position.set(...state.position)
            drone.drone.drone.quaternion.copy(new ui_state.THREE.Quaternion(state.orientation[1], state.orientation[2], state.orientation[3], state.orientation[0]).normalize())
        })
        ui_state.controls.update()
        ui_state.renderer.render(ui_state.scene, ui_state.camera);
        if(window.terminal_execution_id === execution_id){
            requestAnimationFrame(loop)
        }
    }
    loop()
}
main()
