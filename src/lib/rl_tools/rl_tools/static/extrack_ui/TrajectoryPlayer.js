import * as pako from 'https://cdn.jsdelivr.net/npm/pako@2.0.4/+esm';

async function fetchData(url) {
    const response = await fetch(url);
    return await response.json();
}
async function fetchAndDecompressData(url) {
    const response = await fetch(url);
    const compressedData = await response.arrayBuffer();
    const decompressedData = pako.ungzip(new Uint8Array(compressedData), { to: 'string' });
    return JSON.parse(decompressedData);
}

function getFileExtension(path) {
    return path.split('.').pop();
}

export class TrajectoryPlayer{
    constructor(ui_path, size, options) {
        // const experiments_stub = "../../experiments";
        // const modulePath = `${experiments_stub}/experiments/2024-05-25_14-28-34/32e6580_zoo_algorithm_environment/sac_pendulum-v1/0000/ui.esm.js`;
        this.size = size
        this.options = options || {};

        this.ui = import(ui_path)
        this.container = document.createElement('div');
        this.container.classList.add("trajectory-player-container")
        this.container.style.alignItems = "center";
        this.canvas_container = document.createElement('div');
        this.canvas_container.classList.add("trajectory-player-canvas-container")
        this.container.appendChild(this.canvas_container);
        this.loading_text = document.createElement('p');
        this.loading_text.style.display = "none";
        this.container.appendChild(this.loading_text);
        this.controls_container = document.createElement('div');
        this.controls_container.classList.add("trajectory-player-controls-container");

        this.container.appendChild(this.controls_container);

        this.canvas = null
    }
    getCanvas(){
        return this.container;

    }

    async playTrajectories(path) {
        this.loading_text.innerHTML = `Loading Trajectory Data from ${path}`
        this.loading_text.style.display = "inline";

        const trajectoryData = getFileExtension(path) === "json" ? await fetchData(path) : await fetchAndDecompressData(path)
        this.loading_text.style.display = "none";
        let currentEpisode = 0;
        let currentStep = 0;
        let currentEpisodeLength = 0;
        let currentEpisodeReturn = 0;
        let current_episode_return_multi = 0;

        const ui = await this.ui;

        const episode_info = document.createElement('div');
        episode_info.classList.add("trajectory-player-episode-info");

        this.controls_container.replaceChildren(episode_info);
        const previous_button = document.createElement('button');
        previous_button.innerHTML = "Previous Episode";
        this.controls_container.appendChild(previous_button);
        const restart_button = document.createElement('button');
        restart_button.innerHTML = "Restart Episode";
        this.controls_container.appendChild(restart_button);
        const skip_button = document.createElement('button');
        skip_button.innerHTML = "Next Episode";
        this.controls_container.appendChild(skip_button);

        var ratio = window.devicePixelRatio || 1;


        const onResize = () => {
            if(this.canvas){
                let size;
                if(this.size){
                    size = this.size;
                }
                else{
                    size = Math.min(this.canvas_container.clientWidth, this.canvas_container.clientHeight);
                }
                this.canvas.width = size * ratio;
                this.canvas.height = size * ratio;

                this.canvas.style.width = size + 'px';
                this.canvas.style.height = size + 'px';
            }
        }
        const resizeListener = new ResizeObserver(onResize);
        resizeListener.observe(this.canvas_container);
        onResize();
        window.addEventListener('resize', onResize);

        const dt = trajectoryData[0].trajectory[0].dt;
        const single = !Boolean(ui.render_multi)
        if(!single){
            previous_button.style.display = "none";
            skip_button.style.display = "none";
        }
        let current_parameters = null
        let ui_state = null
        let current_state = null
        let current_action = null
        let render_loop = {id: 0}
        let render_loops_running = {}
        let current_parameters_multi = null
        let current_step_data_multi = null

        const render = async () => {
            if (ui.render && ui.init){
                if(!ui_state){
                    if(current_parameters || current_parameters_multi){
                        this.canvas_container.innerHTML = "";
                        this.canvas = document.createElement('canvas');
                        this.canvas.classList.add("trajectory-player-canvas")
                        this.canvas_container.appendChild(this.canvas);
                        onResize()
                        console.log("Init Environment UI")
                        ui_state = await ui.init(this.canvas, {devicePixelRatio: window.devicePixelRatio})
                        if(ui_state.cursor_grab){
                            this.canvas.style.cursor = "grab"
                        }
                    }
                    else{
                        throw new Error('Init but parameters not set')
                    }
                }
                if(ui_state){
                    render_loop.id += 1;
                    await new Promise((resolve) => {
                        let interval_id = null
                        interval_id = setInterval(() => {
                            if(Object.keys(render_loops_running).length === 0){
                                clearInterval(interval_id)
                                resolve();
                            }
                        })
                    })
                    const current_id = render_loop.id;
                    render_loops_running[current_id] = true;
                    const loop = async () => {
                        if(render_loop.id === current_id){
                            if(single){
                                if(current_parameters && current_state && current_action){
                                    await ui.render(ui_state, current_parameters, current_state, current_action)
                                    requestAnimationFrame(loop);
                                    return
                                }
                            }
                            else{
                                if(current_parameters_multi && current_step_data_multi){
                                    await ui.render_multi(ui_state, current_parameters_multi, current_step_data_multi.map(step => step.state), current_step_data_multi.map(step => step.action))
                                    requestAnimationFrame(loop);
                                    return
                                }
                            }
                        }
                        console.log("Stopping render loop: ", current_id)
                        delete render_loops_running[current_id];
                    }
                    if(ui.episode_init){
                        if(single){
                            await ui.episode_init(ui_state, current_parameters)
                        }
                        else{
                            await ui.episode_init_multi(ui_state, current_parameters_multi)
                        }
                    }
                    requestAnimationFrame(loop)
                }
                else{
                    throw new Error('init function not successfull')
                }
            }
        }
        const step = () => {
            // const size = Math.min(this.canvas_container.clientWidth, this.canvas_container.clientHeight);
            // this.canvas.width = size;
            // this.canvas.height = size;
            if(this.options["verbose"]){
                episode_info.innerHTML = `Path: ${path}</br>`;
            }
            else{
                episode_info.innerHTML = "";
            }
            if(single){
                episode_info.innerHTML += `Episode: ${currentEpisode+1}/${trajectoryData.length}, Step: ${currentStep}, Return: ${currentEpisodeReturn.toFixed(2)}`;
                if (currentEpisode < trajectoryData.length) {
                    const { parameters, trajectory } = trajectoryData[currentEpisode];
                    current_parameters = parameters;
                    let skip = false;
                    if (currentStep < trajectory.length) {
                        const { state, action, reward, terminated, dt } = trajectory[currentStep];
                        current_state = state;
                        current_action = action;
                        currentEpisodeLength++;
                        currentEpisodeReturn += reward;
                        if(currentStep === 0){
                            render()
                        }
                        currentStep++;
                        if(terminated){
                            skip = true
                        }
                    } else {
                        skip = true
                    }
                    if(skip){
                        currentStep = 0;
                        currentEpisode++;
                        if(currentEpisode >= trajectoryData.length){
                            currentEpisode = 0;
                        }
                        console.log(`Episode ${currentEpisode} finished. Return = ${currentEpisodeReturn}, Length = ${currentEpisodeLength}`);
                        currentEpisodeLength = 0;
                        currentEpisodeReturn = 0;
                    }
                }
            }
            else{
                episode_info.innerHTML += `Step: ${currentStep}, Return (avg): ${current_episode_return_multi.toFixed(2)}`;
                current_parameters_multi = trajectoryData.map((episode) => episode.parameters);
                current_step_data_multi = trajectoryData.map((episode) => episode.trajectory[currentStep]);
                if(currentStep == 0) {
                    render()
                }
                current_episode_return_multi += trajectoryData.reduce((acc, episode) => acc + episode.trajectory[currentStep].reward, 0) / trajectoryData.length;
                currentStep++;
                if(currentStep >= trajectoryData[0].trajectory.length) {
                    currentStep = 0;
                    current_episode_return_multi = 0;
                }
            }
        }

        function previousEpisode() {
            currentStep = 0;
            currentEpisode--;
            if (currentEpisode < 0) {
                currentEpisode = trajectoryData.length - 1;
            }
            currentEpisodeLength = 0;
            currentEpisodeReturn = 0;
            console.log(`Skipped to Episode ${currentEpisode}`);
        }
        function restartEpisode() {
            currentStep = 0;
            currentEpisodeLength = 0;
            currentEpisodeReturn = 0;
            console.log(`Restarted Episode ${currentEpisode}`);
        }
        function skipEpisode() {
            currentStep = 0;
            currentEpisode++;
            if (currentEpisode >= trajectoryData.length) {
                currentEpisode = 0;
            }
            currentEpisodeLength = 0;
            currentEpisodeReturn = 0;
            console.log(`Skipped to Episode ${currentEpisode}`);
        }

        previous_button.addEventListener('click', previousEpisode);
        restart_button.addEventListener('click', restartEpisode);
        skip_button.addEventListener('click', skipEpisode);


        setInterval(step, dt * 1000);
    }

}

