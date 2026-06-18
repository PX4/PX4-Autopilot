import {main as car_main} from './car/main.js';
import {
    renderCanvas as car_renderCanvas,
    renderCanvasContainer as car_renderCanvasContainer,
    renderControlInputs as car_renderControlInputs,
    renderMessageLabels as car_renderMessageLabels
} from './car/render.js';


const returns_chart_ctx = document.getElementById('returns-chart').getContext('2d');

const data = {
    labels: [],
    datasets: [{
        label: 'Return',
        backgroundColor: '#6fd0cb',
        borderColor: '#7DB9B6',
        data: [],
        borderWidth: 2,
        fill: 'start'
    }]
};

const options = {
    responsive: true,
    maintainAspectRatio: false,
    animation: {
        duration: 0
    },
    scales: {
        xAxes: [{
            display: true,
            gridLines: {
                display: false
            }
        }],
        yAxes: [{
            display: true,
            gridLines: {
                display: false
            },
            ticks: {
                beginAtZero: true,
                suggestedMax: 100
            }
        }]
    }
};

const returns_chart = new Chart(returns_chart_ctx, {
    type: 'line',
    data: data,
    options: options
});

const terminalContainer = document.getElementById("terminal-container");
const terminalOutput = document.getElementById("terminal-output");
function appendToTerminal(text) {
    if(terminalContainer.style.display !== "none"){
        terminalOutput.textContent += text + "\n";
    }
}
const pendulum = document.getElementById("pendulum");
let pendulum_transform_default = getComputedStyle(pendulum).transform;
console.log("Default transform: ", pendulum_transform_default)
function setPendulumAngle(angle) {
    pendulum.style.transform = `${pendulum_transform_default} rotate(${angle - Math.PI}rad)`;
}

const realtime_checkbox = document.getElementById("realtime-checkbox");
const seed_input = document.getElementById("seed-input");

let async_main = (async () => {
    const worker = new Worker('./training_worker.js', {type: "module"});
    worker.addEventListener("error", (error) => {
        console.error("An error occurred in the Web Worker:", error);
    });

    worker.addEventListener("message", (event) => {
        if (event.data.type) {
            if(event.data.payload){
                appendToTerminal(event.data.type + ": " + JSON.stringify(event.data.payload));
            }
            else{
                appendToTerminal(event.data.type);
            }
        }
        // console.log("Message from worker", event.data);
    });
    function sendMessageToWorker(type, payload) {
        return new Promise((resolve, reject) => {
            const id = Math.random().toString(36); // Generate a unique ID for the message
            const msg = {id, type, payload}
            // console.log("Sending message to worker", msg);
            worker.postMessage(msg);

            const handleMessage = (event) => {
                if (event.data.id === id) {
                    worker.removeEventListener('message', handleMessage);
                    resolve(event.data.payload);
                }
            };

            worker.addEventListener('message', handleMessage);
        });
    }

    let sleep = (ms) => {
        return new Promise(resolve => setTimeout(resolve, ms));
    }
    await sleep(500);
    await sendMessageToWorker("initialize", {benchmark: false});
    document.getElementById("training-button").disabled = false;
    document.getElementById("training-button").value = "Start training";


    let training = false;
    document.getElementById("training-button").addEventListener("click", async () => {
        gtag('event', 'pendulum_training_button', {"training": training});
        if(!training){
            training = true;
            document.getElementById("training-button").value = "Stop training";
            returns_chart.data.labels = [];
            returns_chart.data.datasets[0].data = [];
            returns_chart.update();

            let seed = parseInt(seed_input.value);
            if(isNaN(seed)){
                seed = 0;
            }

            await sendMessageToWorker("initialize_training_state", {seed: seed});
            let finished = false;
            let advance = async () => {
                let answer = await sendMessageToWorker("train_one_step");
                finished = answer.training_finished;
                let state = answer.state;
                // console.log("Step: ", answer.step, " Finished: ", finished, " State: ", state, " Episode: ", answer.episode, " Episode return: ", answer.episode_return)
                if(answer.episode_return){
                    returns_chart.data.labels.push(answer.step);
                    returns_chart.data.datasets[0].data.push(answer.episode_return);
                    returns_chart.update({
                        duration: 10,
                        easing: 'easeOutBounce'
                    });
                }
                if(answer.step % 1 === 0){
                    let output = "Step: " + answer.step + " Episode: " + answer.episode;
                    if(answer.episode_return){
                        output += " Episode return: " + answer.episode_return;
                    }
                    appendToTerminal(output);
                }
                setPendulumAngle(state[0])
            }
            const target_dt = 0.05;
            while(!finished && training){
                let start = performance.now();
                await advance();
                let end = performance.now();
                let dt = (end - start) / 1000;
                let sleep_time = Math.max(0, target_dt - dt);
                if(realtime_checkbox.checked){
                    // console.log("Sleeping for ", sleep_time, " seconds")
                    await new Promise(resolve => setTimeout(resolve, sleep_time * 1000));
                }
            }

        }
        else{
            training = false;
            document.getElementById("training-button").value = "Start training";
            await sendMessageToWorker("destroy_training_state");
        }
    });
});


window.addEventListener("DOMContentLoaded", () => {
    const messageContainer = document.getElementById("car-message-container");
    car_renderMessageLabels(messageContainer);
    const controlContainer = document.getElementById("car-control-container");
    car_renderControlInputs(controlContainer);
    const canvasContainer = document.getElementById("car-canvas-container");
    car_renderCanvasContainer(canvasContainer);
})
window.addEventListener("load", () =>{
    if (window.Worker) {
        try {
            async_main();
            car_main();
        } catch (error) {
            console.error("Error running main", error);
        }
    } else {
        console.log('Web Workers are not supported in your browser.');
    }
})

