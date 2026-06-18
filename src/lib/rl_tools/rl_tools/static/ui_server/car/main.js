import {Track} from './track.js';
import {Client as ClientWS} from './client.js';
import {Client as ClientWASM} from './client_wasm.js';


console.log("Car UI")

const forceWASM = false
// const forceWASM = true

const keyThrottleValue = 0.5
const orientationGainSteering = 3
const orientationGainThrottle = 3
let first_orientation = null
let playbackSpeed = 3

let main = async () => {
    const canvas = document.getElementById('car-drawingCanvas');
    const canvasContainer = document.getElementById('car-canvasContainer');
    const resetTrackButton = document.getElementById('car-resetTrackButton');
    const saveTrackButton = document.getElementById('car-saveTrackButton');
    const playButton = document.getElementById('car-playButton');
    const trainButton = document.getElementById('car-trainButton');
    const drawLabel = document.getElementById('car-drawLabel');
    const loadingLabel = document.getElementById('car-loadingLabel');
    const trainLabel = document.getElementById('car-trainLabel');
    const playLabel = document.getElementById('car-playLabel');
    const playbackSpeedCheckbox = document.getElementById('car-playbackSpeedCheckbox');
    const playbackSpeedCheckboxLabel = document.getElementById('car-playbackSpeedCheckboxLabel');

    let response = await fetch('./scenario');
    let Client = ClientWASM;
    if(response.status == 200 && !forceWASM){
        Client = ClientWS;
    }

    // const client = new Client();
    const client = new Client();
    let track = null;
    client.setEnvironmentCallbacks(
        {
            setParametersCallback: (parameters)=>{
                console.log('Parameters:', parameters);
                track = new Track(canvas, parameters);
                loadingLabel.style.display = "none";
                drawLabel.style.display = "block";
                resetTrackButton.style.display = "block";
                saveTrackButton.style.display = "block";
                if(canvasContainer){
                    canvasContainer.style.display = "block";
                }
                track.resizeCanvas()
            },
            setStateCallback: (state)=>{
                const overlay = document.getElementById('car-drawingCanvasTrainingOverlay')
                if(overlay){
                    overlay.style.display = 'none'
                }
                if(track){
                    track.state = [state.state];
                    track.action = [state.action];
                }
            },
            setTruncatedCallback: (state)=>{
                const overlay = document.getElementById('car-drawingCanvasTrainingOverlay')
                if(overlay){
                    overlay.style.display = 'flex'
                    requestAnimationFrame(() => {
                        overlay.classList.add('show');
                    });
                }
            },
            setActionCallback: (data)=>{
                if(track){
                    track.action = [data.action];
                }
            },
        }
    )
    resetTrackButton.addEventListener('click', ()=>{
        track.reset();
    });
    saveTrackButton.addEventListener('click', ()=>{
        track.disable_drawing()
        client.sendMessage("setTrack", track.track);
        resetTrackButton.style.display = "none";
        saveTrackButton.style.display = "none";
        playButton.style.display = "block";
        trainButton.style.display = "block";
        drawLabel.style.display = "none";
    });

    let mode_interactive = false;

    const input_action = {
        "throttle": 0,
        "steering": 0
    }
    document.addEventListener('keydown', function(event) {
        if(mode_interactive){
            let update = false;
            switch(event.key) {
                case "ArrowUp":
                    input_action["throttle"] = keyThrottleValue;
                    update = true;
                    break;
                case "ArrowDown":
                    input_action["throttle"] = -keyThrottleValue;
                    update = true;
                    break;
                case "ArrowLeft":
                    input_action["steering"] = 1;
                    update = true;
                    break;
                case "ArrowRight":
                    input_action["steering"] = -1;
                    update = true;
                    break;
                default:
                    break;
            }
            if(update){
                event.preventDefault();
                client.sendMessage("setAction", [input_action["throttle"], input_action["steering"]]);
            }
        }
    });
    document.addEventListener('keyup', function(event) {
        if(mode_interactive){
            let update = false;
            switch(event.key) {
                case "ArrowUp":
                    input_action["throttle"] = 0;
                    update = true;
                    break;
                case "ArrowDown":
                    input_action["throttle"] = 0;
                    update = true;
                    break;
                case "ArrowLeft":
                    input_action["steering"] = 0;
                    update = true;
                    break;
                case "ArrowRight":
                    input_action["steering"] = 0;
                    update = true;
                    break;
                default:
                    break;
            }
            if(update){
                client.sendMessage("setAction", [input_action["throttle"], input_action["steering"]]);
            }
        }
    });
    playButton.addEventListener('click', ()=>{
        mode_interactive = true
        client.sendMessage("setAction", [0, 0]);
        playButton.style.display = "none";
        playLabel.style.display = "block";
        if (typeof DeviceOrientationEvent !== 'undefined' && typeof DeviceOrientationEvent.requestPermission === 'function') {
            DeviceOrientationEvent.requestPermission()
                .then(permissionState => {
                    if (permissionState === 'granted') {
                        // document.getElementById('car-status').textContent = 'Permission granted';
                    }
                })
                .catch(console.error); // Handle errors
        } else {
            // Handle regular non-iOS 13+ devices
            // window.addEventListener('deviceorientation', handleOrientationEvent);
            // document.getElementById('car-status').textContent = 'Permission API not required';
        }
    });
    trainButton.addEventListener('click', ()=>{
        gtag('event', 'car_training_button');
        mode_interactive = false
        client.sendMessage("startTraining", null);
        playButton.style.display = "none";
        trainButton.style.display = "none";
        trainLabel.style.display = "block";
        playLabel.style.display = "none";
        playbackSpeedCheckboxLabel.style.display = "block";
        client.sendMessage("setPlaybackSpeed", playbackSpeedCheckbox.checked ? 1 : playbackSpeed);
    });
    playbackSpeedCheckbox.addEventListener('change', ()=>{
        client.sendMessage("setPlaybackSpeed", playbackSpeedCheckbox.checked ? 1 : playbackSpeed);
    });
    window.addEventListener('deviceorientation', function(event) {
        if(mode_interactive){
            if(!first_orientation){
                first_orientation = event;
            }
            input_action["steering"] = Math.max(-1, Math.min(1, (-event.gamma + first_orientation.gamma)/90 * orientationGainSteering));
            input_action["throttle"] = Math.max(-1, Math.min(1, (-event.beta + first_orientation.beta)/90 * orientationGainThrottle));
            client.sendMessage("setAction", [input_action["throttle"], input_action["steering"]]);
        }
    });
}

export {main};