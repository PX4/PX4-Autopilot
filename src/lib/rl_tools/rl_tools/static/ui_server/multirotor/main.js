import * as THREE from "../lib/three.module.js"
import {Drone} from "./drone.js"
import {CoordinateSystem} from "./coordinate_system.js"
import {Simulator} from "./simulator.js"
import {default_model} from "./default_model.js"


console.log("Multirotor UI")

async function onload(){
  let scenario = await fetch('scenario')
    .then(response => {
      if (!response.ok) {
        throw new Error('Fetching scenario was not successful ' + response.statusText);
      }
      return response.text();
    })
  console.log("The scenario is: " + scenario)
  let parameters = {
    "capture": false,
    "width": 1920,
    "height": 1080,
    "cameraDistanceToOrigin": 1,
  }

  var canvasContainer = document.getElementById("canvasContainer")
  let simulator = new Simulator(canvasContainer, parameters)
  window.simulator = simulator

  window.drones = {}
  window.origin_coordinate_systems = {}

  window.removeDrone = id => {
    if(id in window.drones){
      // console.log("Removing drone")
      simulator.remove(window.drones[id].get())
      delete window.drones[id]
    }
    if(id in window.origin_coordinate_systems){
      // console.log("Removing origin frame")
      simulator.remove(window.origin_coordinate_systems[id].get())
      delete window.origin_coordinate_systems[id]
    }
  }
  window.addDrone = (
      id,
      origin,
      model,
      {
        displayGlobalCoordinateSystem = true,
        displayIMUCoordinateSystem = true,
        displayActions = true
      }
  ) => {
    window.removeDrone(id)
    window.drones[id] = new Drone(model, origin, null, displayIMUCoordinateSystem, displayActions)
    // drone.get().position.set(0, 0.2, 0.2)
    simulator.add(window.drones[id].get())
    if (displayGlobalCoordinateSystem){
      let cs = new CoordinateSystem(origin, 1, 0.01)
      simulator.add(cs.get())
      window.origin_coordinate_systems[id] = cs
    }
    window
  }

  window.setInfo = (info) => {
    const infoContainer = document.getElementById("infoContainer")
    infoContainer.style.display = "block"
    infoContainer.innerHTML = info
  }

  function onWindowResize(){
    if (!capture){
      const width = canvasContainer.offsetWidth
      const height = canvasContainer.offsetHeight
      simulator.camera.aspect =  width / height
      simulator.camera.updateProjectionMatrix()
      simulator.renderer.setSize(width, height)
    }
  }
  onWindowResize()
  window.addEventListener('resize', onWindowResize, false);



  var button = document.getElementById("button")
  var seed_input = document.getElementById("seed-input")
  var animateButton = function(e) {
    e.preventDefault;
    //reset animation
    e.target.classList.remove('animate');

    e.target.classList.add('animate');
    setTimeout(function(){
      e.target.classList.remove('animate');
    },700);
  };

  button.addEventListener('click', animateButton, false);


  document.addEventListener("keypress", function onPress(event) {
    if (event.key === "u" && event.ctrlKey) {
      document.getElementById("controlContainer").style.display = document.getElementById("controlContainer").style.display == "none" ? "block" : "none"
    }
  });

  window.getFrame = function(){
    return renderer.domElement.toDataURL("image/png");
  }

  var ws = new WebSocket('ws://' + window.location.host + "/ui");

  ws.onopen = function(event) {
    console.log('Connection opened:', event);
  };

  ws.onmessage = function(event) {
    console.log("Message: ", event.data)
    let {channel, data} = JSON.parse(event.data)
    if (channel === "addDrone") {
      window.addDrone(data.id, data.origin, data.model || default_model, data.display_options);
    }
    else{
      if (channel === "setDroneState") {
        if(data.id in window.drones){
          window.drones[data.id].setState(data.data)
        }
        else{
          throw new Error("Drone not found")
        }
      }
      else{
        if (channel === "removeDrone") {
          window.removeDrone(data.id)
        }
        else{
          if (channel === "status") {
            button.innerHTML = "Training progress: " + Math.round(data.progress * 100) + "%"
            if(data.finished && !window.finished){
              window.finished = true;
              document.getElementById("canvasContainer").style.display = "none"
              document.getElementById("controlContainer").style.display = "none"
              var resultContainer = document.getElementById("resultContainer")
              // resultContainer.innerHTML = "Total training time: " + Math.round(data.time) + "s"
              resultContainer.innerHTML = "Finished"
              resultContainer.style.display = "block"
            }
          }
        }
      }
    }
  };

  ws.onerror = function(error) {
    console.error('WebSocket Error:', error);
  };

  ws.onclose = function(event) {
    if (event.wasClean) {
      console.log('Connection closed cleanly, code=', event.code, 'reason=', event.reason);
    } else {
      console.error('Connection died');
    }
  };

  button.addEventListener("click", event => {
    seed_input.style.display = "none"
    button.disabled = true;
    ws.send(JSON.stringify({
      "channel": "startTraining",
      "data": {
        "seed": parseInt(seed_input.value || 0)
      }
    }))
  }, false)
}

window.onload = function(){
  onload()
}