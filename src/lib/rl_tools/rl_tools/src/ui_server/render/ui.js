import * as THREE from "three"
import {OrbitControls} from "three-orbitcontrols"
import {GLTFLoader} from "three-gltfloader"

export class CoordinateSystem{
    constructor(origin, length=1, diameter=0.01) {
        this.cs = new THREE.Group()
        const material_red = new THREE.MeshLambertMaterial({color: 0xAA0000})
        const material_green = new THREE.MeshLambertMaterial({color: 0x00AA00})
        const material_blue = new THREE.MeshLambertMaterial({color: 0x0000AA})
        const line = new THREE.BoxGeometry(length, diameter, diameter)
        var x = new THREE.Mesh( line, material_red);
        x.position.set(length/2, 0, 0)
        var y = new THREE.Mesh( line, material_green);
        y.position.set(0, length/2, 0)
        y.rotation.set(0, 0, Math.PI/2)
        var z = new THREE.Mesh( line, material_blue);
        z.position.set(0, 0, length/2)
        z.rotation.set(0, Math.PI/2, 0)
        this.cs.add(x)
        this.cs.add(y)
        this.cs.add(z)
        this.cs.position.set(origin[0], origin[1], origin[2])
    }
    get(){
        return this.cs
    }
}

function norm(a){
    return Math.sqrt(a.map(x => x**2).reduce((a, c) => a + c, 0))
}

function Matrix4FromRotMatTranspose(rotMat){
    const m = new THREE.Matrix4()
    m.set(
        rotMat[0][0], rotMat[1][0], rotMat[2][0], 0,
        rotMat[0][1], rotMat[1][1], rotMat[2][1], 0,
        rotMat[0][2], rotMat[1][2], rotMat[2][2], 0,
        0, 0, 0, 1)
    return m
}

function Matrix4FromRotMat(rotMat){
    const m = new THREE.Matrix4()
    m.set(
        rotMat[0][0], rotMat[0][1], rotMat[0][2], 0,
        rotMat[1][0], rotMat[1][1], rotMat[1][2], 0,
        rotMat[2][0], rotMat[2][1], rotMat[2][2], 0,
        0, 0, 0, 1)
    return m
}




class State{
    constructor(canvas, {devicePixelRatio, showAxes=false, capture=false, camera_position=[0.5, 0.5, 1], camera_distance=null, interactive=true, conta_url="/conta/"}){
        this.canvas = canvas
        this.devicePixelRatio = devicePixelRatio
        this.showAxes = showAxes
        this.cursor_grab = interactive // Instruct the embedding code to make the cursor a grab cursor
        this.render_tick = 0
        this.capture = capture
        this.camera_position = camera_position
        this.camera_distance = camera_distance
        this.interactive = interactive
        this.IS_MOBILE = this.is_mobile();
        this.lastCanvasWidth = 0
        this.lastCanvasHeight = 0
        this.conta_url = conta_url
    }

    is_mobile() {
        const isIOS = /iP(hone|ad|od)/.test(navigator.platform) || /iPhone|iPad|iPod/.test(navigator.userAgent);
        const isAndroid = /Android/.test(navigator.userAgent);
        const isMobile = /Mobi|Android/i.test(navigator.userAgent) || 'ontouchstart' in window || navigator.maxTouchPoints > 0;
        const isTablet = /iPad|Android(?!.*Mobile)/.test(navigator.userAgent);
        return isIOS || isAndroid || isMobile || isTablet;
    }

    async initialize(){
        const width = this.canvas.width
        const height = this.canvas.height
        this.scene = new THREE.Scene();
        this.camera = new THREE.PerspectiveCamera( 40, width / height, 0.1, 1000 );
        this.scene.add(this.camera);

        this.renderer = new THREE.WebGLRenderer({canvas: this.canvas, antialias: true, alpha: !this.IS_MOBILE, preserveDrawingBuffer: this.capture && !this.IS_MOBILE} );

        const dpr = !this.IS_MOBILE ? this.devicePixelRatio : Math.min(this.devicePixelRatio || window.devicePixelRatio || 1, 2)
        this.renderer.setPixelRatio(dpr)
        this.renderer.setClearColor(0xffffff, 0);

        this.renderer.setSize(width/this.devicePixelRatio, height/this.devicePixelRatio);

        this.lastCanvasWidth = this.canvas.width
        this.lastCanvasHeight = this.canvas.height


        // canvasContainer.appendChild(this.renderer.domElement );

        this.controls = this.interactive ? new OrbitControls(this.camera, this.renderer.domElement) : null;

        // this.controls.enabled = false;
        // window.addEventListener('keydown', (event) => {
        //     if (event.key === 'Alt') {
        //         this.controls.enabled = true;
        //         this.canvas.style.cursor = "grab"
        //     }
        // });

        // window.addEventListener('keyup', (event) => {
        //     if (event.key === 'Alt') {
        //         this.controls.enabled = false;
        //         this.canvas.style.cursor = "default"
        //     }
        // });
        this.canvas.title = "Alt+Drag to rotate the camera. Alt+CTRL+Drag to move the camera."

        this.simulator = new THREE.Group()
        this.simulator.rotation.set(-Math.PI / 2, 0, Math.PI / 2, 'XYZ');

        // const axesHelper = new THREE.AxesHelper(5);
        // this.scene.add(axesHelper)
        this.scene.add(this.simulator)

        var light = new THREE.AmbientLight( 0xffffff,0.5 ); // soft white light
        this.scene.add(light);
        var directionalLight = new THREE.DirectionalLight( 0xffffff, 0.4 )
        directionalLight.position.set(-100, 100, 0)
        directionalLight.target.position.set(0, 0, 0)
        this.scene.add( directionalLight )
        var directionalLight = new THREE.DirectionalLight( 0xffffff, 0.3 )
        directionalLight.position.set(0, 100, 100)
        directionalLight.target.position.set(0, 0, 0)
        this.scene.add( directionalLight )
        var directionalLight = new THREE.DirectionalLight( 0xffffff, 0.2 )
        directionalLight.position.set(0, 100, -100)
        directionalLight.target.position.set(0, 0, 0)
        this.scene.add( directionalLight )

        // this.camera.position.set(...this.camera_position)
        // this.camera.quaternion.set(-0.14, 0.70, 0.14, 0.68)
        // this.controls.target.set(0.0, 0.0, 0.0)
        // this.controls.minDistance = 1
        // this.controls.minDistance = 5
        // this.controls.update()

        this.camera_set = false
        this.THREE = THREE
    }

}


function thrust_direction_to_quaternion(thrust_direction){
    const x = thrust_direction[0];
    const y = thrust_direction[1];
    const z = thrust_direction[2];

    const z_unit = [0.0, 0.0, 1.0];

    let cross_x = z_unit[1] * z - z_unit[2] * y;
    let cross_y = z_unit[2] * x - z_unit[0] * z;
    let cross_z = z_unit[0] * y - z_unit[1] * x;

    const dot = z_unit[0] * x + z_unit[1] * y + z_unit[2] * z;

    const angle = Math.acos(dot);

    const cross_magnitude = Math.sqrt(cross_x * cross_x + cross_y * cross_y + cross_z * cross_z);
    if (cross_magnitude != 0) {
        cross_x /= cross_magnitude;
        cross_y /= cross_magnitude;
        cross_z /= cross_magnitude;
    }

    const half_angle = angle / 2.0;
    const sin_half_angle = Math.sin(half_angle);

    const qw = Math.cos(half_angle);
    const qx = cross_x * sin_half_angle;
    const qy = cross_y * sin_half_angle;
    const qz = cross_z * sin_half_angle;
    return [qw, qx, qy, qz];
}

export class DroneMesh{
  constructor(parameters, origin, displayIMUCoordinateSystem, displayActions, conta_url){
    console.assert(parameters.ui)
    this.group = new THREE.Group()
    const url = `${conta_url}${parameters.ui.model}`
    this.loaded = new GLTFLoader().loadAsync(url)
    this.loaded.then((gltf) => {
      const object = gltf.scene
      const object_group = new THREE.Group()
      object_group.add(object)
//      if(parameters.ui.name == "x500"){
      object_group.rotation.set(Math.PI / 2, 0, Math.PI / 2, 'ZYX')
      const scale = 0.5
      object_group.scale.set(scale, scale, scale)
//      }
      this.group.add(object_group)
    })
    if (displayIMUCoordinateSystem) {
      const scale = 1 //model.mass
      const coordinateSystemLength = Math.cbrt(scale)
      const coordinateSystemThickness = 0.01 * coordinateSystemLength
      this.group.add((new CoordinateSystem([0, 0, 0], coordinateSystemLength, coordinateSystemThickness)).get())
    }
  }
  get(){
    return this.group
  }
  set_action(action){ }
}

export class DroneDefault{
    constructor(parameters, origin, displayIMUCoordinateSystem, displayActions){
        const url = window.location.href;
        const urlObj = new URL(url);
        const params = new URLSearchParams(urlObj.search);
        if(params.has('L2FDisplayActions') === true){
            displayActions = params.get('L2FDisplayActions') === "true";
        }

        // console.log(model)
        this.origin = origin
        this.parameters = parameters
        this.droneFrame = new THREE.Group()
        this.drone = new THREE.Group()
        if(origin){
            this.drone.position.set(...origin)
        }
        // this.drone.add((new CoordinateSystem()).get())
        // this.drone.add((new CoordinateSystem(10 * this.scale, 0.1 * this.scale)).get())
        this.scale = parameters.dynamics.mass
        const material = new THREE.MeshLambertMaterial({color: 0xAAAAAA})
        const clockwise_rotor_material = new THREE.MeshLambertMaterial({color: 0x00FF00})
        const counter_clockwise_rotor_material = new THREE.MeshLambertMaterial({color: 0xFF0000})

        const coordinateSystemLength = Math.cbrt(this.scale)
        const coordinateSystemThickness = 0.01 * coordinateSystemLength

        const centerSize = Math.cbrt(this.scale) / 15
        const centerForm = new THREE.BoxGeometry(centerSize, centerSize, centerSize*0.3)
        const center = new THREE.Mesh( centerForm, material);

        this.parameters.dynamics["imu_position"] = [0, 0, 0]
        this.parameters.dynamics["imu_orientation"] = [1, 0, 0, 0]

        this.imuGroup = new THREE.Group()
        this.imuGroup.position.set(...this.parameters.dynamics.imu_position)
        this.imuGroup.quaternion.set(this.parameters.dynamics.imu_orientation[1], this.parameters.dynamics.imu_orientation[2], this.parameters.dynamics.imu_orientation[3], this.parameters.dynamics.imu_orientation[0])
        if (displayIMUCoordinateSystem) {
            this.imuGroup.add((new CoordinateSystem([0, 0, 0], coordinateSystemLength, coordinateSystemThickness)).get())
        }
        this.drone.add(this.imuGroup)
        this.drone.add(center)

        this.rotors = []

        const averageArmLength = this.parameters.dynamics.rotor_positions.map(position => norm(position)).reduce((a, c) => a + c, 0) / this.parameters.dynamics.rotor_positions.length
        for(const [rotorIndex, rotor_position] of this.parameters.dynamics.rotor_positions.entries()){
            let rotorCageRadiusFactor = 1
            let rotorCageThicknessFactor = 1
            const rotorCageRadius =  averageArmLength/3 * Math.sqrt(rotorCageRadiusFactor)
            const rotorCageThickness = averageArmLength/20 * Math.sqrt(rotorCageThicknessFactor)
            const armGroup = new THREE.Group()
            const length = norm(rotor_position)
            const armDiameter = averageArmLength/10
            const armLength = length - rotorCageRadius
            const armForm = new THREE.CylinderGeometry( armDiameter/2, armDiameter/2, armLength, 8 );
            const rot = new THREE.Quaternion(); // Geometry extends in y -> transform y to relative pos
            rot.setFromUnitVectors(new THREE.Vector3(...[0, 1, 0]), (new THREE.Vector3(...rotor_position)).normalize());
            armGroup.quaternion.set(rot.x, rot.y, rot.z, rot.w)

            const arm = new THREE.Mesh(armForm, material)
            arm.position.set(0, armLength/2, 0)
            armGroup.add(arm)

            const rotorGroup = new THREE.Group()
            rotorGroup.position.set(...rotor_position)

            const thrust_orientation = thrust_direction_to_quaternion(this.parameters.dynamics.rotor_thrust_directions[rotorIndex])
            rotorGroup.quaternion.set(thrust_orientation[3], thrust_orientation[0], thrust_orientation[1], thrust_orientation[2])
            // rotorGroup.add((new CoordinateSystem([0, 0, 0], 0.1, 0.01)).get())
            const rotorCageForm = new THREE.TorusGeometry(rotorCageRadius, rotorCageThickness, 16, 32 );
            const cageMaterial = (this.parameters.dynamics.rotor_thrust_directions[rotorIndex][2] < 0 ? clockwise_rotor_material : counter_clockwise_rotor_material)// new THREE.MeshLambertMaterial({color: 0xAAAAAA})
            const rotorCage = new THREE.Mesh(rotorCageForm, cageMaterial)
            rotorGroup.add(rotorCage)

            const forceArrow = new THREE.ArrowHelper(new THREE.Vector3(0,0,1), new THREE.Vector3(0,0,0 ), 0, 0x000000);
            if(displayActions){
                rotorGroup.add(forceArrow)
            }

            this.drone.add(rotorGroup)
            this.drone.add(armGroup)
            this.droneFrame.add(this.drone)
            this.rotors.push({
                forceArrow,
                rotorCage
            })
        }

    }
    get(){
        return this.droneFrame
    }
    // setState(state){
    //   const mat = Matrix4FromRotMat(state.orientation)
    //   this.droneFrame.quaternion.setFromRotationMatrix(mat)
    //   this.droneFrame.position.set(state.pose.position[0] + this.origin[0], state.pose.position[1] + this.origin[1], state.pose.position[2] + this.origin[2])
    //   const avg_rot_rate = state.rotor_states.reduce((a, c) => a + c["power"], 0)/state.rotor_states.length
    //   state.rotor_states.map((rotorState, i) => {
    //     const forceArrow = this.rotors[i].forceArrow
    //     const rotorCage = this.rotors[i].rotorCage
    //     const min_rpm = this.model.rotors[i].min_rpm
    //     const max_rpm = this.model.rotors[i].max_rpm


    //     const rot_rate = rotorState["power"]
    //     const force_magnitude = (rot_rate - avg_rot_rate)/max_rpm * 10///1000
    //     forceArrow.setDirection(new THREE.Vector3(0, 0, rot_rate)) //Math.sign(force_magnitude)))
    //     forceArrow.setLength(Math.cbrt(this.this.scale)/10) //Math.abs(force_magnitude))
    //   })
    // }
    set_action(action){
        for(let i = 0; i < 4; i++){
            const forceArrow = this.rotors[i].forceArrow
            const force_magnitude = action[i]
            forceArrow.setDirection(new THREE.Vector3(0, 0, force_magnitude))
            forceArrow.setLength(Math.cbrt(this.scale)/10)
        }
    }

}

async function drone_factory(parameters, origin, displayIMUCoordinateSystem, displayActions, conta_url){
  if(parameters.ui && parameters.ui.enable && parameters.ui.model){
    try{
      const model = new DroneMesh(parameters, origin, displayIMUCoordinateSystem, displayActions, conta_url)
      await model.loaded
      return model
    }
    catch(error){
      console.error("An error occurred:", error.message);
    }
  }
  return new DroneDefault(parameters, origin, displayIMUCoordinateSystem, displayActions)
}

export async function init(canvas, options){
    const state = new State(canvas, options)
    await state.initialize()
    return state
}
function clear_episode(ui_state){
    if(ui_state.drone){
        ui_state.simulator.remove(ui_state.drone.get())
        if(ui_state.showAxes){
            ui_state.simulator.remove(ui_state.origin_coordinate_system.get())
        }
    }
    if(ui_state.drones){
        ui_state.drones.map(drone => ui_state.simulator.remove(drone.get()))
        if(ui_state.showAxes){
            ui_state.origin_coordinate_systems.map(cs => ui_state.simulator.remove(cs.get()))
        }
    }
}
function set_camera(ui_state, distance){
    const scale = 1/Math.sqrt(ui_state.camera_position[0]**2 + ui_state.camera_position[1]**2 + ui_state.camera_position[2]**2) * distance
    if(!ui_state.camera_set){
        ui_state.camera.position.set(ui_state.camera_position[0] * scale, ui_state.camera_position[1] * scale, ui_state.camera_position[2] * scale)
        ui_state.camera.lookAt(0, 0, 0)
        ui_state.camera_set = true
        ui_state.controls.update()
    }
}
export async function episode_init(ui_state, parameters){
    let distance = (parameters.ui && parameters.ui.camera_distance) ? parameters.ui.camera_distance : Math.cbrt(parameters.dynamics.mass) * 2
    if(ui_state.camera_distance){
        distance = ui_state.camera_distance
    }
    set_camera(ui_state, distance)
    clear_episode(ui_state)
    ui_state.drone = await drone_factory(parameters, [0, 0, 0], ui_state.showAxes, false, ui_state.conta_url)
    ui_state.simulator.add(ui_state.drone.get())
    if(ui_state.showAxes){
        ui_state.origin_coordinate_system = new CoordinateSystem([0, 0, 0], 1 * scale, 0.01 * scale)
        ui_state.simulator.add(ui_state.origin_coordinate_system.get())
    }
}

export async function episode_init_multi(ui_state, parameters){
    const grid_distance = 0.0
    const grid_size = Math.ceil(Math.sqrt(parameters.length))
    let distance = (grid_distance > 0 ? grid_distance * grid_size * 2 : Math.cbrt(parameters[0].dynamics.mass))
    if(parameters.ui && parameters.ui.camera_distance){
        distance = parameters.ui.camera_distance
    }
    if(ui_state.camera_distance){
        distance = ui_state.camera_distance
    }
    set_camera(ui_state, distance)
    clear_episode(ui_state)
    ui_state.drones = []
    if(!ui_state.showAxes && ui_state.origin_coordinate_systems){
        ui_state.origin_coordinate_systems.forEach(cs => {
            ui_state.simulator.remove(cs.get())
        })
    }
    ui_state.origin_coordinate_systems = []
    await Promise.all(parameters.map(async (parameter, i) => {
        const x = (i % grid_size) * grid_distance
        const y = Math.floor(i / grid_size) * grid_distance
        const drone = await drone_factory(parameter, [x, y, 0], ui_state.showAxes, false, ui_state.conta_url)
        ui_state.simulator.add(drone.get())
        if(ui_state.showAxes){
            const cs = new CoordinateSystem([x, y, 0], 1, 0.01)
            ui_state.simulator.add(cs.get())
            ui_state.origin_coordinate_systems.push(cs)
        }
        ui_state.drones.push(drone)
    }))
}

function update_camera(ui_state){
    const currentWidth = ui_state.canvas.width
    const currentHeight = ui_state.canvas.height
    const hasResized = currentWidth !== ui_state.lastCanvasWidth || currentHeight !== ui_state.lastCanvasHeight

    if (hasResized) {
        const width = currentWidth / ui_state.devicePixelRatio;
        const height = currentHeight / ui_state.devicePixelRatio;

        if (ui_state.camera.aspect !== width / height) {
            ui_state.camera.aspect = width / height;
            ui_state.camera.updateProjectionMatrix();
        }

        if (ui_state.renderer) {
            ui_state.renderer.setPixelRatio(ui_state.devicePixelRatio);
            ui_state.renderer.setSize(width, height, false);
        }

        ui_state.lastCanvasWidth = currentWidth
        ui_state.lastCanvasHeight = currentHeight
    }

    if(ui_state.interactive && ui_state.controls){
        ui_state.controls.update()
    }
    if(ui_state.renderer){
        ui_state.renderer.render(ui_state.scene, ui_state.camera);
    }
    ui_state.render_tick += 1
}

function clip_position(scale, position){
    const extent = Math.cbrt(scale) * 300 // to maybe prevent threejs from exploding
    const max_position = extent
    const min_position = -extent
    return position.map((p) => {
        if(p > max_position){
            return max_position
        }
        else if(p < min_position){
            return min_position
        }
        else{
            return p
        }
    })
}

export async function render(ui_state, parameters, state, action) {
    if(ui_state.drone){
        ui_state.drone.get().position.set(...clip_position(parameters.dynamics.mass, state.position))
        ui_state.drone.get().quaternion.copy(new THREE.Quaternion(state.orientation[1], state.orientation[2], state.orientation[3], state.orientation[0]).normalize())
    }
    update_camera(ui_state)
}

export async function render_multi(ui_state, parameters, states, actions){
    if(ui_state.drones && ui_state.drones.length == states.length){
        states.map((state, i) => {
            const action = actions[i]
            const current_parameters = parameters[i]
            ui_state.drones[i].get().position.set(...clip_position(current_parameters.dynamics.mass, state.position))
            ui_state.drones[i].get().quaternion.copy(new THREE.Quaternion(state.orientation[1], state.orientation[2], state.orientation[3], state.orientation[0]).normalize())
            ui_state.drones[i].set_action(action)
        })
    }
    update_camera(ui_state)
}
