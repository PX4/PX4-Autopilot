import * as THREE from "../lib/three.module.js"
import {CoordinateSystem} from "./coordinate_system.js"
import {norm, Matrix4FromRotMat, Matrix4FromRotMatTranspose} from "./math.js"

class Drone{
  constructor(model, origin, envParams, displayIMUCoordinateSystem, displayActions){
    // console.log(model)
    this.origin = origin
    this.model = model
    this.envParams = envParams
    this.droneFrame = new THREE.Group()
    this.drone = new THREE.Group()
    // this.drone.add((new CoordinateSystem()).get())
    // this.drone.add((new CoordinateSystem(10 * model.mass, 0.1 * model.mass)).get())
    const material = new THREE.MeshLambertMaterial({color: 0xAAAAAA})
    const clockwise_rotor_material = new THREE.MeshLambertMaterial({color: 0x00FF00})
    const counter_clockwise_rotor_material = new THREE.MeshLambertMaterial({color: 0xFF0000})

    const coordinateSystemLength = Math.cbrt(model.mass)
    const coordinateSystemThickness = 0.01 * coordinateSystemLength

    const centerSize = Math.cbrt(model.mass) / 10
    const centerForm = new THREE.BoxGeometry(centerSize, centerSize, centerSize*0.3)
    const center = new THREE.Mesh( centerForm, material);
    // this.drone.quaternion.set(Math.sqrt(0.5), Math.sqrt(0.5), 0,0) // ENUtoNED
    this.imuGroup = new THREE.Group()
    this.imuGroup.position.set(...model.imu.pose.position)
    this.imuGroup.quaternion.setFromRotationMatrix(Matrix4FromRotMat(model.imu.pose.orientation))
    if (displayIMUCoordinateSystem) {
      this.imuGroup.add((new CoordinateSystem([0, 0, 0], coordinateSystemLength, coordinateSystemThickness)).get())
    }
    this.drone.add(this.imuGroup)
    this.drone.add(center)

    this.rotors = []

    const averageArmLength = model.rotors.map(rotor => norm(rotor.pose.position)).reduce((a, c) => a + c, 0) / model.rotors.length
    for(const [rotorIndex, rotor] of model.rotors.entries()){
      let rotorCageRadiusFactor = 1
      let rotorCageThicknessFactor = 1
      if (this.envParams != null){
        const rotorParams = this.envParams.rotors[rotorIndex]
        // console.log(this.envParams)
        if (rotorParams.thrust_curve.factors[1].constructor == Object){
          if (Array.isArray(rotorParams.thrust_curve.factors[1].parameters)){
            const mean1 = rotorParams.thrust_curve.factors[1].parameters.reduce((a, c)=>a+c, 0) / rotorParams.thrust_curve.factors[1].parameters.length
            rotorCageThicknessFactor = rotor.thrust_curve.factor_1/mean1
          }
          else{
            if ("upper" in rotorParams.thrust_curve.factors[1]){
              rotorCageThicknessFactor = rotor.thrust_curve.factor_1/((rotorParams.thrust_curve.factors[1].upper - rotorParams.thrust_curve.factors[1].lower)/2 + rotorParams.thrust_curve.factors[1].lower)
            }
          }
        }
        if (rotorParams.thrust_curve.factors[2].constructor == Object){
          if (Array.isArray(rotorParams.thrust_curve.factors[2].parameters)){
            const mean2 = rotorParams.thrust_curve.factors[2].parameters.reduce((a, c)=>a+c, 0) / rotorParams.thrust_curve.factors[2].parameters.length
            rotorCageThicknessFactor = rotor.thrust_curve.factor_2/mean2
          }
          else{
            if ("upper" in rotorParams.thrust_curve.factors[2]){
              rotorCageThicknessFactor = rotor.thrust_curve.factor_2/((rotorParams.thrust_curve.factors[2].upper - rotorParams.thrust_curve.factors[2].lower)/2 + rotorParams.thrust_curve.factors[2].lower)
            }
          }
        }
      }
      const rotorCageRadius =  averageArmLength/3 * Math.sqrt(rotorCageRadiusFactor)
      const rotorCageThickness = averageArmLength/20 * Math.sqrt(rotorCageThicknessFactor)
      const armGroup = new THREE.Group()
      const length = norm(rotor.pose.position)
      const armDiameter = averageArmLength/10
      const armLength = length - rotorCageRadius
      const armForm = new THREE.CylinderGeometry( armDiameter/2, armDiameter/2, armLength, 8 );
      const rot = new THREE.Quaternion(); // Geometry extends in y -> transform y to relative pos
      rot.setFromUnitVectors(new THREE.Vector3(...[0, 1, 0]), (new THREE.Vector3(...rotor.pose.position)).normalize());
      armGroup.quaternion.set(rot.x, rot.y, rot.z, rot.w)

      const arm = new THREE.Mesh(armForm, material)
      arm.position.set(0, armLength/2, 0)
      armGroup.add(arm)

      const rotorGroup = new THREE.Group()
      rotorGroup.position.set(...rotor.pose.position)
      rotorGroup.quaternion.setFromRotationMatrix(Matrix4FromRotMat(rotor.pose.orientation))
      // rotorGroup.add((new CoordinateSystem([0, 0, 0], 0.1, 0.01)).get())
      const rotorCageForm = new THREE.TorusGeometry(rotorCageRadius, rotorCageThickness, 16, 32 );
      const cageMaterial = (rotor.spin_orientation_clockwise ? clockwise_rotor_material : counter_clockwise_rotor_material)// new THREE.MeshLambertMaterial({color: 0xAAAAAA})
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
  setState(state){
    // console.log(state)
    const mat = Matrix4FromRotMat(state.pose.orientation)
    this.droneFrame.quaternion.setFromRotationMatrix(mat)
    this.droneFrame.position.set(state.pose.position[0] + this.origin[0], state.pose.position[1] + this.origin[1], state.pose.position[2] + this.origin[2])
    const avg_rot_rate = state.rotor_states.reduce((a, c) => a + c["power"], 0)/state.rotor_states.length
    state.rotor_states.map((rotorState, i) => {
      const forceArrow = this.rotors[i].forceArrow
      const rotorCage = this.rotors[i].rotorCage
      const min_rpm = this.model.rotors[i].min_rpm
      const max_rpm = this.model.rotors[i].max_rpm


      const rot_rate = rotorState["power"]
      const force_magnitude = (rot_rate - avg_rot_rate)/max_rpm * 10///1000
      // const force_magnitude = (rot_rate - min_rpm) / (max_rpm - min_rpm)
      forceArrow.setDirection(new THREE.Vector3(0, 0, rot_rate)) //Math.sign(force_magnitude)))
      forceArrow.setLength(Math.cbrt(this.model.mass)/10) //Math.abs(force_magnitude))
      // const c1 = [1, 0, 0]
      // const c2 = [0, 1, 0]
      // const upper_rot_rate_limit = 10000
      // const a = Math.min(1, Math.max(0, (rot_rate / upper_rot_rate_limit + 1)/2))
      // rotorCage.material.color.set(new THREE.Color(c1[0] * a + c2[0]*(1-a), c1[1] * a + c2[1]*(1-a), c1[2] * a + c2[2]*(1-a)))
    })
  }

}

const example_state = {
  "pose": {
    "position": [0, 0, 0],
    "orientation": [
      [1, 0, 0],
      [0, 1, 0],
      [0, 0, 1]
    ],
  },
  "rotor_states": [
    { "rpm": 0 },
    { "rpm": 0 },
    { "rpm": 0 },
    { "rpm": 0 }
  ] 
}


export {Drone, example_state}