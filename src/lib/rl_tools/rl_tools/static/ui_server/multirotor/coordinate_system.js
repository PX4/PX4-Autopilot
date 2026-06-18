import * as THREE from "../lib/three.module.js"

class CoordinateSystem{
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

export {
    CoordinateSystem
}