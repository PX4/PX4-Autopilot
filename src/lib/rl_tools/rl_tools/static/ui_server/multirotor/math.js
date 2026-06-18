
import * as THREE from "../lib/three.module.js"

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

export {
    norm,
    Matrix4FromRotMat,
    Matrix4FromRotMatTranspose
}