import SimulationObject from "../../../common/simulation_object.js";
import {
  IntegrateQuatGlobal,
  IntegrateQuatLocal,
} from "../../../common/util.js";

const { mat3, mat4, vec3, quat } = glMatrix;

export default class Simulation1 {
  object;

  worldInitialAngularMomentum;
  worldCurrentAngularMomentum;

  p5Instance;

  constructor(p5Instance) {
    this.p5Instance = p5Instance;

    this.object = new SimulationObject(
      vec3.fromValues(100, 50, 30),
      vec3.fromValues(0.0, 0.0, 0.0),
      vec3.fromValues(0.0, 0.0, 0.0),
      vec3.fromValues(1.0, 1.0, 1.0),
      vec3.fromValues(0.0, 0.0, 0.0),
      vec3.fromValues(0.0, 0.0, 0.0),
      this.p5Instance.random(10, 100),
      vec3.fromValues(
        Math.floor(this.p5Instance.random(255)),
        Math.floor(this.p5Instance.random(255)),
        Math.floor(this.p5Instance.random(255)),
      ),
      p5Instance,
    );

    const worldAngularVelocity = vec3.fromValues(
      this.p5Instance.random(0.0, 0.01),
      this.p5Instance.random(0.0, 0.01),
      this.p5Instance.random(0.0, 0.01),
    );

    const worldInertialTensor = this.object.getWorldInertialTensor();

    this.worldInitialAngularMomentum = vec3.create();
    vec3.transformMat3(
      this.worldInitialAngularMomentum,
      worldAngularVelocity,
      worldInertialTensor,
    );

    this.worldCurrentAngularMomentum = vec3.create();
  }

  update(dt) {
    const worldInertialTensor = this.object.getWorldInertialTensor();

    const invertWorldInertialTensor = mat3.create();
    mat3.invert(invertWorldInertialTensor, worldInertialTensor);

    const newWorldAngularVelocity = vec3.create();
    vec3.transformMat3(
      newWorldAngularVelocity,
      this.worldInitialAngularMomentum,
      invertWorldInertialTensor,
    );

    IntegrateQuatGlobal(this.object.rotation, newWorldAngularVelocity, dt);

    vec3.transformMat3(
      this.worldCurrentAngularMomentum,
      newWorldAngularVelocity,
      worldInertialTensor,
    );
  }

  draw() {
    this.p5Instance.camera(0, 0, 600, 0, 0, 0, 0, 1, 0);

    this.p5Instance.fill(255, 0, 0);

    this.p5Instance.text(this.worldInitialAngularMomentum, -120, -120, 50);
    this.p5Instance.text(this.worldCurrentAngularMomentum, -120, -100, 50);

    this.object.draw();
  }
}
