import SimulationObject from "../../../common/simulation_object.js";
import {
  IntegrateQuatGlobal,
  IntegrateQuatLocal,
} from "../../../common/util.js";

const { mat3, mat4, vec3, quat } = glMatrix;

export default class Simulation1 {
  object;

  initialAngularMomentum;

  p5Instance;

  constructor(p5Instance) {
    this.p5Instance = p5Instance;

    this.object = new SimulationObject(
      vec3.fromValues(100, 50, 30),
      vec3.fromValues(0.0, 0.0, 0.0),
      vec3.fromValues(0.0, 0.0, 0.0),
      vec3.fromValues(1.0, 1.0, 1.0),
      vec3.fromValues(0.0, 0.0, 0.0),
      vec3.fromValues(
        this.p5Instance.random(-0.001, 0.001),
        this.p5Instance.random(-0.001, 0.001),
        this.p5Instance.random(-0.001, 0.001),
      ),
      this.p5Instance.random(10, 100),
      vec3.fromValues(
        Math.floor(this.p5Instance.random(255)),
        Math.floor(this.p5Instance.random(255)),
        Math.floor(this.p5Instance.random(255)),
      ),
      p5Instance,
    );

    this.initialAngularMomentum = this.object.angularMomentum;
  }

  update(dt) {
    vec3.transformMat3(
      this.object.angularVelocity,
      this.initialAngularMomentum,
      this.object.invertInertialTensor,
    );

    IntegrateQuatLocal(this.object.rotation, this.object.angularVelocity, dt);

    this.object.updateAngularMomentum();
  }

  draw() {
    this.p5Instance.camera(0, 0, 600, 0, 0, 0, 0, 1, 0);

    this.p5Instance.fill(255, 0, 0);

    this.p5Instance.text(this.initialAngularMomentum, -120, -120, 50);
    this.p5Instance.text(this.object.angularMomentum, -120, -100, 50);

    this.object.draw();
  }
}
