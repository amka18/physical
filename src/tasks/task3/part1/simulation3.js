import SimulationObject from "../../../common/simulation_object.js";
import {
  IntegrateQuatGlobal,
  IntegrateQuatLocal,
} from "../../../common/integrators.js";
import {
  OutputVector,
  OutputValue,
  DrawLine,
  DrawAxes,
} from "../../../common/draw_utils.js";

const { mat3, mat4, vec3, quat } = glMatrix;

export default class Simulation3 {
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
        this.p5Instance.random(0.001, 0.003),
        this.p5Instance.random(0.001, 0.003),
        this.p5Instance.random(0.001, 0.003),
      ),
      80,
      vec3.fromValues(
        Math.floor(this.p5Instance.random(255)),
        Math.floor(this.p5Instance.random(255)),
        Math.floor(this.p5Instance.random(255)),
      ),
      p5Instance,
    );

    this.initialAngularMomentum = vec3.clone(this.object.angularMomentum);
  }

  update(dt) {
    const torque = vec3.create();
    vec3.cross(
      torque,
      this.object.nextAngularVelocity,
      this.object.angularMomentum,
    );
    vec3.scale(torque, torque, -1);

    const newAngularMomentum = vec3.create();
    vec3.scaleAndAdd(
      newAngularMomentum,
      this.object.angularMomentum,
      torque,
      dt,
    );

    vec3.transformMat3(
      this.object.nextAngularVelocity,
      newAngularMomentum,
      this.object.invertInertialTensor,
    );

    vec3.copy(this.object.angularMomentum, newAngularMomentum);

    IntegrateQuatLocal(this.object.nextRotation, this.object.nextAngularVelocity, dt);
  }

  draw() {
    this.p5Instance.camera(0, 0, 600, 0, 0, 0, 0, 1, 0);

    OutputVector(
      "init",
      this.initialAngularMomentum,
      3,
      vec3.fromValues(-120, -120, 50),
      vec3.fromValues(10, 20, 10),
      this.p5Instance,
    );

    OutputVector(
      "current",
      this.object.angularMomentum,
      5,
      vec3.fromValues(-70, -120, 50),
      vec3.fromValues(10, 10, 10),
      this.p5Instance,
    );

    OutputValue(
      "energy",
      this.object.getRotationKineticEnergy(),
      3,
      vec3.fromValues(0, -120, 50),
      vec3.fromValues(10, 10, 10),
      this.p5Instance,
    );

    DrawLine(
      this.initialAngularMomentum,
      vec3.fromValues(0.0, 0.0, 0.0),
      200,
      vec3.fromValues(180, 50, 50),
      this.p5Instance,
    );

    DrawLine(
      this.object.angularMomentum,
      vec3.fromValues(0.0, 0.0, 0.0),
      200,
      vec3.fromValues(50, 180, 50),
      this.p5Instance,
    );

    DrawAxes(this.object.nextPosition, this.object.nextRotation, 100, this.p5Instance);

    this.object.draw();
  }
}
