import SimulationObject from "../../common/simulation_object.js";
import {
  IntegrateQuatGlobal,
  IntegrateQuatLocal,
} from "../../common/integrators.js";
import {
  OutputVector,
  OutputValue,
  DrawLine,
  DrawAxes,
} from "../../common/draw_utils.js";

const { mat3, mat4, vec3, quat } = glMatrix;

export default class Simulation3 {
  object;
  camera;

  initialAngularMomentum;

  p5Instance;

  constructor(p5Instance) {
    this.p5Instance = p5Instance;

    this.object = new SimulationObject(
      [40, 80, 80],
      vec3.fromValues(0.0, 0.0, 0.0),
      vec3.fromValues(0.0, 0.0, 0.0),
      vec3.fromValues(1.0, 1.0, 1.0),
      vec3.fromValues(0.0, 0.0, 0.0),
      vec3.fromValues(0.01, -0.01, 0.01),
      10,
      vec3.fromValues(
        Math.floor(this.p5Instance.random(255)),
        Math.floor(this.p5Instance.random(255)),
        Math.floor(this.p5Instance.random(255)),
      ),
      p5Instance,
    );

    this.initialAngularMomentum = vec3.clone(this.object.angularMomentum);
  }

  setCamera() {
    this.camera = this.p5Instance.createCamera();
    this.p5Instance.camera(0, 0, 600, 0, 0, 0, 0, 1, 0);
  }

  update(dt) {
    const obj = this.object;

    vec3.copy(obj.rotation, obj.nextRotation);
    vec3.copy(obj.angularVelocity, obj.nextAngularVelocity);

    const g = vec3.create();
    vec3.cross(g, obj.angularVelocity, obj.angularMomentum);
    vec3.scale(g, g, -1);

    const newAngularMomentum = vec3.create();
    vec3.scaleAndAdd(newAngularMomentum, this.object.angularMomentum, g, dt);

    vec3.transformMat3(
      this.object.nextAngularVelocity,
      newAngularMomentum,
      this.object.invertInertialTensor,
    );

    vec3.copy(obj.angularMomentum, newAngularMomentum);

    IntegrateQuatLocal(
      this.object.nextRotation,
      this.object.nextAngularVelocity,
      dt,
    );
  }

  draw() {
    this.p5Instance.orbitControl();

    this.object.draw();

    DrawLine(
      this.initialAngularMomentum,
      vec3.fromValues(0.0, 0.0, 0.0),
      200,
      vec3.fromValues(180, 50, 50),
      this.p5Instance,
    );

    DrawAxes(
      this.object.position,
      this.object.nextRotation,
      100,
      this.p5Instance,
    );

    const cameraParams = [
      this.camera.eyeX,
      this.camera.eyeY,
      this.camera.eyeZ,
      this.camera.centerX,
      this.camera.centerY,
      this.camera.centerZ,
      0,
      -1,
      0,
    ];

    this.p5Instance.drawingContext.disable(
      this.p5Instance.drawingContext.DEPTH_TEST,
    );

    this.p5Instance.camera();

    this.p5Instance.push();

    this.p5Instance.translate(
      -this.p5Instance.width / 2,
      -this.p5Instance.height / 2,
    );

    OutputVector(
      "init",
      this.initialAngularMomentum,
      4,
      16,
      [10, 20],
      [10, 10, 10],
      this.p5Instance,
    );

    OutputVector(
      "current",
      this.object.angularMomentum,
      4,
      16,
      [10, 40],
      [10, 10, 10],
      this.p5Instance,
    );

    OutputValue(
      "energy",
      this.object.getRotationKineticEnergy(),
      4,
      16,
      [10, 60],
      [10, 10, 10],
      this.p5Instance,
    );

    this.p5Instance.pop();

    this.p5Instance.drawingContext.enable(
      this.p5Instance.drawingContext.DEPTH_TEST,
    );

    this.p5Instance.camera(...cameraParams);
  }
}
