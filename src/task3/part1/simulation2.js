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

export default class Simulation2 {
  object;
  camera;

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
        this.p5Instance.random(0.002, 0.003),
        this.p5Instance.random(0.002, 0.003),
        this.p5Instance.random(0.002, 0.003),
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

  setCamera() {
    this.camera = this.p5Instance.createCamera();
    this.p5Instance.camera(0, 0, 600, 0, 0, 0, 0, 1, 0);
  }

  update(dt) {
    vec3.transformMat3(
      this.object.angularVelocity,
      this.initialAngularMomentum,
      this.object.invertInertialTensor,
    );

    IntegrateQuatLocal(
      this.object.nextRotation,
      this.object.nextAngularVelocity,
      dt,
    );

    this.object.updateAngularMomentum();
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

    this.p5Instance.pop();

    this.p5Instance.drawingContext.enable(
      this.p5Instance.drawingContext.DEPTH_TEST,
    );

    this.p5Instance.camera(...cameraParams);
  }
}
