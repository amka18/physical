import SceneObject from "./scene_object.js";
import {
  updateQuatGlobal,
  updateQuatLocal,
  calculateRotationEnergy,
} from "./utils.js";
import { OutputVector, OutputValue, DrawLine, DrawAxes } from "./draw_utils.js";

const { mat3, mat4, vec3, quat } = glMatrix;

export default class Simulation3 {
  object;
  camera;

  L0;
  L;
  Ek;

  p5Instance;

  constructor(p5Instance) {
    this.p5Instance = p5Instance;

    this.object = new SceneObject(
      [20, 30, 60],
      vec3.fromValues(0, 0, 0),
      vec3.fromValues(0, 0, 0),
      vec3.fromValues(0, 0, 0),
      vec3.fromValues(0.001, 0.01, 0.001),
      vec3.fromValues(20, 0, -20),
      5,
      vec3.fromValues(
        Math.floor(this.p5Instance.random(255)),
        Math.floor(this.p5Instance.random(255)),
        Math.floor(this.p5Instance.random(255)),
      ),
      p5Instance,
    );

    const obj = this.object;

    this.L0 = vec3.create();
    vec3.transformMat3(this.L0, obj.angularVelocity, obj.inertialTensor);

    this.L = vec3.clone(this.L0);

    this.Ek = calculateRotationEnergy(obj.inertialTensor, obj.angularVelocity);
  }

  setCamera() {
    this.camera = this.p5Instance.createCamera();
    this.p5Instance.camera(0, 0, 600, 0, 0, 0, 0, 1, 0);
  }

  update(dt) {
    const obj = this.object;

    const g = vec3.create();
    vec3.cross(g, this.L, obj.angularVelocity);
    const invIg = vec3.create();
    vec3.transformMat3(invIg, g, obj.invInertialTensor);

    obj.angularVelocity[0] = obj.angularVelocity[0] + invIg[0] * dt;
    obj.angularVelocity[1] = obj.angularVelocity[1] + invIg[1] * dt;
    obj.angularVelocity[2] = obj.angularVelocity[2] + invIg[2] * dt;

    vec3.transformMat3(this.L, obj.angularVelocity, obj.inertialTensor);

    this.Ek = calculateRotationEnergy(obj.inertialTensor, obj.angularVelocity);

    updateQuatLocal(obj.rotation, obj.angularVelocity, dt);
  }

  draw() {
    this.p5Instance.orbitControl();

    const obj = this.object;

    obj.draw();

    DrawLine(
      this.L0,
      vec3.fromValues(0.0, 0.0, 0.0),
      200,
      vec3.fromValues(180, 50, 50),
      this.p5Instance,
    );

    DrawAxes(obj.position, obj.rotation, 100, this.p5Instance);

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

    OutputVector("L0", this.L0, 4, 16, [10, 20], [10, 10, 10], this.p5Instance);

    OutputVector("L", this.L, 4, 16, [10, 40], [10, 10, 10], this.p5Instance);

    OutputValue("E", this.Ek, 4, 16, [10, 60], [10, 10, 10], this.p5Instance);

    this.p5Instance.pop();

    this.p5Instance.drawingContext.enable(
      this.p5Instance.drawingContext.DEPTH_TEST,
    );

    this.p5Instance.camera(...cameraParams);
  }
}
