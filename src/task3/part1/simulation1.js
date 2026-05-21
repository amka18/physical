import SceneObject from "./scene_object.js";
import {
  updateQuatGlobal,
  updateQuatLocal,
  calculateRotationEnergy,
} from "./utils.js";
import { OutputVector, OutputValue, DrawLine, DrawAxes } from "./draw_utils.js";

const { mat3, mat4, vec3, quat } = glMatrix;

export default class Simulation1 {
  object;
  camera;

  wL0;
  wL;
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

    const w0 = vec3.create();
    vec3.set(w0, 0.001, 0.01, 0.001);

    const wI = this.object.getWI();

    this.wL0 = vec3.create();
    vec3.transformMat3(this.wL0, w0, wI);

    this.wL = vec3.clone(this.wL0);

    this.Ek = calculateRotationEnergy(wI, w0);
  }

  setCamera() {
    this.camera = this.p5Instance.createCamera();
    this.p5Instance.camera(0, 0, 600, 0, 0, 0, 0, 1, 0);
  }

  update(dt) {
    const obj = this.object;

    const wI = obj.getWI();
    const wInvI = obj.getWInvI();

    const w = vec3.create();
    vec3.transformMat3(w, this.wL0, wInvI);

    updateQuatGlobal(obj.rotation, w, dt);

    vec3.transformMat3(this.wL, w, wI);
    this.Ek = calculateRotationEnergy(wI, w);
  }

  draw() {
    this.p5Instance.orbitControl();

    const obj = this.object;

    obj.draw();

    DrawLine(
      this.wL0,
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

    OutputVector(
      "L0",
      this.wL0,
      4,
      16,
      [10, 20],
      [10, 10, 10],
      this.p5Instance,
    );

    OutputValue("E", this.Ek, 4, 16, [10, 60], [10, 10, 10], this.p5Instance);

    OutputVector("L", this.wL, 4, 16, [10, 40], [10, 10, 10], this.p5Instance);

    this.p5Instance.pop();

    this.p5Instance.drawingContext.enable(
      this.p5Instance.drawingContext.DEPTH_TEST,
    );

    this.p5Instance.camera(...cameraParams);
  }
}
