import SceneObject from "./scene_object.js";
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

export default class Simulation4 {
  object;
  camera;

  L0;
  i;
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

    this.L0 = this.calculateAngularMomentum(
      this.object.inertialTensor,
      this.object.angularVelocity,
    );

    this.Ek = 0.0;

    // this.object = new SimulationObject(
    //   vec3.fromValues(100, 50, 30),
    //   vec3.fromValues(0.0, 0.0, 0.0),
    //   vec3.fromValues(0.0, 0.0, 0.0),
    //   vec3.fromValues(1.0, 1.0, 1.0),
    //   vec3.fromValues(0.0, 0.0, 0.0),
    //   vec3.fromValues(
    //     this.p5Instance.random(0.001, 0.003),
    //     this.p5Instance.random(0.001, 0.003),
    //     this.p5Instance.random(0.001, 0.003),
    //   ),
    //   80,
    //   vec3.fromValues(
    //     Math.floor(this.p5Instance.random(255)),
    //     Math.floor(this.p5Instance.random(255)),
    //     Math.floor(this.p5Instance.random(255)),
    //   ),
    //   p5Instance,
    // );

    // this.initialAngularMomentum = vec3.clone(this.object.angularMomentum);
  }

  setCamera() {
    this.camera = this.p5Instance.createCamera();
    this.p5Instance.camera(0, 0, 600, 0, 0, 0, 0, 1, 0);
  }

  calculateAngularMomentum(I, w) {
    const L = vec3.create();
    vec3.transformMat3(L, w, I);
    return L;
  }

  createSkewMatrix(vector) {
    const matrix = mat3.create();

    matrix[0] = 0;
    matrix[1] = -vector[2];
    matrix[2] = vector[1];
    matrix[3] = vector[2];
    matrix[4] = 0;
    matrix[5] = -vector[0];
    matrix[6] = -vector[1];
    matrix[7] = vector[0];
    matrix[8] = 0;

    return matrix;
  }

  update(dt) {
    const obj = this.object;

    const I = mat3.clone(obj.inertialTensor);
    const invI = mat3.clone(obj.invInertialTensor);
    const w = vec3.clone(obj.angularVelocity);
    const L = this.calculateAngularMomentum(I, w);

    const LSkewMat = this.createSkewMatrix(L);
    const wSkewMat = this.createSkewMatrix(w);

    // вспомагательные расчеты
    const wIMat = mat3.create();
    mat3.multiply(wIMat, wSkewMat, I);
    const invILMat = mat3.create();
    mat3.multiply(invILMat, invI, LSkewMat);
    const invIwIMat = mat3.create();
    mat3.multiply(invIwIMat, invI, wIMat);

    // Расчет G
    const G = mat3.create();
    mat3.sub(G, invILMat, invIwIMat);

    const g = vec3.create();
    vec3.transformMat3(g, w, invILMat);

    const E = mat3.create();
    mat3.identity(E);
    const tempMat1 = mat3.create();
    mat3.multiplyScalar(tempMat1, G, dt);
    const tempMat2 = mat3.create();
    mat3.sub(tempMat2, E, tempMat1);
    const tempMat3 = mat3.create();
    mat3.invert(tempMat3, tempMat2);

    const tempVec1 = vec3.create();
    vec3.scale(tempVec1, g, dt);

    const dw = vec3.create();
    vec3.transformMat3(dw, tempVec1, tempMat3);

    obj.angularVelocity[0] = obj.angularVelocity[0] + dw[0];
    obj.angularVelocity[1] = obj.angularVelocity[1] + dw[1];
    obj.angularVelocity[2] = obj.angularVelocity[2] + dw[2];

    this.i = this.calculateAngularMomentum(I, obj.angularVelocity);

    this.Ek = vec3.dot(this.i, obj.angularVelocity) * 0.5;

    const wquat = quat.create();
    quat.set(
      wquat,
      obj.angularVelocity[0],
      obj.angularVelocity[1],
      obj.angularVelocity[2],
      0.0,
    );

    const dq = quat.create();
    quat.multiply(dq, obj.rotation, wquat);

    obj.rotation[0] = obj.rotation[0] + 0.5 * dq[0] * dt;
    obj.rotation[1] = obj.rotation[1] + 0.5 * dq[1] * dt;
    obj.rotation[2] = obj.rotation[2] + 0.5 * dq[2] * dt;
    obj.rotation[3] = obj.rotation[3] + 0.5 * dq[3] * dt;

    quat.normalize(obj.rotation, obj.rotation);

    // vec3.copy(this.object.angularVelocity, this.object.nextAngularVelocity);
    // const ω = this.object.nextAngularVelocity;

    // this.object.updateAngularMomentum();

    // const gyro = this.object.getGyro();

    // const skewOmega = this.getSkewMatrix(
    //   vec3.clone(this.object.nextAngularVelocity),
    // );

    // const skewL = this.getSkewMatrix(vec3.clone(this.object.angularMomentum));
    // const skewOmegaTimesI = mat3.create();
    // mat3.multiply(skewOmegaTimesI, skewOmega, this.object.inertialTensor);

    // const jacobiCore = mat3.create();
    // mat3.subtract(jacobiCore, skewOmegaTimesI, skewL);

    // mat3.multiplyScalar(jacobiCore, jacobiCore, dt);

    // const jacobi = mat3.create();
    // mat3.add(jacobi, this.object.inertialTensor, jacobiCore);

    // const jacobiInv = mat3.create();
    // mat3.invert(jacobiInv, jacobi);
    // const correction = vec3.create();
    // vec3.transformMat3(correction, gyro, jacobiInv);
    // vec3.scale(correction, correction, dt);
    // vec3.add(this.object.nextAngularVelocity, ω, correction);

    // this.object.updateAngularMomentum();
    // IntegrateQuatLocal(obj.rotation, obj.angularVelocity, dt);
  }

  draw() {
    this.p5Instance.orbitControl();

    this.object.draw();

    DrawLine(
      this.L0,
      vec3.fromValues(0.0, 0.0, 0.0),
      200,
      vec3.fromValues(180, 50, 50),
      this.p5Instance,
    );

    DrawAxes(this.object.position, this.object.rotation, 100, this.p5Instance);

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
      this.L0,
      4,
      16,
      [10, 20],
      [10, 10, 10],
      this.p5Instance,
    );

    OutputVector(
      "current",
      this.i,
      4,
      16,
      [10, 40],
      [10, 10, 10],
      this.p5Instance,
    );

    OutputValue(
      "energy",
      this.Ek,
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
