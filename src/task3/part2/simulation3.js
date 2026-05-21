import SceneObject from "./scene_object.js";
import Plane from "./plane.js";
import Spring from "./spring.js";
import { OutputVector, OutputValue, DrawLine, DrawAxes } from "./draw_utils.js";

const { mat3, mat4, vec3, quat } = glMatrix;

export default class Simulation1 {
  plane;

  object1;
  object2;

  l0;
  r1;
  r2;
  compliance;
  lambda;

  subStepCount;

  p5Instance;
  camera;

  constructor(p5Instance) {
    this.p5Instance = p5Instance;

    this.plane = new Plane(500, [50, 50, 50], p5Instance);

    this.object1 = new SceneObject(
      [40, 80, 80],
      vec3.fromValues(-50, 40.0, 0),
      vec3.fromValues(0, 0, 0),
      vec3.fromValues(0, 0, 0),
      vec3.fromValues(0, 0, 0),
      vec3.fromValues(20, 0, -20),
      10,
      vec3.fromValues(
        Math.floor(this.p5Instance.random(255)),
        Math.floor(this.p5Instance.random(255)),
        Math.floor(this.p5Instance.random(255)),
      ),
      p5Instance,
    );

    this.object2 = new SceneObject(
      [40, 80, 80],
      vec3.fromValues(50, 40.0, 0),
      vec3.fromValues(0, 0, 0),
      vec3.fromValues(0.0, 0, 0),
      vec3.fromValues(0, 0, 0),
      vec3.fromValues(-20, 0, -20),
      10,
      vec3.fromValues(
        Math.floor(this.p5Instance.random(255)),
        Math.floor(this.p5Instance.random(255)),
        Math.floor(this.p5Instance.random(255)),
      ),
      p5Instance,
    );

    this.subStepCount = 10;

    this.l0 = 150;
    this.r1 = vec3.create();
    this.r2 = vec3.create();
    this.compliance = 1000;
    this.lambda = 0.0;
  }

  setCamera() {
    this.camera = this.p5Instance.createCamera();
    this.p5Instance.camera(0, 600, 1000, 0, 0, 0, 0, 1, 0);
  }

  update(dt) {
    const h = dt / this.subStepCount;

    for (let i = 0; i < this.subStepCount; i++) {
      this.XPBDprediction(
        this.object1,
        h,
        vec3.fromValues(0.0, 0.0, 0.0),
        vec3.fromValues(0.0, 0.0, 0.0),
      );

      this.XPBDprediction(
        this.object2,
        h,
        vec3.fromValues(0.0, 0.0, 0.0),
        vec3.fromValues(0.0, 0.0, 0.0),
      );

      this.XPBDPositionSolver(this.object1, this.object2, h);

      this.XPBDUpdateVelocities(this.object1, h);
      this.XPBDUpdateVelocities(this.object2, h);
    }
  }

  XPBDprediction(object, h, fext, rorque) {
    vec3.copy(object.prevPosition, object.position);

    object.velocity[0] = object.velocity[0] + (h * fext[0]) / object.mass;
    object.velocity[1] = object.velocity[1] + (h * fext[1]) / object.mass;
    object.velocity[2] = object.velocity[2] + (h * fext[2]) / object.mass;

    object.position[0] = object.position[0] + h * object.velocity[0];
    object.position[1] = object.position[1] + h * object.velocity[1];
    object.position[2] = object.position[2] + h * object.velocity[2];

    quat.copy(object.prevRotation, object.rotation);

    object.angularVelocity[0] = object.angularVelocity[0];
    object.angularVelocity[1] = object.angularVelocity[1];
    object.angularVelocity[2] = object.angularVelocity[2];

    const wquat = quat.create();
    quat.set(
      wquat,
      object.angularVelocity[0],
      object.angularVelocity[1],
      object.angularVelocity[2],
      0.0,
    );
    const dq = quat.create();
    quat.multiply(dq, wquat, object.rotation);

    object.rotation[0] = object.rotation[0] + 0.5 * dq[0] * h;
    object.rotation[1] = object.rotation[1] + 0.5 * dq[1] * h;
    object.rotation[2] = object.rotation[2] + 0.5 * dq[2] * h;

    quat.normalize(object.rotation, object.rotation);
  }

  XPBDCalculateEffMass(m, arm, n, wInvI) {
    const crossArmN = vec3.create();
    vec3.cross(crossArmN, arm, n);

    const tempVec3 = mat3.create();
    vec3.transformMat3(tempVec3, crossArmN, wInvI);

    const effW = 1 / m + vec3.dot(tempVec3, crossArmN);
    return effW;
  }

  XPBDPositionSolver(object1, object2, h) {
    this.r1 = object1.getWorldAttachmentPoint();
    this.r2 = object2.getWorldAttachmentPoint();

    const dir = vec3.create();
    dir[0] = this.r2[0] - this.r1[0];
    dir[1] = this.r2[1] - this.r1[1];
    dir[2] = this.r2[2] - this.r1[2];

    const l = vec3.length(dir);

    const n = vec3.create();
    vec3.normalize(n, dir);

    const c = l - this.l0;

    const wInvI1 = object1.getWInvI();
    const arm1 = vec3.create();
    vec3.sub(arm1, this.r1, object1.position);
    const effW1 = this.XPBDCalculateEffMass(object1.mass, arm1, n, wInvI1);

    const wInvI2 = object2.getWInvI();
    const arm2 = vec3.create();
    vec3.sub(arm2, this.r2, object2.position);
    const effW2 = this.XPBDCalculateEffMass(object2.mass, arm2, n, wInvI2);

    const alpha = this.compliance / (h * h);

    const dLambda = (-c - alpha * this.lambda) / (effW1 + effW2 + alpha);
    this.lambda = this.lambda + dLambda;
    this.lambda = 0;

    const p = vec3.create();
    vec3.scale(p, n, dLambda);

    object1.position[0] = object1.position[0] - p[0] / object1.mass;
    object1.position[1] = object1.position[1] - p[1] / object1.mass;
    object1.position[2] = object1.position[2] - p[2] / object1.mass;

    object2.position[0] = object2.position[0] + p[0] / object2.mass;
    object2.position[1] = object2.position[1] + p[1] / object2.mass;
    object2.position[2] = object2.position[2] + p[2] / object2.mass;

    const crossArmP1 = vec3.create();
    vec3.cross(crossArmP1, arm1, p);
    const M1 = vec3.create();
    vec3.transformMat3(M1, crossArmP1, wInvI1);
    const q1 = quat.create();
    quat.set(q1, M1[0], M1[1], M1[2], 0.0);
    const dq1 = quat.create();
    quat.multiply(dq1, q1, object1.rotation);
    object1.rotation[0] = object1.rotation[0] - 0.5 * dq1[0];
    object1.rotation[1] = object1.rotation[1] - 0.5 * dq1[1];
    object1.rotation[2] = object1.rotation[2] - 0.5 * dq1[2];
    object1.rotation[3] = object1.rotation[3] - 0.5 * dq1[3];
    quat.normalize(object1.rotation, object1.rotation);

    const crossArmP2 = vec3.create();
    vec3.cross(crossArmP2, arm2, p);
    const M2 = vec3.create();
    vec3.transformMat3(M2, crossArmP2, wInvI2);
    const q2 = quat.create();
    quat.set(q2, M2[0], M2[1], M2[2], 0.0);
    const dq2 = quat.create();
    quat.multiply(dq2, q2, object2.rotation);
    object2.rotation[0] = object2.rotation[0] + 0.5 * dq2[0];
    object2.rotation[1] = object2.rotation[1] + 0.5 * dq2[1];
    object2.rotation[2] = object2.rotation[2] + 0.5 * dq2[2];
    object2.rotation[3] = object2.rotation[3] + 0.5 * dq2[3];
    quat.normalize(object2.rotation, object2.rotation);
  }

  XPBDUpdateVelocities(object, h) {
    object.velocity[0] = (object.position[0] - object.prevPosition[0]) / h;
    object.velocity[1] = (object.position[1] - object.prevPosition[1]) / h;
    object.velocity[2] = (object.position[2] - object.prevPosition[2]) / h;

    const dq = quat.create();
    const prevInvQ = quat.create();
    quat.invert(prevInvQ, object.prevRotation);
    quat.multiply(dq, object.rotation, prevInvQ);

    const w = vec3.create();
    vec3.set(w, (2 * dq[0]) / h, (2 * dq[1]) / h, (2 * dq[2]) / h);
    const ws = dq[3] > 0 ? 1 : -1;

    vec3.scale(object.angularVelocity, w, ws);
  }

  draw() {
    this.p5Instance.orbitControl();

    this.plane.draw();
    this.object1.draw();
    this.object2.draw();

    this.p5Instance.line(
      this.r1[0],
      this.r1[1],
      this.r1[2],
      this.r2[0],
      this.r2[1],
      this.r2[2],
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

    this.p5Instance.pop();

    this.p5Instance.drawingContext.enable(
      this.p5Instance.drawingContext.DEPTH_TEST,
    );

    this.p5Instance.camera(...cameraParams);
  }
}
