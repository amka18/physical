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
  p1;
  p2;
  compliance;

  accumulatedImpulse;

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

    this.l0 = 100;
    this.p1 = vec3.create();
    this.p2 = vec3.create();
    this.compliance = 1000;

    this.accumulatedImpulse = 0.0;
  }

  setCamera() {
    this.camera = this.p5Instance.createCamera();
    this.p5Instance.camera(0, 600, 1000, 0, 0, 0, 0, 1, 0);
  }

  update(dt) {
    const obj1 = this.object1;
    const obj2 = this.object2;

    let predV1 = vec3.create();
    let predW1 = vec3.create();
    let predV2 = vec3.create();
    let predW2 = vec3.create();

    this.pred(
      obj1,
      dt,
      vec3.fromValues(0.0, 0.0, 0.0),
      vec3.fromValues(0.0, 0.0, 0.0),
      predV1,
      predW1,
    );

    this.pred(
      obj2,
      dt,
      vec3.fromValues(0.0, 0.0, 0.0),
      vec3.fromValues(0.0, 0.0, 0.0),
      predV2,
      predW2,
    );

    this.si(obj1, obj2, dt, predV1, predW1, predV2, predW2);
  }

  si(object1, object2, h, predV1, predW1, predV2, predW2) {
    this.p1 = object1.getWorldAttachmentPoint();
    this.p2 = object2.getWorldAttachmentPoint();

    const d = vec3.create();
    vec3.sub(d, this.p2, this.p1);

    const n = vec3.create();
    vec3.normalize(n, d);

    const r1 = vec3.create();
    vec3.sub(r1, this.p1, object1.position);
    const crossR1N = vec3.create();
    vec3.cross(crossR1N, r1, n);

    const t1 = vec3.create();
    vec3.scale(t1, crossR1N, -1);

    const r2 = vec3.create();
    vec3.sub(r2, this.p2, object2.position);
    const crossR2N = vec3.create();
    vec3.cross(crossR2N, r2, n);

    const t2 = vec3.create();
    vec3.scale(t2, n, -1);

    const c = vec3.length(d) - this.l0;

    const dc =
      vec3.dot(t2, predV1) +
      vec3.dot(t1, predW1) +
      vec3.dot(n, predV2) +
      vec3.dot(crossR2N, predW2);

    const wInvI1 = object1.getWInvI();
    const invEffMass1 = this.calculateEffMass(object1.mass, r1, n, wInvI1);

    const wInvI2 = object2.getWInvI();
    const invEffMass2 = this.calculateEffMass(object2.mass, r2, n, wInvI2);

    const appliedDelta = (-c / (invEffMass1 + invEffMass2)) * 0.01;

    object1.position[0] -= (appliedDelta / object1.mass) * n[0];
    object1.position[1] -= (appliedDelta / object1.mass) * n[1];
    object1.position[2] -= (appliedDelta / object1.mass) * n[2];

    object2.position[0] += (appliedDelta / object2.mass) * n[0];
    object2.position[1] += (appliedDelta / object2.mass) * n[1];
    object2.position[2] += (appliedDelta / object2.mass) * n[2];

    const angCorr1 = vec3.create();
    vec3.transformMat3(angCorr1, crossR1N, object1.getWInvI());
    vec3.scale(angCorr1, angCorr1, -appliedDelta);
    if (vec3.length(angCorr1) > 1e-12) {
      const halfAngle = vec3.scale(vec3.create(), angCorr1, 0.5);
      const rotQ1 = quat.create();
      quat.setAxisAngle(
        rotQ1,
        vec3.normalize(vec3.create(), halfAngle),
        vec3.length(angCorr1),
      );
      quat.multiply(object1.rotation, rotQ1, object1.rotation);
      quat.normalize(object1.rotation, object1.rotation);
    }

    const angCorr2 = vec3.create();
    vec3.transformMat3(angCorr2, crossR2N, object2.getWInvI());
    vec3.scale(angCorr2, angCorr2, appliedDelta);
    if (vec3.length(angCorr2) > 1e-12) {
      const halfAngle = vec3.scale(vec3.create(), angCorr2, 0.5);
      const rotQ2 = quat.create();
      quat.setAxisAngle(
        rotQ2,
        vec3.normalize(vec3.create(), halfAngle),
        vec3.length(angCorr2),
      );
      quat.multiply(object2.rotation, rotQ2, object2.rotation);
      quat.normalize(object2.rotation, object2.rotation);
    }
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

  pred(object, h, forceExt, torque, predV, predW) {
    predV[0] = object.velocity[0] + (h * forceExt[0]) / object.mass;
    predV[1] = object.velocity[1] + (h * forceExt[1]) / object.mass;
    predV[2] = object.velocity[2] + (h * forceExt[2]) / object.mass;

    const I = object.inertialTensor;
    const invI = object.invInertialTensor;
    const w = object.angularVelocity;
    const L = this.calculateAngularMomentum(I, w);

    const LSkewMat = this.createSkewMatrix(L);
    const wSkewMat = this.createSkewMatrix(w);

    const wIMat = mat3.create();
    mat3.multiply(wIMat, wSkewMat, I);
    const invILMat = mat3.create();
    mat3.multiply(invILMat, invI, LSkewMat);
    const invIwIMat = mat3.create();
    mat3.multiply(invIwIMat, invI, wIMat);

    const G = mat3.create();
    mat3.sub(G, invILMat, invIwIMat);

    const g = vec3.create();
    vec3.transformMat3(g, w, invILMat);

    const E = mat3.create();
    mat3.identity(E);
    const tempMat1 = mat3.create();
    mat3.multiplyScalar(tempMat1, G, h);
    const tempMat2 = mat3.create();
    mat3.sub(tempMat2, E, tempMat1);
    const tempMat3 = mat3.create();
    mat3.invert(tempMat3, tempMat2);

    const tempVec1 = vec3.create();
    vec3.scale(tempVec1, g, h);

    const dw = vec3.create();
    vec3.transformMat3(dw, tempVec1, tempMat3);

    predW[0] = object.angularVelocity[0] + dw[0];
    predW[1] = object.angularVelocity[1] + dw[1];
    predW[2] = object.angularVelocity[2] + dw[2];
  }

  calculateEffMass(m, arm, n, wInvI) {
    const crossArmN = vec3.create();
    vec3.cross(crossArmN, arm, n);

    const tempVec3 = mat3.create();
    vec3.transformMat3(tempVec3, crossArmN, wInvI);

    const effW = 1 / m + vec3.dot(tempVec3, crossArmN);
    return effW;
  }

  draw() {
    this.p5Instance.orbitControl();

    this.plane.draw();
    this.object1.draw();
    this.object2.draw();

    this.p5Instance.line(
      this.p1[0],
      this.p1[1],
      this.p1[2],
      this.p2[0],
      this.p2[1],
      this.p2[2],
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
