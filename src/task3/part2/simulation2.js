import SceneObject from "./scene_object.js";
import Plane from "./plane.js";
import Spring from "./spring.js";
import { OutputVector, OutputValue, DrawLine, DrawAxes } from "./draw_utils.js";
import { updateQuatLocal, updateQuatGlobal } from "./utils.js";

const { mat3, mat4, vec3, quat } = glMatrix;

export default class Simulation2 {
  plane;

  object;
  spring;

  p;

  subStepCount;

  p5Instance;
  camera;

  constructor(p5Instance) {
    this.p5Instance = p5Instance;

    this.plane = new Plane(500, [50, 50, 50], p5Instance);

    this.object = new SceneObject(
      [40, 80, 80],
      vec3.fromValues(0, 40.0, 0),
      vec3.fromValues(0, 0, 0),
      vec3.fromValues(0, 0, 0),
      vec3.fromValues(0, 0, 0),
      vec3.fromValues(20, 0, 10),
      10,
      vec3.fromValues(
        Math.floor(this.p5Instance.random(255)),
        Math.floor(this.p5Instance.random(255)),
        Math.floor(this.p5Instance.random(255)),
      ),
      p5Instance,
    );

    this.spring = new Spring(vec3.fromValues(250, 40, 0), 0.001, 0.001, 210);

    this.subStepCount = 10;
  }

  setCamera() {
    this.camera = this.p5Instance.createCamera();
    this.p5Instance.camera(0, 600, 1000, 0, 0, 0, 0, 1, 0);
  }

  update(dt) {
    const obj = this.object;
    const spring = this.spring;

    let predV = vec3.create();
    let predW = vec3.create();

    this.SIPredict(obj, dt, predV, predW);

    this.SISpring(obj, spring, dt, predV, predW);

    this.SIUpdate(obj, dt);
  }

  SISpring(object, spring, h, predV, predW) {
    this.p = object.getWorldAttachmentPoint();

    const d = vec3.create();
    vec3.sub(d, spring.anchorPos, this.p);

    const n = vec3.create();
    vec3.normalize(n, d);

    const r = vec3.create();
    vec3.sub(r, this.p, object.position);
    const crossRN = vec3.create();
    vec3.cross(crossRN, r, n);

    const dl = vec3.length(d) - spring.restLength;

    const dc = -vec3.dot(n, predV) - vec3.dot(crossRN, predW);

    const wInvI = object.getWInvI();
    const invEffMass = this.SICalculateInvEffMass(object.mass, r, n, wInvI);

    const omega = Math.sqrt(spring.stiffness / invEffMass);
    const k = spring.stiffness;
    const c = (2 * spring.damping * omega) / invEffMass;
    const denom = c + h * k;
    const beta = (h * k) / denom;
    const gamma = 1.0 / denom;

    const lambda = -(dc + (beta * dl) / h) / gamma;

    predV[0] = predV[0] - (n[0] * lambda) / object.mass;
    predV[1] = predV[1] - (n[1] * lambda) / object.mass;
    predV[2] = predV[2] - (n[2] * lambda) / object.mass;

    const torque = vec3.create();
    vec3.scale(torque, crossRN, -lambda);
    const dw = vec3.create();
    vec3.transformMat3(dw, torque, object.getWInvI());
    predW[0] += dw[0];
    predW[1] += dw[1];
    predW[2] += dw[2];

    vec3.copy(object.velocity, predV);
    vec3.copy(object.angularVelocity, predW);
  }

  SICalculateInvEffMass(m, arm, n, wInvI) {
    const crossArmN = vec3.create();
    vec3.cross(crossArmN, arm, n);

    const tempVec3 = mat3.create();
    vec3.transformMat3(tempVec3, crossArmN, wInvI);

    const effW = 1 / m + vec3.dot(tempVec3, crossArmN);
    return effW;
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

  SIPredict(object, dt, predV, predW) {
    predV[0] = object.velocity[0];
    predV[1] = object.velocity[1];
    predV[2] = object.velocity[2];

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
    mat3.multiplyScalar(tempMat1, G, dt);
    const tempMat2 = mat3.create();
    mat3.sub(tempMat2, E, tempMat1);
    const tempMat3 = mat3.create();
    mat3.invert(tempMat3, tempMat2);

    const tempVec1 = vec3.create();
    vec3.scale(tempVec1, g, dt);

    const dw = vec3.create();
    vec3.transformMat3(dw, tempVec1, tempMat3);

    predW[0] = object.angularVelocity[0] + dw[0];
    predW[1] = object.angularVelocity[1] + dw[1];
    predW[2] = object.angularVelocity[2] + dw[2];
  }

  SIUpdate(object, h) {
    object.position[0] = object.position[0] + object.velocity[0] * h;
    object.position[1] = object.position[1] + object.velocity[1] * h;
    object.position[2] = object.position[2] + object.velocity[2] * h;

    updateQuatGlobal(object.rotation, object.angularVelocity, h);
  }

  draw() {
    this.p5Instance.orbitControl();

    const obj = this.object;
    const spring = this.spring;

    this.plane.draw();
    obj.draw();

    this.p5Instance.line(
      this.p[0],
      this.p[1],
      this.p[2],
      spring.anchorPos[0],
      spring.anchorPos[1],
      spring.anchorPos[2],
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
