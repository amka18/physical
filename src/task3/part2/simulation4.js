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
      vec3.fromValues(0.01, 0, 0),
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
    this.p1 = vec3.create();
    this.p2 = vec3.create();
    this.compliance = 1000;
    this.lambda = 0.0;
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

    this.int(obj1, dt);
    this.int(obj2, dt);
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

    const b = c * 0.1/h;

    const wInvI1 = object1.getWInvI();
    const effMass1 = this.XPBDCalculateEffMass(object1.mass, r1, n, wInvI1);

    const wInvI2 = object2.getWInvI();
    const effMass2 = this.XPBDCalculateEffMass(object2.mass, r2, n, wInvI2);

    const lambda = (-dc - b) / (effMass1 + effMass2);

    const I1crossR1N = vec3.create();

    predV1[0] = predV1[0] - (n[0] * lambda) / object1.mass;
    predV1[1] = predV1[1] - (n[1] * lambda) / object1.mass;
    predV1[2] = predV1[2] - (n[2] * lambda) / object1.mass;

    predV2[0] = predV2[0] + (n[0] * lambda) / object2.mass;
    predV2[1] = predV2[1] + (n[1] * lambda) / object2.mass;
    predV2[2] = predV2[2] + (n[2] * lambda) / object2.mass;

    // predW1[0] = predW1[0] - crossR1N[0]*

    vec3.copy(object1.velocity, predV1);

    vec3.copy(object2.velocity, predV2);
  }

  pred(object, h, forceExt, torque, predV, predW) {
    predV[0] = object.velocity[0] + (h * forceExt[0]) / object.mass;
    predV[1] = object.velocity[1] + (h * forceExt[1]) / object.mass;
    predV[2] = object.velocity[2] + (h * forceExt[2]) / object.mass;

    // расчитать с героскопическим слагаемым
    predW[0] = object.angularVelocity[0] + h * torque[0]; // I-1
    predW[1] = object.angularVelocity[1] + h * torque[1];
    predW[2] = object.angularVelocity[2] + h * torque[2];
  }

  int(object, h) {
    object.position[0] = object.position[0] + object.velocity[0] * h;
    object.position[1] = object.position[1] + object.velocity[1] * h;
    object.position[2] = object.position[2] + object.velocity[2] * h;

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
