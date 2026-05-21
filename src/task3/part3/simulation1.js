import SceneObject from "./scene_object.js";
import Plane from "./plane.js";
import Spring from "./spring.js";
import {
  updateQuatGlobal,
  updateQuatLocal,
  calculateRotationEnergy,
} from "./utils.js";
import { OutputVector, OutputValue, DrawLine, DrawAxes } from "./draw_utils.js";

const { mat3, mat4, vec3, quat } = glMatrix;

export default class Simulation1 {
  plane;
  spring;
  object;

  subStepCount;

  p5Instance;
  camera;

  constructor(p5Instance) {
    this.p5Instance = p5Instance;

    this.plane = new Plane(500, [50, 50, 50], p5Instance);

    this.spring = new Spring(
      vec3.fromValues(0.0, 40.0, 250.0),
      0.001,
      0.001,
      180,
    );

    this.object = new SceneObject(
      [80, 80, 80],
      vec3.fromValues(0, 40.0, 0),
      vec3.fromValues(0, 0, 0),
      vec3.fromValues(0, 0, 0),
      vec3.fromValues(0, 0, 0),
      vec3.fromValues(1, 0, 40),
      10,
      vec3.fromValues(
        Math.floor(this.p5Instance.random(255)),
        Math.floor(this.p5Instance.random(255)),
        Math.floor(this.p5Instance.random(255)),
      ),
      p5Instance,
    );

    this.subStepCount = 10;
  }

  setCamera() {
    this.camera = this.p5Instance.createCamera();
    this.p5Instance.camera(0, 600, 1000, 0, 0, 0, 0, 1, 0);
  }

  update(dt) {
    const h = dt / this.subStepCount;

    for (let i = 0; i < this.subStepCount; i++) {
      const obj = this.object;

      const spring = this.spring;
      spring.attachmentPos = obj.getWorldAttachmentPoint();
      const n = vec3.create();
      vec3.sub(n, spring.attachmentPos, spring.anchorPos);
      spring.currentLength = vec3.length(n);
      vec3.normalize(n, n);
      const dl = spring.currentLength - spring.restLength;
      const fe = vec3.create();
      vec3.scale(fe, n, -spring.stiffness * dl);

      const attachmentV = vec3.create();
      const tempV = vec3.create();
      vec3.cross(tempV, obj.angularVelocity, obj.attachmentPoint);
      vec3.add(attachmentV, obj.velocity, tempV);
      const fd = vec3.create();
      vec3.scale(fd, attachmentV, spring.damping);

      const f = vec3.create();
      if (vec3.length(f) < 0.001) {
        vec3.set(f, 0, 0, 0.0, 0.0);
      }
      vec3.sub(f, fe, fd);
      const torque = vec3.create();
      vec3.cross(torque, obj.attachmentPoint, f);

      vec3.copy(obj.prevPosition, obj.position);
      vec3.scaleAndAdd(obj.velocity, obj.velocity, f, h / obj.mass);
      vec3.scaleAndAdd(obj.position, obj.position, obj.velocity, h);

      quat.copy(obj.prevRotation, obj.rotation);
      const I = obj.getWI();
      const invI = obj.getWInvI();
      const tempVec1 = vec3.create();
      vec3.transformMat3(tempVec1, torque, invI);
      vec3.scaleAndAdd(obj.angularVelocity, obj.angularVelocity, tempVec1, h);
      updateQuatGlobal(obj.rotation, obj.angularVelocity, h);

      const dx = vec3.create();
      vec3.sub(dx, obj.position, obj.prevPosition);
      vec3.scale(obj.velocity, dx, 1 / h);

      const invQprev = quat.create();
      quat.conjugate(invQprev, obj.prevRotation);
      const dq = quat.create();
      quat.multiply(dq, obj.rotation, invQprev);
      quat.normalize(dq, dq);

      const w = vec3.fromValues(
        (2 * dq[0]) / h,
        (2 * dq[1]) / h,
        (2 * dq[2]) / h,
      );
      const sign = dq[3] >= 0 ? 1 : -1;
      vec3.scale(w, w, sign);
      vec3.copy(obj.angularVelocity, w);
    }
  }

  draw() {
    this.p5Instance.orbitControl();

    this.plane.draw();
    this.object.draw();

    const spring = this.spring;

    this.p5Instance.line(
      spring.anchorPos[0],
      spring.anchorPos[1],
      spring.anchorPos[2],
      spring.attachmentPos[0],
      spring.attachmentPos[1],
      spring.attachmentPos[2],
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
