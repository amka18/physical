import SceneObject from "./scene_object.js";
import Plane from "./plane.js";
import {
  XPBD_PredictPositions,
  XPBD_UpdateVelocities,
  XPBD_SolveGroundCollision,
  XPBD_SolveObjectsCollision,
} from "./xpbd.js";
import { detectGroundCollision, detectObjectCollision } from "./collision.js";

const { mat3, mat4, vec3, quat } = glMatrix;

export default class Simulation1 {
  plane;

  objects;

  groundContacts;
  collisionContacts;

  subStepCount;
  collisionIterCount;

  p5Instance;
  camera;

  constructor(p5Instance) {
    this.p5Instance = p5Instance;

    this.plane = new Plane(500, [50, 50, 50], p5Instance);

    const objPositions = [
      vec3.fromValues(0, 150, 0),
      vec3.fromValues(100, 150, 0),
      vec3.fromValues(-100, 150, 0),
      vec3.fromValues(0, 150, 100),
      vec3.fromValues(0, 150, -100),
      vec3.fromValues(0, 250, 0),
      vec3.fromValues(100, 250, 0),
      vec3.fromValues(-100, 250, 0),
      vec3.fromValues(0, 250, 100),
      vec3.fromValues(0, 250, -100),
    ];

    this.objects = [];

    for (const objPosition of objPositions) {
      const obj = new SceneObject(
        [40, 40, 40],
        objPosition,
        vec3.fromValues(
          this.p5Instance.random(-180, 180),
          this.p5Instance.random(-180, 180),
          this.p5Instance.random(-180, 180),
        ),
        vec3.fromValues(0, 0, 0),
        vec3.fromValues(0.0, 0.0, 0.0),
        Math.floor(this.p5Instance.random(1, 10)),
        vec3.fromValues(
          Math.floor(this.p5Instance.random(255)),
          Math.floor(this.p5Instance.random(255)),
          Math.floor(this.p5Instance.random(255)),
        ),
        p5Instance,
      );
      this.objects.push(obj);
    }

    this.collisionContacts = [];
    this.groundContacts = [];

    this.subStepCount = 10;
    this.collisionIterCount = 5;
  }

  setCamera() {
    this.camera = this.p5Instance.createCamera();
    this.p5Instance.camera(0, 600, 1000, 0, 0, 0, 0, 1, 0);
    this.draw();
  }

  detectCollisions() {
    detectGroundCollision(this.objects, this.groundContacts);
    detectObjectCollision(this.objects, this.collisionContacts);
  }

  update(dt) {
    const h = dt / this.subStepCount;

    for (let i = 0; i < this.subStepCount; i++) {
      for (const obj of this.objects) {
        XPBD_PredictPositions(obj, h);
      }

      this.detectCollisions();

      for (const groungContact of this.groundContacts) {
        XPBD_SolveGroundCollision(
          groungContact.object,
          groungContact.contactVert,
          groungContact.penetration,
          groungContact.lambda,
          h,
        );
      }

      for (const collisionContact of this.collisionContacts) {
        XPBD_SolveObjectsCollision(
          collisionContact.object1,
          collisionContact.object2,
          collisionContact.contactVert1,
          collisionContact.contactVert2,
          collisionContact.normal,
          collisionContact.penetration,
          collisionContact.lambda,
          h,
        );
      }

      for (const obj of this.objects) {
        XPBD_UpdateVelocities(obj, h);
      }
    }
  }

  draw() {
    this.p5Instance.orbitControl();
    this.plane.draw();

    for (const obj of this.objects) {
      obj.draw();
    }

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
