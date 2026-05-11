import GameObject from "../../../common/simulation_object.js";
import Plane from "../../../common/plane.js";
import { GroundConstraint } from "../../../common/collision.js";

const { mat4, vec3, quat } = glMatrix;

export default class GameEngine {
  plane;
  objects;

  p5Instance;

  constructor(p5Instance) {
    this.p5Instance = p5Instance;

    this.plane = new Plane(500, vec3.fromValues(100, 200, 100), p5Instance);

    this.objects = [];

    const object1 = new GameObject(
      vec3.fromValues(
        this.p5Instance.random(50, 95),
        this.p5Instance.random(50, 95),
        this.p5Instance.random(50, 95),
      ),
      vec3.fromValues(0.0, -400, 0.0),
      vec3.fromValues(
        this.p5Instance.random(180),
        this.p5Instance.random(180),
        this.p5Instance.random(180),
      ),
      vec3.fromValues(1.0, 1.0, 1.0),
      vec3.fromValues(0.0, -this.p5Instance.random(0.1), 0.0),
      vec3.fromValues(
        this.p5Instance.random(0.001),
        this.p5Instance.random(0.001),
        this.p5Instance.random(0.001),
      ),
      this.p5Instance.random(10, 100),
      vec3.fromValues(
        Math.floor(this.p5Instance.random(255)),
        Math.floor(this.p5Instance.random(255)),
        Math.floor(this.p5Instance.random(255)),
      ),
      p5Instance,
    );
    this.objects.push(object1);
  }

  update(dt) {
    for (let object of this.objects) {
      vec3.copy(object.prevPosition, object.position);
      quat.copy(object.prevRotation, object.rotation);
    }

    for (let object of this.objects) {
      vec3.scaleAndAdd(object.position, object.position, object.velocity, dt);

      // добавим
      let g = vec3.fromValues(0.0, 0.001, 0.0);
      vec3.scaleAndAdd(object.position, object.position, g, dt);

      let wquat = quat.fromValues(
        object.angularVelocity[0],
        object.angularVelocity[1],
        object.angularVelocity[2],
        0.0,
      );
      quat.multiply(wquat, wquat, object.rotation);
      quat.scale(wquat, wquat, dt * 0.5);
      quat.add(object.rotation, object.rotation, wquat);
      quat.normalize(object.rotation, object.rotation);
    }

    // коллизии или ограничени
    // на box
    // между собой
    // проверяем что не вышла за пределы коробк
    for (let object of this.objects) {
      GroundConstraint(object);
    }

    // обновляем скорость по изменению позиции
    for (let object of this.objects) {
      vec3.sub(object.velocity, object.position, object.prevPosition);
      vec3.scale(object.velocity, object.velocity, 1.0 / dt);

      // let deltaQuat = quat.create();
      // quat.conjugate(object.prevRotationRaw, object.prevRotationRaw);
      // quat.multiply(deltaQuat, object.rotation, object.prevRotationRaw);

      // // Извлекаем векторную часть (оси) и нормализуем
      // let angle = 2 * Math.acos(Math.min(1, Math.abs(deltaQuat[3])));
      // let axis = vec3.fromValues(deltaQuat[0], deltaQuat[1], deltaQuat[2]);
      // vec3.normalize(axis, axis);

      // // Угловая скорость = угол * ось / dt
      // vec3.scale(object.angularVelocity, axis, angle / dt);
    }
  }

  draw() {
    this.p5Instance.camera(0, -1000, 600, 0, 0, 0, 0, 1, 0);

    this.plane.draw();

    for (let object of this.objects) {
      object.draw();
    }
  }
}
