import GameObject from "./game_object.js";
import Box from "./box.js";

const { mat4, vec3, quat } = glMatrix;

export default class GameEngine {
  #p5Instance = null;

  #box;
  #objects;

  constructor(p5Instance) {
    this.#p5Instance = p5Instance;

    this.#box = new Box(vec3.fromValues(500, 500, 500), p5Instance);

    this.#objects = [];

    for (let i = 0; i < 10; i++) {
      const xPosition = (Math.floor(this.#p5Instance.random(4)) - 2) * 100.0;
      const yPosition = (Math.floor(this.#p5Instance.random(4)) - 2) * 100.0;
      const zPosition = (Math.floor(this.#p5Instance.random(4)) - 2) * 100.0;

      const obj = new GameObject(
        vec3.fromValues(
          this.#p5Instance.random(50, 95),
          this.#p5Instance.random(50, 95),
          this.#p5Instance.random(50, 95),
        ),
        vec3.fromValues(xPosition, yPosition, zPosition),
        vec3.fromValues(
          this.#p5Instance.random(180),
          this.#p5Instance.random(180),
          this.#p5Instance.random(180),
        ),
        vec3.fromValues(1.0, 1.0, 1.0),
        vec3.fromValues(
          this.#p5Instance.random(0.1) - 0.05,
          this.#p5Instance.random(0.1) - 0.05,
          this.#p5Instance.random(0.1) - 0.05,
        ),
        vec3.fromValues(
          this.#p5Instance.random(0.001),
          this.#p5Instance.random(0.001),
          this.#p5Instance.random(0.001),
        ),
        this.#p5Instance.random(10, 100),
        vec3.fromValues(
          Math.floor(this.#p5Instance.random(255)),
          Math.floor(this.#p5Instance.random(255)),
          Math.floor(this.#p5Instance.random(255)),
        ),
        p5Instance,
      );
      this.#objects.push(obj);
    }
  }

  init() {}

  update(dt) {
    // обновляем старые позиции
    for (let object of this.#objects) {
      vec3.copy(object.prevPosition, object.position);
      quat.copy(object.prevRotation, object.rotation);
    }

    // предварительно обновляем позиции и вращение
    for (let object of this.#objects) {
      vec3.scaleAndAdd(object.position, object.position, object.velocity, dt);

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
    for (let object of this.#objects) {
    }

    // обновляем скорость по изменению позиции
    for (let object of this.#objects) {
      vec3.sub(object.velocity, object.position, object.prevPosition);
      vec3.scale(object.velocity, object.velocity, 1.0 / dt);
    }
  }

  draw() {
    this.#p5Instance.camera(400, -300, 1000, 0, 0, 0, 0, 1, 0);

    this.#box.draw();

    for (let object of this.#objects) {
      object.draw();
    }
  }
}
