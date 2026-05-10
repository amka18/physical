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
          this.#p5Instance.random(10),
          this.#p5Instance.random(10),
          this.#p5Instance.random(10),
        ),
        vec3.fromValues(
          this.#p5Instance.random(10),
          this.#p5Instance.random(10),
          this.#p5Instance.random(10),
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

  }

  draw() {
    this.#p5Instance.camera(400, -300, 1000, 0, 0, 0, 0, 1, 0);

    this.#box.draw();

    for (let object of this.#objects) {
      object.draw();
    }
  }
}
