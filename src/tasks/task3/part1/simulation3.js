import SimulationObject from "../../../common/simulation_object.js";
import {
  IntegrateQuatGlobal,
  IntegrateQuatLocal,
} from "../../../common/util.js";

const { mat3, mat4, vec3, quat } = glMatrix;

export default class Simulation3 {
  object;

  initialAngularMomentum;

  p5Instance;

  constructor(p5Instance) {
    this.p5Instance = p5Instance;

    this.object = new SimulationObject(
      vec3.fromValues(100, 50, 30),
      vec3.fromValues(0.0, 0.0, 0.0),
      vec3.fromValues(0.0, 0.0, 0.0),
      vec3.fromValues(1.0, 1.0, 1.0),
      vec3.fromValues(0.0, 0.0, 0.0),
      vec3.fromValues(
        this.p5Instance.random(-0.001, 0.001),
        this.p5Instance.random(-0.001, 0.001),
        this.p5Instance.random(-0.001, 0.001),
      ),
      this.p5Instance.random(10, 100),
      vec3.fromValues(
        Math.floor(this.p5Instance.random(255)),
        Math.floor(this.p5Instance.random(255)),
        Math.floor(this.p5Instance.random(255)),
      ),
      p5Instance,
    );

    this.initialAngularMomentum = this.object.angularMomentum;
  }

  update(dt) {
    const torque = vec3.create();
    vec3.cross(
      torque,
      this.object.angularVelocity,
      this.object.angularMomentum,
    );
    vec3.scale(torque, torque, -1); // torque = -ω × L

    // Шаг 2: Обновляем момент (явный Эйлер)
    // L_new = L_old + dt * (-ω × L)
    const newAngularMomentum = vec3.create();
    vec3.scaleAndAdd(
      newAngularMomentum,
      this.object.angularMomentum,
      torque,
      dt,
    );

    // Шаг 3: Обновляем угловую скорость через новый момент
    // ω = I⁻¹ * L_new
    vec3.transformMat3(
      this.object.angularVelocity,
      newAngularMomentum,
      this.object.invertInertialTensor,
    );

    // Шаг 4: Обновляем момент в объекте
    vec3.copy(this.object.angularMomentum, newAngularMomentum);

    // Шаг 5: Интегрируем вращение
    IntegrateQuatLocal(this.object.rotation, this.object.angularVelocity, dt);
  }

  draw() {
    this.p5Instance.camera(0, 0, 600, 0, 0, 0, 0, 1, 0);

    this.p5Instance.fill(255, 0, 0);

    this.p5Instance.text(this.initialAngularMomentum, -120, -120, 50);
    this.p5Instance.text(this.object.angularMomentum, -120, -100, 50);

    this.object.draw();
  }
}
