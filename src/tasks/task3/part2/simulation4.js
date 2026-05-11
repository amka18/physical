import SimulationObject from "../../../common/simulation_object.js";
import {
  IntegrateQuatGlobal,
  IntegrateQuatLocal,
} from "../../../common/util.js";

const { mat3, mat4, vec3, quat } = glMatrix;

export default class Simulation1 {
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

  skewMatrix(v, out) {
    out[0] = 0;
    out[1] = -v[2];
    out[2] = v[1];
    out[3] = v[2];
    out[4] = 0;
    out[5] = -v[0];
    out[6] = -v[1];
    out[7] = v[0];
    out[8] = 0;
    return out;
  }

  update(dt) {
    // ----- ШАГ 1: Получаем текущие параметры -----
    const I = this.object.inertialTensor; // локальный тензор инерции
    const ω = this.object.angularVelocity; // текущая угловая скорость

    // ----- ШАГ 2: Вычисляем текущий момент L = I * ω -----
    const L = vec3.create();
    vec3.transformMat3(L, ω, I);

    // ----- ШАГ 3: Вычисляем гироскопический член gyro = ω × L -----
    const gyro = vec3.create();
    vec3.cross(gyro, ω, L);

    // ----- ШАГ 4: Строим кососимметричные матрицы -----
    const skewOmega = mat3.create(); // [ω]×
    const skewL = mat3.create(); // [L]×

    this.skewMatrix(ω, skewOmega);
    this.skewMatrix(L, skewL);

    // ----- ШАГ 5: Вычисляем матрицу Якоби -----
    // J = I + dt * ( [L]× - [ω]× * I )
    // Это линеаризация функции f(ω) = -I⁻¹·(ω × Iω)

    // Шаг 5a: Вычисляем [ω]× * I
    const skewOmegaTimesI = mat3.create();
    mat3.multiply(skewOmegaTimesI, skewOmega, I);

    // Шаг 5b: Вычисляем [L]× - [ω]× * I
    const jacobiCore = mat3.create();
    mat3.subtract(jacobiCore, skewOmegaTimesI, skewL);

    // Шаг 5c: Умножаем на dt
    mat3.multiplyScalar(jacobiCore, jacobiCore, dt);

    // Шаг 5d: J = I + dt * ([L]× - [ω]× * I)
    const jacobi = mat3.create();
    mat3.add(jacobi, I, jacobiCore);

    // ----- ШАГ 6: Вычисляем обратную матрицу Якоби -----
    const jacobiInv = mat3.create();
    const success = mat3.invert(jacobiInv, jacobi);

    if (!success) {
      // Если матрица вырождена, используем явный метод как fallback
      console.warn("Jacobi matrix is singular, using explicit fallback");
      const Iinv = this.object.invertInertialTensor;
      const alpha = vec3.create();
      vec3.transformMat3(alpha, gyro, Iinv);
      vec3.scale(alpha, alpha, -1);
      vec3.scaleAndAdd(this.object.angularVelocity, ω, alpha, dt);
    } else {
      // ----- ШАГ 7: Вычисляем поправку к угловой скорости -----
      // Δω = J⁻¹ * (dt * gyro)
      const correction = vec3.create();
      vec3.transformMat3(correction, gyro, jacobiInv);
      vec3.scale(correction, correction, dt);

      // ----- ШАГ 8: Обновляем угловую скорость ω_new = ω + Δω -----
      vec3.add(this.object.angularVelocity, ω, correction);
    }

    // ----- ШАГ 9: Обновляем момент импульса из новой скорости -----
    vec3.transformMat3(
      this.object.angularMomentum,
      this.object.angularVelocity,
      I,
    );

    // ----- ШАГ 10: Интегрируем вращение (обновляем кватернион) -----
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
