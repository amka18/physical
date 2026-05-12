import SimulationObject from "../../../common/simulation_object.js";
import {
  IntegrateQuatGlobal,
  IntegrateQuatLocal,
} from "../../../common/integrators.js";
import {
  OutputVector,
  OutputValue,
  DrawLine,
  DrawAxes,
} from "../../../common/draw_utils.js";

const { mat3, mat4, vec3, quat } = glMatrix;

export default class Simulation4 {
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
        this.p5Instance.random(0.001, 0.003),
        this.p5Instance.random(0.001, 0.003),
        this.p5Instance.random(0.001, 0.003),
      ),
      80,
      vec3.fromValues(
        Math.floor(this.p5Instance.random(255)),
        Math.floor(this.p5Instance.random(255)),
        Math.floor(this.p5Instance.random(255)),
      ),
      p5Instance,
    );

    this.initialAngularMomentum = vec3.clone(this.object.angularMomentum);
  }

  getSkewMatrix(vector) {
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

  update(dt) {
    // // ----- ШАГ 1: Получаем текущие параметры -----
    // const I = this.object.inertialTensor; // локальный тензор инерции
    vec3.copy(this.object.prevAngularVelocity, this.object.angularVelocity);

    const ω = this.object.angularVelocity; // текущая угловая скорость

    // // ----- ШАГ 2: Вычисляем текущий момент L = I * ω -----
    // const L = vec3.create();
    // vec3.transformMat3(L, ω, I);

    this.object.updateAngularMomentum();

    // ----- ШАГ 3: Вычисляем гироскопический член gyro = ω × L -----
    // const gyro = vec3.create();
    // vec3.cross(gyro, ω, L);
    const gyro = this.object.getGyro();

    // ----- ШАГ 4: Строим кососимметричные матрицы -----
    // const skewOmega = mat3.create(); // [ω]×
    // const skewL = mat3.create(); // [L]×
    const skewOmega = this.getSkewMatrix(
      vec3.clone(this.object.angularVelocity),
    );

    const skewL = this.getSkewMatrix(vec3.clone(this.object.angularMomentum));

    // this.skewMatrix(ω, skewOmega);
    // this.skewMatrix(L, skewL);

    // ----- ШАГ 5: Вычисляем матрицу Якоби -----
    // J = I + dt * ( [L]× - [ω]× * I )
    // Это линеаризация функции f(ω) = -I⁻¹·(ω × Iω)

    // Шаг 5a: Вычисляем [ω]× * I
    const skewOmegaTimesI = mat3.create();
    mat3.multiply(skewOmegaTimesI, skewOmega, this.object.inertialTensor);

    // Шаг 5b: Вычисляем [L]× - [ω]× * I
    const jacobiCore = mat3.create();
    mat3.subtract(jacobiCore, skewOmegaTimesI, skewL);

    // Шаг 5c: Умножаем на dt
    mat3.multiplyScalar(jacobiCore, jacobiCore, dt);

    // Шаг 5d: J = I + dt * ([L]× - [ω]× * I)
    const jacobi = mat3.create();
    mat3.add(jacobi, this.object.inertialTensor, jacobiCore);

    // ----- ШАГ 6: Вычисляем обратную матрицу Якоби -----
    const jacobiInv = mat3.create();
    const success = mat3.invert(jacobiInv, jacobi);

    // if (!success) {
    //   // Если матрица вырождена, используем явный метод как fallback
    //   console.warn("Jacobi matrix is singular, using explicit fallback");
    //   const Iinv = this.object.invertInertialTensor;
    //   const alpha = vec3.create();
    //   vec3.transformMat3(alpha, gyro, Iinv);
    //   vec3.scale(alpha, alpha, -1);
    //   vec3.scaleAndAdd(this.object.angularVelocity, ω, alpha, dt);
    // } else {
    //   // ----- ШАГ 7: Вычисляем поправку к угловой скорости -----
    //   // Δω = J⁻¹ * (dt * gyro)
    const correction = vec3.create();
    vec3.transformMat3(correction, gyro, jacobiInv);
    vec3.scale(correction, correction, dt);

    // ----- ШАГ 8: Обновляем угловую скорость ω_new = ω + Δω -----
    vec3.add(this.object.angularVelocity, ω, correction);
    // }

    // ----- ШАГ 9: Обновляем момент импульса из новой скорости -----
    // vec3.transformMat3(
    //   this.object.angularMomentum,
    //   this.object.angularVelocity,
    //   I,
    // );
    this.object.updateAngularMomentum();

    // ----- ШАГ 10: Интегрируем вращение (обновляем кватернион) -----
    IntegrateQuatLocal(this.object.rotation, this.object.angularVelocity, dt);
  }

  draw() {
    this.p5Instance.camera(0, 0, 600, 0, 0, 0, 0, 1, 0);

    OutputVector(
      "init",
      this.initialAngularMomentum,
      3,
      vec3.fromValues(-120, -120, 50),
      vec3.fromValues(10, 20, 10),
      this.p5Instance,
    );

    OutputVector(
      "current",
      this.object.angularMomentum,
      5,
      vec3.fromValues(-70, -120, 50),
      vec3.fromValues(10, 10, 10),
      this.p5Instance,
    );

    OutputValue(
      "energy",
      this.object.getRotationKineticEnergy(),
      3,
      vec3.fromValues(0, -120, 50),
      vec3.fromValues(10, 10, 10),
      this.p5Instance,
    );

    DrawLine(
      this.initialAngularMomentum,
      vec3.fromValues(0.0, 0.0, 0.0),
      200,
      vec3.fromValues(180, 50, 50),
      this.p5Instance,
    );

    DrawLine(
      this.object.angularMomentum,
      vec3.fromValues(0.0, 0.0, 0.0),
      200,
      vec3.fromValues(50, 180, 50),
      this.p5Instance,
    );

    DrawAxes(this.object.position, this.object.rotation, 100, this.p5Instance);

    this.object.draw();
  }
}
