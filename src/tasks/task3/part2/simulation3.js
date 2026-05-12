import SimulationObject from "../../../common/simulation_object.js";
import {
  IntegrateQuatGlobal,
  IntegrateQuatLocal,
} from "../../../common/integrators.js";

const { mat3, mat4, vec3, quat } = glMatrix;

export default class Simulation1 {
  object1;
  object2;
  
  constraintRestLength;
  constraintStiffness = 1000.0; // Жесткость (вместо compliance)
  
  localAttachPoint1;
  localAttachPoint2;
  
  p5Instance;

  constructor(p5Instance) {
    this.p5Instance = p5Instance;

    // Создаем первое тело
    this.object1 = new SimulationObject(
      vec3.fromValues(60, 60, 60),
      vec3.fromValues(-80, 50, 0),  // Подняли повыше
      vec3.fromValues(0.0, 0.0, 0.0),
      vec3.fromValues(1.0, 1.0, 1.0),
      vec3.fromValues(0.0, 0.0, 0.0),
      vec3.fromValues(0.0, 0.0, 0.0),
      10.0,
      vec3.fromValues(255, 0, 0),
      p5Instance,
    );
    
    // Создаем второе тело
    this.object2 = new SimulationObject(
      vec3.fromValues(60, 60, 60),
      vec3.fromValues(80, 60, 0),   // Подняли повыше
      vec3.fromValues(0.0, 0.0, 0.0),
      vec3.fromValues(1.0, 1.0, 1.0),
      vec3.fromValues(0.0, 0.0, 0.0),
      vec3.fromValues(0.0, 0.0, 0.0),
      10.0,
      vec3.fromValues(0, 0, 255),
      p5Instance,
    );
    
    // Точки прикрепления (на гранях кубов)
    this.localAttachPoint1 = vec3.fromValues(30, 0, 0);  // Правая грань первого куба
    this.localAttachPoint2 = vec3.fromValues(-30, 0, 0); // Левая грань второго куба
    
    // Начальная длина ограничения
    const p1 = this.getWorldPoint(this.object1, this.localAttachPoint1);
    const p2 = this.getWorldPoint(this.object2, this.localAttachPoint2);
    this.constraintRestLength = vec3.distance(p1, p2);
  }

  getWorldPoint(object, localPoint) {
    const worldPoint = vec3.create();
    vec3.transformQuat(worldPoint, localPoint, object.nextRotation);
    vec3.add(worldPoint, worldPoint, object.nextPosition);
    return worldPoint;
  }

  /**
   * Решение ограничения методом Sequential Impulses (проекция скоростей)
   * Более стабильный подход для твердых тел
   */
  solveConstraintSequential(dt) {
    // Получаем мировые координаты точек
    const p1 = this.getWorldPoint(this.object1, this.localAttachPoint1);
    const p2 = this.getWorldPoint(this.object2, this.localAttachPoint2);
    
    // Вычисляем вектор и расстояние
    const delta = vec3.create();
    vec3.subtract(delta, p2, p1);
    const dist = vec3.length(delta);
    
    if (dist < 0.001) return;
    
    // Ошибка ограничения
    const error = dist - this.constraintRestLength;
    
    // Если ошибка маленькая, пропускаем
    if (Math.abs(error) < 0.01) return;
    
    // Направление ограничения
    const n = vec3.create();
    vec3.scale(n, delta, 1.0 / dist);
    
    // Векторы от центров масс к точкам
    const r1 = vec3.create();
    const r2 = vec3.create();
    vec3.subtract(r1, p1, this.object1.nextPosition);
    vec3.subtract(r2, p2, this.object2.nextPosition);
    
    // Вычисляем относительную скорость в точке ограничения
    // v_rel = (v2 + ω2 × r2) - (v1 + ω1 × r1)
    const v1_at_point = vec3.create();
    const v2_at_point = vec3.create();
    const omega1_cross_r1 = vec3.create();
    const omega2_cross_r2 = vec3.create();
    
    vec3.cross(omega1_cross_r1, this.object1.nextAngularVelocity, r1);
    vec3.cross(omega2_cross_r2, this.object2.nextAngularVelocity, r2);
    
    vec3.add(v1_at_point, this.object1.nextVelocity, omega1_cross_r1);
    vec3.add(v2_at_point, this.object2.nextVelocity, omega2_cross_r2);
    
    const v_rel = vec3.create();
    vec3.subtract(v_rel, v2_at_point, v1_at_point);
    
    // Скорость вдоль ограничения
    const v_rel_n = vec3.dot(v_rel, n);
    
    // Вычисляем эффективную массу
    const r1xn = vec3.create();
    const r2xn = vec3.create();
    vec3.cross(r1xn, r1, n);
    vec3.cross(r2xn, r2, n);
    
    const I1_world = this.object1.getWorldInertialTensor();
    const I2_world = this.object2.getWorldInertialTensor();
    
    const I1_inv = mat3.create();
    const I2_inv = mat3.create();
    mat3.invert(I1_inv, I1_world);
    mat3.invert(I2_inv, I2_world);
    
    const temp1 = vec3.create();
    const temp2 = vec3.create();
    vec3.transformMat3(temp1, r1xn, I1_inv);
    vec3.transformMat3(temp2, r2xn, I2_inv);
    
    const w1 = 1.0 / this.object1.mass + vec3.dot(r1xn, temp1);
    const w2 = 1.0 / this.object2.mass + vec3.dot(r2xn, temp2);
    const effectiveMass = 1.0 / (w1 + w2);
    
    // Baumgarte stabilization для позиционной ошибки
    const beta = 0.2; // Коэффициент коррекции позиции
    const nextPositionCorrection = beta * error / dt;
    
    // Вычисляем импульс
    let impulse = -(v_rel_n + nextPositionCorrection) * effectiveMass;
    
    // Ограничиваем импульс
    const maxImpulse = 100.0;
    impulse = Math.min(maxImpulse, Math.max(-maxImpulse, impulse));
    
    // Применяем импульс к телам
    const impulseVec = vec3.create();
    vec3.scale(impulseVec, n, impulse);
    
    // Линейные импульсы
    const linearImpulse1 = vec3.create();
    const linearImpulse2 = vec3.create();
    vec3.scale(linearImpulse1, impulseVec, 1.0 / this.object1.mass);
    vec3.scale(linearImpulse2, impulseVec, -1.0 / this.object2.mass);
    
    vec3.add(this.object1.nextVelocity, this.object1.nextVelocity, linearImpulse1);
    vec3.add(this.object2.nextVelocity, this.object2.nextVelocity, linearImpulse2);
    
    // Угловые импульсы
    const angularImpulse1 = vec3.create();
    const angularImpulse2 = vec3.create();
    vec3.transformMat3(angularImpulse1, r1xn, I1_inv);
    vec3.transformMat3(angularImpulse2, r2xn, I2_inv);
    vec3.scale(angularImpulse1, angularImpulse1, impulse);
    vec3.scale(angularImpulse2, angularImpulse2, -impulse);
    
    vec3.add(this.object1.nextAngularVelocity, this.object1.nextAngularVelocity, angularImpulse1);
    vec3.add(this.object2.nextAngularVelocity, this.object2.nextAngularVelocity, angularImpulse2);
  }

  update(dt) {
    // Ограничиваем шаг времени
    dt = Math.min(dt, 0.033);
    
    // 1. Обновляем угловые моменты с гироскопическим слагаемым
    const torque1 = vec3.create();
    const torque2 = vec3.create();
    
    // Гироскопический момент: τ = -ω × L
    vec3.cross(torque1, this.object1.nextAngularVelocity, this.object1.angularMomentum);
    vec3.cross(torque2, this.object2.nextAngularVelocity, this.object2.angularMomentum);
    vec3.scale(torque1, torque1, -1);
    vec3.scale(torque2, torque2, -1);
    
    // Обновляем угловые моменты (Simplectic Euler)
    vec3.scaleAndAdd(this.object1.angularMomentum, this.object1.angularMomentum, torque1, dt);
    vec3.scaleAndAdd(this.object2.angularMomentum, this.object2.angularMomentum, torque2, dt);
    
    // Обновляем угловые скорости
    vec3.transformMat3(this.object1.nextAngularVelocity, this.object1.angularMomentum, this.object1.invertInertialTensor);
    vec3.transformMat3(this.object2.nextAngularVelocity, this.object2.angularMomentum, this.object2.invertInertialTensor);
    
    // 2. Применяем силы (гравитацию)
    const gravity = vec3.fromValues(0, -20.0, 0); // Усиленная гравитация
    const force1 = vec3.create();
    const force2 = vec3.create();
    vec3.scale(force1, gravity, this.object1.mass);
    vec3.scale(force2, gravity, this.object2.mass);
    
    // Обновляем линейные скорости
    vec3.scaleAndAdd(this.object1.nextVelocity, this.object1.nextVelocity, force1, dt / this.object1.mass);
    vec3.scaleAndAdd(this.object2.nextVelocity, this.object2.nextVelocity, force2, dt / this.object2.mass);
    
    // 3. Решаем ограничения несколько раз (итеративно)
    const iterations = 5;
    for (let i = 0; i < iterations; i++) {
      this.solveConstraintSequential(dt);
    }
    
    // 4. Интегрируем позиции
    vec3.scaleAndAdd(this.object1.nextPosition, this.object1.nextPosition, this.object1.nextVelocity, dt);
    vec3.scaleAndAdd(this.object2.nextPosition, this.object2.nextPosition, this.object2.nextVelocity, dt);
    
    // 5. Интегрируем вращения
    IntegrateQuatLocal(this.object1.nextRotation, this.object1.nextAngularVelocity, dt);
    IntegrateQuatLocal(this.object2.nextRotation, this.object2.nextAngularVelocity, dt);
    
    // 6. Обновляем угловые моменты для следующего шага
    this.object1.updateAngularMomentum();
    this.object2.updateAngularMomentum();
    
    // 7. Простое ограничение пола
    const floorY = -150;
    if (this.object1.nextPosition[1] - 30 < floorY) {
      this.object1.nextPosition[1] = floorY + 30;
      this.object1.nextVelocity[1] *= -0.3;
    }
    if (this.object2.nextPosition[1] - 30 < floorY) {
      this.object2.nextPosition[1] = floorY + 30;
      this.object2.nextVelocity[1] *= -0.3;
    }
  }

  draw() {
    this.p5Instance.background(40);
    this.p5Instance.camera(0, 0, 500, 0, 0, 0, 0, 1, 0);
    
    // Рисуем пол
    this.p5Instance.push();
    this.p5Instance.stroke(100);
    this.p5Instance.strokeWeight(1);
    for (let x = -300; x <= 300; x += 50) {
      this.p5Instance.line(x, -150, -300, x, -150, 300);
    }
    this.p5Instance.pop();
    
    // Информация
    this.p5Instance.fill(255, 255, 255);
    this.p5Instance.textSize(14);
    this.p5Instance.text(`Sequential Impulses + Baumgarte`, -250, -200, 50);
    this.p5Instance.text(`Rest length: ${this.constraintRestLength.toFixed(1)}`, -250, -170, 50);
    
    // Точки прикрепления и линия связи
    const p1 = this.getWorldPoint(this.object1, this.localAttachPoint1);
    const p2 = this.getWorldPoint(this.object2, this.localAttachPoint2);
    const currentDist = vec3.distance(p1, p2);
    
    this.p5Instance.text(`Current distance: ${currentDist.toFixed(1)}`, -250, -140, 50);
    
    // Желтая линия связи
    this.p5Instance.stroke(255, 255, 0);
    this.p5Instance.strokeWeight(3);
    this.p5Instance.line(p1[0], p1[1], p1[2], p2[0], p2[1], p2[2]);
    
    // Точки прикрепления
    this.p5Instance.push();
    this.p5Instance.fill(255, 255, 0);
    this.p5Instance.noStroke();
    this.p5Instance.translate(p1[0], p1[1], p1[2]);
    this.p5Instance.sphere(5);
    this.p5Instance.pop();
    
    this.p5Instance.push();
    this.p5Instance.fill(255, 255, 0);
    this.p5Instance.noStroke();
    this.p5Instance.translate(p2[0], p2[1], p2[2]);
    this.p5Instance.sphere(5);
    this.p5Instance.pop();
    
    // Отрисовка тел
    this.object1.draw();
    this.object2.draw();
  }
}