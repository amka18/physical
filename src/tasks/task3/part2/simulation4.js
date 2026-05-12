import SimulationObject from "../../../common/simulation_object.js";
import {
  IntegrateQuatGlobal,
  IntegrateQuatLocal,
} from "../../../common/intergrators.js";

const { mat3, mat4, vec3, quat } = glMatrix;

export default class Simulation1 {
  object1;
  object2;
  
  // Параметры ограничения расстояния
  constraintRestLength;     // Исходная длина связи
  constraintRestLengthSqr;  // Квадрат длины (для оптимизации)
  
  // Параметры Sequential Impulses
  baumgarteBeta = 0.2;      // Коэффициент Baumgarte для коррекции позиционной ошибки
  
  // Точки прикрепления (в локальных координатах тел)
  localAttachPoint1;
  localAttachPoint2;
  
  p5Instance;

  constructor(p5Instance) {
    this.p5Instance = p5Instance;

    // ========== СОЗДАНИЕ ПЕРВОГО ТВЕРДОГО ТЕЛА ==========
    // Параметры: (размеры, позиция, вращение, масштаб, 
    //            линейная скорость, угловая скорость, масса, цвет, p5)
    this.object1 = new SimulationObject(
      vec3.fromValues(60, 60, 60),      // Размеры куба 60x60x60
      vec3.fromValues(-100, 80, 0),     // Позиция: левее центра, выше для падения
      vec3.fromValues(0.0, 0.0, 0.0),   // Начальное вращение
      vec3.fromValues(1.0, 1.0, 1.0),   // Масштаб
      vec3.fromValues(0.0, 0.0, 0.0),   // Начальная линейная скорость
      vec3.fromValues(0.5, 0.3, 0.2),   // Начальная угловая скорость (для эффекта)
      10.0,                             // Масса 10 кг
      vec3.fromValues(255, 100, 100),   // Светло-красный цвет
      p5Instance,
    );
    
    // ========== СОЗДАНИЕ ВТОРОГО ТВЕРДОГО ТЕЛА ==========
    this.object2 = new SimulationObject(
      vec3.fromValues(60, 60, 60),      // Размеры куба 60x60x60
      vec3.fromValues(100, 80, 0),      // Позиция: правее центра, выше для падения
      vec3.fromValues(0.0, 0.0, 0.0),   // Начальное вращение
      vec3.fromValues(1.0, 1.0, 1.0),   // Масштаб
      vec3.fromValues(0.0, 0.0, 0.0),   // Начальная линейная скорость
      vec3.fromValues(-0.4, 0.5, 0.3),  // Начальная угловая скорость (в другую сторону)
      10.0,                             // Масса 10 кг
      vec3.fromValues(100, 100, 255),   // Светло-синий цвет
      p5Instance,
    );
    
    // ========== НАСТРОЙКА ОГРАНИЧЕНИЯ ==========
    // Точки прикрепления НЕ в центре масс (на поверхностях кубов)
    // Для куба 60x60 половина = 30, поэтому точки на гранях
    this.localAttachPoint1 = vec3.fromValues(30, 0, 0);   // Правая грань первого куба
    this.localAttachPoint2 = vec3.fromValues(-30, 0, 0);  // Левая грань второго куба
    
    // Вычисляем начальную длину ограничения между точками
    const worldPoint1 = this.getWorldPoint(this.object1, this.localAttachPoint1);
    const worldPoint2 = this.getWorldPoint(this.object2, this.localAttachPoint2);
    this.constraintRestLength = vec3.distance(worldPoint1, worldPoint2);
    this.constraintRestLengthSqr = this.constraintRestLength * this.constraintRestLength;
  }

  /**
   * Преобразование локальной точки тела в мировые координаты
   * @param {SimulationObject} object - твердое тело
   * @param {vec3} localPoint - точка в локальной системе тела
   * @returns {vec3} точка в мировой системе координат
   * 
   * Формула: p_world = R * p_local + position
   * где R - матрица поворота (кватернион)
   */
  getWorldPoint(object, localPoint) {
    const worldPoint = vec3.create();
    vec3.transformQuat(worldPoint, localPoint, object.rotation);
    vec3.add(worldPoint, worldPoint, object.position);
    return worldPoint;
  }

  /**
   * ============================================================================
   * SEQUENTIAL IMPULSES METHOD for Distance Constraint
   * ============================================================================
   * 
   * ТЕОРИЯ (на основе статьи Erin Catto "Iterative Dynamics with Sequential Impulses"):
   * 
   * 1. Ограничение расстояния между двумя точками на твердых телах:
   *    C = |p2 - p1| - d = 0, где d - исходная длина
   * 
   * 2. Скоростное ограничение (производная от C):
   *    Ċ = (p2 - p1)/|p2 - p1| · (v2 + ω2 × r2 - v1 - ω1 × r1) = n · v_rel
   *    где n - единичный вектор направления, r - вектор от центра масс к точке
   * 
   * 3. Для удовлетворения ограничения применяем импульс P:
   *    Δv = P / m
   *    Δω = I⁻¹ · (r × P)
   * 
   * 4. Эффективная масса (скаляр) в направлении ограничения:
   *    1/m_eff = 1/m₁ + 1/m₂ + (r₁ × n)ᵀ·I₁⁻¹·(r₁ × n) + (r₂ × n)ᵀ·I₂⁻¹·(r₂ × n)
   * 
   * 5. Импульс для достижения нулевой относительной скорости:
   *    P = -v_rel_n / m_eff
   * 
   * 6. BAUMGARTE STABILIZATION: коррекция позиционной ошибки
   *    Добавляем член для исправления накопленной ошибки позиции:
   *    P = -(v_rel_n + β·C/Δt) / m_eff
   *    где β (0..1) - коэффициент Baumgarte
   * 
   * 7. Sequential Impulses: многократно применяем импульсы для всех ограничений
   *    (в нашем случае одно ограничение, но итерации улучшают сходимость)
   * 
   * ============================================================================
   */
  solveDistanceConstraint(dt) {
    // --- ШАГ 1: Получаем мировые координаты точек прикрепления ---
    const p1 = this.getWorldPoint(this.object1, this.localAttachPoint1);
    const p2 = this.getWorldPoint(this.object2, this.localAttachPoint2);
    
    // --- ШАГ 2: Вычисляем вектор и расстояние между точками ---
    const delta = vec3.create();
    vec3.subtract(delta, p2, p1);
    const distance = vec3.length(delta);
    
    // Защита от деления на ноль
    if (distance < 0.0001) return;
    
    // --- ШАГ 3: Направление ограничения (нормаль) ---
    // n = (p2 - p1) / |p2 - p1|
    const n = vec3.create();
    vec3.scale(n, delta, 1.0 / distance);
    
    // --- ШАГ 4: Позиционная ошибка для Baumgarte ---
    // C = current_distance - rest_length
    const positionError = distance - this.constraintRestLength;
    
    // --- ШАГ 5: Векторы от центров масс к точкам прикрепления ---
    const r1 = vec3.create();
    const r2 = vec3.create();
    vec3.subtract(r1, p1, this.object1.position);
    vec3.subtract(r2, p2, this.object2.position);
    
    // --- ШАГ 6: Вычисляем скорости точек в мировом пространстве ---
    // v_point = v_cm + ω × r
    const v1_at_point = vec3.create();
    const v2_at_point = vec3.create();
    const omega1_cross_r1 = vec3.create();
    const omega2_cross_r2 = vec3.create();
    
    vec3.cross(omega1_cross_r1, this.object1.angularVelocity, r1);
    vec3.cross(omega2_cross_r2, this.object2.angularVelocity, r2);
    
    vec3.add(v1_at_point, this.object1.linearVelocity, omega1_cross_r1);
    vec3.add(v2_at_point, this.object2.linearVelocity, omega2_cross_r2);
    
    // --- ШАГ 7: Относительная скорость вдоль направления ограничения ---
    const v_rel = vec3.create();
    vec3.subtract(v_rel, v2_at_point, v1_at_point);
    const v_rel_n = vec3.dot(v_rel, n);
    
    // --- ШАГ 8: Вычисление ЭФФЕКТИВНОЙ МАССЫ ---
    // Для точечной массы: 1/m_eff = 1/m1 + 1/m2
    // Для твердого тела добавляется вращательный вклад: (r × n)ᵀ · I⁻¹ · (r × n)
    
    // Вращательный вклад для тела 1
    const r1xn = vec3.create();
    vec3.cross(r1xn, r1, n);
    
    // Получаем мировой тензор инерции и его обратный
    const I1_world = this.object1.getWorldInertialTensor();
    const I1_inv = mat3.create();
    mat3.invert(I1_inv, I1_world);
    
    // Вычисляем (r × n)ᵀ · I⁻¹ · (r × n)
    const temp1 = vec3.create();
    vec3.transformMat3(temp1, r1xn, I1_inv);
    const rotationalInertia1 = vec3.dot(r1xn, temp1);
    
    // Вращательный вклад для тела 2
    const r2xn = vec3.create();
    vec3.cross(r2xn, r2, n);
    
    const I2_world = this.object2.getWorldInertialTensor();
    const I2_inv = mat3.create();
    mat3.invert(I2_inv, I2_world);
    
    const temp2 = vec3.create();
    vec3.transformMat3(temp2, r2xn, I2_inv);
    const rotationalInertia2 = vec3.dot(r2xn, temp2);
    
    // Полная обратная эффективная масса
    const invEffectiveMass = 
      (1.0 / this.object1.mass) + 
      (1.0 / this.object2.mass) + 
      rotationalInertia1 + 
      rotationalInertia2;
    
    // Эффективная масса
    const effectiveMass = (Math.abs(invEffectiveMass) > 0.0001) ? 1.0 / invEffectiveMass : 0.0;
    
    // --- ШАГ 9: Вычисление необходимого импульса ---
    // С Baumgarte stabilization: P = -(v_rel_n + β·C/Δt) / m_eff
    const baumgarteTerm = this.baumgarteBeta * positionError / dt;
    let impulseMagnitude = -(v_rel_n + baumgarteTerm) * effectiveMass;
    
    // Ограничиваем импульс для стабильности (опционально)
    const maxImpulse = 500.0;
    impulseMagnitude = Math.min(maxImpulse, Math.max(-maxImpulse, impulseMagnitude));
    
    // --- ШАГ 10: Применение импульса к телам ---
    // Импульс как вектор: P = P_magnitude * n
    const impulseVec = vec3.create();
    vec3.scale(impulseVec, n, impulseMagnitude);
    
    // Линейный импульс (меняет линейную скорость)
    // Δv = P / m
    const linearImpulse1 = vec3.create();
    const linearImpulse2 = vec3.create();
    vec3.scale(linearImpulse1, impulseVec, 1.0 / this.object1.mass);
    vec3.scale(linearImpulse2, impulseVec, -1.0 / this.object2.mass);
    
    vec3.add(this.object1.linearVelocity, this.object1.linearVelocity, linearImpulse1);
    vec3.add(this.object2.linearVelocity, this.object2.linearVelocity, linearImpulse2);
    
    // Угловой импульс (меняет угловую скорость)
    // Δω = I⁻¹ · (r × P)
    const angularImpulse1 = vec3.create();
    const angularImpulse2 = vec3.create();
    vec3.transformMat3(angularImpulse1, r1xn, I1_inv);
    vec3.transformMat3(angularImpulse2, r2xn, I2_inv);
    vec3.scale(angularImpulse1, angularImpulse1, impulseMagnitude);
    vec3.scale(angularImpulse2, angularImpulse2, -impulseMagnitude);
    
    vec3.add(this.object1.angularVelocity, this.object1.angularVelocity, angularImpulse1);
    vec3.add(this.object2.angularVelocity, this.object2.angularVelocity, angularImpulse2);
  }

  /**
   * ОСНОВНОЙ МЕТОД ОБНОВЛЕНИЯ ФИЗИКИ
   * 
   * Алгоритм Sequential Impulses:
   * 1. Сохраняем текущие состояния
   * 2. Применяем внешние силы (гравитация)
   * 3. Обновляем угловые моменты с гироскопическим слагаемым
   * 4. ИТЕРАТИВНО решаем ограничения (Sequential Impulses)
   * 5. Интегрируем позиции и вращения
   * 6. Обновляем угловые моменты
   */
  update(dt) {
    // Ограничиваем шаг времени для стабильности
    dt = Math.min(dt, 0.033);
    
    // ===== ШАГ 1: Сохраняем текущие состояния =====
    const oldPos1 = vec3.clone(this.object1.position);
    const oldPos2 = vec3.clone(this.object2.position);
    const oldRot1 = quat.clone(this.object1.rotation);
    const oldRot2 = quat.clone(this.object2.rotation);
    
    // ===== ШАГ 2: Применяем внешние силы (гравитация) =====
    // F = m * g, где g = 9.8 м/с²
    const gravity = vec3.fromValues(0, -9.8, 0);
    const force1 = vec3.create();
    const force2 = vec3.create();
    vec3.scale(force1, gravity, this.object1.mass);
    vec3.scale(force2, gravity, this.object2.mass);
    
    // Обновляем линейные скорости: v = v + (F/m) * dt
    vec3.scaleAndAdd(this.object1.linearVelocity, this.object1.linearVelocity, force1, dt / this.object1.mass);
    vec3.scaleAndAdd(this.object2.linearVelocity, this.object2.linearVelocity, force2, dt / this.object2.mass);
    
    // ===== ШАГ 3: Обновляем вращения с гироскопическим слагаемым =====
    // Simplectic Euler с учетом гироскопического момента
    // Уравнение Эйлера для вращения: dL/dt = τ = -ω × L
    const torque1 = vec3.create();
    const torque2 = vec3.create();
    
    // Гироскопический момент: τ = -ω × L
    vec3.cross(torque1, this.object1.angularVelocity, this.object1.angularMomentum);
    vec3.cross(torque2, this.object2.angularVelocity, this.object2.angularMomentum);
    vec3.scale(torque1, torque1, -1);
    vec3.scale(torque2, torque2, -1);
    
    // Обновляем угловые моменты (явный Эйлер)
    vec3.scaleAndAdd(this.object1.angularMomentum, this.object1.angularMomentum, torque1, dt);
    vec3.scaleAndAdd(this.object2.angularMomentum, this.object2.angularMomentum, torque2, dt);
    
    // Обновляем угловые скорости: ω = I⁻¹ · L
    vec3.transformMat3(this.object1.angularVelocity, this.object1.angularMomentum, this.object1.invertInertialTensor);
    vec3.transformMat3(this.object2.angularVelocity, this.object2.angularMomentum, this.object2.invertInertialTensor);
    
    // ===== ШАГ 4: ИТЕРАТИВНОЕ РЕШЕНИЕ ОГРАНИЧЕНИЙ =====
    // Sequential Impulses: несколько итераций улучшают точность
    // Чем больше итераций, тем жестче связь
    const iterations = 8;
    for (let iteration = 0; iteration < iterations; iteration++) {
      this.solveDistanceConstraint(dt);
    }
    
    // ===== ШАГ 5: ИНТЕГРИРОВАНИЕ ПОЗИЦИЙ И ВРАЩЕНИЙ =====
    // Явный Эйлер для позиций: x_new = x_old + v * dt
    vec3.scaleAndAdd(this.object1.position, this.object1.position, this.object1.linearVelocity, dt);
    vec3.scaleAndAdd(this.object2.position, this.object2.position, this.object2.linearVelocity, dt);
    
    // Интегрирование вращений через кватернионы
    IntegrateQuatLocal(this.object1.rotation, this.object1.angularVelocity, dt);
    IntegrateQuatLocal(this.object2.rotation, this.object2.angularVelocity, dt);
    
    // Нормализация кватернионов для численной стабильности
    quat.normalize(this.object1.rotation, this.object1.rotation);
    quat.normalize(this.object2.rotation, this.object2.rotation);
    
    // ===== ШАГ 6: ОБНОВЛЕНИЕ УГЛОВЫХ МОМЕНТОВ =====
    this.object1.updateAngularMomentum();
    this.object2.updateAngularMomentum();
    
    // ===== ДОПОЛНИТЕЛЬНО: Ограничение пола =====
    const floorY = -150;
    const halfSize = 30;
    
    if (this.object1.position[1] - halfSize < floorY) {
      this.object1.position[1] = floorY + halfSize;
      this.object1.linearVelocity[1] *= -0.3; // Отскок
      // Небольшое трение
      this.object1.linearVelocity[0] *= 0.98;
      this.object1.linearVelocity[2] *= 0.98;
    }
    
    if (this.object2.position[1] - halfSize < floorY) {
      this.object2.position[1] = floorY + halfSize;
      this.object2.linearVelocity[1] *= -0.3;
      this.object2.linearVelocity[0] *= 0.98;
      this.object2.linearVelocity[2] *= 0.98;
    }
  }

  /**
   * ОТРИСОВКА СЦЕНЫ
   */
  draw() {
    this.p5Instance.background(30);
    this.p5Instance.camera(0, 0, 500, 0, 0, 0, 0, 1, 0);
    
    // Рисуем пол (сетка для ориентации)
    this.p5Instance.push();
    this.p5Instance.stroke(80);
    this.p5Instance.strokeWeight(1);
    for (let x = -300; x <= 300; x += 50) {
      this.p5Instance.line(x, -150, -300, x, -150, 300);
    }
    for (let z = -300; z <= 300; z += 50) {
      this.p5Instance.line(-300, -150, z, 300, -150, z);
    }
    this.p5Instance.pop();
    
    // Отображаем информацию
    this.p5Instance.fill(255, 255, 255);
    this.p5Instance.textSize(14);
    this.p5Instance.text("SEQUENTIAL IMPULSES + BAUMGARTE", -250, -200, 50);
    this.p5Instance.text(`Rest length: ${this.constraintRestLength.toFixed(1)}`, -250, -170, 50);
    this.p5Instance.text(`Baumgarte β: ${this.baumgarteBeta}`, -250, -140, 50);
    
    // Получаем мировые координаты точек прикрепления
    const p1 = this.getWorldPoint(this.object1, this.localAttachPoint1);
    const p2 = this.getWorldPoint(this.object2, this.localAttachPoint2);
    const currentDist = vec3.distance(p1, p2);
    
    this.p5Instance.text(`Current distance: ${currentDist.toFixed(1)}`, -250, -110, 50);
    this.p5Instance.text(`Error: ${(currentDist - this.constraintRestLength).toFixed(2)}`, -250, -80, 50);
    
    // Рисуем желтую линию связи
    this.p5Instance.stroke(255, 255, 0);
    this.p5Instance.strokeWeight(3);
    this.p5Instance.line(p1[0], p1[1], p1[2], p2[0], p2[1], p2[2]);
    
    // Рисуем желтые точки прикрепления
    this.p5Instance.push();
    this.p5Instance.fill(255, 255, 0);
    this.p5Instance.noStroke();
    this.p5Instance.translate(p1[0], p1[1], p1[2]);
    this.p5Instance.sphere(6);
    this.p5Instance.pop();
    
    this.p5Instance.push();
    this.p5Instance.fill(255, 255, 0);
    this.p5Instance.noStroke();
    this.p5Instance.translate(p2[0], p2[1], p2[2]);
    this.p5Instance.sphere(6);
    this.p5Instance.pop();
    
    // Отрисовка тел
    this.object1.draw();
    this.object2.draw();
  }
}