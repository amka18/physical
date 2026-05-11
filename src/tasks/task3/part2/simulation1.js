import SimulationObject from "../../../common/simulation_object.js";
import {
  IntegrateQuatGlobal,
  IntegrateQuatLocal,
} from "../../../common/util.js";

const { mat3, mat4, vec3, quat } = glMatrix;

export default class Simulation1 {
  object;

  worldInitialAngularMomentum;
  worldCurrentAngularMomentum;

  p5Instance;

  // Параметры пружинки
  springAnchor;
  localAttachmentPoint;
  springStiffness;
  springRestLength;
  springDamping;

  springPoints;
  
  // Для отладки
  debugText;
  
  // Для накопления сил
  accumulatedForce;
  accumulatedTorque;

  constructor(p5Instance) {
    this.p5Instance = p5Instance;

    this.object = new SimulationObject(
      vec3.fromValues(80, 80, 80),     // куб 80x80x80
      vec3.fromValues(0, -120, 0),      // начальная позиция
      vec3.fromValues(0, 0, 0),        // начальный поворот
      vec3.fromValues(1, 1, 1),        // масштаб
      vec3.fromValues(0, 0, 0),        // начальная скорость
      vec3.fromValues(0, 0, 0),        // начальная угловая скорость
      10,                               // масса
      vec3.fromValues(200, 100, 50),   // цвет
      p5Instance,
    );

    // Инициализация пружинки
    this.springAnchor = vec3.fromValues(0, 200, 0);
    
    // Точка крепления к углу куба (локальные координаты)
    // Так как куб 80x80x80, его углы на расстоянии 40 от центра
    this.localAttachmentPoint = vec3.fromValues(0, 20, 40);
    
    this.springStiffness = 100.0;      // Жесткость пружины
    this.springRestLength = 100.0;      // Длина покоя
    this.springDamping = 15.0;          // Демпфирование

    this.springPoints = [];
    this.debugText = "";
    
    this.accumulatedForce = vec3.create();
    this.accumulatedTorque = vec3.create();

    const worldAngularVelocity = vec3.fromValues(0, 0, 0);
    const worldInertialTensor = this.object.getWorldInertialTensor();

    this.worldInitialAngularMomentum = vec3.create();
    vec3.transformMat3(
      this.worldInitialAngularMomentum,
      worldAngularVelocity,
      worldInertialTensor,
    );

    this.worldCurrentAngularMomentum = vec3.create();
  }

  /**
   * ИСПРАВЛЕННАЯ версия с правильной физикой пружины
   */
  update(dt) {
    if (dt > 0.033) dt = 0.033;
    if (dt < 0.001) return;
    
    // Обнуляем накопленные силы
    vec3.zero(this.accumulatedForce);
    vec3.zero(this.accumulatedTorque);
    
    // ============ ВЫЧИСЛЕНИЕ СИЛЫ ПРУЖИНЫ ============
    // Получаем матрицу поворота
    const rotMatrix = mat3.create();
    mat3.fromQuat(rotMatrix, this.object.rotation);
    
    // Вычисляем мировые координаты точки крепления
    const r_local = this.localAttachmentPoint; // локальный вектор от ЦМ до точки крепления
    const r_world = vec3.create();
    vec3.transformMat3(r_world, r_local, rotMatrix);
    
    const worldAttachmentPoint = vec3.create();
    vec3.add(worldAttachmentPoint, this.object.position, r_world);
    
    // Вектор от точки крепления до анкера
    const delta = vec3.create();
    vec3.sub(delta, this.springAnchor, worldAttachmentPoint);
    
    const currentLength = vec3.length(delta);
    
    if (currentLength > 0.001) {
      // Направление силы (от точки крепления к анкеру)
      const direction = vec3.create();
      vec3.normalize(direction, delta);
      
      // Сила упругости (закон Гука)
      const displacement = currentLength - this.springRestLength;
      const springForceMag = this.springStiffness * displacement;
      
      // Скорость точки крепления
      const worldAngularVelocity = this.object.getWorldAngularVelocity();
      const pointVelocity = vec3.create();
      
      // v_point = v_cm + ω × r_world
      vec3.cross(pointVelocity, worldAngularVelocity, r_world);
      vec3.add(pointVelocity, pointVelocity, this.object.linearVelocity);
      
      // Проекция скорости на направление пружины
      const velocityAlongSpring = vec3.dot(pointVelocity, direction);
      
      // Демпфирующая сила (противоположна скорости)
      const dampingForceMag = this.springDamping * velocityAlongSpring;
      
      // Полная сила, действующая на точку крепления (НЕ на центр масс!)
      const totalForceMag = springForceMag - dampingForceMag;
      
      // Вектор силы в мировых координатах
      const forceAtPoint = vec3.create();
      vec3.scale(forceAtPoint, direction, totalForceMag);
      
      // ВАЖНО: Сила, действующая на тело, приложена к точке крепления
      // Это создает и линейное ускорение ЦМ, и вращение
      
      // Линейная сила (та же самая)
      vec3.add(this.accumulatedForce, this.accumulatedForce, forceAtPoint);
      
      // Момент силы: τ = r_world × F
      const torque = vec3.create();
      vec3.cross(torque, r_world, forceAtPoint);
      vec3.add(this.accumulatedTorque, this.accumulatedTorque, torque);
    }
    
    // ============ ПРИМЕНЕНИЕ СИЛ К ТЕЛУ ============
    // Линейное ускорение: a = F / m
    const linearAcc = vec3.create();
    vec3.scale(linearAcc, this.accumulatedForce, 1.0 / this.object.mass);
    
    // Добавляем гравитацию
    const gravity = vec3.fromValues(0, -9.8, 0);
    vec3.add(linearAcc, linearAcc, gravity);
    
    // Момент силы переводим в локальные координаты для обновления угловой скорости
    const rotMatrix2 = mat3.create();
    mat3.fromQuat(rotMatrix2, this.object.rotation);
    const rotMatrixTranspose = mat3.create();
    mat3.transpose(rotMatrixTranspose, rotMatrix2);
    
    const localTorque = vec3.create();
    vec3.transformMat3(localTorque, this.accumulatedTorque, rotMatrixTranspose);
    
    // Угловое ускорение: α = I^(-1) * τ
    const angularAcc = vec3.create();
    vec3.transformMat3(angularAcc, localTorque, this.object.invertInertialTensor);
    
    // ============ SIMPLECTIC EULER ============
    // Шаг 1: Обновляем скорости
    vec3.scaleAndAdd(this.object.linearVelocity, this.object.linearVelocity, linearAcc, dt);
    vec3.scaleAndAdd(this.object.angularVelocity, this.object.angularVelocity, angularAcc, dt);
    
    // Небольшое демпфирование для стабильности
    vec3.scale(this.object.angularVelocity, this.object.angularVelocity, 0.999);
    
    // Шаг 2: Обновляем позицию, используя НОВЫЕ скорости
    vec3.scaleAndAdd(this.object.position, this.object.position, this.object.linearVelocity, dt);
    
    // Обновляем поворот
    const newWorldAngularVelocity = this.object.getWorldAngularVelocity();
    IntegrateQuatGlobal(this.object.rotation, newWorldAngularVelocity, dt);
    quat.normalize(this.object.rotation, this.object.rotation);
    
    // Обновляем угловой момент
    this.object.updateAngularMomentum();
    
    // Сохраняем точки для визуализации
    // Пересчитываем мировую точку крепления с новыми значениями
    const updatedRotMatrix = mat3.create();
    mat3.fromQuat(updatedRotMatrix, this.object.rotation);
    const updatedR_World = vec3.create();
    vec3.transformMat3(updatedR_World, this.localAttachmentPoint, updatedRotMatrix);
    const updatedWorldPoint = vec3.create();
    vec3.add(updatedWorldPoint, this.object.position, updatedR_World);
    
    this.springPoints = [vec3.clone(this.springAnchor), vec3.clone(updatedWorldPoint)];
    
    // Отладочная информация
    const displacement = currentLength > 0 ? currentLength - this.springRestLength : 0;
    this.debugText = `Length: ${currentLength.toFixed(1)} | Disp: ${displacement.toFixed(1)} | Force: ${(this.springStiffness * Math.abs(displacement)).toFixed(1)}`;
  }

  draw() {
    // Настраиваем камеру
    this.p5Instance.camera(0, 100, 600, 0, 100, 0, 0, 1, 0);
    
    // Освещение
    this.p5Instance.ambientLight(80);
    this.p5Instance.directionalLight(255, 255, 255, 0, 1, -1);
    this.p5Instance.directionalLight(200, 200, 200, 1, 0, 0);
    
    // Отладочный текст
    this.p5Instance.fill(255, 255, 0);
    this.p5Instance.textSize(14);
    this.p5Instance.text(this.debugText, -250, -180, 50);
    this.p5Instance.text(`Pos: ${this.object.position[0].toFixed(1)}, ${this.object.position[1].toFixed(1)}, ${this.object.position[2].toFixed(1)}`, -250, -160, 50);
    this.p5Instance.text(`Vel: ${this.object.linearVelocity[1].toFixed(1)}`, -250, -140, 50);
    this.p5Instance.text(`AngVel: ${this.object.angularVelocity[0].toFixed(2)}, ${this.object.angularVelocity[1].toFixed(2)}, ${this.object.angularVelocity[2].toFixed(2)}`, -250, -120, 50);

    // Рисуем пружинку
    if (this.springPoints && this.springPoints.length === 2) {
      this.drawSpring(this.springPoints[0], this.springPoints[1]);
      
      // Анкер (верхняя точка)
      this.p5Instance.push();
      this.p5Instance.translate(
        this.springPoints[0][0],
        this.springPoints[0][1],
        this.springPoints[0][2],
      );
      this.p5Instance.fill(100, 255, 100);
      this.p5Instance.noStroke();
      this.p5Instance.sphere(6);
      this.p5Instance.pop();

      // Точка крепления на объекте
      this.p5Instance.push();
      this.p5Instance.translate(
        this.springPoints[1][0],
        this.springPoints[1][1],
        this.springPoints[1][2],
      );
      this.p5Instance.fill(255, 200, 100);
      this.p5Instance.noStroke();
      this.p5Instance.sphere(5);
      this.p5Instance.pop();
    }

    // Рисуем объект
    this.object.draw();
    
    // Рисуем центр масс
    this.p5Instance.push();
    this.p5Instance.translate(this.object.position[0], this.object.position[1], this.object.position[2]);
    this.p5Instance.fill(255, 0, 0);
    this.p5Instance.sphere(3);
    this.p5Instance.pop();
  }
  
  /**
   * Рисует спираль пружинки
   */
  drawSpring(point1, point2) {
    const start = point1;
    const end = point2;
    
    const direction = vec3.create();
    vec3.sub(direction, end, start);
    const length = vec3.length(direction);
    
    if (length < 0.01) return;
    
    const normalizedDir = vec3.create();
    vec3.normalize(normalizedDir, direction);
    
    // Находим перпендикулярные векторы
    let perp1 = vec3.create();
    if (Math.abs(normalizedDir[0]) < 0.9) {
      perp1 = vec3.fromValues(1, 0, 0);
    } else {
      perp1 = vec3.fromValues(0, 1, 0);
    }
    
    const temp = vec3.create();
    vec3.cross(temp, perp1, normalizedDir);
    vec3.normalize(perp1, temp);
    
    const perp2 = vec3.create();
    vec3.cross(perp2, normalizedDir, perp1);
    
    const turns = 6;
    const radius = 6;
    
    this.p5Instance.stroke(100, 200, 100);
    this.p5Instance.strokeWeight(2);
    this.p5Instance.noFill();
    
    this.p5Instance.beginShape();
    for (let i = 0; i <= 80; i++) {
      const t = i / 80;
      const angle = t * Math.PI * 2 * turns;
      
      const pos = vec3.create();
      vec3.scaleAndAdd(pos, start, normalizedDir, t * length);
      
      const offset = vec3.create();
      vec3.scale(offset, perp1, Math.cos(angle) * radius);
      const offset2 = vec3.create();
      vec3.scale(offset2, perp2, Math.sin(angle) * radius);
      vec3.add(offset, offset, offset2);
      
      vec3.add(pos, pos, offset);
      this.p5Instance.vertex(pos[0], pos[1], pos[2]);
    }
    this.p5Instance.endShape();
  }
}