import SimulationObject from "../../../common/simulation_object.js";
import {
  IntegrateQuatGlobal,
  IntegrateQuatLocal,
} from "../../../common/util.js";

const { mat3, mat4, vec3, quat } = glMatrix;

export default class Simulation2 {
  object;

  worldInitialAngularMomentum;
  worldCurrentAngularMomentum;

  p5Instance;

  // Параметры пружины
  springAnchor;
  localAttachmentPoint;
  springRestLength;
  springStiffness;
  springDamping;

  // Для визуализации
  springPoints;
  debugText;

  // Для накопления сил
  accumulatedForce;
  accumulatedTorque;

  constructor(p5Instance) {
    this.p5Instance = p5Instance;

    this.object = new SimulationObject(
      vec3.fromValues(80, 80, 80),
      vec3.fromValues(0, 150, 0),
      vec3.fromValues(10, 15, 5), // небольшой начальный поворот
      vec3.fromValues(1, 1, 1),
      vec3.fromValues(0, 0, 0),
      vec3.fromValues(0, 0, 0),
      15,
      vec3.fromValues(200, 150, 100),
      p5Instance,
    );

    // НАСТРОЙКА ПРУЖИНЫ
    this.springAnchor = vec3.fromValues(0, 220, 0);
    this.localAttachmentPoint = vec3.fromValues(35, 35, 35);
    this.springRestLength = 120;

    // Подобранные параметры для стабильности
    this.springStiffness = 80.0;
    this.springDamping = 25.0;

    this.springPoints = [];
    this.debugText = "";

    this.accumulatedForce = vec3.create();
    this.accumulatedTorque = vec3.create();
  }

  update(dt) {
    if (dt > 0.033) dt = 0.033;
    if (dt < 0.001) return;

    // Обнуляем накопленные силы
    vec3.zero(this.accumulatedForce);
    vec3.zero(this.accumulatedTorque);

    // ============ ВЫЧИСЛЕНИЕ СИЛЫ ПРУЖИНЫ ============
    const rotMatrix = mat3.create();
    mat3.fromQuat(rotMatrix, this.object.nextRotation);

    // Вектор от центра масс до точки крепления в мировых координатах
    const r_world = vec3.create();
    vec3.transformMat3(r_world, this.localAttachmentPoint, rotMatrix);

    // Мировая позиция точки крепления
    const worldAttachmentPoint = vec3.create();
    vec3.add(worldAttachmentPoint, this.object.nextPosition, r_world);

    // Вектор от точки крепления до анкера
    const delta = vec3.create();
    vec3.sub(delta, this.springAnchor, worldAttachmentPoint);
    const currentLength = vec3.length(delta);

    if (currentLength > 0.001) {
      // Направление силы
      const direction = vec3.create();
      vec3.normalize(direction, delta);

      // Сила упругости (закон Гука)
      const displacement = currentLength - this.springRestLength;
      const springForceMag = this.springStiffness * displacement;

      // Скорость точки крепления: v_point = v_cm + ω × r
      const worldAngularVelocity = this.object.getWorldAngularVelocity();
      const pointVelocity = vec3.create();
      vec3.cross(pointVelocity, worldAngularVelocity, r_world);
      vec3.add(pointVelocity, pointVelocity, this.object.nextVelocity);

      // Скорость растяжения вдоль пружины
      const stretchVelocity = vec3.dot(pointVelocity, direction);

      // Демпфирующая сила
      const dampingForceMag = this.springDamping * stretchVelocity;

      // Полная сила (знак: положительная = растяжение)
      const totalForceMag = -(springForceMag + dampingForceMag);

      // Вектор силы в точке крепления
      const forceAtPoint = vec3.create();
      vec3.scale(forceAtPoint, direction, totalForceMag);

      // Добавляем силу к центру масс
      vec3.add(this.accumulatedForce, this.accumulatedForce, forceAtPoint);

      // Момент силы: τ = r × F
      const torque = vec3.create();
      vec3.cross(torque, r_world, forceAtPoint);
      vec3.add(this.accumulatedTorque, this.accumulatedTorque, torque);
    }

    // ============ ГРАВИТАЦИЯ ============
    const gravity = vec3.fromValues(0, -12.0, 0);
    const gravityForce = vec3.create();
    vec3.scale(gravityForce, gravity, this.object.mass);
    vec3.add(this.accumulatedForce, this.accumulatedForce, gravityForce);

    // ============ ПРИМЕНЕНИЕ СИЛ ============
    // Линейное ускорение
    const linearAcc = vec3.create();
    vec3.scale(linearAcc, this.accumulatedForce, 1.0 / this.object.mass);

    // Момент силы в локальные координаты
    const rotMatrix2 = mat3.create();
    mat3.fromQuat(rotMatrix2, this.object.nextRotation);
    const rotMatrixTranspose = mat3.create();
    mat3.transpose(rotMatrixTranspose, rotMatrix2);

    const localTorque = vec3.create();
    vec3.transformMat3(localTorque, this.accumulatedTorque, rotMatrixTranspose);

    // Угловое ускорение
    const angularAcc = vec3.create();
    vec3.transformMat3(
      angularAcc,
      localTorque,
      this.object.invertInertialTensor,
    );

    // ============ SIMPLECTIC EULER ============
    // Обновляем скорости
    vec3.scaleAndAdd(
      this.object.nextVelocity,
      this.object.nextVelocity,
      linearAcc,
      dt,
    );
    vec3.scaleAndAdd(
      this.object.nextAngularVelocity,
      this.object.nextAngularVelocity,
      angularAcc,
      dt,
    );

    // Демпфирование для стабильности
    vec3.scale(this.object.nextVelocity, this.object.nextVelocity, 0.999);
    vec3.scale(this.object.nextAngularVelocity, this.object.nextAngularVelocity, 0.998);

    // Обновляем позицию
    vec3.scaleAndAdd(
      this.object.nextPosition,
      this.object.nextPosition,
      this.object.nextVelocity,
      dt,
    );

    // Обновляем поворот
    const newWorldAngularVelocity = this.object.getWorldAngularVelocity();
    IntegrateQuatGlobal(this.object.nextRotation, newWorldAngularVelocity, dt);
    quat.normalize(this.object.nextRotation, this.object.nextRotation);

    // Обновляем угловой момент
    this.object.updateAngularMomentum();

    // ============ ВИЗУАЛИЗАЦИЯ ============
    const updatedRotMatrix = mat3.create();
    mat3.fromQuat(updatedRotMatrix, this.object.nextRotation);
    const updatedR_World = vec3.create();
    vec3.transformMat3(
      updatedR_World,
      this.localAttachmentPoint,
      updatedRotMatrix,
    );
    const updatedWorldPoint = vec3.create();
    vec3.add(updatedWorldPoint, this.object.nextPosition, updatedR_World);
    this.springPoints = [
      vec3.clone(this.springAnchor),
      vec3.clone(updatedWorldPoint),
    ];

    // Отладка
//     const currentLength = vec3.distance(this.springAnchor, updatedWorldPoint);
//     const displacement = currentLength - this.springRestLength;
//     const forceMag = Math.abs(this.springStiffness * displacement);
//     const angularVelMag = vec3.length(this.object.nextAngularVelocity);

//     this.debugText = `SPRING ON RIGID BODY
// Length: ${currentLength.toFixed(1)} / ${this.springRestLength}
// Displacement: ${displacement.toFixed(1)}
// Spring Force: ${forceMag.toFixed(1)}
// Stiffness: ${this.springStiffness} | Damping: ${this.springDamping}
// Angular Velocity: ${angularVelMag.toFixed(2)} rad/s
// Pos Y: ${this.object.nextPosition[1].toFixed(1)}
// Vel Y: ${this.object.nextVelocity[1].toFixed(1)}`;

    // Защита от вылета
    if (
      Math.abs(this.object.nextPosition[1]) > 400 ||
      Math.abs(this.object.nextPosition[0]) > 400
    ) {
      this.object.nextPosition = vec3.fromValues(0, 150, 0);
      this.object.nextVelocity = vec3.fromValues(0, 0, 0);
      this.object.nextAngularVelocity = vec3.fromValues(0, 0, 0);
      this.object.nextRotation = quat.create();
      console.log("Object reset - flew too far");
    }
  }

  draw() {
    this.p5Instance.camera(0, 100, 600, 0, 100, 0, 0, 1, 0);

    this.p5Instance.ambientLight(80);
    this.p5Instance.directionalLight(255, 255, 255, 0, 1, -1);
    this.p5Instance.directionalLight(200, 200, 200, 1, 0, 0);

    // Отладочный текст
    this.p5Instance.fill(255, 255, 0);
    this.p5Instance.textSize(11);
    const lines = this.debugText.split("\n");
    for (let i = 0; i < lines.length; i++) {
      this.p5Instance.text(lines[i], -320, -200 + i * 18, 50);
    }

    // Рисуем пружинку
    if (this.springPoints && this.springPoints.length === 2) {
      this.drawSpring(this.springPoints[0], this.springPoints[1]);

      // Анкер
      this.p5Instance.push();
      this.p5Instance.translate(
        this.springPoints[0][0],
        this.springPoints[0][1],
        this.springPoints[0][2],
      );
      this.p5Instance.fill(100, 255, 100);
      this.p5Instance.noStroke();
      this.p5Instance.sphere(8);
      this.p5Instance.pop();

      // Точка крепления
      this.p5Instance.push();
      this.p5Instance.translate(
        this.springPoints[1][0],
        this.springPoints[1][1],
        this.springPoints[1][2],
      );
      this.p5Instance.fill(255, 200, 100);
      this.p5Instance.noStroke();
      this.p5Instance.sphere(6);
      this.p5Instance.pop();

      // Линия от центра масс до точки крепления
      this.p5Instance.stroke(255, 100, 100);
      this.p5Instance.strokeWeight(1);
      this.p5Instance.line(
        this.object.nextPosition[0],
        this.object.nextPosition[1],
        this.object.nextPosition[2],
        this.springPoints[1][0],
        this.springPoints[1][1],
        this.springPoints[1][2],
      );
    }

    this.object.draw();

    // Центр масс
    this.p5Instance.push();
    this.p5Instance.translate(
      this.object.nextPosition[0],
      this.object.nextPosition[1],
      this.object.nextPosition[2],
    );
    this.p5Instance.fill(255, 0, 0);
    this.p5Instance.noStroke();
    this.p5Instance.sphere(4);
    this.p5Instance.pop();
  }

  drawSpring(point1, point2) {
    const start = point1;
    const end = point2;
    const direction = vec3.create();
    vec3.sub(direction, end, start);
    const length = vec3.length(direction);
    if (length < 0.01) return;

    const normalizedDir = vec3.create();
    vec3.normalize(normalizedDir, direction);

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

    const displacement = length - this.springRestLength;
    const forceIntensity = Math.min(1, Math.abs(displacement) / 30);
    const turns = 6;
    const radius = 6;

    const r = 100 + forceIntensity * 155;
    const g = 200 - forceIntensity * 100;
    const b = 100 - forceIntensity * 50;

    this.p5Instance.stroke(r, g, b);
    this.p5Instance.strokeWeight(2 + forceIntensity);
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
