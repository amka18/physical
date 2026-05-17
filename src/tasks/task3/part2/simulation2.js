import SimulationObject from "../../../common/simulation_object.js";
import Plane from "../../../common/plane.js";
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
import { GetLength } from "../../../common/utils.js";

const { mat3, mat4, vec3, quat } = glMatrix;

export default class Simulation2 {
  object;
  plane;

  p5Instance;
  camera;

  springAnchorPoint;
  worldAttachmentPoint;

  springCurrentLength;
  springRestLength;

  springStiffness;
  springDamping;
  springFreqParam;

  springForce1;
  springForce2;

  constructor(p5Instance) {
    this.p5Instance = p5Instance;

    this.plane = new Plane(500, [50, 150, 50], p5Instance);

    this.object = new SimulationObject(
      [80, 80, 80],
      vec3.fromValues(0, 40, 0),
      vec3.fromValues(0, 0, 0),
      vec3.fromValues(1, 1, 1),
      vec3.fromValues(0, 0, 0),
      vec3.fromValues(0, 0, 0),
      10,
      vec3.fromValues(
        Math.floor(this.p5Instance.random(255)),
        Math.floor(this.p5Instance.random(255)),
        Math.floor(this.p5Instance.random(255)),
      ),
      p5Instance,
    );

    this.springAnchorPoint = vec3.fromValues(0, 40, 250);
    this.object.addAttachmentPoint(vec3.fromValues(10.0, 0.0, 40.0));
    this.worldAttachmentPoint = vec3.fromValues(0.0, 0.0, 0.0);
    this.springCurrentLength = 0.0;
    this.springRestLength = 150.0;
    this.springStiffness = 0.0001;
    this.springDamping = 0.00001;
    this.springForce1 = 0.0;
    this.springForce2 = 0.0;
    this.springFreqParam = 0.001;
  }

  setCamera() {
    this.camera = this.p5Instance.createCamera();
    this.p5Instance.camera(0, 600, 1000, 0, 0, 0, 0, 1, 0);
  }

  update(dt) {
    const obj = this.object;

    // Копируем текущее состояние в "предыдущее"
    vec3.copy(obj.position, obj.nextPosition);
    quat.copy(obj.rotation, obj.nextRotation);
    vec3.copy(obj.velocity, obj.nextVelocity);
    vec3.copy(obj.angularVelocity, obj.nextAngularVelocity);

    // Получаем мировую позицию точки крепления пружины на объекте
    this.worldAttachmentPoint = obj.getWorldPositionAttachmentPoint();

    // Вычисляем вектор от якоря к точке крепления на объекте
    const springToObjectDir = vec3.create();
    vec3.sub(
      springToObjectDir,
      this.worldAttachmentPoint,
      this.springAnchorPoint,
    );
    this.springCurrentLength = vec3.length(springToObjectDir);

    // Нормализуем направление (Якобиан J)
    const springObjectDirN = vec3.create();
    vec3.normalize(springObjectDirN, springToObjectDir);

    // Вычисляем рычаг (r) от центра масс объекта до точки крепления
    const arm = vec3.create();
    vec3.sub(arm, this.worldAttachmentPoint, obj.position);

    // Вычисляем tempCross1 = r x n (для угловой части)
    const tempCross1 = vec3.create();
    vec3.cross(tempCross1, arm, springObjectDirN);

    // Вычисляем эффективную массу (K = 1/m + (r x n)^T * I^-1 * (r x n))
    // invertWorldInertialTensor должен быть мировым обратным тензором инерции
    const invertWorldInertialTensor = obj.getWorldInvertInertialTensor();
    const temp2 = vec3.create();
    vec3.transformMat3(temp2, tempCross1, invertWorldInertialTensor);
    const rotationalTemp = vec3.dot(tempCross1, temp2);
    const invEffMass = 1.0 / obj.mass + rotationalTemp; // Это K
    const effMass = 1.0 / invEffMass; // Это 1/K

    // --- Параметры мягкого ограничения ---
    // ВАЖНО: Используем частоту в Герцах! Например, 5 Гц.
    const frequencyHz = 5.0; // Замените на this.frequencyHz
    const dampingRatio = this.springDamping; // Обычно от 0 до 1 (у вас 0.01)

    const omega = 2.0 * Math.PI * frequencyHz;
    const k = effMass * omega * omega;
    const c = 2.0 * effMass * dampingRatio * omega;

    // Вычисляем beta и gamma (softness)
    const demon = c + dt * k; // Знаменатель
    const beta = (dt * k) / demon; // Коэффициент Baumgarte
    const gamma = 1.0 / demon; // Softness (податливость)

    // Вычисляем ошибку constraint'а (C)
    const C = this.springRestLength - this.springCurrentLength;

    // Вычисляем относительную скорость вдоль constraint'а (Jv)
    // Скорость точки крепления с учетом вращения
    const attachmentPointVelocity = vec3.clone(obj.velocity);
    const rotateVelocity = vec3.create();
    vec3.cross(rotateVelocity, obj.angularVelocity, arm);
    vec3.add(attachmentPointVelocity, attachmentPointVelocity, rotateVelocity);

    // Проекция относительной скорости на направление constraint'а
    const Jv = vec3.dot(springObjectDirN, attachmentPointVelocity);

    // Вычисляем lambda (импульс) по формуле из статьи:
    // lambda = - (Jv + beta * C / dt) / (K + gamma)
    const lambda = -(Jv + (beta * C) / dt) / (invEffMass + gamma);

    // Применяем линейный импульс
    const impulseLinear = vec3.create();
    vec3.scale(impulseLinear, springObjectDirN, lambda);
    vec3.scaleAndAdd(
      obj.nextVelocity,
      obj.velocity,
      impulseLinear,
      1.0 / obj.mass,
    );

    // Применяем угловой импульс
    const impulseAngular = vec3.create();
    vec3.scale(impulseAngular, tempCross1, lambda);
    vec3.transformMat3(
      impulseAngular,
      impulseAngular,
      invertWorldInertialTensor,
    );
    vec3.add(obj.nextAngularVelocity, obj.angularVelocity, impulseAngular);

    // Интегрируем позицию и ориентацию
    vec3.scaleAndAdd(obj.nextPosition, obj.position, obj.nextVelocity, dt);

    const halfAngle = vec3.create();
    vec3.scale(halfAngle, obj.nextAngularVelocity, dt * 0.5);
    // Исправление: quat.fromValues принимает (x, y, z, w)
    const deltaRotation = quat.fromValues(
      halfAngle[0],
      halfAngle[1],
      halfAngle[2],
      1,
    );
    quat.normalize(deltaRotation, deltaRotation);
    quat.multiply(obj.nextRotation, deltaRotation, obj.rotation);
  }

  draw() {
    this.p5Instance.orbitControl();

    this.plane.draw();
    this.object.draw();

    this.p5Instance.line(
      this.springAnchorPoint[0],
      this.springAnchorPoint[1],
      this.springAnchorPoint[2],
      this.worldAttachmentPoint[0],
      this.worldAttachmentPoint[1],
      this.worldAttachmentPoint[2],
    );

    const cameraParams = [
      this.camera.eyeX,
      this.camera.eyeY,
      this.camera.eyeZ,
      this.camera.centerX,
      this.camera.centerY,
      this.camera.centerZ,
      0,
      -1,
      0,
    ];

    this.p5Instance.drawingContext.disable(
      this.p5Instance.drawingContext.DEPTH_TEST,
    );

    this.p5Instance.camera();

    this.p5Instance.push();

    this.p5Instance.translate(
      -this.p5Instance.width / 2,
      -this.p5Instance.height / 2,
    );

    OutputValue(
      "l0 = ",
      this.springRestLength,
      3,
      20,
      [10, 20],
      [100, 30, 0],
      this.p5Instance,
    );

    OutputValue(
      "dl = ",
      this.springCurrentLength,
      3,
      20,
      [10, 50],
      [100, 30, 0],
      this.p5Instance,
    );

    this.p5Instance.pop();

    this.p5Instance.drawingContext.enable(
      this.p5Instance.drawingContext.DEPTH_TEST,
    );

    this.p5Instance.camera(...cameraParams);
  }
}
