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

export default class Simulation1 {
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

    this.springAnchorPoint = vec3.fromValues(-50, 40, 250);
    this.object.addAttachmentPoint(vec3.fromValues(20.0, 0.0, 40.0));
    this.worldAttachmentPoint = vec3.fromValues(0.0, 0.0, 0.0);
    this.springCurrentLength = 0.0;
    this.springRestLength = 200.0;
    this.springStiffness = 0.00001;
    this.springDamping = 0.001;
    this.springForce1 = 0.0;
    this.springForce2 = 0.0;
  }

  setCamera() {
    this.camera = this.p5Instance.createCamera();
  }

  update(dt) {
    // ОБНОВЛЯЕМ
    const obj = this.object;

    vec3.copy(obj.position, obj.nextPosition);
    quat.copy(obj.rotation, obj.nextRotation);
    vec3.copy(obj.velocity, obj.nextVelocity);
    vec3.copy(obj.angularVelocity, obj.nextAngularVelocity);

    // НАХОДИМ СИЛЫ
    // обновляем мировую координату точки соприкосеовения
    this.worldAttachmentPoint = obj.getWorldPositionAttachmentPoint();

    // расчитываем закон гука
    const springToObjectDir = vec3.create();
    vec3.sub(
      springToObjectDir,
      this.worldAttachmentPoint,
      this.springAnchorPoint,
    );
    const springObjectDirN = vec3.create();
    vec3.normalize(springObjectDirN, springToObjectDir);
    this.springCurrentLength = vec3.length(springToObjectDir);
    this.springForce1 =
      (this.springRestLength - this.springCurrentLength) * this.springStiffness;
    const force1 = vec3.create();
    vec3.normalize(force1, springToObjectDir);
    vec3.scale(force1, force1, this.springForce1);

    // демпфирование
    const attachmentPointVelocity = vec3.clone(obj.velocity);
    const rotateVelocity = vec3.create();
    vec3.cross(rotateVelocity, obj.angularVelocity, obj.localAnchor);
    vec3.add(attachmentPointVelocity, attachmentPointVelocity, rotateVelocity);

    const force2 = vec3.create();
    vec3.scale(force2, attachmentPointVelocity, this.springDamping);

    const force = vec3.create();
    vec3.add(force, force, force1);
    vec3.sub(force, force, force2);

    // переводм силу в локальные координаты
    const rotationMatrix = mat3.create();
    mat3.fromQuat(rotationMatrix, obj.rotation);
    const invertRotationMatrix = mat3.create();
    mat3.invert(invertRotationMatrix, rotationMatrix);
    const localForce = vec3.create();
    vec3.transformMat3(localForce, force, invertRotationMatrix);

    // расчитываем локальный момент
    const localTorque = vec3.create();
    vec3.cross(localTorque, obj.localAnchor, force);

    // РАСЧЕТ ЭФФЕКТИВНОЙ МАССЫ
    const arm = vec3.create();
    vec3.sub(arm, this.worldAttachmentPoint, obj.position);
    const tempN = vec3.create(); //
    vec3.scale(tempN, springObjectDirN, -1);
    const tempCross1 = vec3.create();
    vec3.cross(tempCross1, arm, tempN);
    const invertWorldInertialTensor = obj.getWorldInvertInertialTensor();
    const temp2 = vec3.create();
    vec3.transformMat3(temp2, tempCross1, invertWorldInertialTensor);
    const rotationalTemp = vec3.dot(tempCross1, temp2);
    const invEffMass = 1 / obj.mass + rotationalTemp;

    // update velocity (обновляем скорости)
    const acceleration = vec3.create();
    vec3.scale(acceleration, force, invEffMass);
    vec3.scaleAndAdd(obj.nextVelocity, obj.velocity, acceleration, dt);

    const angularAcceleration = vec3.create();
    vec3.transformMat3(
      angularAcceleration,
      localTorque,
      obj.invertInertialTensor,
    );
    vec3.scaleAndAdd(
      obj.nextAngularVelocity,
      obj.angularVelocity,
      angularAcceleration,
      dt,
    );

    // // передвигаем позиции
    vec3.scaleAndAdd(obj.nextPosition, obj.position, obj.nextVelocity, dt);
    IntegrateQuatLocal(obj.nextRotation, obj.nextAngularVelocity, dt);
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
