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
  springCurrentLength;
  springRestLength;
  springStiffness;
  springApplicationPoint;
  springForceValue;
  springDamping;

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

    this.object.addForce(
      vec3.fromValues(0.0, 0.0, 0.0),
      vec3.fromValues(0.0, 0.0, 0.0),
    );

    // Инициализация пружинки
    this.springAnchorPoint = vec3.fromValues(0, 40, 250);
    this.springCurrentLength = 0.0;
    this.springRestLength = 15.0;
    this.springStiffness = 0.1;
    this.springDamping = 0.1;
    this.springApplicationPoint = vec3.clone(
      this.object.getWorldApplicationPoint,
    );
    this.springForceValue = 0.0;
  }

  setCamera() {
    this.camera = this.p5Instance.createCamera();
  }

  update(dt) {
    this.springApplicationPoint = vec3.clone(
      this.object.getWorldApplicationPoint(),
    );

    // присваеваем копираем в текущие

    const springDirection = vec3.create();
    vec3.sub(
      springDirection,
      this.springApplicationPoint,
      this.springAnchorPoint,
    );
    this.springCurrentLength_ = vec3.length(springDirection);

    const forceDirection = vec3.create();
    vec3.normalize(forceDirection, springDirection);

    const dL = this.springCurrentLength - this.springRestLength;

    const worldApplicationPointVelocity =
      this.object.getWorldApplicationPointVelocity();

    const velocityAlongSpring = vec3.dot(
      worldApplicationPointVelocity,
      springDirection,
    );

    const dampingForceMag = this.springDamping * velocityAlongSpring;

    const springForceMag = vec3.create();
    vec3.scale(springForceMag, forceDirection, dL * this.springStiffness);

    // сила в мировых
    const totalForceMag = springForceMag + dampingForceMag;

    // переведем силу в локальные // сделаем это в функции
    const localForce = vec3.create();

    const rotMatrix2 = mat3.create();
    mat3.fromQuat(rotMatrix2, this.object.rotation);
    const invertRotation = mat3.create();
    mat3.invert(invertRotation, rotMatrix2);

    vec3.transformMat3(localForce, totalForceMag, invertRotation);

    this.object.updateTorque(localForce);
  }

  draw() {
    // Настраиваем камеру
    this.p5Instance.orbitControl();

    this.plane.draw();

    const springDirection = vec3.create();
    vec3.sub(
      springDirection,
      this.springApplicationPoint,
      this.springAnchorPoint,
    );
    DrawLine(
      springDirection,
      this.springAnchorPoint,
      this.springCurrentLength,
      [100, 0, 0],
      this.p5Instance,
    );

    // Рисуем объект
    this.object.draw();

    // 2d

    const camParams = [
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
      "rest l",
      this.springRestLength,
      3,
      20,
      [10, 10],
      [100, 30, 0],
      this.p5Instance,
    );

    OutputValue(
      "cur l",
      this.springCurrentLength,
      3,
      20,
      [10, 50],
      [100, 30, 0],
      this.p5Instance,
    );

    this.p5Instance.fill(255, 0, 0);
    this.p5Instance.textSize(30);
    this.p5Instance.textAlign(this.p5Instance.LEFT, this.p5Instance.BOTTOM);
    this.p5Instance.text("test", 100, 100);

    this.p5Instance.pop();

    this.p5Instance.drawingContext.enable(
      this.p5Instance.drawingContext.DEPTH_TEST,
    );

    this.p5Instance.camera(...camParams);
  }
}
