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
  camera;

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

  setCamera() {
    this.camera = this.p5Instance.createCamera();
    this.p5Instance.camera(0, 0, 600, 0, 0, 0, 0, 1, 0);
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
    vec3.copy(this.object.angularVelocity, this.object.nextAngularVelocity);
    const ω = this.object.nextAngularVelocity;

    this.object.updateAngularMomentum();

    const gyro = this.object.getGyro();

    const skewOmega = this.getSkewMatrix(
      vec3.clone(this.object.nextAngularVelocity),
    );

    const skewL = this.getSkewMatrix(vec3.clone(this.object.angularMomentum));
    const skewOmegaTimesI = mat3.create();
    mat3.multiply(skewOmegaTimesI, skewOmega, this.object.inertialTensor);

    const jacobiCore = mat3.create();
    mat3.subtract(jacobiCore, skewOmegaTimesI, skewL);

    mat3.multiplyScalar(jacobiCore, jacobiCore, dt);

    const jacobi = mat3.create();
    mat3.add(jacobi, this.object.inertialTensor, jacobiCore);

    const jacobiInv = mat3.create();
    mat3.invert(jacobiInv, jacobi);
    const correction = vec3.create();
    vec3.transformMat3(correction, gyro, jacobiInv);
    vec3.scale(correction, correction, dt);
    vec3.add(this.object.nextAngularVelocity, ω, correction);

    this.object.updateAngularMomentum();
    IntegrateQuatLocal(
      this.object.nextRotation,
      this.object.nextAngularVelocity,
      dt,
    );
  }

  draw() {
    this.p5Instance.orbitControl();

    this.object.draw();

    DrawLine(
      this.initialAngularMomentum,
      vec3.fromValues(0.0, 0.0, 0.0),
      200,
      vec3.fromValues(180, 50, 50),
      this.p5Instance,
    );

    DrawAxes(
      this.object.position,
      this.object.nextRotation,
      100,
      this.p5Instance,
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

    OutputVector(
      "init",
      this.initialAngularMomentum,
      4,
      16,
      [10, 20],
      [10, 10, 10],
      this.p5Instance,
    );

    OutputVector(
      "current",
      this.object.angularMomentum,
      4,
      16,
      [10, 40],
      [10, 10, 10],
      this.p5Instance,
    );

    OutputValue(
      "energy",
      this.object.getRotationKineticEnergy(),
      4,
      16,
      [10, 60],
      [10, 10, 10],
      this.p5Instance,
    );

    this.p5Instance.pop();

    this.p5Instance.drawingContext.enable(
      this.p5Instance.drawingContext.DEPTH_TEST,
    );

    this.p5Instance.camera(...cameraParams);
  }
}
