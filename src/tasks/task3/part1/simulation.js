import SimulationObject from "../../../common/simulation_object.js";
import { IntegrateQuat } from "../../../common/util.js";

const { mat4, vec3, quat } = glMatrix;

export default class Simulation {
  object;

  p5Instance;

  constructor(p5Instance) {
    this.p5Instance = p5Instance;

    this.object = new SimulationObject(
      vec3.fromValues(100, 50, 30),
      vec3.fromValues(0.0, 0.0, 0.0),
      vec3.fromValues(
        this.p5Instance.random(-90, 90),
        this.p5Instance.random(-90, 90),
        this.p5Instance.random(-90, 90),
      ),
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
  }

  update(dt) {
    // // расчет углового ускорения
    // const a = vec3.create();

    // const g = vec3.create();

    // const inverseT = mat3.create();
    // mat3.invert(inverseT, this.object.inertialTensor);

    // mat3.multiplyVec3(
    //   g,
    //   this.object.inertialTensor,
    //   this.object.angularVelocity,
    // );

    // const l = vec3.create();
    // vec3.cross(l, this.object.angularVelocity, g);

    const dQuat = IntegrateQuat(
      this.object.rotation,
      this.object.angularVelocity,
      dt,
    );
    quat.add(this.object.rotation, this.object.rotation, dQuat);
    quat.normalize(this.object.rotation, this.object.rotation);

    this.object.updateWorldInertialTensor();
  }

  draw() {
    this.p5Instance.camera(0, 0, 600, 0, 0, 0, 0, 1, 0);

    this.object.draw();
  }
}
