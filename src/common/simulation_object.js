const { mat3, mat4, vec3, vec4, quat } = glMatrix;

export default class SimulationObject {
  // global
  position;
  prevPosition;

  rotation;
  prevRotation;

  scale;

  linearVelocity;
  prevLinearVelocity;

  // local
  angularVelocity;
  prevAngularVelocity;

  inertialTensor;
  invertInertialTensor;

  angularMomentum;

  mass;

  localVertices;
  edgeIndices;
  faceIndices;

  worldVertices;

  color;

  p5Instance;

  /**
   * Конструктор SimulationObject
   * @param {number[]} dimensions - [width, height, depth]
   * @param {number[]} position - [x, y, z]
   * @param {number[]} rotation - [eulerX, eulerY, eulerZ] в градусах
   * @param {number[]} scale - [x, y, z]
   * @param {number[]} linearVelocity - [vx, vy, vz]
   * @param {number[]} angularVelocity - [wx, wy, wz]
   * @param {number} mass - масса объекта
   * @param {number[]} color - [r, g, b]
   * @param {p5} p5Instance - экземпляр p5.js
   */
  constructor(
    dimensions,
    position,
    rotation,
    scale,
    velocity,
    angularVelocity,
    mass,
    color,
    p5Instance,
  ) {
    this.position = position ? vec3.clone(position) : vec3.create();
    this.prevPosition = vec3.clone(this.position);

    this.rotation = quat.create();
    quat.fromEuler(this.rotation, rotation[0], rotation[1], rotation[2]);
    this.prevRotation = quat.clone(this.rotation);

    this.scale = scale ? vec3.clone(scale) : vec3.fromValues(1.0, 1.0, 1.0);

    this.linearVelocity = velocity ? vec3.clone(velocity) : vec3.create();
    this.prevLinearVelocity = vec3.clone(this.linearVelocity);

    this.angularVelocity = angularVelocity
      ? vec3.clone(angularVelocity)
      : vec3.fromValues(0.0, 0.0, 0.0);
    this.prevAngularVelocity = vec3.clone(this.angularVelocity);

    this.#initializeInertiaTensor(
      dimensions[0],
      dimensions[1],
      dimensions[2],
      mass,
    );

    this.angularMomentum = vec3.create();
    this.updateAngularMomentum();

    this.mass = mass;

    this.#initializeVertices(dimensions[0], dimensions[1], dimensions[2]);

    this.color = color
      ? vec3.clone(color)
      : vec3.fromValues(200.0, 200.0, 200.0);

    this.p5Instance = p5Instance;
  }

  /**
   *
   * @returns {number}
   */
  getRotationKineticEnergy() {
    this.updateAngularMomentum();
    const energy = 0.5 * vec3.dot(this.angularVelocity, this.angularMomentum);
    return energy;
  }

  /**
   * Получение мировой угловой скорости
   * ω_world = R * ω_local
   * @returns {vec3} Мировая угловая скорость
   */
  getWorldAngularVelocity() {
    const rotationMatrix = mat3.create();

    mat3.fromQuat(rotationMatrix, this.rotation);

    const worldAngularVelocity = vec3.create();
    vec3.transformMat3(
      worldAngularVelocity,
      this.angularVelocity,
      rotationMatrix,
    );

    return worldAngularVelocity;
  }

  /**
   * Получение мирового тензора инерции
   * I_world = R * I_local * R^T
   * @returns {mat3} Мировой тензор инерции
   */
  getWorldInertialTensor() {
    const rotationMatrix = mat3.create();
    mat3.fromQuat(rotationMatrix, this.rotation);

    const transposeRotationMatrix = mat3.create();
    mat3.transpose(transposeRotationMatrix, rotationMatrix);

    const tempMatrix = mat3.create();
    mat3.multiply(tempMatrix, this.inertialTensor, transposeRotationMatrix);

    const worldInertialTensor = mat3.create();
    mat3.multiply(worldInertialTensor, rotationMatrix, tempMatrix);

    return worldInertialTensor;
  }

  getGyro() {
    const gyro = vec3.create();
    vec3.cross(gyro, this.angularVelocity, this.angularMomentum);

    return gyro;
  }

  /**
   * Получение мирового углового момента
   * L_world = R * L_local
   * @returns {vec3} Мировой угловой момент
   */
  getWorldAngularMomentum() {
    this.updateAngularMomentum();

    const rotationMatrix = mat3.create();

    mat3.fromQuat(rotationMatrix, this.rotation);

    const worldAngularMomentum = vec3.create();
    vec3.transformMat3(
      worldAngularMomentum,
      this.angularMomentum,
      rotationMatrix,
    );

    return worldAngularMomentum;
  }

  /**
   * Обновление локального углового момента
   * L_local = I_local * ω_local
   */
  updateAngularMomentum() {
    vec3.transformMat3(
      this.angularMomentum,
      this.angularVelocity,
      this.inertialTensor,
    );
  }

  /**
   * Отрисовка объекта
   */
  draw() {
    this.#updateWorldVertices();

    this.p5Instance.push();
    this.p5Instance.fill(this.color[0], this.color[1], this.color[2]);
    this.p5Instance.noStroke();

    for (let face of this.faceIndices) {
      this.p5Instance.beginShape();
      for (let index of face) {
        const vert = this.worldVertices[index];
        this.p5Instance.vertex(vert[0], vert[1], vert[2]);
      }
      this.p5Instance.endShape(this.p5Instance.CLOSE);
    }

    this.p5Instance.stroke(0);
    this.p5Instance.noFill();
    for (let edge of this.edgeIndices) {
      const vertex1 = this.worldVertices[edge[0]];
      const vertex2 = this.worldVertices[edge[1]];
      this.p5Instance.line(
        vertex1[0],
        vertex1[1],
        vertex1[2],
        vertex2[0],
        vertex2[1],
        vertex2[2],
      );
    }

    this.p5Instance.pop();
  }

  /**
   * Инициализация тензора инерции
   * @param {number} width - ширина
   * @param {number} height - высота
   * @param {number} depth - глубина
   * @param {number} mass - масса
   */
  #initializeInertiaTensor(width, height, depth, mass) {
    const Ixx = (mass / 12.0) * (height ** 2 + depth ** 2);
    const Iyy = (mass / 12.0) * (width ** 2 + depth ** 2);
    const Izz = (mass / 12.0) * (width ** 2 + height ** 2);

    this.inertialTensor = mat3.fromValues(
      Ixx,
      0.0,
      0.0,
      0.0,
      Iyy,
      0.0,
      0.0,
      0.0,
      Izz,
    );

    this.invertInertialTensor = mat3.create();
    mat3.invert(this.invertInertialTensor, this.inertialTensor);
  }

  /**
   * Инициализация вершин куба
   * @param {number} width - ширина
   * @param {number} height - высота
   * @param {number} depth - глубина
   */
  #initializeVertices(width, height, depth) {
    const a = width / 2.0;
    const b = height / 2.0;
    const c = depth / 2.0;

    this.localVertices = [
      vec3.fromValues(-a, -b, c),
      vec3.fromValues(a, -b, c),
      vec3.fromValues(a, b, c),
      vec3.fromValues(-a, b, c),
      vec3.fromValues(-a, -b, -c),
      vec3.fromValues(a, -b, -c),
      vec3.fromValues(a, b, -c),
      vec3.fromValues(-a, b, -c),
    ];

    this.edgeIndices = [
      [0, 1],
      [1, 2],
      [2, 3],
      [3, 0],
      [4, 7],
      [7, 6],
      [6, 5],
      [5, 4],
      [3, 7],
      [4, 0],
      [6, 2],
      [1, 5],
    ];

    this.faceIndices = [
      [0, 1, 2],
      [0, 2, 3],
      [4, 7, 6],
      [4, 6, 5],
      [3, 7, 4],
      [3, 4, 0],
      [2, 1, 5],
      [2, 5, 6],
      [3, 2, 6],
      [3, 6, 7],
      [0, 4, 5],
      [0, 5, 1],
    ];

    this.worldVertices = this.localVertices.map((vert) => vec3.clone(vert));
  }

  /**
   * Обновление мировых координат вершин
   */
  #updateWorldVertices() {
    const modelMatrix = mat4.create();

    mat4.fromRotationTranslationScale(
      modelMatrix,
      this.rotation,
      this.position,
      this.scale,
    );

    for (let vertInd = 0; vertInd < this.localVertices.length; vertInd++) {
      const localVert = this.localVertices[vertInd];

      const worldVert = vec4.fromValues(
        localVert[0],
        localVert[1],
        localVert[2],
        1.0,
      );

      vec4.transformMat4(worldVert, worldVert, modelMatrix);

      vec3.set(
        this.worldVertices[vertInd],
        worldVert[0],
        worldVert[1],
        worldVert[2],
      );
    }
  }
}
