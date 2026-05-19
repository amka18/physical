const { mat3, mat4, vec3, vec4, quat } = glMatrix;

export default class SceneObject {
  position;
  prevPosition;

  rotation;
  prevRotation;

  scale;

  velocity;
  angularVelocity;

  inertialTensor;

  mass;

  attachmentPoint;

  worldVertices;

  localVertices;
  edgeIndices;
  faceIndices;

  color;

  p5Instance;

  /**
   *
   * @param {*} dimensions
   * @param {*} position
   * @param {*} rotation
   * @param {*} velocity
   * @param {*} angularVelocity
   * @param {*} attachmentPoint
   * @param {*} mass
   * @param {*} color
   * @param {*} p5Instance
   */
  constructor(
    dimensions,
    position,
    rotation,
    velocity,
    angularVelocity,
    attachmentPoint,
    mass,
    color,
    p5Instance,
  ) {
    this.position = position ? vec3.clone(position) : vec3.create();
    this.prevPosition = vec3.clone(this.position);

    this.rotation = quat.fromValues(0.0, 0.0, 0.0, 1.0);
    if (rotation) {
      quat.fromEuler(this.rotation, rotation[0], rotation[1], rotation[2]);
    }
    this.prevRotation = quat.clone(this.rotation);

    this.scale = vec3.fromValues(1.0, 1.0, 1.0);

    this.velocity = velocity
      ? vec3.clone(velocity)
      : vec3.fromValues(0.0, 0.0, 0.0);

    this.angularVelocity = angularVelocity
      ? vec3.clone(angularVelocity)
      : vec3.fromValues(0.0, 0.0, 0.0);

    this.attachmentPoint = attachmentPoint
      ? vec3.clone(attachmentPoint)
      : vec3.fromValues(0.0, 0.0, 0.0);

    this.mass = mass;

    this.#initializeInertialTensor(
      dimensions[0],
      dimensions[1],
      dimensions[2],
      mass,
    );

    this.#initializeVertices(dimensions[0], dimensions[1], dimensions[2]);

    this.color = color
      ? vec3.clone(color)
      : vec3.fromValues(200.0, 200.0, 200.0);

    this.p5Instance = p5Instance;
  }

  getWInvI() {
    const invI = mat3.create();
    mat3.set(
      invI,
      1 / this.inertialTensor[0],
      0.0,
      0.0,
      0.0,
      1 / this.inertialTensor[4],
      0.0,
      0.0,
      0.0,
      1 / this.inertialTensor[8],
    );
    const rotMat = mat3.create();
    mat3.fromQuat(rotMat, this.rotation);
    const tRotMat = mat3.create();
    mat3.transpose(tRotMat, rotMat);
    const tempMat = mat3.create();
    mat3.multiply(tempMat, rotMat, invI);
    const wInvI = mat3.create();
    mat3.multiply(wInvI, tempMat, tRotMat);

    return wInvI;
  }

  getGero() {
    const gero = vec3.create();

    const L = vec3.create();
    // const Iw = this.getWorldInertialTensor();
    // vec3.transformMat3(angularMomentum, this.angularVelocity, Iw);
    // vec3.cross(gero, this.angularVelocity, angularMomentum);

    return gero;
  }

  getWorldAttachmentPoint() {
    const lr = vec3.clone(this.attachmentPoint);

    const rotMat = mat3.create();
    mat3.fromQuat(rotMat, this.rotation);

    const wr = vec3.create();
    vec3.transformMat3(wr, lr, rotMat);

    wr[0] = wr[0] + this.position[0];
    wr[1] = wr[1] + this.position[1];
    wr[2] = wr[2] + this.position[2];

    return wr;
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
  #initializeInertialTensor(width, height, depth, mass) {
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
