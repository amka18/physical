const { mat4, vec3, vec4, quat } = glMatrix;

export default class GameObject {
  #position;
  #rotation;
  #scale;

  #prevPosition;
  #velocity;
  #prevRotation;
  #angularVelocity;

  #mass;
  #inverseMatrix;
  #inertialTensor;
  #inertialWorldTensor;

  #localVertices;
  #edgesIndices;
  #facesIndices;

  #modelMatrix;
  #worldVertices;

  #color;

  #p5Instance;

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
    this.#position = position ? vec3.clone(position) : vec3.create();

    this.#rotation = quat.create();
    quat.fromEuler(this.#rotation, rotation[0], rotation[1], rotation[2]);

    this.#scale = scale ? vec3.clone(scale) : vec3.fromValues(1.0, 1.0, 1.0);

    this.#velocity = velocity ? vec3.clone(velocity) : vec3.create();

    this.#angularVelocity = angularVelocity
      ? vec3.clone(angularVelocity)
      : vec3.create();

    this.#mass = mass;

    this.#prevPosition = vec3.clone(this.#position);
    this.#prevRotation = quat.clone(this.#rotation);

    this.#inverseMatrix = this.#mass > 0 ? 1.0 / this.#mass : 0;

    const a = dimensions[0] / 2.0;
    const b = dimensions[1] / 2.0;
    const c = dimensions[2] / 2.0;

    this.#localVertices = [
      vec3.fromValues(-a, -b, c),
      vec3.fromValues(a, -b, c),
      vec3.fromValues(a, b, c),
      vec3.fromValues(-a, b, c),
      vec3.fromValues(-a, -b, -c),
      vec3.fromValues(a, -b, -c),
      vec3.fromValues(a, b, -c),
      vec3.fromValues(-a, b, -c),
    ];

    this.#edgesIndices = [
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

    this.#facesIndices = [
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

    this.#worldVertices = this.#localVertices.map((vert) => vec3.clone(vert));
    this.#modelMatrix = mat4.create();
    this.#updateWorldVertices();

    this.#color = color
      ? vec3.clone(color)
      : vec3.fromValues(200.0, 200.0, 200.0);

    this.#p5Instance = p5Instance;
  }

  draw() {
    this.#updateWorldVertices();

    this.#p5Instance.push();
    this.#p5Instance.fill(this.#color[0], this.#color[1], this.#color[2]);
    this.#p5Instance.noStroke();

    for (let face of this.#facesIndices) {
      this.#p5Instance.beginShape();
      for (let index of face) {
        const vert = this.#worldVertices[index];
        this.#p5Instance.vertex(vert[0], vert[1], vert[2]);
      }
      this.#p5Instance.endShape(this.#p5Instance.CLOSE);
    }

    this.#p5Instance.stroke(0);
    this.#p5Instance.noFill();
    for (let edge of this.#edgesIndices) {
      const vertex1 = this.#worldVertices[edge[0]];
      const vertex2 = this.#worldVertices[edge[1]];
      this.#p5Instance.line(
        vertex1[0],
        vertex1[1],
        vertex1[2],
        vertex2[0],
        vertex2[1],
        vertex2[2],
      );
    }

    this.#p5Instance.pop();
  }

  #updateWorldVertices() {
    mat4.fromRotationTranslationScale(
      this.#modelMatrix,
      this.#rotation,
      this.#position,
      this.#scale,
    );

    for (let vertInd = 0; vertInd < this.#localVertices.length; vertInd++) {
      const localVert = this.#localVertices[vertInd];

      const worldVert = vec4.fromValues(
        localVert[0],
        localVert[1],
        localVert[2],
        1.0,
      );

      vec4.transformMat4(worldVert, worldVert, this.#modelMatrix);

      vec3.set(
        this.#worldVertices[vertInd],
        worldVert[0],
        worldVert[1],
        worldVert[2],
      );
    }
  }
}
