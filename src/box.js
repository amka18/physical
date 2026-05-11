const { vec3 } = glMatrix;

export default class Box {
  worldVertices;
  edgeIndices;
  bounds;

  p5Instance;

  constructor(dimensions, p5Instance) {
    const a = dimensions[0] / 2.0;
    const b = dimensions[1] / 2.0;
    const c = dimensions[2] / 2.0;

    this.worldVertices = [
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

    this.bounds = [-a, a, -b, b, -c, c];

    this.p5Instance = p5Instance;
  }

  draw() {
    this.p5Instance.push();

    this.p5Instance.stroke(0);
    this.p5Instance.strokeWeight(2);
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
}
