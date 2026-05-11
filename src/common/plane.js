const { vec3 } = glMatrix;

export default class Plane {
  worldVertices;

  color;

  p5Instance;

  constructor(size, color, p5Instance) {
    const halfSize = size / 2.0;

    this.worldVertices = [
      vec3.fromValues(-halfSize, 0, halfSize),
      vec3.fromValues(halfSize, 0, halfSize),
      vec3.fromValues(halfSize, 0, -halfSize),
      vec3.fromValues(-halfSize, 0, -halfSize),
    ];

    this.color = color ? color : vec3.fromValues(100.0, 200.0, 100.0);

    this.p5Instance = p5Instance;
  }

  draw() {
    this.p5Instance.push();
    this.p5Instance.fill(this.color[0], this.color[1], this.color[2]);
    this.p5Instance.noStroke();

    this.p5Instance.beginShape(this.p5Instance.QUADS);
    for (let vertex of this.worldVertices) {
      this.p5Instance.vertex(vertex[0], vertex[1], vertex[2]);
    }
    this.p5Instance.endShape(this.p5Instance.CLOSE);

    this.p5Instance.pop();
  }
}
