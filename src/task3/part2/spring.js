const { mat3, mat4, vec3, quat } = glMatrix;

export default class Spring {
  anchorPos;
  attachmentPos;

  stiffness;
  damping;

  restLength;
  currentLength;

  /**
   *
   * @param {*} anchorPos
   * @param {*} stiffness
   * @param {*} damping
   * @param {*} restLength
   */
  constructor(anchorPos, stiffness, damping, restLength) {
    this.anchorPos = anchorPos
      ? vec3.clone(anchorPos)
      : vec3.fromValues(0.0, 0.0, 0.0);

    this.attachmentPos = vec3.create();

    this.stiffness = stiffness;
    this.damping = damping;

    this.restLength = restLength;
    this.currentLength = this.restLength;
  }
}
