const { vec3, quat } = glMatrix;

/**
 *
 * @param {*} nextRotation
 * @param {*} globalAngularVelocity
 * @param {*} dt
 */
export function IntegrateQuatGlobal(nextRotation, globalAngularVelocity, dt) {
  const wQuat = quat.fromValues(
    globalAngularVelocity[0],
    globalAngularVelocity[1],
    globalAngularVelocity[2],
    0.0,
  );

  const dQuat = quat.create();
  quat.multiply(dQuat, wQuat, nextRotation);
  quat.scale(dQuat, dQuat, 0.5 * dt);

  quat.add(nextRotation, nextRotation, dQuat);
  quat.normalize(nextRotation, nextRotation);
}

/**
 *
 * @param {*} nextRotation
 * @param {*} globalAngularVelocity
 * @param {*} dt
 */
export function IntegrateQuatLocal(nextRotation, localAngularVelocity, dt) {
  const wQuat = quat.fromValues(
    localAngularVelocity[0],
    localAngularVelocity[1],
    localAngularVelocity[2],
    0.0,
  );

  const dQuat = quat.create();
  quat.multiply(dQuat, nextRotation, wQuat);
  quat.scale(dQuat, dQuat, 0.5 * dt);

  quat.add(nextRotation, nextRotation, dQuat);
  quat.normalize(nextRotation, nextRotation);
}
