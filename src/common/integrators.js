const { vec3, quat } = glMatrix;

/**
 *
 * @param {*} nextRotation
 * @param {*} globalAngularVelocity
 * @param {*} dt
 */
export function IntegrateQuatGlobal(rotation, angularVelocity, dt) {
  const wQuat = quat.fromValues(
    angularVelocity[0],
    angularVelocity[1],
    angularVelocity[2],
    0.0,
  );

  const dQuat = quat.create();
  quat.multiply(dQuat, wQuat, rotation);
  quat.scale(dQuat, dQuat, 0.5 * dt);

  quat.add(rotation, rotation, dQuat);
  quat.normalize(rotation, rotation);
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
  // quat.multiply(dQuat, nextRotation, wQuat);
  quat.multiply(dQuat, nextRotation, wQuat);
  quat.scale(dQuat, dQuat, 0.5 * dt);

  quat.add(nextRotation, nextRotation, dQuat);
  quat.normalize(nextRotation, nextRotation);
}
