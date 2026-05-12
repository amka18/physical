const { vec3, quat } = glMatrix;

/**
 *
 * @param {*} rotation
 * @param {*} globalAngularVelocity
 * @param {*} dt
 */
export function IntegrateQuatGlobal(rotation, globalAngularVelocity, dt) {
  const wQuat = quat.fromValues(
    globalAngularVelocity[0],
    globalAngularVelocity[1],
    globalAngularVelocity[2],
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
 * @param {*} rotation
 * @param {*} globalAngularVelocity
 * @param {*} dt
 */
export function IntegrateQuatLocal(rotation, localAngularVelocity, dt) {
  const wQuat = quat.fromValues(
    localAngularVelocity[0],
    localAngularVelocity[1],
    localAngularVelocity[2],
    0.0,
  );

  const dQuat = quat.create();
  quat.multiply(dQuat, rotation, wQuat);
  quat.scale(dQuat, dQuat, 0.5 * dt);

  quat.add(rotation, rotation, dQuat);
  quat.normalize(rotation, rotation);
}
