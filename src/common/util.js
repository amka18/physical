const { vec3, quat } = glMatrix;

export function IntegrateQuat(rotation, angularVelocity, dt) {
  const wQuat = quat.fromValues(
    angularVelocity[0],
    angularVelocity[1],
    angularVelocity[2],
    0.0,
  );

  const dQuat = quat.create();
  quat.multiply(dQuat, wQuat, rotation);
  quat.scale(dQuat, dQuat, 0.5 * dt);

  return dQuat;
}
