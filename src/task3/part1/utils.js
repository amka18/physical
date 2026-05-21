const { mat3, mat4, vec3, quat } = glMatrix;

export function updateQuatGlobal(q, w, dt) {
  const wquat = quat.create();
  quat.set(wquat, w[0], w[1], w[2], 0.0);

  const dq = quat.create();
  quat.multiply(dq, wquat, q);

  q[0] = q[0] + 0.5 * dq[0] * dt;
  q[1] = q[1] + 0.5 * dq[1] * dt;
  q[2] = q[2] + 0.5 * dq[2] * dt;
  q[3] = q[3] + 0.5 * dq[3] * dt;

  quat.normalize(q, q);
}

export function updateQuatLocal(q, w, dt) {
  const wquat = quat.create();
  quat.set(wquat, w[0], w[1], w[2], 0.0);

  const dq = quat.create();
  quat.multiply(dq, q, wquat);

  q[0] = q[0] + 0.5 * dq[0] * dt;
  q[1] = q[1] + 0.5 * dq[1] * dt;
  q[2] = q[2] + 0.5 * dq[2] * dt;
  q[3] = q[3] + 0.5 * dq[3] * dt;

  quat.normalize(q, q);
}

export function calculateRotationEnergy(I, w) {
  const L = vec3.create();
  vec3.transformMat3(L, w, I);
  return vec3.dot(L, w) * 0.5;
}
