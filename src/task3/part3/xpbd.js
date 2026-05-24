import SceneObject from "./scene_object.js";
import { updateQuatGlobal, updateQuatLocal } from "./utils.js";

const { mat3, mat4, vec3, vec4, quat } = glMatrix;

export function XPBD_PredictPositions(object, h) {
  vec3.copy(object.prevPosition, object.position);

  const g = 0.0001;
  object.velocity[0] = object.velocity[0];
  object.velocity[1] = object.velocity[1] - g * h;
  object.velocity[2] = object.velocity[2];

  object.position[0] = object.position[0] + object.velocity[0] * h;
  object.position[1] = object.position[1] + object.velocity[1] * h;
  object.position[2] = object.position[2] + object.velocity[2] * h;

  quat.copy(object.prevRotation, object.rotation);

  object.angularVelocity[0] = object.angularVelocity[0];
  object.angularVelocity[1] = object.angularVelocity[1];
  object.angularVelocity[2] = object.angularVelocity[2];

  const wquat = quat.create();
  quat.set(
    wquat,
    object.angularVelocity[0],
    object.angularVelocity[1],
    object.angularVelocity[2],
    0.0,
  );

  const dq = quat.create();
  quat.multiply(dq, wquat, object.rotation);

  object.rotation[0] = object.rotation[0] + 0.5 * dq[0] * h;
  object.rotation[1] = object.rotation[1] + 0.5 * dq[1] * h;
  object.rotation[2] = object.rotation[2] + 0.5 * dq[2] * h;
  object.rotation[3] = object.rotation[3] + 0.5 * dq[3] * h;

  quat.normalize(object.rotation, object.rotation);
}

export function XPBD_UpdateVelocities(object, h) {
  object.velocity[0] = (object.position[0] - object.prevPosition[0]) / h;
  object.velocity[1] = (object.position[1] - object.prevPosition[1]) / h;
  object.velocity[2] = (object.position[2] - object.prevPosition[2]) / h;

  const dq = quat.create();
  const prevInvQuat = quat.create();
  quat.conjugate(prevInvQuat, object.prevRotation);
  quat.multiply(dq, object.rotation, prevInvQuat);

  const ws = dq[3] >= 0 ? 1 : -1;
  const w = vec3.create();
  vec3.set(
    w,
    ((2 * dq[0]) / h) * ws,
    ((2 * dq[1]) / h) * ws,
    ((2 * dq[2]) / h) * ws,
  );

  vec3.copy(object.angularVelocity, w);
}

export function XPBD_SolveGroundCollision(
  object,
  contactVert,
  penetration,
  lambda,
  h,
) {
  const n = vec3.fromValues(0.0, 1.0, 0.0);

  const wInvI = object.getWInvI();

  const r = vec3.create();
  vec3.sub(r, contactVert, object.position);

  const w = XPBD_GetInvEffMass(object.mass, r, n, wInvI);

  const alpha = 0 / h ** 2;

  const dLambda = (penetration - alpha * lambda) / (w + alpha);

  const p = vec3.create();
  vec3.scale(p, n, dLambda);

  XPBD_CorrectPosition(object, p);
  XPBD_CorrectRotation(object, r, p, wInvI);
}

export function XPBD_SolveObjectsCollision(
  object1,
  object2,
  contactVert1,
  contactVert2,
  normal,
  penetration,
  lambda,
  h,
) {
  const n = vec3.create();
  vec3.normalize(n, normal);

  const wInvI1 = object1.getWInvI();
  const wInvI2 = object2.getWInvI();

  const r1 = vec3.create();
  vec3.sub(r1, contactVert1, object1.position);

  const r2 = vec3.create();
  vec3.sub(r2, contactVert2, object2.position);

  const w1 = XPBD_GetInvEffMass(object1.mass, r1, n, wInvI1);
  const w2 = XPBD_GetInvEffMass(object2.mass, r2, n, wInvI2);

  const alpha = 0 / h ** 2;

  const dLambda = (penetration - alpha * lambda) / (w1 + w2 + alpha);

  const p = vec3.create();
  vec3.scale(p, n, dLambda);

  const p_ = vec3.create();
  vec3.scale(p_, p, -1);

  XPBD_CorrectPosition(object1, p_);
  XPBD_CorrectPosition(object2, p);

  XPBD_CorrectRotation(object1, r1, p_, wInvI1);
  XPBD_CorrectRotation(object2, r2, p, wInvI2);
}

function XPBD_GetInvEffMass(m, r, n, wInvI) {
  const crossRN = vec3.create();
  vec3.cross(crossRN, r, n);

  const temp = mat3.create();
  vec3.transformMat3(temp, crossRN, wInvI);

  const invEffMass = 1 / m + vec3.dot(temp, crossRN);
  return invEffMass;
}

function XPBD_CorrectPosition(object, p) {
  object.position[0] = object.position[0] + p[0] / object.mass;
  object.position[1] = object.position[1] + p[1] / object.mass;
  object.position[2] = object.position[2] + p[2] / object.mass;
}

function XPBD_CorrectRotation(object, r, p, wInvI) {
  const crossRP = vec3.create();
  vec3.cross(crossRP, r, p);
  const temp = vec3.create();
  vec3.transformMat3(temp, crossRP, wInvI);
  const q = quat.create();
  quat.set(q, temp[0], temp[1], temp[2], 0.0);
  const dq = quat.create();
  quat.multiply(dq, q, object.rotation);

  object.rotation[0] = object.rotation[0] + 0.5 * dq[0];
  object.rotation[1] = object.rotation[1] + 0.5 * dq[1];
  object.rotation[2] = object.rotation[2] + 0.5 * dq[2];
  object.rotation[3] = object.rotation[3] + 0.5 * dq[3];

  quat.normalize(object.rotation, object.rotation);
}
