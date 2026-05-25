import SceneObject from "./scene_object.js";

const { mat3, mat4, vec3, vec4, quat } = glMatrix;

export function SI_PredictVelocities(object, h) {
  vec3.copy(object.prevVelocity, object.velocity);

  const g = 0.0001;

  object.velocity[0] = object.velocity[0];
  object.velocity[1] = object.velocity[1] - g * h;
  object.velocity[2] = object.velocity[2];

  vec3.copy(object.prevAngularVelocity, object.angularVelocity);

  const rotMat = mat3.create();
  mat3.fromQuat(rotMat, object.rotation);
  const invRotMat = mat3.create();
  mat3.invert(invRotMat, rotMat);

  const wl = vec3.create();
  vec3.transformMat3(wl, object.angularVelocity, invRotMat);

  const dwl = computeGeroDw(
    object.inertialTensor,
    object.invInertialTensor,
    wl,
    h,
  );
  const dw = vec3.create();
  vec3.transformMat3(dw, dwl, rotMat);

  object.angularVelocity[0] = object.angularVelocity[0] + dw[0];
  object.angularVelocity[0] = object.angularVelocity[1] + dw[1];
  object.angularVelocity[0] = object.angularVelocity[2] + dw[2];
}

export function SI_SolveGroundCollision(
  object,
  contactVert,
  penetration,
  lambda,
  h,
) {
  const n = vec3.fromValues(0.0, 1.0, 0.0);

  const r = vec3.create();
  vec3.sub(r, contactVert, object.position);
  const crossRN = vec3.create();
  vec3.cross(crossRN, r, n);

  const c = -penetration;

  const dc =
    vec3.dot(object.velocity, n) + vec3.dot(object.angularVelocity, crossRN);

  const wInvI = object.getWInvI();
  const w = SI_GetInvEffMass(object.mass, r, n, wInvI);

  const b = 0.3;

  const dp = (-dc - (b * c) / h) / w;

  const p = vec3.create();
  vec3.scale(p, n, dp);

  SI_CorrectVelocity(object, p);
  SI_CorrectAngulatVelocity(object, r, p);
}

export function SI_SolveObjectsCollision(
  object1,
  object2,
  contactVert1,
  contactVert2,
  normal,
  penetration,
  lambda,
  h,
) {
  const d = vec3.create();
  vec3.sub(d, contactVert2, contactVert1);

  const n = vec3.create();
  vec3.normalize(n, d);

  const r1 = vec3.create();
  vec3.sub(r1, contactVert1, object1.position);
  const crossR1N = vec3.create();
  vec3.cross(crossR1N, r1, n);

  const temp1 = vec3.create();
  vec3.scale(temp1, crossR1N, -1);

  const r2 = vec3.create();
  vec3.sub(r2, contactVert2, object2.position);
  const crossR2N = vec3.create();
  vec3.cross(crossR2N, r2, n);

  const temp2 = vec3.create();
  vec3.scale(temp2, n, -1);

  const c = penetration;

  const dc =
    vec3.dot(temp2, object1.velocity) +
    vec3.dot(temp1, object1.angularVelocity) +
    vec3.dot(n, object2.velocity) +
    vec3.dot(crossR2N, object2.angularVelocity);

  const wInvI1 = object1.getWInvI();
  const w1 = SI_GetInvEffMass(object1.mass, r1, n, wInvI1);

  const wInvI2 = object2.getWInvI();
  const w2 = SI_GetInvEffMass(object2.mass, r2, n, wInvI2);

  const b = 0.7;

  const dLambda = (-dc - (b * c) / h) / (w1 + w2);

  const p = vec3.create();
  vec3.scale(p, n, dLambda);
  const p_ = vec3.create();
  vec3.scale(p_, p, -1);

  SI_CorrectVelocity(object1, p_);
  SI_CorrectAngulatVelocity(object1, r1, p_);
  SI_CorrectVelocity(object2, p);
  SI_CorrectAngulatVelocity(object2, r2, p);
}

export function SI_UpdatePositionAndRotation(object, h) {
  object.position[0] = object.position[0] + object.velocity[0] * h;
  object.position[1] = object.position[1] + object.velocity[1] * h;
  object.position[2] = object.position[2] + object.velocity[2] * h;

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

  quat.normalize(object.rotation, object.rotation);
}

function SI_CorrectVelocity(object, p) {
  object.velocity[0] = object.velocity[0] + p[0] / object.mass;
  object.velocity[1] = object.velocity[1] + p[1] / object.mass;
  object.velocity[2] = object.velocity[2] + p[2] / object.mass;
}

function SI_CorrectAngulatVelocity(object, r, p) {
  const crossRP = vec3.create();
  vec3.cross(crossRP, r, p);

  const dw = vec3.create();
  vec3.transformMat3(dw, crossRP, object.getWInvI());
  object.angularVelocity[0] += dw[0];
  object.angularVelocity[1] += dw[1];
  object.angularVelocity[2] += dw[2];
}

function SI_GetInvEffMass(m, r, n, wInvI) {
  const crossRN = vec3.create();
  vec3.cross(crossRN, r, n);

  const temp = mat3.create();
  vec3.transformMat3(temp, crossRN, wInvI);

  const invEffMass = 1 / m + vec3.dot(temp, crossRN);
  return invEffMass;
}

function computeGeroDw(I, invI, w, h) {
  const L = getAngularMomentum(I, w);

  const LSkewMat = getSkewMatrix(L);
  const wSkewMat = getSkewMatrix(w);

  const wIMat = mat3.create();
  mat3.multiply(wIMat, wSkewMat, I);
  const invILMat = mat3.create();
  mat3.multiply(invILMat, invI, LSkewMat);
  const invIwIMat = mat3.create();
  mat3.multiply(invIwIMat, invI, wIMat);

  const G = mat3.create();
  mat3.sub(G, invILMat, invIwIMat);

  const g = vec3.create();
  vec3.transformMat3(g, w, invILMat);

  const E = mat3.create();
  mat3.identity(E);
  const tempMat1 = mat3.create();
  mat3.multiplyScalar(tempMat1, G, h);
  const tempMat2 = mat3.create();
  mat3.sub(tempMat2, E, tempMat1);
  const tempMat3 = mat3.create();
  mat3.invert(tempMat3, tempMat2);

  const tempVec1 = vec3.create();
  vec3.scale(tempVec1, g, h);

  const dw = vec3.create();
  vec3.transformMat3(dw, tempVec1, tempMat3);

  return dw;
}

function getAngularMomentum(I, w) {
  const L = vec3.create();
  vec3.transformMat3(L, w, I);
  return L;
}

function getSkewMatrix(vector) {
  const matrix = mat3.create();

  matrix[0] = 0;
  matrix[1] = -vector[2];
  matrix[2] = vector[1];
  matrix[3] = vector[2];
  matrix[4] = 0;
  matrix[5] = -vector[0];
  matrix[6] = -vector[1];
  matrix[7] = vector[0];
  matrix[8] = 0;

  return matrix;
}
