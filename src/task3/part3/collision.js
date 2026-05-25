import SceneObject from "./scene_object.js";

const { mat3, mat4, vec3, vec4, quat } = glMatrix;

export function detectGroundCollision(objects, groundContacts) {
  groundContacts.length = 0;

  for (const object of objects) {
    const verts = object.getWVerts();

    for (const vert of verts) {
      if (vert[1] < 0.0) {
        groundContacts.push({
          object: object,
          penetration: -vert[1],
          contactVert: vec3.clone(vert),
          lambda: 0.0,
        });
      }
    }
  }
}

export function detectObjectCollision(objects, collisionContacts) {
  collisionContacts.length = 0;

  for (let i = 0; i < objects.length; i++) {
    for (let j = i + 1; j < objects.length; j++) {
      const contacts = SAT_detectObjectCollision(objects[i], objects[j]);

      for (const contact of contacts) {
        collisionContacts.push(contact);
      }
    }
  }
}

function getObjectAxis(object) {
  const rotMat = mat3.create();
  mat3.fromQuat(rotMat, object.rotation);

  return [
    vec3.fromValues(rotMat[0], rotMat[1], rotMat[2]),
    vec3.fromValues(rotMat[3], rotMat[4], rotMat[5]),
    vec3.fromValues(rotMat[6], rotMat[7], rotMat[8]),
  ];
}

function getProjOnAxis(axis, objectAxes, halfDimensions) {
  return (
    halfDimensions[0] * Math.abs(vec3.dot(axis, objectAxes[0])) +
    halfDimensions[1] * Math.abs(vec3.dot(axis, objectAxes[1])) +
    halfDimensions[2] * Math.abs(vec3.dot(axis, objectAxes[2]))
  );
}

function checkVertInside(vert, object, halfSize) {
  const localVert = vec3.create();

  const rotMat = mat3.create();
  mat3.fromQuat(rotMat, object.rotation);
  const invRotMat = mat3.create();
  mat3.invert(invRotMat, rotMat);

  vec3.sub(localVert, vert, object.position);
  vec3.transformMat3(localVert, localVert, invRotMat);

  const absLocalVert = vec3.create();
  for (let i = 0; i < 3; i++) {
    absLocalVert[i] = Math.abs(localVert[i]);
  }

  const dists = vec3.create();
  vec3.subtract(dists, halfSize, absLocalVert);

  return dists[0] >= 0 && dists[1] >= 0 && dists[2] >= 0;
}

function SAT_detectObjectCollision(object1, object2, collisionContacts) {
  const axes1 = getObjectAxis(object1);
  const halfDimensions1 = vec3.clone(object1.dimensions);
  vec3.scale(halfDimensions1, halfDimensions1, 0.5);

  const axes2 = getObjectAxis(object2);
  const halfDimensions2 = vec3.clone(object2.dimensions);
  vec3.scale(halfDimensions2, halfDimensions2, 0.5);

  const fromObj1ToObj2 = vec3.create();
  vec3.subtract(fromObj1ToObj2, object1.position, object2.position);

  let minOverlap = Infinity;
  let bestAxis = vec3.create();

  const tryAxis = (axis) => {
    const len = vec3.length(axis);
    if (len < 1e-9) {
      return true;
    }

    const currentAxis = vec3.clone(axis);
    vec3.scale(currentAxis, currentAxis, 1.0 / len);

    const projOnAxis = (axis, bodyAxes, halfSize) => {
      return (
        halfSize[0] * Math.abs(vec3.dot(axis, bodyAxes[0])) +
        halfSize[1] * Math.abs(vec3.dot(axis, bodyAxes[1])) +
        halfSize[2] * Math.abs(vec3.dot(axis, bodyAxes[2]))
      );
    };

    const projA = projOnAxis(currentAxis, axes1, halfDimensions1);
    const projB = projOnAxis(currentAxis, axes2, halfDimensions2);

    const signedDist = vec3.dot(fromObj1ToObj2, currentAxis);
    const dist = Math.abs(signedDist);
    const overlap = projA + projB - dist;
    if (overlap < 0) {
      return false;
    }

    if (overlap < minOverlap) {
      minOverlap = overlap;
      const sign = signedDist >= 0 ? 1 : -1;
      bestAxis = vec3.clone(currentAxis);
      vec3.scale(bestAxis, bestAxis, sign);
    }

    return true;
  };

  for (let i = 0; i < 3; i++) {
    if (!tryAxis(axes1[i])) {
      return [];
    }
  }

  for (let i = 0; i < 3; i++) {
    if (!tryAxis(axes2[i])) {
      return [];
    }
  }

  for (let i = 0; i < 3; i++) {
    for (let j = 0; j < 3; j++) {
      const axis = vec3.create();
      vec3.cross(axis, axes1[i], axes2[j]);
      if (!tryAxis(axis)) {
        return [];
      }
    }
  }

  let contacts = [];

  const verts1 = object1.getWVerts();
  const verts2 = object2.getWVerts();

  const normal1 = vec3.clone(bestAxis);
  const normal2 = vec3.create();
  vec3.scale(normal2, normal1, -1);

  for (const vert1 of verts1) {
    if (checkVertInside(vert1, object2, halfDimensions2)) {
      const vert2 = vec3.create();
      vec3.scaleAndAdd(vert2, vert1, normal1, minOverlap);

      contacts.push({
        object1: object1,
        object2: object2,
        contactVert1: vec3.clone(vert1),
        contactVert2: vec3.clone(vert2),
        normal: vec3.clone(normal2),
        penetration: minOverlap,
        lambda: 0,
      });
    }
  }

  for (const vert of verts2) {
    if (checkVertInside(vert, object1, halfDimensions1)) {
      const vertContact2 = vec3.create();
      vec3.scaleAndAdd(vertContact2, vert, normal2, minOverlap);

      contacts.push({
        object1: object2,
        object2: object1,
        contactVert1: vec3.clone(vert),
        contactVert2: vec3.clone(vertContact2),
        normal: vec3.clone(normal1),
        penetration: minOverlap,
        lambda: 0,
      });
    }
  }

  if (contacts.length === 0) {
    const firstContact = vec3.clone(normal2);
    vec3.scale(firstContact, firstContact, halfDimensions1[0]);
    vec3.transformMat4(firstContact, firstContact, object1.getModelMatrix());

    const secondContact = vec3.clone(normal1);
    vec3.scale(secondContact, secondContact, halfDimensions2[0]);
    vec3.transformMat4(secondContact, secondContact, object2.getModelMatrix());

    contacts.push({
      object1: object1,
      object2: object2,
      contactVert1: vec3.clone(firstContact),
      contactVert2: vec3.clone(secondContact),
      normal: vec3.clone(normal2),
      penetration: minOverlap,
      lambda: 0,
    });
  }

  return contacts;
}

export function detectObjectCollisionSpatialGrid(objects, collisionContacts) {
  collisionContacts.length = 0;
  if (objects.length < 2) return;

  const min = vec3.create();
  const max = vec3.create();
  const first = objects[0];
  const halfFirst = vec3.create();
  vec3.scale(halfFirst, first.dimensions, 0.5);
  vec3.sub(min, first.position, halfFirst);
  vec3.add(max, first.position, halfFirst);

  for (let i = 1; i < objects.length; i++) {
    const obj = objects[i];
    const half = vec3.scale(vec3.create(), obj.dimensions, 0.5);
    const objMin = vec3.sub(vec3.create(), obj.position, half);
    const objMax = vec3.add(vec3.create(), obj.position, half);
    vec3.min(min, min, objMin);
    vec3.max(max, max, objMax);
  }

  let maxExtent = 0;
  for (const obj of objects) {
    const extent = Math.max(
      obj.dimensions[0],
      obj.dimensions[1],
      obj.dimensions[2],
    );
    if (extent > maxExtent) maxExtent = extent;
  }
  const cellSize = maxExtent;
  if (cellSize <= 0) return;

  const grid = new Map();

  for (let i = 0; i < objects.length; i++) {
    const obj = objects[i];
    const half = vec3.scale(vec3.create(), obj.dimensions, 0.5);
    const objMin = vec3.sub(vec3.create(), obj.position, half);
    const objMax = vec3.add(vec3.create(), obj.position, half);

    const ixMin = Math.floor((objMin[0] - min[0]) / cellSize);
    const iyMin = Math.floor((objMin[1] - min[1]) / cellSize);
    const izMin = Math.floor((objMin[2] - min[2]) / cellSize);
    const ixMax = Math.floor((objMax[0] - min[0]) / cellSize);
    const iyMax = Math.floor((objMax[1] - min[1]) / cellSize);
    const izMax = Math.floor((objMax[2] - min[2]) / cellSize);

    for (let ix = ixMin; ix <= ixMax; ix++) {
      for (let iy = iyMin; iy <= iyMax; iy++) {
        for (let iz = izMin; iz <= izMax; iz++) {
          const key = `${ix},${iy},${iz}`;
          let cell = grid.get(key);
          if (!cell) {
            cell = [];
            grid.set(key, cell);
          }
          cell.push(i);
        }
      }
    }
  }

  const processedPairs = new Set();

  for (const cell of grid.values()) {
    if (cell.length < 2) continue;
    for (let a = 0; a < cell.length; a++) {
      const idxA = cell[a];
      for (let b = a + 1; b < cell.length; b++) {
        const idxB = cell[b];
        const pairKey = idxA < idxB ? `${idxA},${idxB}` : `${idxB},${idxA}`;
        if (processedPairs.has(pairKey)) continue;
        processedPairs.add(pairKey);

        const contacts = SAT_detectObjectCollision(
          objects[idxA],
          objects[idxB],
        );
        for (const contact of contacts) {
          collisionContacts.push(contact);
        }
      }
    }
  }
}
