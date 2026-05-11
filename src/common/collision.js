import Box from "./box.js";
import GameObject from "./simulation_object.js";

const { mat4, vec3, quat } = glMatrix;

function SATCollision(object1, object2) {}

export function GroundConstraint(object) {
  let offset = 0.0;
  let interact = false;

  for (let vertex of object.worldVertices) {
    if (vertex[1] > 0.0) {
      if (vertex[1] > offset) {
        offset = vertex[1];
        interact = true;
      }
    }
  }

  if (interact) {
    object.position[1] -= offset;
  }
}
