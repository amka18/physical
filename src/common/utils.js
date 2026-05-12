const { mat3, mat4, vec3, quat } = glMatrix;

export function GetLength(point1, point2) {
  const temp = vec3.create();
  vec3.sub(temp, point1, point2);

  const length = vec3.length(temp);

  return length;
}
