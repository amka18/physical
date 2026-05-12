const { vec3, quat, mat4 } = glMatrix;

/**
 * Форматирует вектор для вывода с заданным количеством знаков после запятой
 * @param {vec3} vector
 * @param {number} decimals
 * @returns {string}
 */
function FormatVector(vector, decimals) {
  //   return `[${vector.map((v) => v.toFixed(decimals)).join(", ")}]`;
  return `${vector[0].toFixed(decimals)} ${vector[1].toFixed(decimals)} ${vector[2].toFixed(decimals)}`;
}

/**
 *
 * @param {*} prompt
 * @param {*} vector
 * @param {*} decimals
 * @param {*} position
 * @param {*} color
 * @param {*} p5Instance
 */
export function OutputVector(
  prompt,
  vector,
  decimals,
  position,
  color,
  p5Instance,
) {
  const formattedVector = FormatVector(vector, decimals);

  p5Instance.push();

  p5Instance.resetMatrix();

  p5Instance.fill(color[0], color[1], color[2]);
  p5Instance.text(
    `${prompt}: ${formattedVector}`,
    position[0],
    position[1],
    position[2],
  );

  p5Instance.pop();
}

/**
 *
 * @param {*} prompt
 * @param {*} value
 * @param {*} decimals
 * @param {*} position
 * @param {*} color
 * @param {*} p5Instance
 */
export function OutputValue(
  prompt,
  value,
  decimals,
  size,
  position,
  color,
  p5Instance,
) {
  p5Instance.push();

  // p5Instance.resetMatrix();

  // p5Instance.translate(-p5Instance.width / 2, -p5Instance.height / 2);

  p5Instance.textSize(size);

  p5Instance.fill(color[0], color[1], color[2]);
  p5Instance.text(
    `${prompt}: ${value.toFixed(decimals)}`,
    position[0],
    position[1],
  );

  p5Instance.pop();
}

/**
 *
 * @param {*} direction
 * @param {*} position
 * @param {*} length
 * @param {*} color
 * @param {*} p5Instance
 */
export function DrawLine(direction, position, length, color, p5Instance) {
  const normalizeDirection = vec3.create();
  vec3.normalize(normalizeDirection, direction);

  const point2 = vec3.create();
  vec3.scaleAndAdd(point2, position, normalizeDirection, length);

  p5Instance.push();

  p5Instance.stroke(color[0], color[1], color[2]);
  p5Instance.strokeWeight(2);
  p5Instance.line(
    position[0],
    position[1],
    position[2],
    point2[0],
    point2[1],
    point2[2],
  );

  p5Instance.pop();
}

/**
 *
 * @param {*} position
 * @param {*} orientation
 * @param {*} length
 * @param {*} p5Instance
 */
export function DrawAxes(position, orientation, length, p5Instance) {
  const axes = [
    { dir: [1, 0, 0], color: [255, 0, 0], name: "X" },
    { dir: [0, 1, 0], color: [0, 255, 0], name: "Y" },
    { dir: [0, 0, 1], color: [0, 0, 255], name: "Z" },
  ];

  p5Instance.push();

  p5Instance.strokeWeight(2);

  axes.forEach((axis) => {
    const rotatedDirection = vec3.create();
    vec3.transformQuat(rotatedDirection, axis.dir, orientation);

    const end = vec3.create();
    vec3.scaleAndAdd(end, position, rotatedDirection, length);

    p5Instance.stroke(axis.color[0], axis.color[1], axis.color[2]);
    p5Instance.line(
      position[0],
      position[1],
      position[2],
      end[0],
      end[1],
      end[2],
    );
  });

  p5Instance.pop();
}
