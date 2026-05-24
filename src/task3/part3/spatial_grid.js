// Класс пространственной сетки
class SpatialGrid {
  constructor(cellSize) {
    this.cellSize = cellSize;
    this.cells = new Map();
  }

  clear() {
    this.cells.clear();
  }

  insert(index, aabb) {
    const { minX, minY, maxX, maxY } = aabb;
    const startCol = Math.floor(minX / this.cellSize);
    const endCol   = Math.floor(maxX / this.cellSize);
    const startRow = Math.floor(minY / this.cellSize);
    const endRow   = Math.floor(maxY / this.cellSize);

    for (let row = startRow; row <= endRow; row++) {
      for (let col = startCol; col <= endCol; col++) {
        const key = `${col},${row}`;
        if (!this.cells.has(key)) {
          this.cells.set(key, []);
        }
        this.cells.get(key).push(index);
      }
    }
  }

  // Собрать все уникальные пары кандидатов (индексы объектов)
  getCandidatePairs() {
    const pairSet = new Set();

    for (const indices of this.cells.values()) {
      // Перебор всех пар внутри одной ячейки
      for (let i = 0; i < indices.length; i++) {
        for (let j = i + 1; j < indices.length; j++) {
          const a = indices[i];
          const b = indices[j];
          // Каноническое представление пары (меньший индекс первый)
          const key = a < b ? `${a},${b}` : `${b},${a}`;
          pairSet.add(key);
        }
      }
    }

    // Преобразуем Set<string> в массив [i, j]
    return Array.from(pairSet).map(str => str.split(',').map(Number));
  }
}

// Основная функция обнаружения коллизий с использованием сетки
export function detectObjectCollision(objects, collisionContacts) {
  collisionContacts.length = 0;

  if (objects.length < 2) return;

  // Параметры сетки – можно подбирать под конкретную сцену
  // Здесь cellSize вычисляется как средний размер объекта,
  // но можно задать константу, например 100.
  let totalWidth = 0;
  let totalHeight = 0;
  for (const obj of objects) {
    // Предполагаем наличие width/height; если нет – адаптируйте
    totalWidth += obj.width || 0;
    totalHeight += obj.height || 0;
  }
  const avgWidth = totalWidth / objects.length;
  const avgHeight = totalHeight / objects.length;
  const cellSize = Math.max(avgWidth, avgHeight, 1); // минимум 1

  const grid = new SpatialGrid(cellSize);

  // Заполняем сетку
  for (let i = 0; i < objects.length; i++) {
    const obj = objects[i];
    const aabb = {
      minX: obj.x,
      minY: obj.y,
      maxX: obj.x + (obj.width || 0),
      maxY: obj.y + (obj.height || 0)
    };
    grid.insert(i, aabb);
  }

  // Получаем уникальные пары
  const pairs = grid.getCandidatePairs();

  // Для каждой пары выполняем точное SAT‑тестирование
  for (const [i, j] of pairs) {
    const contacts = SAT_detectObjectCollision(objects[i], objects[j]);
    for (const contact of contacts) {
      collisionContacts.push(contact);
    }
  }
}