// import SimulationObject from "../../../common/simulation_object.js";
// import Plane from "../../../common/plane.js";
// import { GroundConstraint, BoxBoxCollisionXPBD } from "../../../common/collision.js";

// const { mat4, vec3, quat } = glMatrix;

// export default class Simulation1 {
//   plane;
//   objects;
//   p5Instance;
  
//   // Параметры XPBD
//   iterations = 10;
//   compliance = 0.0001;
//   gravity = vec3.fromValues(0, -0.001, 0);

//   constructor(p5Instance) {
//     this.p5Instance = p5Instance;
//     this.plane = new Plane(500, vec3.fromValues(100, 200, 100), p5Instance);
    
//     // Создаем 10 случайных объектов
//     this.objects = [];
//     const colors = [
//       [255, 100, 100], [100, 255, 100], [100, 100, 255],
//       [255, 255, 100], [255, 100, 255], [100, 255, 255],
//       [255, 200, 100], [200, 100, 255], [100, 200, 255],
//       [255, 150, 150]
//     ];
    
//     for (let i = 0; i < 10; i++) {
//       // Размеры куба (ширина, высота, глубина)
//       const width = 50;
//       const height = 50;
//       const depth = 50;
      
//       // Позиции: разбрасываем по пространству
//       const x = -200 + (i % 5) * 100;
//       const z = -150 + Math.floor(i / 5) * 150;
//       const y = 100 + i * 20; // На разной высоте, чтобы падали
      
//       this.objects.push(
//         new SimulationObject(
//           vec3.fromValues(width, height, depth), // dimensions
//           vec3.fromValues(x, y, z), // nextPosition
//           vec3.fromValues(0, 0, 0), // nextRotation (начальный поворот)
//           vec3.fromValues(1, 1, 1), // scale (оставляем 1)
//           vec3.fromValues(0, 0, 0), // linear velocity
//           vec3.fromValues(0, 0, 0), // angular velocity
//           15, // mass
//           vec3.fromValues(colors[i][0], colors[i][1], colors[i][2]), // color
//           p5Instance,
//         ),
//       );
//     }
//   }

//   update(dt) {
//     if (dt > 0.033) dt = 0.033;
    
//     // Сохраняем предыдущие позиции
//     for (let object of this.objects) {
//       vec3.copy(object.position, object.nextPosition);
//       quat.copy(object.rotation, object.nextRotation);
//       vec3.copy(object.prevLinearVelocity, object.nextVelocity);
//     }
    
//     // XPBD итерации
//     for (let iter = 0; iter < this.iterations; iter++) {
//       // Предсказание позиций (только на первой итерации)
//       if (iter === 0) {
//         for (let object of this.objects) {
//           // Применяем гравитацию и обновляем позицию
//           vec3.scaleAndAdd(object.nextPosition, object.nextPosition, object.nextVelocity, dt);
//           vec3.scaleAndAdd(object.nextPosition, object.nextPosition, this.gravity, dt * dt);
          
//           // Обновляем вращение, если есть угловая скорость
//           if (vec3.length(object.nextAngularVelocity) > 0.001) {
//             let wquat = quat.fromValues(
//               object.nextAngularVelocity[0],
//               object.nextAngularVelocity[1],
//               object.nextAngularVelocity[2],
//               0.0
//             );
//             quat.multiply(wquat, wquat, object.nextRotation);
//             quat.scale(wquat, wquat, dt * 0.5);
//             quat.add(object.nextRotation, object.nextRotation, wquat);
//             quat.normalize(object.nextRotation, object.nextRotation);
//           }
//         }
//       }
      
//       // Проверяем столкновения между всеми парами объектов
//       for (let i = 0; i < this.objects.length; i++) {
//         for (let j = i + 1; j < this.objects.length; j++) {
//           this.solveBoxBoxCollision(this.objects[i], this.objects[j]);
//         }
//       }
      
//       // Проверяем столкновения с землей
//       for (let object of this.objects) {
//         this.solveGroundCollision(object);
//       }
//     }
    
//     // Обновляем скорости после всех итераций
//     for (let object of this.objects) {
//       // Линейная скорость
//       vec3.sub(object.nextVelocity, object.nextPosition, object.position);
//       vec3.scale(object.nextVelocity, object.nextVelocity, 1.0 / dt);
      
//       // Угловая скорость (если было вращение)
//       let deltaQuat = quat.create();
//       let prevConj = quat.clone(object.rotation);
//       quat.conjugate(prevConj, prevConj);
//       quat.multiply(deltaQuat, object.nextRotation, prevConj);
      
//       let angle = 2 * Math.acos(Math.min(0.9999, Math.abs(deltaQuat[3])));
//       let axis = vec3.fromValues(deltaQuat[0], deltaQuat[1], deltaQuat[2]);
//       let len = vec3.length(axis);
//       if (len > 0.0001 && angle > 0.0001) {
//         vec3.scale(axis, axis, 1.0 / len);
//         vec3.scale(object.nextAngularVelocity, axis, angle / dt);
//       } else {
//         vec3.set(object.nextAngularVelocity, 0, 0, 0);
//       }
//     }
//   }
  
//   solveGroundCollision(object) {
//     // Получаем реальные размеры объекта из его локальных вершин
//     let minY = Infinity;
//     let maxY = -Infinity;
    
//     // Вычисляем AABB объекта в мировых координатах
//     for (let vert of object.worldVertices) {
//       if (vert[1] < minY) minY = vert[1];
//       if (vert[1] > maxY) maxY = vert[1];
//     }
    
//     const groundY = 0;
    
//     if (minY < groundY) {
//       const penetration = groundY - minY;
//       // Корректируем позицию
//       object.nextPosition[1] += penetration;
      
//       // Обновляем мировые вершины после коррекции
//       object.draw(); // Это обновит worldVertices
      
//       // Отскок
//       if (object.nextVelocity[1] < 0) {
//         object.nextVelocity[1] *= -0.3;
//       }
//     }
//   }
  
//   solveBoxBoxCollision(objA, objB) {
//     // Получаем AABB для каждого объекта
//     const aabbA = this.getAABB(objA);
//     const aabbB = this.getAABB(objB);
    
//     // Проверяем пересечение AABB
//     if (aabbA.maxX < aabbB.minX || aabbB.maxX < aabbA.minX) return;
//     if (aabbA.maxY < aabbB.minY || aabbB.maxY < aabbA.minY) return;
//     if (aabbA.maxZ < aabbB.minZ || aabbB.maxZ < aabbA.minZ) return;
    
//     // Вычисляем перекрытие
//     const overlapX = Math.min(aabbA.maxX, aabbB.maxX) - Math.max(aabbA.minX, aabbB.minX);
//     const overlapY = Math.min(aabbA.maxY, aabbB.maxY) - Math.max(aabbA.minY, aabbB.minY);
//     const overlapZ = Math.min(aabbA.maxZ, aabbB.maxZ) - Math.max(aabbA.minZ, aabbB.minZ);
    
//     // Находим наименьшее перекрытие для определения нормали
//     let normal = vec3.create();
//     let penetration = overlapX;
//     vec3.set(normal, 1, 0, 0);
    
//     if (overlapY < penetration) {
//       penetration = overlapY;
//       vec3.set(normal, 0, 1, 0);
//     }
    
//     if (overlapZ < penetration) {
//       penetration = overlapZ;
//       vec3.set(normal, 0, 0, 1);
//     }
    
//     // Определяем направление нормали
//     const centerA = objA.nextPosition;
//     const centerB = objB.nextPosition;
//     const dir = vec3.subtract(vec3.create(), centerB, centerA);
    
//     if (vec3.dot(dir, normal) < 0) {
//       vec3.negate(normal, normal);
//     }
    
//     // XPBD коррекция
//     const invMassA = 1.0 / objA.mass;
//     const invMassB = 1.0 / objB.mass;
//     const totalInvMass = invMassA + invMassB;
    
//     if (totalInvMass > 0) {
//       const correction = penetration / (totalInvMass + this.compliance / (this.iterations * this.iterations));
//       const correctionVec = vec3.scale(vec3.create(), normal, correction);
      
//       vec3.scaleAndAdd(objA.nextPosition, objA.nextPosition, correctionVec, -invMassA);
//       vec3.scaleAndAdd(objB.nextPosition, objB.nextPosition, correctionVec, invMassB);
      
//       // Коррекция скоростей
//       const relVelocity = vec3.subtract(vec3.create(), objB.nextVelocity, objA.nextVelocity);
//       const velAlong = vec3.dot(relVelocity, normal);
      
//       if (velAlong < 0) {
//         const restitution = 0.3;
//         const impulseMag = -(1 + restitution) * velAlong / totalInvMass;
//         const impulse = vec3.scale(vec3.create(), normal, impulseMag);
        
//         vec3.scaleAndAdd(objA.nextVelocity, objA.nextVelocity, impulse, -invMassA);
//         vec3.scaleAndAdd(objB.nextVelocity, objB.nextVelocity, impulse, invMassB);
//       }
//     }
//   }
  
//   getAABB(object) {
//     // Обновляем мировые вершины
//     object.draw();
    
//     let minX = Infinity, minY = Infinity, minZ = Infinity;
//     let maxX = -Infinity, maxY = -Infinity, maxZ = -Infinity;
    
//     for (let vert of object.worldVertices) {
//       minX = Math.min(minX, vert[0]);
//       minY = Math.min(minY, vert[1]);
//       minZ = Math.min(minZ, vert[2]);
//       maxX = Math.max(maxX, vert[0]);
//       maxY = Math.max(maxY, vert[1]);
//       maxZ = Math.max(maxZ, vert[2]);
//     }
    
//     return { minX, minY, minZ, maxX, maxY, maxZ };
//   }

//   draw() {
//     this.p5Instance.camera(0, -600, 600, 0, 100, 0, 0, 1, 0);
    
//     // Включаем освещение для лучшей видимости
//     this.p5Instance.ambientLight(100);
//     this.p5Instance.directionalLight(255, 255, 255, 0, -1, -1);
    
//     this.plane.draw();
    
//     for (let object of this.objects) {
//       object.draw();
//     }
//   }
// }
