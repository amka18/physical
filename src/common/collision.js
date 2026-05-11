const { vec3 } = glMatrix;

export function GroundConstraint(object) {
  const halfSize = 40;
  const minY = object.position[1] - halfSize;
  
  if (minY < 0) {
    object.position[1] += -minY;
    if (object.linearVelocity[1] < 0) {
      object.linearVelocity[1] *= -0.3;
    }
  }
}

export class BoxBoxCollisionXPBD {
  static solve(objA, objB, compliance, iterations) {
    const halfSize = 40;
    const posA = objA.position;
    const posB = objB.position;
    
    const delta = vec3.subtract(vec3.create(), posB, posA);
    
    const overlapX = (halfSize + halfSize) - Math.abs(delta[0]);
    const overlapY = (halfSize + halfSize) - Math.abs(delta[1]);
    const overlapZ = (halfSize + halfSize) - Math.abs(delta[2]);
    
    if (overlapX > 0 && overlapY > 0 && overlapZ > 0) {
      let normal = vec3.create();
      let penetration = overlapX;
      vec3.set(normal, Math.sign(delta[0]), 0, 0);
      
      if (overlapY < penetration) {
        penetration = overlapY;
        vec3.set(normal, 0, Math.sign(delta[1]), 0);
      }
      
      if (overlapZ < penetration) {
        penetration = overlapZ;
        vec3.set(normal, 0, 0, Math.sign(delta[2]));
      }
      
      const invMassA = 1.0 / objA.mass;
      const invMassB = 1.0 / objB.mass;
      const totalInvMass = invMassA + invMassB;
      
      if (totalInvMass > 0) {
        const correction = penetration / (totalInvMass + compliance / (iterations * iterations));
        const correctionA = vec3.scale(vec3.create(), normal, correction * invMassA);
        const correctionB = vec3.scale(vec3.create(), normal, -correction * invMassB);
        
        vec3.add(objA.position, objA.position, correctionA);
        vec3.add(objB.position, objB.position, correctionB);
        
        return true;
      }
    }
    return false;
  }
}