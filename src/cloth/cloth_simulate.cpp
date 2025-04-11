#include "cloth.h"
#include <cmath>

void Cloth::AdjustInterpolated() {
  for (int i = 0; i < nx; i++) {
    for (int j = 0; j < ny; j++) {
      ClothParticle& p = particles[i][j];
      if (p.type != Particle::Interp) {
        continue;
      }
      int  distance = scale(p.layer);
      bool rowInterp = i % distance == distance / 2;
      int  iDelta = 0;
      int  jDelta = 0;
      if (rowInterp) {
        iDelta = distance / 2;
      } else {
        jDelta = distance / 2;
      }
      p.inPlaceInterp(particles[i + iDelta][j + jDelta],
                      particles[i - iDelta][j - jDelta]);
    }
  }
}

inline bool shouldSimulate(int t, const ClothParticle& p) {
  return p.type == Particle::Active && ((1 << p.layer) - 1 >= t);
}

Vec3f forceToPoint(double k, const ClothParticle& source,
                   const ClothParticle& adjacent) {
  const Vec3f length = adjacent.position - source.position;
  Vec3f       lengthNorm = length;
  lengthNorm.Normalize();
  const Vec3f originalLength =
      adjacent.originalPosition - source.originalPosition;
  const Vec3f dist = originalLength.Length() * lengthNorm;
  const Vec3f total = k * (length - dist);
  return total;
}
Vec3f Cloth::reduceForceOverOffsets(int i, int j, const Offset::Vec& offsets,
                                    double k) const {
  Vec3f                force = Vec3f::zero();
  const ClothParticle& p = getParticle(i, j);

  const int distance = scale(p.layer);
  const int forceMultiplier = 1 << p.layer;

  for (const Offset::Offset& offset : offsets) {
    int iOffset = i + offset.first * distance;
    int jOffset = j + offset.second * distance;
    if (!inBounds(iOffset, jOffset)) {
      continue;
    }
    const ClothParticle& adj = getParticle(iOffset, jOffset);
    if (adj.type == Particle::None) {
      continue;
    }
    force += forceToPoint(k * (double)forceMultiplier, p, adj);
  }
  return force;
}

void Cloth::updateForces(int t, vector<vector<Vec3f>>& forces) {
  using std::pair;
  for (int i = 0; i < nx; i++) {
    for (int j = 0; j < ny; j++) {
      const ClothParticle& p = getParticle(i, j);
      if (!shouldSimulate(t, p)) {
        continue;
      }
      forces[i][j] = Vec3f::zero();
      forces[i][j] +=
          reduceForceOverOffsets(i, j, Offset::Structural, k_structural);
      forces[i][j] += reduceForceOverOffsets(i, j, Offset::Shear, k_shear);
      forces[i][j] += reduceForceOverOffsets(i, j, Offset::Bend, k_bend);
      Vec3f gForce = Vec3f(GLOBAL_args->mesh_data->gravity);
      gForce *= p.mass;
      forces[i][j] += gForce;
      Vec3f dampingForce = damping * p.velocity;
      forces[i][j] -= dampingForce;
      forces[i][j] /= p.mass;
    }
  }
}

void Cloth::updateVelocities(int t, const vector<vector<Vec3f>>& forces) {
  for (int i = 0; i < nx; i++) {
    for (int j = 0; j < ny; j++) {
      ClothParticle& p = particles[i][j];
      if (!shouldSimulate(t, p)) {
        continue;
      }
      double timestep = GLOBAL_args->mesh_data->timestep;
      timestep /= (double)scale(0);
      timestep *= (double)scale(p.layer);
      p.velocity += forces[i][j] * timestep;
    }
  }
}

void Cloth::updatePositions() {
  for (int i = 0; i < nx; i++) {
    for (int j = 0; j < ny; j++) {
      ClothParticle& p = particles[i][j];
      if (p.type == Particle::Active) {
        double timestep = GLOBAL_args->mesh_data->timestep;
        timestep /= (double)scale(0);
        timestep *= (double)scale(p.layer);
        p.position += p.velocity * timestep;
      }
    }
  }
}

void Cloth::correctParticle(int i, int j, const Offset::Vec& offsets,
                            double k) {
  ClothParticle& p = particles[i][j];
  const int      distance = scale(p.layer);

  for (const Offset::Offset offset : offsets) {
    int iIdx = i + offset.first * distance;
    int jIdx = j + offset.second * distance;
    if (!inBounds(iIdx, jIdx)) {
      continue;
    }

    ClothParticle& adjacent = particles[iIdx][jIdx];
    if (adjacent.type == Particle::None) {
      continue;
    }

    const Vec3f distance = p.position - adjacent.position;
    const Vec3f original = p.originalPosition - adjacent.originalPosition;

    double stretchRatio = distance.Length() / original.Length();
    if (stretchRatio - 1.0 > k) {
      const double moveRatio = 1 - ((k + 1) / stretchRatio);
      Vec3f        moveVector = distance;

      moveVector *= moveRatio;

      if (adjacent.type == Particle::Active) {
        moveVector /= 2;
        p.position -= moveVector;
        adjacent.position += moveVector;
      } else {
        p.position -= moveVector;
      }
    }
  }
}

void Cloth::correctPositions() {
  for (int i = 0; i < nx; i++) {
    for (int j = 0; j < ny; j++) {
      const ClothParticle& p = getParticle(i, j);
      if (p.type != Particle::Active) {
        continue;
      }
      correctParticle(i, j, Offset::Structural, correction);
      correctParticle(i, j, Offset::Shear, correction);
    }
  }
}

void Cloth::performTimestepSimulation(int t) {
  static vector<vector<Vec3f>> forces;
  if ((int)forces.size() != nx || (int)forces[0].size() != ny) {
    forces = vector<vector<Vec3f>>(nx, vector<Vec3f>(ny, Vec3f::zero()));
  }

  updateForces(t, forces);
  updateVelocities(t, forces);
  updatePositions();
  correctPositions();
}

void Cloth::Animate() {
  for (int t = 0; t < (1 << maximumSubdivision); t++) {
    performTimestepSimulation(t);
  }
  AdjustInterpolated();
}

float getAngle(Vec3f p1, Vec3f p2, Vec3f p3){
  Vec3f p12 = p1 - p2;
  Vec3f p23 = p2 - p3;
  p12.Normalize();
  p23.Normalize();
  return acos(p12.Dot3(p23));
}


std::vector<std::vector<bool>> Cloth::getShouldSubdivide(float threshold){
  std::vector<std::vector<bool>> result;
  for(int i = 0; i < (int)particles.size(); i++){
    std::vector<bool> rowResults;
    for(int j = 0; j < (int)particles[i].size(); j ++){
      ClothParticle p = getParticle(i, j);
      Vec3f pos = getParticle(i, j).position;
      Vec3f north = pos;
      Vec3f south = pos;
      Vec3f east = pos;
      Vec3f west = pos;
      if (i - 1 >= 0)
        north = getParticle(i - 1, j).position;
      if (i + 1 < nx)
        south = getParticle(i + 1, j).position;
      if (j - 1 >= 0)
        east = getParticle(i, j - 1).position;
      if (j + 1 < ny)
        west = getParticle(i, j + 1).position;
      if(i - 1 >= 0 && i + 1 < nx){
        if(j - 1 >= 0 && j + 1 < ny){
          float t1 = getAngle(south, pos, north);
          float t2 = getAngle(west, pos, east);
          std::cout << t1 << " " << t2 << std::endl;
          if(t1 > threshold || t2 > threshold){
            rowResults.push_back(true);
            continue;
          }
        }
        else{
          float t1 = getAngle(south, pos, north);
          if(t1 > threshold){
            rowResults.push_back(true);
            continue;
        }   
      }
    }
    else if(j - 1 >= 0 && j + 1 < ny){
      float t1 = getAngle(east, pos, west);
          if(t1 > threshold){
            rowResults.push_back(true);
            continue;
    }   
    }
    rowResults.push_back(false);
  }
    result.push_back(rowResults);
  }
  return result;
}


void Cloth::subdivide() {
  // Todo: figure out threshold
}
