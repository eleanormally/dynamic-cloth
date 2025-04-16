#include <cmath>
#include "cloth.h"

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
      const ClothParticle& l = getParticle(i - iDelta, j - jDelta);
      const ClothParticle& r = getParticle(i + iDelta, j + jDelta);
      assert(l.type != Particle::None);
      assert(r.type != Particle::None);

      p.inPlaceInterp(l, r);
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
  static int animations;
  if (animations == timestamps_per_subdivision - 1) {
    animations = 0;
    Subdivide();
  } else {
    animations++;
  }
  for (int t = (1 << maximumSubdivision) - 1; t >= 0; t--) {
    performTimestepSimulation(t);
  }
  AdjustInterpolated();
}

float getAngle(const Vec3f& p1, const Vec3f& p2, const Vec3f& p3) {
  Vec3f p12 = p1 - p2;
  Vec3f p23 = p2 - p3;
  p12.Normalize();
  p23.Normalize();
  return acos(p12.Dot3(p23));
}

vector<vector<bool>> Cloth::getShouldSubdivide(float threshold) {
  vector<vector<bool>> result(nx, vector<bool>(ny, false));
  for (int i = 0; i < nx; i++) {
    for (int j = 0; j < ny; j++) {
      const ClothParticle& p = getParticle(i, j);
      if (p.type != Particle::Active) {
        continue;
      }
      for (const Offset::Offset offset : Offset::Cardinal) {
        int distance = scale(p.layer);
        int deltaI = distance * offset.first;
        int deltaJ = distance * offset.second;
        if (!inBounds(i + deltaI, j + deltaJ) ||
            !inBounds(i - deltaI, j - deltaJ)) {
          continue;
        }

        const ClothParticle& l = getParticle(i - deltaI, j - deltaJ);
        const ClothParticle& r = getParticle(i + deltaI, j + deltaJ);
        if (l.type == Particle::None || r.type == Particle::None) {
          continue;
        }

        float angle = getAngle(l.position, p.position, r.position);
        if (angle > threshold) {
          result[i][j] = true;
          break;
        }
      }
    }
  }
  return result;
}

void Cloth::Subdivide() {
  vector<vector<bool>> subdivisions = getShouldSubdivide(subdivision_angle);

  bool increaseDensity = false;
  for (int i = 0; i < nx; i++) {
    for (int j = 0; j < ny; j++) {
      const ClothParticle& p = getParticle(i, j);
      if (p.layer < subdivision_limit && p.layer == maximumSubdivision &&
          subdivisions[i][j]) {
        increaseDensity = true;
      }
    }
  }
  if (increaseDensity) {
    IncreaseClothDensity();
  }
  //using subdivisions.size() since we may have increased nx/ny above
  for (int i = 0; i < (int)subdivisions.size(); i++) {
    for (int j = 0; j < (int)subdivisions[0].size(); j++) {
      if (subdivisions[i][j]) {
        int x = increaseDensity ? i * 2 : i;
        int y = increaseDensity ? j * 2 : j;
        SubdivideAboutPoint(x, y);
      }
    }
  }
}
