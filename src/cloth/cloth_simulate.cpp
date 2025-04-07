#include "cloth.h"
void Cloth::AdjustInterpolated() {
  for (int i = 0; i < nx; i++) {
    for (int j = 0; j < ny; j++) {
      ClothParticle& p = particles[i][j];
      if (p.type != Particle::Interp) {
        continue;
      }
      int distance = 1 << (maximumSubdivision - p.layer - 1);
      assert(distance >= 0);  //should never be interpolating across base layer
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

bool shouldSimulate(int t, const ClothParticle& p) {
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
Vec3f Cloth::reduceForceOverOffsets(
    int i, int j, const vector<std::pair<int, int>>& offsets) const {
  Vec3f                force = Vec3f::zero();
  const ClothParticle& p = getParticle(i, j);
  const int            distance = 1 << (maximumSubdivision - p.layer);
  for (const std::pair<int, int>& offset : offsets) {
    int iOffset = i + offset.first * distance;
    int jOffset = j + offset.second * distance;
    if (iOffset < 0 || jOffset < 0 || iOffset >= nx || jOffset >= ny) {
      continue;
    }
    const auto k = springs.find(PosPair(i, j, iOffset, jOffset, p.layer));
    assert(k != springs.end());
    const ClothParticle& adj =
        getParticle(i + offset.first * distance, j + offset.second * distance);
    force += forceToPoint(k->second, p, adj);
  }
  return force;
}

void Cloth::updateForces(int t, vector<vector<Vec3f>>& forces) {
  using std::pair;
  const static vector<pair<int, int>> structuralOffsets = {
      {0, 1},
      {0, -1},
      {1, 0},
      {-1, 0},
  };
  const static vector<pair<int, int>> bendOffsets = {
      {0, 2},
      {0, -2},
      {2, 0},
      {-2, 0},
  };
  const static vector<pair<int, int>> shearOffsets = {
      {1, 1},
      {1, -1},
      {-1, 1},
      {-1, -1},
  };
  for (int i = 0; i < nx; i++) {
    for (int j = 0; j < ny; j++) {
      const ClothParticle& p = getParticle(i, j);
      if (!shouldSimulate(t, p)) {
        continue;
      }
      forces[i][j] = GLOBAL_args->mesh_data->gravity;
      forces[i][j] += reduceForceOverOffsets(i, j, structuralOffsets);
      forces[i][j] += reduceForceOverOffsets(i, j, shearOffsets);
      forces[i][j] += reduceForceOverOffsets(i, j, bendOffsets);
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
      timestep /= (double)(1 << maximumSubdivision);
      timestep *= (double)(1 << (maximumSubdivision - p.layer));
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
        timestep /= (double)(1 << maximumSubdivision);
        timestep *= (double)(1 << (maximumSubdivision - p.layer));
        p.position += p.velocity * timestep;
      }
    }
  }
}

void Cloth::correctPositions() {
  for (const auto& pair : springs) {
    ClothParticle& a = particles[pair.first.a.x][pair.first.a.y];
    ClothParticle& b = particles[pair.first.b.x][pair.first.b.y];
    assert(a.type != Particle::None);
    assert(b.type != Particle::None);

    const Vec3f distance = a.position - b.position;
    const Vec3f originalDistance = a.originalPosition - b.originalPosition;

    double stretchRatio = distance.Length() / originalDistance.Length();
    if (stretchRatio - 1.0 < correction) {
      continue;
    }
    const double ratio = 1 - ((correction + 1) / stretchRatio);
    if (a.type != Particle::Active && b.type != Particle::Active) {
      continue;
    }
    if (a.type != Particle::Active) {
      Vec3f moveVector = distance;
      moveVector *= ratio;
      b.position += moveVector;
    } else if (b.type != Particle::Active) {
      Vec3f moveVector = distance;
      moveVector *= ratio;
      a.position -= moveVector;
    } else {
      Vec3f moveVector = distance;
      moveVector *= ratio;
      moveVector /= 2;
      a.position -= moveVector;
      b.position += moveVector;
    }
  }
}

void Cloth::performTimestepSimulation(int t) {
  static vector<vector<Vec3f>> forces;
  if ((int)forces.size() != nx || (int)forces[0].size() != ny) {
    forces = vector<vector<Vec3f>>(nx, vector<Vec3f>(ny, Vec3f::zero()));
  }

  updateForces(t, forces);
  for (int i = 0; i < nx; i++) {
    for (int j = 0; j < ny; j++) {
      std::cout << forces[i][j] << ", ";
    }
    std::cout << std::endl;
  }
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
