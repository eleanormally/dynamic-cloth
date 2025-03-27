#include "cloth.h"

static const std::pair<int, int> structuralOffsets[4] = {
    std::make_pair(1, 0), std::make_pair(-1, 0), std::make_pair(0, 1),
    std::make_pair(0, -1)};
static const std::pair<int, int> shearOffsets[4] = {
    std::make_pair(1, 1), std::make_pair(1, -1), std::make_pair(-1, 1),
    std::make_pair(-1, -1)};
static const std::pair<int, int> flexionOffsets[4] = {
    std::make_pair(2, 0), std::make_pair(-2, 0), std::make_pair(0, 2),
    std::make_pair(0, -2)};
std::pair<int, int> offsetPosition(const int i, const int j,
                                   const std::pair<int, int> offset) {
  return std::make_pair(i + offset.first, j + offset.second);
}

Vec3f Cloth::reduceForcePositionsWithConstant(
    const double k, const std::pair<int, int> offsets[4], const int i,
    const int j) {

  const ClothParticle& particle = getParticle(i, j);
  const Vec3f& particleStartingPos = particle.getOriginalPosition();
  const Vec3f& particlePos = particle.getPosition();

  Vec3f reduced;
  for (int c = 0; c < 4; c++) {
    const auto newPos =
        std::make_pair(offsets[c].first + i, offsets[c].second + j);
    if (newPos.first < 0 || newPos.second < 0 || newPos.first >= nx ||
        newPos.second >= ny) {
      continue;
    }
    const ClothParticle& adjParticle = getParticle(newPos.first, newPos.second);
    const Vec3f lengthVector = adjParticle.getPosition() - particlePos;
    Vec3f lengthVectorNorm = lengthVector;
    lengthVectorNorm.Normalize();
    const Vec3f originalLengthVector =
        adjParticle.getOriginalPosition() - particleStartingPos;
    const Vec3f dist = originalLengthVector.Length() * lengthVectorNorm;
    const Vec3f total = k * (lengthVector - dist);
    reduced += total;
  }
  return reduced;
}

void Cloth::correctOffsetParticlesWithConstant(
    double k, const std::pair<int, int> offsets[4], const int i, const int j) {
  ClothParticle& p = getParticle(i, j);
  for (int c = 0; c < 4; c++) {
    const auto newPos =
        std::make_pair(offsets[c].first + i, offsets[c].second + j);
    if (newPos.first < 0 || newPos.second < 0 || newPos.first >= nx ||
        newPos.second >= ny) {
      continue;
    }
    ClothParticle& adjP = getParticle(newPos.first, newPos.second);
    const auto dist = p.getPosition() - adjP.getPosition();
    const auto originalDist =
        p.getOriginalPosition() - adjP.getOriginalPosition();
    auto stretchRatio = dist.Length() / originalDist.Length();
    if (stretchRatio - 1.0 > k) {
      if (p.isFixed() && adjP.isFixed()) {
        continue;
      }
      const double ratio = 1 - ((k + 1) / stretchRatio);
      if (p.isFixed()) {
        Vec3f moveVector = dist;
        moveVector *= ratio;
        adjP.setPosition(adjP.getPosition() + moveVector);
      } else if (adjP.isFixed()) {
        Vec3f moveVector = dist;
        moveVector *= ratio;
        p.setPosition(p.getPosition() - moveVector);
      } else {
        Vec3f moveVector = dist;
        moveVector *= ratio;
        moveVector /= 2;
        p.setPosition(p.getPosition() - moveVector);
        adjP.setPosition(adjP.getPosition() + moveVector);
      }
    }
  }
}

void Cloth::Animate() {

  // Update Forces
  for (int i = 0; i < nx; i++) {
    for (int j = 0; j < ny; j++) {
      ClothParticle& p = getParticle(i, j);
      if (p.isFixed())
        continue;
      Vec3f netForce;
      netForce += reduceForcePositionsWithConstant(k_structural,
                                                   structuralOffsets, i, j);
      netForce += reduceForcePositionsWithConstant(k_shear, shearOffsets, i, j);
      netForce +=
          reduceForcePositionsWithConstant(k_bend, flexionOffsets, i, j);

      Vec3f gForce = Vec3f(GLOBAL_args->mesh_data->gravity);
      gForce *= p.getMass();
      netForce += gForce;

      Vec3f dampingForce = damping * getParticle(i, j).getVelocity();
      netForce -= dampingForce;

      // set accel
      netForce /= p.getMass();
      p.setAcceleration(netForce);
    }
  }

  // Integration
  for (int i = 0; i < nx; i++) {
    for (int j = 0; j < ny; j++) {
      if (getParticle(i, j).isFixed())
        continue;
      ClothParticle& p = getParticle(i, j);
      p.setVelocity(p.getVelocity() +
                    (GLOBAL_args->mesh_data->timestep * p.getAcceleration()));
      p.setPosition(p.getPosition() +
                    (GLOBAL_args->mesh_data->timestep * p.getVelocity()));
    }
  }

  // Corrections
  for (int i = 0; i < nx; i++) {
    for (int j = 0; j < ny; j++) {
      correctOffsetParticlesWithConstant(provot_structural_correction,
                                         structuralOffsets, i, j);
      correctOffsetParticlesWithConstant(provot_shear_correction, shearOffsets,
                                         i, j);
    }
  }
}
