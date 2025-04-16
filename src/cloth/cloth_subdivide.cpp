#include "cloth.h"
void Cloth::IncreaseClothDensity() {
  maximumSubdivision++;
  vector<vector<ClothParticle>> newParticles(
      (nx * 2) - 1, vector<ClothParticle>((ny * 2) - 1, ClothParticle::none()));
  for (int i = 0; i < nx; i++) {
    for (int j = 0; j < ny; j++) {
      newParticles[i * 2][j * 2] = std::move(particles[i][j]);
    }
  }
  nx = (nx * 2) - 1;
  ny = (ny * 2) - 1;
  particles = std::move(newParticles);
}

void Cloth::AddSubdividedParticles(int i, int j, int distance) {
  ClothParticle& p = particles[i][j];

  for (const Offset::Offset offset : Offset::Surrounding) {
    const int xIdx = i + (distance * offset.first);
    const int yIdx = j + (distance * offset.second);

    if (!inBounds(xIdx, yIdx)) {
      continue;
    }

    ClothParticle& newParticle = particles[xIdx][yIdx];

    if (newParticle.type == Particle::Fixed) {
      continue;
    }

    const int      interpXIdx = i + (distance * offset.first * 2);
    const int      interpYIdx = j + (distance * offset.second * 2);
    ClothParticle& interpParticle = particles[interpXIdx][interpYIdx];
    assert(interpParticle.type != Particle::None);
    newParticle = ClothParticle::interpActive(p, interpParticle);
    newParticle.layer = p.layer;
  }
}
void Cloth::AddInterpolatedParticles(int i, int j, int distance) {
  int interpLayer = getParticle(i, j).layer - 1;
  for (const Offset::Offset offset : Offset::Interp) {
    const int xIdx = i + (distance * offset.first);
    const int yIdx = j + (distance * offset.second);
    if (!inBounds(xIdx, yIdx)) {
      continue;
    }

    ClothParticle& newParticle = particles[xIdx][yIdx];
    if (newParticle.type == Particle::Fixed ||
        newParticle.type == Particle::Active) {
      continue;
    }
    //interpolating between the adjacent active points we just created
    if (newParticle.type == Particle::None) {
      if (abs(offset.first) == 1) {
        ClothParticle& interpA = particles[xIdx - distance][yIdx];
        ClothParticle& interpB = particles[xIdx + distance][yIdx];
        assert(interpA.type != Particle::None);
        assert(interpB.type != Particle::None);
        newParticle = ClothParticle::interp(interpA, interpB);
      } else {
        ClothParticle& interpA = particles[xIdx][yIdx - distance];
        ClothParticle& interpB = particles[xIdx][yIdx + distance];
        assert(interpA.type != Particle::None);
        assert(interpB.type != Particle::None);
        newParticle = ClothParticle::interp(interpA, interpB);
      }
    }
    newParticle.layer = interpLayer;
  }
}

void Cloth::SubdivideAboutPoint(int i, int j) {
  ClothParticle& p = particles[i][j];
  if (p.layer >= subdivision_limit) {
    return;
  }
  assert(p.layer < maximumSubdivision);
  assert(p.type == Particle::Active || p.type == Particle::Fixed);

  const int distance = scale(p.layer + 1);

  //add previous layers up to current layer
  for (const Offset::Offset offset : Offset::Surrounding) {
    break;
    const int xIdx = i + (2 * distance * offset.first);
    const int yIdx = j + (2 * distance * offset.second);
    if (!inBounds(xIdx, yIdx)) {
      continue;
    }
    if (particles[xIdx][yIdx].type == Particle::None) {
      p.layer--;
      SubdivideAboutPoint(i, j);
      p.layer++;
    }
  }

  p.layer++;
  AddSubdividedParticles(i, j, distance);
  AddInterpolatedParticles(i, j, distance);
}
