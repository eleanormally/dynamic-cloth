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
  std::unordered_map<PosPair, double> newSprings;
  for (const std::pair<PosPair, double> oldSpring : springs) {
    newSprings[oldSpring.first * 2] = oldSpring.second;
  }
  springs = std::move(newSprings);
}

void Cloth::AddSubdividedParticles(int i, int j, int distance) {
  ClothParticle& p = particles[i][j];

  using std::pair;
  static const vector<pair<int, int>> offsets = {
      {0, 1}, {0, -1}, {1, 0}, {-1, 0}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1},
  };

  for (const pair<int, int> offset : offsets) {
    const int xIdx = i + (distance * offset.first);
    const int yIdx = j + (distance * offset.second);

    if (xIdx < 0 || xIdx >= nx || yIdx < 0 || yIdx >= ny) {
      continue;
    }

    ClothParticle& newParticle = particles[xIdx][yIdx];

    if (newParticle.type == Particle::Active ||
        newParticle.type == Particle::Fixed) {
      continue;
    }

    if (newParticle.type == Particle::Interp) {
      newParticle.type = Particle::Active;
      newParticle.layer = p.layer;
    } else {
      const int      interpXIdx = i + (distance * offset.first * 2);
      const int      interpYIdx = j + (distance * offset.second * 2);
      ClothParticle& interpParticle = particles[interpXIdx][interpYIdx];
      assert(interpParticle.type != Particle::None);
      newParticle = ClothParticle::interpActive(p, interpParticle);
      newParticle.layer = p.layer;
    }
  }
}
void Cloth::AddInterpolatedParticles(int i, int j, int distance) {
  using std::pair;

  static const vector<pair<int, int>> interpOffsets = {
      {1, 2}, {1, -2}, {2, 1}, {2, -1}, {-1, 2}, {-1, -2}, {-2, 1}, {-2, -1},
  };
  for (const pair<int, int> offset : interpOffsets) {
    const int xIdx = i + (distance * offset.first);
    const int yIdx = j + (distance * offset.second);
    if (xIdx < 0 || xIdx >= nx || yIdx < 0 || yIdx >= ny) {
      continue;
    }

    ClothParticle& newParticle = particles[xIdx][yIdx];
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
  }
}

void Cloth::SubdivideAboutPoint(int i, int j) {
  ClothParticle& p = particles[i][j];
  assert(p.layer < maximumSubdivision);
  assert(p.type == Particle::Active || p.type == Particle::Fixed);

  using std::pair;
  static const vector<pair<int, int>> offsets = {
      {0, 1}, {0, -1}, {1, 0}, {-1, 0}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1},
  };

  const int distance = 1 << (maximumSubdivision - p.layer - 1);
  //add previous layers up to current layer
  for (const pair<int, int> offset : offsets) {
    const int xIdx = i + (2 * distance * offset.first);
    const int yIdx = j + (2 * distance * offset.second);
    if (xIdx < 0 || xIdx >= nx || yIdx < 0 || yIdx >= ny) {
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
