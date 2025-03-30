#include <fstream>
#include "../args/argparser.h"
#include "../libs/utils.h"
#include "../render/meshdata.h"
#include "cloth.h"

extern MeshData* mesh_data;

// ================================================================================
// ================================================================================

Cloth::Cloth(ArgParser* _args) {
  args = _args;

  // open the file
  std::ifstream istr(std::string(args->path + '/' + args->cloth_file).c_str());
  assert(istr.good());
  std::string token;

  // read in the simulation parameters
  istr >> token >> k_structural;
  assert(token == "k_structural");  // (units == N/m)  (N = kg*m/s^2)
  istr >> token >> k_shear;
  assert(token == "k_shear");
  istr >> token >> k_bend;
  assert(token == "k_bend");
  istr >> token >> damping;
  assert(token == "damping");
  // NOTE: correction factor == .1, means springs shouldn't stretch more than
  // 10%
  //       correction factor == 100, means don't do ainitialY correction
  istr >> token >> provot_structural_correction;
  assert(token == "provot_structural_correction");
  istr >> token >> provot_shear_correction;
  assert(token == "provot_shear_correction");

  // the cloth dimensions
  istr >> token >> initialX >> initialY;
  nx = initialX;
  ny = initialY;
  assert(token == "m");
  assert(initialX >= 2 && initialY >= 2);

  // the corners of the cloth
  // (units == meters)
  Vec3f  a, b, c, d;
  double x, y, z;
  istr >> token >> x >> y >> z;
  assert(token == "p");
  a.set(x, y, z);
  istr >> token >> x >> y >> z;
  assert(token == "p");
  b.set(x, y, z);
  istr >> token >> x >> y >> z;
  assert(token == "p");
  c.set(x, y, z);
  istr >> token >> x >> y >> z;
  assert(token == "p");
  d.set(x, y, z);

  // fabric weight  (units == kg/m^2)
  // denim ~300 g/m^2
  // silk ~70 g/m^2
  double fabric_weight;
  istr >> token >> fabric_weight;
  assert(token == "fabric_weight");
  double area = AreaOfTriangle(a, b, c) + AreaOfTriangle(a, c, d);

  // create the particles
  particles = vector<vector<ClothParticle>>(
      initialX, vector<ClothParticle>(initialY, ClothParticle()));
  double mass = area * fabric_weight / double(initialX * initialY);
  for (int i = 0; i < initialX; i++) {
    double x = i / double(initialX - 1);
    Vec3f  ab = float(1 - x) * a + float(x) * b;
    Vec3f  dc = float(1 - x) * d + float(x) * c;
    for (int j = 0; j < initialY; j++) {
      double y = j / double(initialY - 1);
      particles[i][j] = ClothParticle();
      ClothParticle& p = particles[i][j];
      Vec3f          abdc = float(1 - y) * ab + float(y) * dc;
      p.position = abdc;
      p.velocity = Vec3f::zero();
      p.mass = mass;
      p.type = Particle::Active;
    }
  }

  // create the springs
  for (int i = 1; i < initialX - 1; i++) {
    for (int j = 1; j < initialY - 1; j++) {
      //structural
      springs[PosPair(i, j, i + 1, j)] = k_structural;
      springs[PosPair(i, j, i - 1, j)] = k_structural;
      springs[PosPair(i, j, i, j + 1)] = k_structural;
      springs[PosPair(i, j, i, j - 1)] = k_structural;
      //shear
      springs[PosPair(i, j, i + 1, j + 1)] = k_shear;
      springs[PosPair(i, j, i + 1, j - 1)] = k_shear;
      springs[PosPair(i, j, i - 1, j + 1)] = k_shear;
      springs[PosPair(i, j, i - 1, j - 1)] = k_shear;
      //bend
      springs[PosPair(i, j, i + 2, j)] = k_bend;
      springs[PosPair(i, j, i - 2, j)] = k_bend;
      springs[PosPair(i, j, i, j + 2)] = k_bend;
      springs[PosPair(i, j, i, j - 2)] = k_bend;
    }
  }

  // parse timestep
  istr >> token >> mesh_data->timestep;
  assert(token == "timestep");
  assert(mesh_data->timestep > 0.0);

  // the fixed particles
  while (istr >> token) {
    assert(token == "f");
    int    i, j;
    double x, y, z;
    istr >> i >> j >> x >> y >> z;
    ClothParticle& p = particles[i][j];
    p.position = Vec3f(x, y, z);
    p.type = Particle::Fixed;
  }

  computeBoundingBox();
}

// ================================================================================

void Cloth::computeBoundingBox() {
  // TODO: update for new data structure
  box = BoundingBox(getParticle(0, 0).position);
  for (int i = 0; i < initialX; i++) {
    for (int j = 0; j < initialY; j++) {
      if (hasParticle(i, j)) {
        box.Extend(getParticle(i, j).position);
      }
    }
  }
}

// ================================================================================
