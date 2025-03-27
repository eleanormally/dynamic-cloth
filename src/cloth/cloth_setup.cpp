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
  //       correction factor == 100, means don't do any correction
  istr >> token >> provot_structural_correction;
  assert(token == "provot_structural_correction");
  istr >> token >> provot_shear_correction;
  assert(token == "provot_shear_correction");

  // the cloth dimensions
  istr >> token >> nx >> ny;
  assert(token == "m");
  assert(nx >= 2 && ny >= 2);

  // the corners of the cloth
  // (units == meters)
  Vec3f a, b, c, d;
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
  particles = new ClothParticle[nx * ny];
  double mass = area * fabric_weight / double(nx * ny);
  for (int i = 0; i < nx; i++) {
    double x = i / double(nx - 1);
    Vec3f ab = float(1 - x) * a + float(x) * b;
    Vec3f dc = float(1 - x) * d + float(x) * c;
    for (int j = 0; j < ny; j++) {
      double y = j / double(ny - 1);
      ClothParticle& p = getParticle(i, j);
      Vec3f abdc = float(1 - y) * ab + float(y) * dc;
      p.setOriginalPosition(abdc);
      p.setPosition(abdc);
      p.setVelocity(Vec3f(0, 0, 0));
      p.setMass(mass);
      p.setFixed(false);
    }
  }

  // parse timestep
  istr >> token >> mesh_data->timestep;
  assert(token == "timestep");
  assert(mesh_data->timestep > 0.0);

  // the fixed particles
  while (istr >> token) {
    assert(token == "f");
    int i, j;
    double x, y, z;
    istr >> i >> j >> x >> y >> z;
    ClothParticle& p = getParticle(i, j);
    p.setPosition(Vec3f(x, y, z));
    p.setFixed(true);
  }

  computeBoundingBox();
}

// ================================================================================

void Cloth::computeBoundingBox() {
  box = BoundingBox(getParticle(0, 0).getPosition());
  for (int i = 0; i < nx; i++) {
    for (int j = 0; j < ny; j++) {
      box.Extend(getParticle(i, j).getPosition());
      box.Extend(getParticle(i, j).getOriginalPosition());
    }
  }
}

// ================================================================================
