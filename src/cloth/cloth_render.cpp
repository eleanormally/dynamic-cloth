#include <cassert>
#include <cstring>
#include "../args/argparser.h"
#include "../libs/utils.h"
#include "../render/meshdata.h"
#include "cloth.h"

extern MeshData* mesh_data;

// ================================================================================

void Cloth::AddWireFrameTriangle(float*& current, const Vec3f& apos,
                                 const Vec3f& bpos, const Vec3f& cpos,
                                 const Vec3f& anormal, const Vec3f& bnormal,
                                 const Vec3f& cnormal, const Vec3f& _abcolor,
                                 const Vec3f& _bccolor, const Vec3f& _cacolor) {

  Vec3f white = Vec3f(1, 1, 1);
  Vec3f xpos = (apos + bpos + cpos) * (1 / 3.0f);
  Vec3f xnormal = (anormal + bnormal + cnormal);
  xnormal.Normalize();

  Vec3f abcolor = _abcolor;
  Vec3f bccolor = _bccolor;
  Vec3f cacolor = _cacolor;
  if (!mesh_data->wireframe) {
    abcolor = white;
    bccolor = white;
    cacolor = white;
  }

  // Draw 3 triangles if wireframe is active
  float12 ta = {float(apos.x()),    float(apos.y()),    float(apos.z()),    1,
                float(anormal.x()), float(anormal.y()), float(anormal.z()), 0,
                float(abcolor.r()), float(abcolor.g()), float(abcolor.b()), 1};
  float12 tb = {float(bpos.x()),    float(bpos.y()),    float(bpos.z()),    1,
                float(bnormal.x()), float(bnormal.y()), float(bnormal.z()), 0,
                float(abcolor.r()), float(abcolor.g()), float(abcolor.b()), 1};
  float12 tc = {float(xpos.x()),    float(xpos.y()),    float(xpos.z()),    1,
                float(xnormal.x()), float(xnormal.y()), float(xnormal.z()), 0,
                float(white.r()),   float(white.g()),   float(white.b()),   1};
  memcpy(current, &ta, sizeof(float) * 12);
  current += 12;
  memcpy(current, &tb, sizeof(float) * 12);
  current += 12;
  memcpy(current, &tc, sizeof(float) * 12);
  current += 12;

  ta = {float(bpos.x()),    float(bpos.y()),    float(bpos.z()),    1,
        float(bnormal.x()), float(bnormal.y()), float(bnormal.z()), 0,
        float(bccolor.r()), float(bccolor.g()), float(bccolor.b()), 1};
  tb = {float(cpos.x()),    float(cpos.y()),    float(cpos.z()),    1,
        float(cnormal.x()), float(cnormal.y()), float(cnormal.z()), 0,
        float(bccolor.r()), float(bccolor.g()), float(bccolor.b()), 1};
  tc = {float(xpos.x()),    float(xpos.y()),    float(xpos.z()),    1,
        float(xnormal.x()), float(xnormal.y()), float(xnormal.z()), 0,
        float(white.r()),   float(white.g()),   float(white.b()),   1};
  memcpy(current, &ta, sizeof(float) * 12);
  current += 12;
  memcpy(current, &tb, sizeof(float) * 12);
  current += 12;
  memcpy(current, &tc, sizeof(float) * 12);
  current += 12;

  ta = {float(cpos.x()),    float(cpos.y()),    float(cpos.z()),    1,
        float(cnormal.x()), float(cnormal.y()), float(cnormal.z()), 0,
        float(cacolor.r()), float(cacolor.g()), float(cacolor.b()), 1};
  tb = {float(apos.x()),    float(apos.y()),    float(apos.z()),    1,
        float(anormal.x()), float(anormal.y()), float(anormal.z()), 0,
        float(abcolor.r()), float(cacolor.g()), float(cacolor.b()), 1};
  tc = {float(xpos.x()),    float(xpos.y()),    float(xpos.z()),    1,
        float(xnormal.x()), float(xnormal.y()), float(xnormal.z()), 0,
        float(white.r()),   float(white.g()),   float(white.b()),   1};
  memcpy(current, &ta, sizeof(float) * 12);
  current += 12;
  memcpy(current, &tb, sizeof(float) * 12);
  current += 12;
  memcpy(current, &tc, sizeof(float) * 12);
  current += 12;
}

void Cloth::PackMesh() {

  int new_cloth_tri_count = 0;

  if (mesh_data->surface) {
    new_cloth_tri_count += 3 * 4 * (nx - 1) * (ny - 1);
  }
  if (mesh_data->velocity) {
    new_cloth_tri_count += 12 * (nx) * (ny);
  }
  if (mesh_data->force) {
    new_cloth_tri_count += 12 * nx * ny;
  }
  if (mesh_data->bounding_box) {
    new_cloth_tri_count += 12 * 12;
  }
  if (mesh_data->clothTriCount != new_cloth_tri_count) {
    delete[] mesh_data->clothTriData;
    mesh_data->clothTriCount = new_cloth_tri_count;
    // allocate space for the new data
    if (mesh_data->clothTriCount == 0) {
      mesh_data->clothTriData = 0;
    } else {
      mesh_data->clothTriData = new float[12 * 3 * mesh_data->clothTriCount];
    }
  }

  // Loop over all of the triangles
  float* current = mesh_data->clothTriData;

  if (mesh_data->surface) {
    PackClothSurface(current);
  }
  if (mesh_data->velocity) {
    PackClothVelocities(current);
  }
  if (mesh_data->bounding_box) {
    PackBoundingBox(current, getBoundingBox());
  }
}

void Cloth::PackClothSurface(float*& current) {
  //TODO: dynamic allocate buffer size;
  // like the last assignment...  to make wireframe edges...
  //
  //   a-----------------------b
  //   |\                     /|
  //   |  \                 /  |
  //   |    \             /    |
  //   |      \         /      |
  //   |        \     /        |
  //   |          \ /          |
  //   |           x           |
  //   |          / \          |
  //   |        /     \        |
  //   |      /         \      |
  //   |    /             \    |
  //   |  /                 \  |
  //   |/                     \|
  //   d-----------------------c
  //

  // mesh surface positions & normals
  for (int i = 0; i < nx - 1; i++) {
    for (int j = 0; j < ny - 1; j++) {
      const ClothParticle& p = getParticle(i, j);
      if (p.type == Particle::None) {
        continue;
      }
      int  diagonal = 1;
      bool shouldUse = true;
      while (true) {
        if (i + diagonal >= nx || j + diagonal >= ny) {
          shouldUse = false;
          break;
        }
        if (getParticle(i + diagonal, j + diagonal).type != Particle::None) {
          break;
        }
        diagonal++;
      }
      if (!shouldUse) {
        continue;
      }
      const ClothParticle& endPoint = getParticle(i + diagonal, j + diagonal);

      const Vec3f a_pos = p.position;
      const Vec3f c_pos = endPoint.position;

      const ClothParticle& nextPoint = getParticle(i + diagonal, j);
      const ClothParticle& adjacentPoint = getParticle(i, j + diagonal);

      if (nextPoint.type == Particle::None ||
          adjacentPoint.type == Particle::None) {
        continue;
      }

      const Vec3f d_pos = nextPoint.position;
      const Vec3f b_pos = adjacentPoint.position;

      Vec3f x_pos = (a_pos + b_pos + c_pos + d_pos) * 0.25f;

      Vec3f a_normal = computeGouraudNormal(i, j, diagonal);
      Vec3f b_normal = computeGouraudNormal(i, j + diagonal, diagonal);
      Vec3f c_normal =
          computeGouraudNormal(i + diagonal, j + diagonal, diagonal);
      Vec3f d_normal = computeGouraudNormal(i + diagonal, j, diagonal);
      if (!mesh_data->gouraud) {
        // compute normals at each corner and average them
        Vec3f top = b_pos - a_pos;
        Vec3f bottom = c_pos - d_pos;
        Vec3f horiz = (top + bottom);
        horiz.Normalize();
        Vec3f left = d_pos - a_pos;
        Vec3f right = c_pos - b_pos;
        Vec3f vert = (left + right);
        vert.Normalize();
        Vec3f normal;
        Vec3f::Cross3(normal, horiz, vert);
        normal.Normalize();
        a_normal = b_normal = c_normal = d_normal = normal;
      }

      Vec3f x_normal = (a_normal + b_normal + c_normal + d_normal);
      x_normal.Normalize();

      AddWireFrameTriangle(current, a_pos, b_pos, x_pos, a_normal, b_normal,
                           x_normal, Vec3f::zero(), Vec3f::zero(),
                           Vec3f::zero());
      AddWireFrameTriangle(current, b_pos, c_pos, x_pos, b_normal, c_normal,
                           x_normal, Vec3f::zero(), Vec3f::zero(),
                           Vec3f::zero());
      AddWireFrameTriangle(current, c_pos, d_pos, x_pos, c_normal, d_normal,
                           x_normal, Vec3f::zero(), Vec3f::zero(),
                           Vec3f::zero());
      AddWireFrameTriangle(current, d_pos, a_pos, x_pos, d_normal, a_normal,
                           x_normal, Vec3f::zero(), Vec3f::zero(),
                           Vec3f::zero());
      j += diagonal - 1;
    }
  }
}

void Cloth::PackClothVelocities(float*& current) {
  float thickness = 0.005 * mesh_data->bb_max_dim;
  float dt = mesh_data->timestep;

  // velocity & force visualization
  for (int i = 0; i < nx; i++) {
    for (int j = 0; j < ny; j++) {
      const optional<ClothParticle>& p = getParticle(i, j);
      if (p.has_value()) {
        const Vec3f& pos = p->position;
        const Vec3f& vel = p->velocity;
        addEdgeGeometry(current, pos, pos + dt * 100 * vel, Vec3f(1, 0, 0),
                        Vec3f(1, 1, 0), thickness, thickness);
      }
    }
  }
}
// ================================================================================
// a helper functions
// ================================================================================

Vec3f Cloth::computeGouraudNormal(int i, int j, int distance) const {
  assert(i >= 0 && i < nx && j >= 0 && j < ny);

  Vec3f pos = getParticle(i, j).position;
  Vec3f north = pos;
  Vec3f south = pos;
  Vec3f east = pos;
  Vec3f west = pos;

  if (i - distance >= 0) {
    if (getParticle(i - distance, j).type != Particle::None) {
      north = getParticle(i - distance, j).position;
    }
  }
  if (i + distance < nx) {
    if (getParticle(i + distance, j).type != Particle::None) {
      south = getParticle(i + distance, j).position;
    }
  }
  if (j - distance >= 0) {
    if (getParticle(i, j - distance).type != Particle::None) {
      east = getParticle(i, j - distance).position;
    }
  }
  if (j + distance < ny) {
    if (getParticle(i, j + distance).type != Particle::None) {
      west = getParticle(i, j + distance).position;
    }
  }

  Vec3f vns = north - south;
  Vec3f vwe = west - east;
  vns.Normalize();
  vwe.Normalize();

  // compute normals at each corner and average them
  Vec3f normal;
  Vec3f::Cross3(normal, vns, vwe);
  normal.Normalize();
  return normal;
}

// ================================================================================

void Cloth::DebugPrintCloth() const {
  for (int i = 0; i < nx; i++) {
    for (int j = 0; j < ny; j++) {
      switch (particles[i][j].type) {
        case Particle::Fixed:
          std::cout << "F" << particles[i][j].layer;
          break;
        case Particle::Active:
          std::cout << "A" << particles[i][j].layer;
          break;
        case Particle::None:
          std::cout << "  ";
          break;
        case Particle::Interp:
          std::cout << "I" << particles[i][j].layer;
          break;
      }
      std::cout << " ";
    }
    std::cout << std::endl;
  }
}
