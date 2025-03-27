#include <cstring>
#include <fstream>
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

Vec3f super_elastic_color(const ClothParticle& a, const ClothParticle& b,
                          double correction) {

  Vec3f a_o, b_o, a_c, b_c;
  a_c = a.getPosition();
  b_c = b.getPosition();
  a_o = a.getOriginalPosition();
  b_o = b.getOriginalPosition();
  double length_o, length;
  length = (a_c - b_c).Length();
  length_o = (a_o - b_o).Length();

  if (length >= (1 + 0.99 * correction) * length_o) {
    // spring is too long, make it cyan
    return Vec3f(0, 1, 1);
  } else if (length <= (1 - 0.99 * correction) * length_o) {
    // spring is too short, make it yellow
    return Vec3f(1, 1, 0);
  } else {
    return Vec3f(0, 0, 0);
  }
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
  if (mesh_data->force) {
    PackClothForces(current);
  }
  if (mesh_data->bounding_box) {
    PackBoundingBox(current, getBoundingBox());
  }
}

void Cloth::PackClothSurface(float*& current) {
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

      const ClothParticle& a = getParticle(i, j);
      const ClothParticle& b = getParticle(i, j + 1);
      const ClothParticle& c = getParticle(i + 1, j + 1);
      const ClothParticle& d = getParticle(i + 1, j);

      const Vec3f& a_pos = a.getPosition();
      const Vec3f& b_pos = b.getPosition();
      const Vec3f& c_pos = c.getPosition();
      const Vec3f& d_pos = d.getPosition();

      Vec3f x_pos = (a_pos + b_pos + c_pos + d_pos) * 0.25f;

      Vec3f a_normal = computeGouraudNormal(i, j);
      Vec3f b_normal = computeGouraudNormal(i, j + 1);
      Vec3f c_normal = computeGouraudNormal(i + 1, j + 1);
      Vec3f d_normal = computeGouraudNormal(i + 1, j);
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

      Vec3f ab_color = super_elastic_color(a, b, provot_structural_correction);
      Vec3f bc_color = super_elastic_color(b, c, provot_structural_correction);
      Vec3f cd_color = super_elastic_color(c, d, provot_structural_correction);
      Vec3f da_color = super_elastic_color(d, a, provot_structural_correction);

      Vec3f ac_color = super_elastic_color(a, c, provot_shear_correction);
      Vec3f bd_color = super_elastic_color(b, d, provot_shear_correction);

      AddWireFrameTriangle(current, a_pos, b_pos, x_pos, a_normal, b_normal,
                           x_normal, ab_color, bd_color, ac_color);
      AddWireFrameTriangle(current, b_pos, c_pos, x_pos, b_normal, c_normal,
                           x_normal, bc_color, ac_color, bd_color);
      AddWireFrameTriangle(current, c_pos, d_pos, x_pos, c_normal, d_normal,
                           x_normal, cd_color, bd_color, ac_color);
      AddWireFrameTriangle(current, d_pos, a_pos, x_pos, d_normal, a_normal,
                           x_normal, da_color, ac_color, bd_color);
    }
  }
}

void Cloth::PackClothVelocities(float*& current) {

  float thickness = 0.005 * mesh_data->bb_max_dim;
  float dt = mesh_data->timestep;

  // velocity & force visualization
  for (int i = 0; i < nx; i++) {
    for (int j = 0; j < ny; j++) {
      const ClothParticle& p = getParticle(i, j);
      const Vec3f& pos = p.getPosition();
      const Vec3f& vel = p.getVelocity();

      addEdgeGeometry(current, pos, pos + dt * 100 * vel, Vec3f(1, 0, 0),
                      Vec3f(1, 1, 0), thickness, thickness);
    }
  }
}

void Cloth::PackClothForces(float*& current) {
  const float dt = mesh_data->timestep;
  const float thickness = 0.005 * mesh_data->bb_max_dim;

  for (int i = 0; i < nx; i++) {
    for (int j = 0; j < ny; j++) {
      const ClothParticle particle = getParticle(i, j);
      const Vec3f& force = particle.getForce();
      const Vec3f& pos = particle.getPosition();
      addEdgeGeometry(current, pos, pos + dt * 100 * force, Vec3f(0, 0, 1),
                      Vec3f(0, 0, 1), thickness, thickness);
    }
  }

  // *********************************************************************
  // ASSIGNMENT:
  //
  // Visualize the forces
  //
  // *********************************************************************
}

// ================================================================================
// a helper functions
// ================================================================================

Vec3f Cloth::computeGouraudNormal(int i, int j) const {
  assert(i >= 0 && i < nx && j >= 0 && j < ny);

  Vec3f pos = getParticle(i, j).getPosition();
  Vec3f north = pos;
  Vec3f south = pos;
  Vec3f east = pos;
  Vec3f west = pos;

  if (i - 1 >= 0)
    north = getParticle(i - 1, j).getPosition();
  if (i + 1 < nx)
    south = getParticle(i + 1, j).getPosition();
  if (j - 1 >= 0)
    east = getParticle(i, j - 1).getPosition();
  if (j + 1 < ny)
    west = getParticle(i, j + 1).getPosition();

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
