#ifndef _CLOTH_H_
#define _CLOTH_H_

#include <vector>

#include "../args/argparser.h"
#include "../libs/boundingbox.h"
#include "../libs/vectors.h"

// =====================================================================================
// Cloth Particles
// =====================================================================================

class ClothParticle {
 public:
  // ACCESSORS
  const Vec3f& getOriginalPosition() const { return original_position; }
  const Vec3f& getPosition() const { return position; }
  const Vec3f& getVelocity() const { return velocity; }
  const Vec3f& getAcceleration() const { return acceleration; }
  Vec3f getForce() const { return float(mass) * acceleration; }
  double getMass() const { return mass; }
  bool isFixed() const { return fixed; }
  // MODIFIERS
  void setOriginalPosition(const Vec3f& p) { original_position = p; }
  void setPosition(const Vec3f& p) { position = p; }
  void setVelocity(const Vec3f& v) { velocity = v; }
  void setAcceleration(const Vec3f& a) { acceleration = a; }
  void setMass(double m) { mass = m; }
  void setFixed(bool b) { fixed = b; }

 private:
  // REPRESENTATION
  Vec3f original_position;
  Vec3f position;
  Vec3f velocity;
  Vec3f acceleration;
  double mass;
  bool fixed;
};

// =====================================================================================
// Cloth System
// =====================================================================================

class Cloth {

 public:
  Cloth(ArgParser* args);
  ~Cloth() { delete[] particles; }

  // ACCESSORS
  const BoundingBox& getBoundingBox() const { return box; }

  // PAINTING & ANIMATING
  void PackMesh();
  void PackClothSurface(float*& current);
  void PackClothVelocities(float*& current);
  void PackClothForces(float*& current);
  void Animate();

 private:
  // PRIVATE ACCESSORS
  const ClothParticle& getParticle(int i, int j) const {
    assert(i >= 0 && i < nx && j >= 0 && j < ny);
    return particles[i + j * nx];
  }
  ClothParticle& getParticle(int i, int j) {
    assert(i >= 0 && i < nx && j >= 0 && j < ny);
    return particles[i + j * nx];
  }

  Vec3f computeGouraudNormal(int i, int j) const;

  // HELPER FUNCTION
  void computeBoundingBox();

  Vec3f reduceForcePositionsWithConstant(const double k,
                                         const std::pair<int, int> offsets[4],
                                         const int i, const int j);
  void correctOffsetParticlesWithConstant(const double k,
                                          const std::pair<int, int> offsets[4],
                                          const int i, const int j);

  // HELPER FUNCTIONS FOR ANIMATION
  void AddWireFrameTriangle(float*& current, const Vec3f& apos,
                            const Vec3f& bpos, const Vec3f& cpos,
                            const Vec3f& anormal, const Vec3f& bnormal,
                            const Vec3f& cnormal, const Vec3f& abcolor,
                            const Vec3f& bccolor, const Vec3f& cacolor);

  // REPRESENTATION
  ArgParser* args;
  // grid data structure
  int nx, ny;
  ClothParticle* particles;
  BoundingBox box;
  // simulation parameters
  double damping;
  // spring constants
  double k_structural;
  double k_shear;
  double k_bend;
  // correction thresholds
  double provot_structural_correction;
  double provot_shear_correction;
};

// ========================================================================

#endif
