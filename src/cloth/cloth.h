#ifndef _CLOTH_H_
#define _CLOTH_H_

#include <optional>
#include <vector>

#include "../args/argparser.h"
#include "../libs/boundingbox.h"
#include "../libs/vectors.h"
#include "offsets.h"

// =====================================================================================
// Cloth Particles
// =====================================================================================

using std::optional;
using std::vector;

enum class Particle {
  Fixed,
  Active,
  None,
  Interp,
};

class ClothParticle {
 public:
  Particle type;
  int      layer;
  double   mass;
  Vec3f    position;
  Vec3f    originalPosition;
  Vec3f    velocity;

  void inPlaceInterp(const ClothParticle& a, const ClothParticle& b) {
    position = Vec3f::interp(a.position, b.position);
    velocity = Vec3f::interp(a.velocity, b.velocity);
    originalPosition = Vec3f::interp(a.originalPosition, b.originalPosition);
  }

  ClothParticle() : type(Particle::None) {}
  static ClothParticle none() { return ClothParticle(); }
  static ClothParticle interp(const ClothParticle& a, const ClothParticle& b) {
    ClothParticle p;
    p.type = Particle::Interp;
    p.mass = a.mass + b.mass / 2;
    p.inPlaceInterp(a, b);
    return p;
  }
  static ClothParticle interpActive(const ClothParticle& a,
                                    const ClothParticle& b) {
    ClothParticle p;
    p.type = Particle::Active;
    p.mass = a.mass + b.mass / 2;
    p.inPlaceInterp(a, b);
    return p;
  }
};

// =====================================================================================
// Cloth System
// =====================================================================================
class Cloth {
 private:
  void DebugPrintCloth() const;
  void AdjustInterpolated();
  void IncreaseClothDensity();

  void SubdivideAboutPoint(int i, int j);
  void AddSubdividedParticles(int i, int j, int distance);
  void AddInterpolatedParticles(int i, int j, int distance);

  void  performTimestepSimulation(int t);
  void  updateForces(int t, vector<vector<Vec3f>>& forces);
  void  updateVelocities(int t, const vector<vector<Vec3f>>& forces);
  void  updatePositions();
  void  correctPositions();
  void  correctParticle(int i, int j, const Offset::Vec& offsets, double k);
  Vec3f reduceForceOverOffsets(int i, int j, const Offset::Vec& offsets,
                               double k) const;

  inline int scale(int layer) const {
    return 1 << (maximumSubdivision - layer);
  }
  inline bool inBounds(int i, int j) const {
    return i >= 0 && i < nx && j >= 0 && j < ny;
  }

 public:
  Cloth(ArgParser* args);

  // ACCESSORS
  const BoundingBox& getBoundingBox() const { return box; }

  // PAINTING & ANIMATING
  void PackMesh();
  void PackClothSurface(float*& current);
  void PackClothVelocities(float*& current);
  void Animate();

 private:
  // PRIVATE ACCESSORS
  const ClothParticle& getParticle(int i, int j) const {
    // TODO: update for data structure
    return particles[i][j];
  }
  bool hasParticle(int i, int j) const {
    return particles[i][j].type != Particle::None;
  }

  Vec3f computeGouraudNormal(int i, int j, int distance) const;

  // HELPER FUNCTION
  void computeBoundingBox();

  // HELPER FUNCTIONS FOR ANIMATION
  void AddWireFrameTriangle(float*& current, const Vec3f& apos,
                            const Vec3f& bpos, const Vec3f& cpos,
                            const Vec3f& anormal, const Vec3f& bnormal,
                            const Vec3f& cnormal, const Vec3f& abcolor,
                            const Vec3f& bccolor, const Vec3f& cacolor);

  // REPRESENTATION
  ArgParser* args;
  // grid data structure
  int                           initialX;
  int                           initialY;
  int                           nx;
  int                           ny;
  int                           maximumSubdivision;
  vector<vector<ClothParticle>> particles;
  BoundingBox                   box;
  // simulation parameters
  double damping;
  // spring constants
  double k_structural;
  double k_shear;
  double k_bend;
  // correction thresholds
  double correction;
};

// ========================================================================

#endif
