#ifndef _CLOTH_H_
#define _CLOTH_H_

#include <optional>
#include <unordered_map>
#include <vector>

#include "../args/argparser.h"
#include "../libs/boundingbox.h"
#include "../libs/vectors.h"

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
  Vec3f    velocity;
  ClothParticle() : type(Particle::None) {}
};

// =====================================================================================
// Cloth System
// =====================================================================================

typedef struct Pos {
  int x;
  int y;
  Pos(int _x, int _y) : x(_x), y(_y) {}
  bool operator==(const Pos& p) const { return x == p.x && y == p.y; }
} Pos;
using std::hash;
template <>
struct std::hash<Pos> {
  size_t operator()(const Pos& p) const {
    return hash<int>{}(p.x) ^ (hash<int>{}(p.y) << 1);
  }
};

typedef struct PosPair {
  Pos a;
  Pos b;
  PosPair(Pos _a, Pos _b) : a(_a), b(_b) {}
  PosPair(int x1, int y1, int x2, int y2) : a(Pos(x1, y1)), b(Pos(x2, y2)) {}
  bool operator==(const PosPair& pp) const {
    return (pp.a == a && pp.b == b) || (pp.a == b && pp.b == a);
  }
} PosPair;
template <>
struct std::hash<PosPair> {
  //making sure that pospair hashes to the same value no matter the order
  size_t operator()(const PosPair& pp) const {
    return hash<Pos>{}(pp.a) ^ hash<Pos>{}(pp.b);
  }
};

class Cloth {

 public:
  Cloth(ArgParser* args);

  // ACCESSORS
  const BoundingBox& getBoundingBox() const { return box; }

  // PAINTING & ANIMATING
  void PackMesh();
  void PackClothSurface(float*& current);
  void PackClothVelocities(float*& current);
  void Animate();
  std::vector<std::vector<bool>> getShouldSubdivide(float threshold);
  void subdivide();
  void interpolate();

 private:
  // PRIVATE ACCESSORS
  const ClothParticle& getParticle(int i, int j) const {
    // TODO: update for data structure
    return particles[i][j];
  }
  bool hasParticle(int i, int j) const {
    return particles[i][j].type != Particle::None;
  }

  Vec3f computeGouraudNormal(int i, int j) const;

  // HELPER FUNCTION
  void computeBoundingBox();

  Vec3f reduceForcePositionsWithConstant(const double              k,
                                         const std::pair<int, int> offsets[4],
                                         const int i, const int j);
  void  correctOffsetParticlesWithConstant(const double              k,
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
  int                                 initialX;
  int                                 initialY;
  int                                 nx;
  int                                 ny;
  int                                 maximumSubdivision;
  vector<vector<ClothParticle>>       particles;
  std::unordered_map<PosPair, double> springs;
  BoundingBox                         box;
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
