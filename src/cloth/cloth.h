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

typedef struct Pos {
  int x;
  int y;
  Pos(int _x, int _y) : x(_x), y(_y) {}
  Pos() {}
  bool operator==(const Pos& p) const { return x == p.x && y == p.y; }
  void operator*=(const int m) {
    x *= m;
    y *= m;
  }
  bool operator<(const Pos& p) const {
    if (x != p.x) {
      return x < p.x;
    }
    return y < p.y;
  }
} Pos;
using std::hash;
template <>
struct std::hash<Pos> {
  size_t operator()(const Pos& p) const {
    return (hash<int>()(p.x) << 16) ^ hash<int>()(p.y);
  }
};

typedef struct PosPair {
  Pos a;
  Pos b;
  int layer;
  PosPair(Pos _a, Pos _b, int _layer) : a(_a), b(_b), layer(_layer) {
    if (b < a) {
      std::swap(a, b);
    }
  }
  PosPair(int x1, int y1, int x2, int y2, int _layer)
      : a(Pos(x1, y1)), b(Pos(x2, y2)), layer(_layer) {
    if (b < a) {
      std::swap(a, b);
    }
  }
  bool operator==(const PosPair& pp) const {
    return layer == pp.layer && pp.a == a && pp.b == b;
  }
  void operator*=(int m) {
    a.x *= m;
    a.y *= m;
    b.x *= m;
    b.y *= m;
  }
  friend PosPair operator*(const PosPair& p, const int m) {
    PosPair out = p;
    out *= m;
    return out;
  }
} PosPair;
template <>
struct std::hash<PosPair> {
  //making sure that pospair hashes to the same value no matter the order
  size_t operator()(const PosPair& pp) const {
    return ((hash<Pos>()(pp.a) << 1 ^ hash<Pos>()(pp.b)) >> 1) ^
           (hash<int>()(pp.layer + 1) << 16);
  }
};

class Cloth {
 private:
  void DebugPrintCloth() const;
  void AdjustInterpolated();
  void IncreaseClothDensity();

  void SubdivideAboutPoint(int i, int j);
  void AddSubdividedParticles(int i, int j, int distance);
  void AddInterpolatedParticles(int i, int j, int distance);
  void RegenerateSprings(int i, int j, int distance);

  void  performTimestepSimulation(int t);
  void  updateForces(int t, vector<vector<Vec3f>>& forces);
  void  updateVelocities(int t, const vector<vector<Vec3f>>& forces);
  void  updatePositions();
  void  correctPositions();
  Vec3f reduceForceOverOffsets(
      int i, int j, const vector<std::pair<int, int>>& offsets) const;

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
  double correction;
};

// ========================================================================

#endif
