#include "utils.h"
#include <cstring>
#include <iostream>
#include "../render/meshdata.h"

// since glLineWidth is gone...
// instead we'll draw a rectangular box
// (should probably use a geometry shader instead)
void addEdgeGeometry(float*& current, const Vec3f& a, const Vec3f& b,
                     const Vec3f& acolor, const Vec3f& bcolor, float a_th,
                     float b_th) {

  // find perpendicular axes
  Vec3f dir = (b - a);
  Vec3f one;
  Vec3f two;
  if (std::min(a_th, b_th) < 0.0000001 ||
      dir.Length() < 0.01 * std::min(a_th, b_th)) {
    dir = one = two = Vec3f(0, 0, 0);
  } else {
    dir.Normalize();
    Vec3f tmp;
    Vec3f::Cross3(tmp, dir, Vec3f(1, 0, 0));
    if (tmp.Length() < 0.1) {
      Vec3f::Cross3(tmp, dir, Vec3f(0, 0, 1));
    }
    tmp.Normalize();
    Vec3f::Cross3(one, dir, tmp);
    assert(fabs(one.Length() - 1.0) < 0.001);
    Vec3f::Cross3(two, dir, one);
    assert(fabs(two.Length() - 1.0) < 0.001);
  }

  Vec3f a1 = a - one * a_th - two * a_th;
  Vec3f a2 = a - one * a_th + two * a_th;
  Vec3f a3 = a + one * a_th + two * a_th;
  Vec3f a4 = a + one * a_th - two * a_th;

  Vec3f b1 = b - one * b_th - two * b_th;
  Vec3f b2 = b - one * b_th + two * b_th;
  Vec3f b3 = b + one * b_th + two * b_th;
  Vec3f b4 = b + one * b_th - two * b_th;

  float12 ta, tb, tc;

  // draw 6 sides of the box
  ta = {float(a1.x()),     float(a1.y()),     float(a1.z()),     1,
        float(-one.x()),   float(-one.y()),   float(-one.z()),   0,
        float(acolor.r()), float(acolor.g()), float(acolor.b()), 1};
  tb = {float(b2.x()),     float(b2.y()),     float(b2.z()),     1,
        float(-one.x()),   float(-one.y()),   float(-one.z()),   0,
        float(bcolor.r()), float(bcolor.g()), float(bcolor.b()), 1};
  tc = {float(a2.x()),     float(a2.y()),     float(a2.z()),     1,
        float(-one.x()),   float(-one.y()),   float(-one.z()),   0,
        float(acolor.r()), float(acolor.g()), float(acolor.b()), 1};
  memcpy(current, &ta, sizeof(float) * 12);
  current += 12;
  memcpy(current, &tb, sizeof(float) * 12);
  current += 12;
  memcpy(current, &tc, sizeof(float) * 12);
  current += 12;
  ta = {float(a1.x()),     float(a1.y()),     float(a1.z()),     1,
        float(-one.x()),   float(-one.y()),   float(-one.z()),   0,
        float(acolor.r()), float(acolor.g()), float(acolor.b()), 1};
  tb = {float(b1.x()),     float(b1.y()),     float(b1.z()),     1,
        float(-one.x()),   float(-one.y()),   float(-one.z()),   0,
        float(bcolor.r()), float(bcolor.g()), float(bcolor.b()), 1};
  tc = {float(b2.x()),     float(b2.y()),     float(b2.z()),     1,
        float(-one.x()),   float(-one.y()),   float(-one.z()),   0,
        float(bcolor.r()), float(bcolor.g()), float(bcolor.b()), 1};
  memcpy(current, &ta, sizeof(float) * 12);
  current += 12;
  memcpy(current, &tb, sizeof(float) * 12);
  current += 12;
  memcpy(current, &tc, sizeof(float) * 12);
  current += 12;

  ta = {float(a2.x()),     float(a2.y()),     float(a2.z()),     1,
        float(two.x()),    float(two.y()),    float(two.z()),    0,
        float(acolor.r()), float(acolor.g()), float(acolor.b()), 1};
  tb = {float(b3.x()),     float(b3.y()),     float(b3.z()),     1,
        float(two.x()),    float(two.y()),    float(two.z()),    0,
        float(bcolor.r()), float(bcolor.g()), float(bcolor.b()), 1};
  tc = {float(a3.x()),     float(a3.y()),     float(a3.z()),     1,
        float(two.x()),    float(two.y()),    float(two.z()),    0,
        float(acolor.r()), float(acolor.g()), float(acolor.b()), 1};
  memcpy(current, &ta, sizeof(float) * 12);
  current += 12;
  memcpy(current, &tb, sizeof(float) * 12);
  current += 12;
  memcpy(current, &tc, sizeof(float) * 12);
  current += 12;
  ta = {float(a2.x()),     float(a2.y()),     float(a2.z()),     1,
        float(two.x()),    float(two.y()),    float(two.z()),    0,
        float(acolor.r()), float(acolor.g()), float(acolor.b()), 1};
  tb = {float(b2.x()),     float(b2.y()),     float(b2.z()),     1,
        float(two.x()),    float(two.y()),    float(two.z()),    0,
        float(bcolor.r()), float(bcolor.g()), float(bcolor.b()), 1};
  tc = {float(b3.x()),     float(b3.y()),     float(b3.z()),     1,
        float(two.x()),    float(two.y()),    float(two.z()),    0,
        float(bcolor.r()), float(bcolor.g()), float(bcolor.b()), 1};
  memcpy(current, &ta, sizeof(float) * 12);
  current += 12;
  memcpy(current, &tb, sizeof(float) * 12);
  current += 12;
  memcpy(current, &tc, sizeof(float) * 12);
  current += 12;

  ta = {float(a3.x()),     float(a3.y()),     float(a3.z()),     1,
        float(one.x()),    float(one.y()),    float(one.z()),    0,
        float(acolor.r()), float(acolor.g()), float(acolor.b()), 1};
  tb = {float(b4.x()),     float(b4.y()),     float(b4.z()),     1,
        float(one.x()),    float(one.y()),    float(one.z()),    0,
        float(bcolor.r()), float(bcolor.g()), float(bcolor.b()), 1};
  tc = {float(a4.x()),     float(a4.y()),     float(a4.z()),     1,
        float(one.x()),    float(one.y()),    float(one.z()),    0,
        float(acolor.r()), float(acolor.g()), float(acolor.b()), 1};
  memcpy(current, &ta, sizeof(float) * 12);
  current += 12;
  memcpy(current, &tb, sizeof(float) * 12);
  current += 12;
  memcpy(current, &tc, sizeof(float) * 12);
  current += 12;
  ta = {float(a3.x()),     float(a3.y()),     float(a3.z()),     1,
        float(one.x()),    float(one.y()),    float(one.z()),    0,
        float(acolor.r()), float(acolor.g()), float(acolor.b()), 1};
  tb = {float(b3.x()),     float(b3.y()),     float(b3.z()),     1,
        float(one.x()),    float(one.y()),    float(one.z()),    0,
        float(bcolor.r()), float(bcolor.g()), float(bcolor.b()), 1};
  tc = {float(b4.x()),     float(b4.y()),     float(b4.z()),     1,
        float(one.x()),    float(one.y()),    float(one.z()),    0,
        float(bcolor.r()), float(bcolor.g()), float(bcolor.b()), 1};
  memcpy(current, &ta, sizeof(float) * 12);
  current += 12;
  memcpy(current, &tb, sizeof(float) * 12);
  current += 12;
  memcpy(current, &tc, sizeof(float) * 12);
  current += 12;

  ta = {float(a4.x()),     float(a4.y()),     float(a4.z()),     1,
        float(-two.x()),   float(-two.y()),   float(-two.z()),   0,
        float(acolor.r()), float(acolor.g()), float(acolor.b()), 1};
  tb = {float(b1.x()),     float(b1.y()),     float(b1.z()),     1,
        float(-two.x()),   float(-two.y()),   float(-two.z()),   0,
        float(bcolor.r()), float(bcolor.g()), float(bcolor.b()), 1};
  tc = {float(a1.x()),     float(a1.y()),     float(a1.z()),     1,
        float(-two.x()),   float(-two.y()),   float(-two.z()),   0,
        float(acolor.r()), float(acolor.g()), float(acolor.b()), 1};
  memcpy(current, &ta, sizeof(float) * 12);
  current += 12;
  memcpy(current, &tb, sizeof(float) * 12);
  current += 12;
  memcpy(current, &tc, sizeof(float) * 12);
  current += 12;
  ta = {float(a4.x()),     float(a4.y()),     float(a4.z()),     1,
        float(-two.x()),   float(-two.y()),   float(-two.z()),   0,
        float(acolor.r()), float(acolor.g()), float(acolor.b()), 1};
  tb = {float(b4.x()),     float(b4.y()),     float(b4.z()),     1,
        float(-two.x()),   float(-two.y()),   float(-two.z()),   0,
        float(bcolor.r()), float(bcolor.g()), float(bcolor.b()), 1};
  tc = {float(b1.x()),     float(b1.y()),     float(b1.z()),     1,
        float(-two.x()),   float(-two.y()),   float(-two.z()),   0,
        float(bcolor.r()), float(bcolor.g()), float(bcolor.b()), 1};
  memcpy(current, &ta, sizeof(float) * 12);
  current += 12;
  memcpy(current, &tb, sizeof(float) * 12);
  current += 12;
  memcpy(current, &tc, sizeof(float) * 12);
  current += 12;

  //top
  ta = {float(a1.x()),     float(a1.y()),     float(a1.z()),     1,
        float(dir.x()),    float(dir.y()),    float(dir.z()),    0,
        float(acolor.r()), float(acolor.g()), float(acolor.b()), 1};
  tb = {float(a2.x()),     float(a2.y()),     float(a2.z()),     1,
        float(dir.x()),    float(dir.y()),    float(dir.z()),    0,
        float(acolor.r()), float(acolor.g()), float(acolor.b()), 1};
  tc = {float(a3.x()),     float(a3.y()),     float(a3.z()),     1,
        float(dir.x()),    float(dir.y()),    float(dir.z()),    0,
        float(acolor.r()), float(acolor.g()), float(acolor.b()), 1};
  memcpy(current, &ta, sizeof(float) * 12);
  current += 12;
  memcpy(current, &tb, sizeof(float) * 12);
  current += 12;
  memcpy(current, &tc, sizeof(float) * 12);
  current += 12;
  ta = {float(a1.x()),     float(a1.y()),     float(a1.z()),     1,
        float(dir.x()),    float(dir.y()),    float(dir.z()),    0,
        float(acolor.r()), float(acolor.g()), float(acolor.b()), 1};
  tb = {float(a3.x()),     float(a3.y()),     float(a3.z()),     1,
        float(dir.x()),    float(dir.y()),    float(dir.z()),    0,
        float(acolor.r()), float(acolor.g()), float(acolor.b()), 1};
  tc = {float(a4.x()),     float(a4.y()),     float(a4.z()),     1,
        float(dir.x()),    float(dir.y()),    float(dir.z()),    0,
        float(acolor.r()), float(acolor.g()), float(acolor.b()), 1};
  memcpy(current, &ta, sizeof(float) * 12);
  current += 12;
  memcpy(current, &tb, sizeof(float) * 12);
  current += 12;
  memcpy(current, &tc, sizeof(float) * 12);
  current += 12;

  //bottom
  ta = {float(b1.x()),     float(b1.y()),     float(b1.z()),     1,
        float(-dir.x()),   float(-dir.y()),   float(-dir.z()),   0,
        float(bcolor.r()), float(bcolor.g()), float(bcolor.b()), 1};
  tb = {float(b3.x()),     float(b3.y()),     float(b3.z()),     1,
        float(-dir.x()),   float(-dir.y()),   float(-dir.z()),   0,
        float(bcolor.r()), float(bcolor.g()), float(bcolor.b()), 1};
  tc = {float(b2.x()),     float(b2.y()),     float(b2.z()),     1,
        float(-dir.x()),   float(-dir.y()),   float(-dir.z()),   0,
        float(bcolor.r()), float(bcolor.g()), float(bcolor.b()), 1};
  memcpy(current, &ta, sizeof(float) * 12);
  current += 12;
  memcpy(current, &tb, sizeof(float) * 12);
  current += 12;
  memcpy(current, &tc, sizeof(float) * 12);
  current += 12;
  ta = {float(b1.x()),     float(b1.y()),     float(b1.z()),     1,
        float(-dir.x()),   float(-dir.y()),   float(-dir.z()),   0,
        float(acolor.r()), float(bcolor.g()), float(bcolor.b()), 1};
  tb = {float(b4.x()),     float(b4.y()),     float(b4.z()),     1,
        float(-dir.x()),   float(-dir.y()),   float(-dir.z()),   0,
        float(acolor.r()), float(bcolor.g()), float(bcolor.b()), 1};
  tc = {float(b3.x()),     float(b3.y()),     float(b3.z()),     1,
        float(-dir.x()),   float(-dir.y()),   float(-dir.z()),   0,
        float(acolor.r()), float(bcolor.g()), float(bcolor.b()), 1};
  memcpy(current, &ta, sizeof(float) * 12);
  current += 12;
  memcpy(current, &tb, sizeof(float) * 12);
  current += 12;
  memcpy(current, &tc, sizeof(float) * 12);
  current += 12;
}

void addCubeGeometry(float*& current, Vec3f pt[8], const Vec3f& color) {

  float12 ta, tb, tc;

  // left
  ta = {float(pt[0].x()), float(pt[0].y()), float(pt[0].z()), 1, -1, 0, 0, 0,
        float(color.r()), float(color.g()), float(color.b()), 1};
  tb = {float(pt[1].x()), float(pt[1].y()), float(pt[1].z()), 1, -1, 0, 0, 0,
        float(color.r()), float(color.g()), float(color.b()), 1};
  tc = {float(pt[2].x()), float(pt[2].y()), float(pt[2].z()), 1, -1, 0, 0, 0,
        float(color.r()), float(color.g()), float(color.b()), 1};
  memcpy(current, &ta, sizeof(float) * 12);
  current += 12;
  memcpy(current, &tb, sizeof(float) * 12);
  current += 12;
  memcpy(current, &tc, sizeof(float) * 12);
  current += 12;
  ta = {float(pt[1].x()), float(pt[1].y()), float(pt[1].z()), 1, -1, 0, 0, 0,
        float(color.r()), float(color.g()), float(color.b()), 1};
  tb = {float(pt[3].x()), float(pt[3].y()), float(pt[3].z()), 1, -1, 0, 0, 0,
        float(color.r()), float(color.g()), float(color.b()), 1};
  tc = {float(pt[2].x()), float(pt[2].y()), float(pt[2].z()), 1, -1, 0, 0, 0,
        float(color.r()), float(color.g()), float(color.b()), 1};
  memcpy(current, &ta, sizeof(float) * 12);
  current += 12;
  memcpy(current, &tb, sizeof(float) * 12);
  current += 12;
  memcpy(current, &tc, sizeof(float) * 12);
  current += 12;

  // right
  ta = {float(pt[4].x()), float(pt[4].y()), float(pt[4].z()), 1, 0, 1, 0, 0,
        float(color.r()), float(color.g()), float(color.b()), 1};
  tb = {float(pt[6].x()), float(pt[6].y()), float(pt[6].z()), 1, 0, 1, 0, 0,
        float(color.r()), float(color.g()), float(color.b()), 1};
  tc = {float(pt[5].x()), float(pt[5].y()), float(pt[5].z()), 1, 0, 1, 0, 0,
        float(color.r()), float(color.g()), float(color.b()), 1};
  memcpy(current, &ta, sizeof(float) * 12);
  current += 12;
  memcpy(current, &tb, sizeof(float) * 12);
  current += 12;
  memcpy(current, &tc, sizeof(float) * 12);
  current += 12;
  ta = {float(pt[5].x()), float(pt[5].y()), float(pt[5].z()), 1, 0, 1, 0, 0,
        float(color.r()), float(color.g()), float(color.b()), 1};
  tb = {float(pt[6].x()), float(pt[6].y()), float(pt[6].z()), 1, 0, 1, 0, 0,
        float(color.r()), float(color.g()), float(color.b()), 1};
  tc = {float(pt[7].x()), float(pt[7].y()), float(pt[7].z()), 1, 0, 1, 0, 0,
        float(color.r()), float(color.g()), float(color.b()), 1};
  memcpy(current, &ta, sizeof(float) * 12);
  current += 12;
  memcpy(current, &tb, sizeof(float) * 12);
  current += 12;
  memcpy(current, &tc, sizeof(float) * 12);
  current += 12;

  // bottom
  ta = {float(pt[0].x()), float(pt[0].y()), float(pt[0].z()), 1, 0, -1, 0, 0,
        float(color.r()), float(color.g()), float(color.b()), 1};
  tb = {float(pt[4].x()), float(pt[4].y()), float(pt[4].z()), 1, 0, -1, 0, 0,
        float(color.r()), float(color.g()), float(color.b()), 1};
  tc = {float(pt[1].x()), float(pt[1].y()), float(pt[1].z()), 1, 0, -1, 0, 0,
        float(color.r()), float(color.g()), float(color.b()), 1};
  memcpy(current, &ta, sizeof(float) * 12);
  current += 12;
  memcpy(current, &tb, sizeof(float) * 12);
  current += 12;
  memcpy(current, &tc, sizeof(float) * 12);
  current += 12;
  ta = {float(pt[1].x()), float(pt[1].y()), float(pt[1].z()), 1, 0, -1, 0, 0,
        float(color.r()), float(color.g()), float(color.b()), 1};
  tb = {float(pt[4].x()), float(pt[4].y()), float(pt[4].z()), 1, 0, -1, 0, 0,
        float(color.r()), float(color.g()), float(color.b()), 1};
  tc = {float(pt[5].x()), float(pt[5].y()), float(pt[5].z()), 1, 0, -1, 0, 0,
        float(color.r()), float(color.g()), float(color.b()), 1};
  memcpy(current, &ta, sizeof(float) * 12);
  current += 12;
  memcpy(current, &tb, sizeof(float) * 12);
  current += 12;
  memcpy(current, &tc, sizeof(float) * 12);
  current += 12;

  // top
  ta = {float(pt[2].x()), float(pt[2].y()), float(pt[2].z()), 1, 0, 1, 0, 0,
        float(color.r()), float(color.g()), float(color.b()), 1};
  tb = {float(pt[3].x()), float(pt[3].y()), float(pt[3].z()), 1, 0, 1, 0, 0,
        float(color.r()), float(color.g()), float(color.b()), 1};
  tc = {float(pt[6].x()), float(pt[6].y()), float(pt[6].z()), 1, 0, 1, 0, 0,
        float(color.r()), float(color.g()), float(color.b()), 1};
  memcpy(current, &ta, sizeof(float) * 12);
  current += 12;
  memcpy(current, &tb, sizeof(float) * 12);
  current += 12;
  memcpy(current, &tc, sizeof(float) * 12);
  current += 12;
  ta = {float(pt[3].x()), float(pt[3].y()), float(pt[3].z()), 1, 0, 1, 0, 0,
        float(color.r()), float(color.g()), float(color.b()), 1};
  tb = {float(pt[7].x()), float(pt[7].y()), float(pt[7].z()), 1, 0, 1, 0, 0,
        float(color.r()), float(color.g()), float(color.b()), 1};
  tc = {float(pt[6].x()), float(pt[6].y()), float(pt[6].z()), 1, 0, 1, 0, 0,
        float(color.r()), float(color.g()), float(color.b()), 1};
  memcpy(current, &ta, sizeof(float) * 12);
  current += 12;
  memcpy(current, &tb, sizeof(float) * 12);
  current += 12;
  memcpy(current, &tc, sizeof(float) * 12);
  current += 12;

  // front
  ta = {float(pt[1].x()), float(pt[1].y()), float(pt[1].z()), 1, 0, 0, 1, 0,
        float(color.r()), float(color.g()), float(color.b()), 1};
  tb = {float(pt[5].x()), float(pt[5].y()), float(pt[5].z()), 1, 0, 0, 1, 0,
        float(color.r()), float(color.g()), float(color.b()), 1};
  tc = {float(pt[3].x()), float(pt[3].y()), float(pt[3].z()), 1, 0, 0, 1, 0,
        float(color.r()), float(color.g()), float(color.b()), 1};
  memcpy(current, &ta, sizeof(float) * 12);
  current += 12;
  memcpy(current, &tb, sizeof(float) * 12);
  current += 12;
  memcpy(current, &tc, sizeof(float) * 12);
  current += 12;
  ta = {float(pt[5].x()), float(pt[5].y()), float(pt[5].z()), 1, 0, 0, 1, 0,
        float(color.r()), float(color.g()), float(color.b()), 1};
  tb = {float(pt[7].x()), float(pt[7].y()), float(pt[7].z()), 1, 0, 0, 1, 0,
        float(color.r()), float(color.g()), float(color.b()), 1};
  tc = {float(pt[3].x()), float(pt[3].y()), float(pt[3].z()), 1, 0, 0, 1, 0,
        float(color.r()), float(color.g()), float(color.b()), 1};
  memcpy(current, &ta, sizeof(float) * 12);
  current += 12;
  memcpy(current, &tb, sizeof(float) * 12);
  current += 12;
  memcpy(current, &tc, sizeof(float) * 12);
  current += 12;

  // back
  ta = {float(pt[0].x()), float(pt[0].y()), float(pt[0].z()), 1, 0, 0, -1, 0,
        float(color.r()), float(color.g()), float(color.b()), 1};
  tb = {float(pt[2].x()), float(pt[2].y()), float(pt[2].z()), 1, 0, 0, -1, 0,
        float(color.r()), float(color.g()), float(color.b()), 1};
  tc = {float(pt[4].x()), float(pt[4].y()), float(pt[4].z()), 1, 0, 0, -1, 0,
        float(color.r()), float(color.g()), float(color.b()), 1};
  memcpy(current, &ta, sizeof(float) * 12);
  current += 12;
  memcpy(current, &tb, sizeof(float) * 12);
  current += 12;
  memcpy(current, &tc, sizeof(float) * 12);
  current += 12;
  ta = {float(pt[4].x()), float(pt[4].y()), float(pt[4].z()), 1, 0, 0, -1, 0,
        float(color.r()), float(color.g()), float(color.b()), 1};
  tb = {float(pt[2].x()), float(pt[2].y()), float(pt[2].z()), 1, 0, 0, -1, 0,
        float(color.r()), float(color.g()), float(color.b()), 1};
  tc = {float(pt[6].x()), float(pt[6].y()), float(pt[6].z()), 1, 0, 0, -1, 0,
        float(color.r()), float(color.g()), float(color.b()), 1};
  memcpy(current, &ta, sizeof(float) * 12);
  current += 12;
  memcpy(current, &tb, sizeof(float) * 12);
  current += 12;
  memcpy(current, &tc, sizeof(float) * 12);
  current += 12;
}

void PackBoundingBox(float*& current, const BoundingBox& bbox) {

  Vec3f min = bbox.getMin();
  Vec3f max = bbox.getMax();
  Vec3f diff = max - min;

  float thickness = diff.Length() * 0.002;

  Vec3f pts[8] = {min + Vec3f(0, 0, 0),
                  min + Vec3f(0, 0, diff.z()),
                  min + Vec3f(0, diff.y(), 0),
                  min + Vec3f(0, diff.y(), diff.z()),
                  min + Vec3f(diff.x(), 0, 0),
                  min + Vec3f(diff.x(), 0, diff.z()),
                  min + Vec3f(diff.x(), diff.y(), 0),
                  min + Vec3f(diff.x(), diff.y(), diff.z())};

  addEdgeGeometry(current, pts[0], pts[1], Vec3f(0, 0, 0), Vec3f(0, 0, 0),
                  thickness, thickness);
  addEdgeGeometry(current, pts[2], pts[3], Vec3f(0, 0, 0), Vec3f(0, 0, 0),
                  thickness, thickness);
  addEdgeGeometry(current, pts[4], pts[5], Vec3f(0, 0, 0), Vec3f(0, 0, 0),
                  thickness, thickness);
  addEdgeGeometry(current, pts[6], pts[7], Vec3f(0, 0, 0), Vec3f(0, 0, 0),
                  thickness, thickness);

  addEdgeGeometry(current, pts[0], pts[2], Vec3f(0, 0, 0), Vec3f(0, 0, 0),
                  thickness, thickness);
  addEdgeGeometry(current, pts[1], pts[3], Vec3f(0, 0, 0), Vec3f(0, 0, 0),
                  thickness, thickness);
  addEdgeGeometry(current, pts[4], pts[6], Vec3f(0, 0, 0), Vec3f(0, 0, 0),
                  thickness, thickness);
  addEdgeGeometry(current, pts[5], pts[7], Vec3f(0, 0, 0), Vec3f(0, 0, 0),
                  thickness, thickness);

  addEdgeGeometry(current, pts[0], pts[4], Vec3f(0, 0, 0), Vec3f(0, 0, 0),
                  thickness, thickness);
  addEdgeGeometry(current, pts[1], pts[5], Vec3f(0, 0, 0), Vec3f(0, 0, 0),
                  thickness, thickness);
  addEdgeGeometry(current, pts[2], pts[6], Vec3f(0, 0, 0), Vec3f(0, 0, 0),
                  thickness, thickness);
  addEdgeGeometry(current, pts[3], pts[7], Vec3f(0, 0, 0), Vec3f(0, 0, 0),
                  thickness, thickness);
}
