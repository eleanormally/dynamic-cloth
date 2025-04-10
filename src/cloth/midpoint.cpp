#include <cmath>
#include "cloth.h"

#define MAX_ITERS 100
#define EPSILON 0.01
double solve(std::function<double(double)> equation, double upperBound) {
  double lowerBound = 0;
  int    iters = 0;
  double midpoint = (upperBound + lowerBound) / 2;
  double value = equation(midpoint);

  while (value > EPSILON && iters < MAX_ITERS) {
    if (value < 0) {
      upperBound = midpoint;
    } else {
      lowerBound = midpoint;
    }
    midpoint = (upperBound + lowerBound) / 2;
    value = equation(midpoint);

    iters++;
  }
  return midpoint;
}

//See midpoint pdf writeup
Vec3f calculateHangingMidpoint(const ClothParticle& p1, const ClothParticle& p2,
                               const double k) {
  double length = (p1.originalPosition - p2.originalPosition).Length() * k;

  double Bx = Vec3f(p2.position.x() - p1.position.x(),
                    p2.position.y() - p1.position.y(), 0)
                  .Length();
  double By = p2.position.z() - p1.position.z();

  double c = (By / Bx) + 1;

  using std::pow;
  auto equation = [c, Bx, length](double a) {
    return pow(c + a * Bx, 1.5) - pow(c - a * Bx, 1.5) - 3 * a * length;
  };

  double a = solve(equation, 10.0);

  double midpointX = Bx / 2;
  double midpointHeight =
      a * midpointX * midpointX + ((By / Bx) - a * Bx) * midpointX;

  Vec3f midpoint = Vec3f::interp(p1.position, p2.position);
  midpoint.setz(midpointHeight);
  return midpoint;
}
