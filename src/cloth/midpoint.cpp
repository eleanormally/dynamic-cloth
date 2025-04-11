#include <cmath>
#include "cloth.h"

#define MAX_ITERS 100
#define EPSILON 0.00001
double solve(std::function<double(double)> equation, double upperBound) {
  double lowerBound = 0;
  int    iters = 0;
  double midpoint = (upperBound + lowerBound) / 2;
  double value = equation(midpoint);

  while (abs(value) > EPSILON && iters < MAX_ITERS) {
    if (value > 0) {
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
  //TODO incorporate spring stiffness into hanging
  const double length =
      (p1.originalPosition - p2.originalPosition).Length() * k;
  if ((p2.position - p1.position).Length() > length) {
    return Vec3f::interp(p1.position, p2.position);
  }

  const double Bx = Vec3f(p2.position.x() - p1.position.x(), 0,
                          p2.position.z() - p1.position.z())
                        .Length();
  const double By = p2.position.y() - p1.position.y();

  const double m = By / Bx;

  using std::pow;
  const auto antiderivative = [m, Bx](double a, double x) constexpr {
    const double q = -1 * a * Bx + 2 * a * x + m;
    const double v = sqrt(q * q + 1);
    const double qv = q / v;
    return -0.25 * Bx * v + 0.5 * x * v + m * v / (4 * a) +
           (log(1 + qv) - log(1 - qv)) / (8 * a);
  };

  auto equation = [&antiderivative, Bx, length](double a) {
    return antiderivative(a, Bx) - antiderivative(a, 0) - length;
  };

  double a = solve(equation, 20.0);

  double midpointX = Bx / 2;
  double midpointHeight =
      a * midpointX * midpointX + ((By / Bx) - a * Bx) * midpointX;

  Vec3f midpoint = Vec3f::interp(p1.position, p2.position);
  midpoint.sety(midpointHeight);
  return midpoint;
}
