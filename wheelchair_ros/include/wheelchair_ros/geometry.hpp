/* Provides classes helpful for geometric computations */
#ifndef GEOMETRY_HPP_
#define GEOMETRY_HPP_

#include <iostream>

class Vector3D {
public:
  double v[3];

public:
  Vector3D();
  Vector3D(double x, double y, double z);
  Vector3D(const Vector3D &other);
  Vector3D& operator=(const Vector3D &other);

  double dot(const Vector3D &other) const;
  Vector3D cross(const Vector3D &other) const;
  double length() const;
  Vector3D normalized() const;

  Vector3D operator*(double r) const;
  Vector3D operator/(double denom) const;
};

Vector3D operator*(double l, const Vector3D &r);
std::ostream& operator<<(std::ostream &s, const Vector3D &v);

#endif //GEOMETRY_HPP_
