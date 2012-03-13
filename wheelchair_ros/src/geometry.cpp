/* Provides classes helpful for geometric computation */
#include <cmath>
#include <iostream>
#include "wheelchair_ros/geometry.hpp"

using namespace std;

Vector3D::Vector3D() {
  v[0] = v[1] = v[2] = 0;
}

Vector3D::Vector3D(double x, double y, double z) {
  v[0] = x;
  v[1] = y;
  v[2] = z;
}

Vector3D::Vector3D(const Vector3D &other) {
  v[0] = other.v[0];
  v[1] = other.v[1];
  v[2] = other.v[2];
}

Vector3D& Vector3D::operator=(const Vector3D &other) {
  v[0] = other.v[0];
  v[1] = other.v[1];
  v[2] = other.v[2];
  return *this;
}

double Vector3D::dot(const Vector3D &other) const {
  return v[0]*other.v[0] + v[1]*other.v[1] + v[2]*other.v[2];
}

Vector3D Vector3D::cross(const Vector3D& other) const {
  return Vector3D(
      v[1]*other.v[2] - v[2]*other.v[1],
      v[2]*other.v[0] - v[0]*other.v[2],
      v[0]*other.v[1] - v[1]*other.v[0]);
}

double Vector3D::length() const {
  return sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

Vector3D Vector3D::normalized() const {
  return Vector3D(*this) / length();
}

Vector3D Vector3D::operator*(double r) const {
  return Vector3D(v[0]*r, v[1]*r, v[2]*r);
}

Vector3D Vector3D::operator/(double d) const {
  return Vector3D(v[0]/d, v[1]/d, v[2]/d);
}

Vector3D operator*(double l, const Vector3D &r) {
  return Vector3D(r.v[0]*l, r.v[1]*l, r.v[2]*l);
}

ostream& operator<<(ostream &s, const Vector3D &v) {
  s << "[" << v.v[0] << "," << v.v[1] << "," << v.v[2] << "]" << endl;
  return s;
}

