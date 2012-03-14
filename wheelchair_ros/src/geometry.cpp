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
  s << "[" << v.v[0] << "," << v.v[1] << "," << v.v[2] << "]";
  return s;
}

Point3D::Point3D() {
  p[0] = p[1] = p[2] = 0;
}

Point3D::Point3D(double x, double y, double z) {
  p[0] = x;
  p[1] = y;
  p[2] = z;
}

Point3D::Point3D(const Point3D &other) {
  p[0] = other.p[0];
  p[1] = other.p[1];
  p[2] = other.p[2];
}

Point3D& Point3D::operator=(const Point3D &other) {
  p[0] = other.p[0];
  p[1] = other.p[1];
  p[2] = other.p[2];
  return *this;
}

Vector3D Point3D::operator-(const Point3D &r) const {
  return Vector3D(p[0] - r.p[0], p[1] - r.p[1], p[2] - r.p[2]);
}

Point3D Point3D::operator+(const Vector3D &r) const {
  return Point3D(p[0] + r.v[0], p[1] + r.v[1], p[2] + r.v[2]);
}

Point3D operator+(const Vector3D &l, const Point3D &r) {
  return Point3D(l.v[0] + r.p[0], l.v[1] + r.p[1], l.v[2] + r.p[2]);
}

ostream& operator<<(ostream &s, const Point3D &p) {
  s << "{" << p.p[0] << "," << p.p[1] << "," << p.p[2] << "}";
  return s;
}

bool Polygon::contains(const Point3D &p) const {
  if (size() <= 2) return false;

  /* Find the forward normal of the polygon.  */
  Vector3D norm (
      (m_vertices[1] - m_vertices[0]).cross(m_vertices[2] - m_vertices[1]));
  norm = norm.normalized();

  /* Check intersection-of-halfspaces on each edge of the polygon */
  for (unsigned i=0; i < size(); ++i) {
    Vector3D tangent (m_vertices[(i+1)%size()] - m_vertices[i]);
    tangent = tangent.normalized();
    
    Vector3D binorm (norm.cross(tangent));

    if ( (p - m_vertices[i]).dot(binorm) < 0 ) {
      return false;  
    }
  }
  return true;
}

ostream& operator<<(ostream& s, const Polygon &p) {
  s << "[";
  for (unsigned i=0; i<p.size(); ++i) {
    if (i) s << "  ";
    s << p[i] << ",";
    if ((i+1) < p.size()) s << endl;
  }
  s << "]";
  return s;
}

void geometryTests() {
  Vector3D v1 (1, -1, 0);
  Vector3D v2 (1, 1, 0);
  cerr << v1 << " dot " << v2 << " = " << v1.dot(v2) << endl;
  cerr << v1 << " cross " << v2 << " = " << v1.cross(v2) << endl;
  Point3D p1 (1, 0, 1);
  Point3D p2 (0, 1, 1);
  cerr << p1 << " - " << p2 << " = " << (p1 - p2) << endl;
 
  Polygon poly;
  poly.push(Point3D(-1, -1, 0));
  poly.push(Point3D(1, -1, 0));
  poly.push(Point3D(1, 1, 0));
  poly.push(Point3D(-1, 1, 0));
  cerr << poly << " :" << endl;
  Point3D p3 (0, 0, 0);
  cerr << "contains " << p3 << ": " << poly.contains(p3) << endl;
  Point3D p4 (2, 2, 2);
  cerr << "contains " << p4 << ": " << poly.contains(p4) << endl;
  Point3D p5 (0.9, 0.9, 40);
  cerr << "contains " << p5 << ": " << poly.contains(p5) << endl;
  Point3D p6 (-0.9, 0.9, 0);
  cerr << "contains " << p6 << ": " << poly.contains(p6) << endl;
  Point3D p7 (1.01, -1.01, 0);
  cerr << "contains " << p7 << ": " << poly.contains(p7) << endl;
}

