/* Provides classes helpful for geometric computations */
#ifndef GEOMETRY_HPP_
#define GEOMETRY_HPP_

#include <iostream>
#include <vector>
#include <GL/gl.h>

class Vector3D {
public:
  GLdouble v[3];

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

class Point3D {
public:
  GLdouble p[3];

public:
  Point3D();
  Point3D(double x, double y, double z);
  Point3D(const Point3D &other);
  Point3D& operator=(const Point3D &other);

  Vector3D operator-(const Point3D &r) const;
  Point3D operator+(const Vector3D &r) const;
};

Point3D operator+(const Vector3D &l, const Point3D &r);
std::ostream& operator<<(std::ostream &s, const Point3D &p);

class Polygon {
private:
  std::vector<Point3D> m_vertices;

public:
  void push(const Point3D &p) 
    { m_vertices.push_back(p); }
  unsigned size() const
    { return m_vertices.size(); }
  Point3D& operator[](unsigned i) 
    { return m_vertices[i]; }
  const Point3D& operator[](unsigned i) const
    { return m_vertices[i]; }

  /* Check if this polygon contains a point.  Assumes that this is a convex
   * planar polygon, that vertices are given in counter-clockwise order,  and
   * that the point is within the plane of the polygon */
  bool contains(const Point3D &p) const;
};

std::ostream& operator<<(std::ostream &s, const Polygon &p);

class Quaternion {
public:
  double a, b, c, d;

public:
  Quaternion() : a(1), b(0), c(0), d(0) {  } // (This is the identity quaternion)
  Quaternion(double pa, double pb, double pc, double pd):
    a(pa), b(pb), c(pc), d(pd)
    { }
  Quaternion(const Quaternion &other) : 
    a(other.a), b(other.b), c(other.c), d(other.d) 
    { }
  /** Initialize a quat with a rotation about an axis
   *  @param rad   the rotation, in radians
   *  @param axis  the axis about which to rotate. */
  Quaternion(double rad, const Vector3D &axis);

  Quaternion& operator =(const Quaternion &other);
  Quaternion operator*(const Quaternion &r) const;

  double angle() { return 2*acos(a); }
  double x() { return 2*asin(b); }
  double y() { return 2*asin(c); }
  double z() { return 2*asin(d); }

  friend std::ostream& operator<<(std::ostream &os, const Quaternion &q);
};

class Matrix4x4 {
public:
  GLdouble m[16];  // The matrix, in column-major order

public:
  Matrix4x4();
  Matrix4x4(const Matrix4x4 &other);
  Matrix4x4(const Quaternion &other)
    {*this = other;}
  Matrix4x4& operator=(const Matrix4x4 &other);
  Matrix4x4& operator=(const Quaternion &other);

  Vector3D operator*(const Vector3D &r) const;
  Point3D operator*(const Point3D &r) const;

  GLdouble& at(unsigned i, unsigned j)
    { return m[i + 4*j]; }
  const GLdouble& at(unsigned i, unsigned j) const
    { return m[i + 4*j]; }
};

std::ostream& operator<<(std::ostream &s, const Matrix4x4 &m);

void geometryTests();

#endif //GEOMETRY_HPP_

