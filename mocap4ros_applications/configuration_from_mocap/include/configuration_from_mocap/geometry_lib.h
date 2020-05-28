// Copyright 2020 National Institute of Advanced Industrial Science and Technology, Japan
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Most of this is based on:
// Schneider and Eberly, 2003. Computational Tools for Computer Graphics
// Morgan Kaufman Publishers */

#ifndef GEOMETRY_LIB_H
#define GEOMETRY_LIB_H

#include <cmath>
#include <optional>
#include <any>

namespace geometry_lib
{

const float PRECISION = 0.001;

struct Vector2D
{
  Vector2D() {}
  Vector2D(double a, double b) : a(a), b(b) {}
  double a;
  double b;
};

Vector2D operator-(const Vector2D& v1, const Vector2D& v2)
{
  return Vector2D(v1.a - v2.a, v1.b - v2.b);
}

Vector2D operator+(const Vector2D& v1, const Vector2D& v2)
{
  return Vector2D(v1.a + v2.a, v1.b + v2.b);
}

struct Vector3D
{
  Vector3D() {}
  Vector3D(double a, double b, double c) : a(a), b(b), c(c) {}
  double a;
  double b;
  double c;
  double norm()
  {
    return std::sqrt(std::pow(a, 2) + std::pow(b, 2) + std::pow(c, 2));
  }
  Vector3D normalize()
  {
    double nrm = norm();
    return Vector3D(a / nrm,
                    b / nrm,
                    c / nrm);
  }
};

Vector3D operator-(const Vector3D& v1, const Vector3D& v2)
{
  return Vector3D(v1.a - v2.a, v1.b - v2.b, v1.c - v2.c);
}

Vector3D operator+(const Vector3D& v1, const Vector3D& v2)
{
  return Vector3D(v1.a + v2.a, v1.b + v2.b, v1.c + v2.c);
}

Vector3D operator*(const Vector3D& vec, double s)
{
  Vector3D res(s * vec.a,
               s * vec.b,
               s * vec.c);
  return res;
}

Vector3D operator*(double s, const Vector3D& vec)
{
  return vec * s;
}

Vector3D operator/(const Vector3D& vec, double s)
{
  return Vector3D(vec.a / s,
                  vec.b / s,
                  vec.c / s);
}

Vector3D cross(const Vector3D& left, const Vector3D& right)
{
  return Vector3D(left.b * right.c - left.c * right.b,
                  left.c * right.a - left.a * right.c,
                  left.a * right.b - left.b * right.a);
}

double dot(const Vector3D& v1, const Vector3D& v2)
{
  return v1.a * v2.a + v1.b * v2.b + v1.c * v2.c;
}

struct Plane
{
  Plane() {}
  Plane(double a, double b, double c, double d) : a(a), b(b), c(c), d(d) {}
  // ax + by + cz = d
  // normal = [ a b c ]
  // d = ax0 + by0 + cz0
  double a;
  double b;
  double c;
  double d;

  Vector3D normal() const
  {
    return Vector3D(a, b, c);
  }
};

bool operator==(const Plane& p1, const Plane& p2)
{
  return std::abs(p1.a - p2.a) < PRECISION &&
         std::abs(p1.b - p2.b) < PRECISION &&
         std::abs(p1.c - p2.c) < PRECISION &&
         std::abs(p1.d - p2.d) < PRECISION;
}

bool operator!=(const Plane& p1, const Plane& p2)
{
  return !(p1 == p2);
}

struct Point2D
{
  Point2D() {}
  Point2D(double x, double y) : x(x), y(y) {}
  double x;
  double y;
};

Vector2D operator-(const Point2D& p1, const Point2D& p2)
{
  return Vector2D(p1.x - p2.x, p1.y - p2.y);
}

Vector2D operator+(const Point2D& p1, const Point2D& p2)
{
  return Vector2D(p1.x + p2.x, p1.y + p2.y);
}

struct Point3D
{
  Point3D() {}
  Point3D(double x, double y, double z) : x(x), y(y), z(z) {}
  double x;
  double y;
  double z;
};

Point3D operator-(const Point3D& point, const Vector3D& vector)
{
  Point3D res(point.x - vector.a,
              point.y - vector.b,
              point.z - vector.c);
  return res;
}

Vector3D operator-(const Point3D& p1, const Point3D& p2)
{
  return Vector3D(p1.x - p2.x, p1.y - p2.y, p1.z - p2.z);
}

Point3D operator+(const Point3D& point, const Vector3D& vector)
{
  return Point3D(point.x + vector.a,
                 point.y + vector.b,
                 point.z + vector.c);
}

bool operator<(const Point3D& p1, const Point3D& p2)
{
  return p1.x + PRECISION < p2.x &&
         p1.y + PRECISION < p2.y &&
         p1.z + PRECISION < p2.z;
}

bool operator==(const Point3D& p1, const Point3D& p2)
{
  return std::abs(p1.x - p2.x) < PRECISION &&
         std::abs(p1.y - p2.y) < PRECISION &&
         std::abs(p1.z - p2.z) < PRECISION;
}

bool operator!=(const Point3D& p1, const Point3D& p2)
{
  return !(p1 == p2);
}

double dot(const Point3D& v1, const Point3D& v2)
{
  return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

double dot(const Vector3D& v1, const Point3D& v2)
{
  return v1.a * v2.x + v1.b * v2.y + v1.c * v2.z;
}

double dot(const Point3D& v1, const Vector3D& v2)
{
  return v1.x * v2.a + v1.y * v2.b + v1.z * v2.c;
}

struct Circle
{
  Circle() {}
  Circle(double x, double y, double radius) : center(x, y), radius(radius) {}
  Circle(Point2D center, double radius) : center(center), radius(radius) {}
  Point2D center;
  double radius;
};

struct Circle3D
{
  Circle3D() {}
  Circle3D(double x, double y, double z, double radius, Plane plane) :
    center(x, y, z), radius(radius), plane(plane) {}
  Point3D center;
  double radius;
  Plane plane;
};

struct Sphere
{
  Sphere() {}
  Sphere(double x, double y, double z, double radius) : center_3(x, y, z), radius(radius) {}
  Sphere(Point3D center_3, double radius) : center_3(center_3), radius(radius) {}
  Point3D center_3;
  double radius;
};

bool operator==(const Circle& a, const Circle& b)
{
  return std::abs(a.center.x - b.center.x) < PRECISION &&
         std::abs(a.center.y - b.center.y) < PRECISION &&
         std::abs(a.radius - b.radius) < PRECISION;
}

double distance(const Point3D& point, const Plane& plane)
{
  return dot(point, plane.normal()) + plane.d;
}

std::optional<Circle3D> intersect(const Sphere& sphere, const Plane& plane)
{
  double distance_to_plane = distance(sphere.center_3, plane);
  if (distance_to_plane > sphere.radius) 
    return {}; // there is no intersection

  Circle3D circle;
  circle.plane = plane;
  // Assume sphere's center is relative to a point in the plane
  // Let Q be the image of the sphere's center to the plane
  Vector3D normal = plane.normal();
  Vector3D normal_scaled_by_dist = distance_to_plane * normal;
  Point3D Q = sphere.center_3 - normal_scaled_by_dist;
  circle.center.x = Q.x;
  circle.center.y = Q.y;
  circle.center.z = Q.z;
  circle.radius = std::sqrt(std::pow(sphere.radius, 2) - std::pow(distance_to_plane, 2));
  return circle;
}

std::optional<Circle3D> intersect(const Plane& plane, const Sphere& sphere)
{
  return intersect(sphere, plane);
}

// <Circle,pair<Point2D,Point2D>,Point2D>
std::any intersect(const Circle& a, const Circle& b)
{
  double d = std::hypot(a.center.x - b.center.x, a.center.y - b.center.y);
  double r_sum = a.radius + b.radius;
  if (a == b)
  {
    // treat intersection as a new circle
    return std::any_cast<Circle>(Circle(a.center.x, a.center.y, a.radius));
  }
  else if (d > r_sum || d < std::abs(a.radius - b.radius))
  {
    // no overlap
    return {};
  }
  /*else if (d == r_sum)
  {
    // single point along the line crossing both centers, scaled by the radius 
    double dx = abs(a.center.x - b.center.x);
    double dy = abs(a.center.y - b.center.y);
    double scale_x_nom = (a.center.x < b.center.x) ? a.radius : b.radius;
    double scale_y_nom = (a.center.y < b.center.y) ? a.radius : b.radius;
    double da = (scale_x_nom / (a.radius + b.radius)) * dx;
    double db = (scale_y_nom / (a.radius + b.radius)) * dy;
    return Point2D(((a.center.x < b.center.x) ? a.center.x : b.center.x) + da,
                   ((a.center.y < b.center.y) ? a.center.y : b.center.y) + db);
  }*/
  else
  {
    // two points overlap
    // distance between center of both circles
    double dx = std::abs(a.center.x - b.center.x);
    double dy = std::abs(a.center.y - b.center.y);
    double a2 = a.radius*a.radius;
    double b2 = b.radius*b.radius;
    double d2 = d*d;
    // distance from center of circle a to middle point of crossing circles
    double d0 = (a2 - b2 + d2) / (2*d);
    double h = std::sqrt(a2-d0*d0);
    // middle point of crossing circles
    double cross_x = a.center.x + (b.center.x - a.center.x) * (d0/d);
    double cross_y = a.center.y + (b.center.y - a.center.y) * (d0/d);
    // intersection point 0 
    double ip0x = cross_x + (b.center.y - a.center.y) * (h/d);
    double ip0y = cross_y - (b.center.x - a.center.x) * (h/d);
    // intersection point 1
    double ip1x = cross_x - (b.center.y - a.center.y) * (h/d);
    double ip1y = cross_y + (b.center.x - a.center.x) * (h/d);
    return std::make_pair(Point2D(ip0x, ip0y), Point2D(ip1x, ip1y));
  }
}

std::any intersect(const Circle3D& a, const Circle3D& b)
{
  if (a.plane != b.plane)
    return {}; // not implemented

  Vector3D line_between_centers = b.center - a.center;
  double distance = line_between_centers.norm();
  double r_sum = a.radius + b.radius;

  if (distance > r_sum)
    return {};

  auto v = line_between_centers.normalize(); 
  auto w = cross(a.plane.normal(), v);
  double ra2 = a.radius * a.radius;
  double rb2 = b.radius * b.radius;
  double d2 = distance * distance;
  double d0 = (ra2 - rb2 + d2) / (2*distance);
  auto h = std::sqrt(ra2 - std::pow(d0, 2));
  auto cpatv = a.center + d0 * v;
  auto hw = h * w;
  return std::make_pair(cpatv + hw, cpatv - hw);
}

}

#endif
