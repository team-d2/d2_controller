#ifndef D2__CONTROLLER__VECTOR3_HPP_
#define D2__CONTROLLER__VECTOR3_HPP_

#include <cmath>

namespace d2::controller
{

struct Vector3
{
  double x, y, z;
};

namespace {

constexpr auto length2(const Vector3 & v) {
  return v.x * v.x + v.y * v.y + v.z * v.z;
}

constexpr auto length(const Vector3 & v) {
  return std::sqrt(length2(v));
}

constexpr auto operator+(const Vector3 & v1, const Vector3 & v2)
{
  return Vector3 {
    v1.x + v2.x,
    v1.y + v2.y,
    v1.z + v2.z
  };

}

constexpr auto operator-(const Vector3 & v1, const Vector3 & v2)
{
  return Vector3 {
    v1.x - v2.x,
    v1.y - v2.y,
    v1.z - v2.z
  };
}

constexpr auto operator*(const Vector3 & v, const double scalar)
{
  return Vector3 {
    v.x * scalar,
    v.y * scalar,
    v.z * scalar
  };
}

constexpr auto operator/(const Vector3 & v, const double scalar)
{
  return v * (1.0 / scalar);
}

constexpr auto distance2(const Vector3 & p1, const Vector3 & p2) {
  return length2(p1 - p2);
}

constexpr auto distance(const Vector3 & p1, const Vector3 & p2) {
  return length(p1 - p2);
}

constexpr auto dot(const Vector3 & v1, const Vector3 & v2)
{
  return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

constexpr auto rot(const Vector3 & v1, const Vector3 & v2)
{
  return Vector3{
    v1.y * v2.z - v1.z * v2.y,
    v1.z * v2.x - v1.x * v2.z,
    v1.x * v2.y - v1.y * v2.x
  };
}

}

}

#endif // D2__CONTROLLER__VECTOR3_HPP_
