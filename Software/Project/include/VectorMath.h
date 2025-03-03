#ifndef VECTOR_MATH_H
#define VECTOR_MATH_H

#include <math.h>
#include <stdlib.h>

/*
   Vector3 Class
   -------------
   A 3-dimensional vector with common operations.
*/
template <typename T>
class Vector3
{
public:
  T x, y, z;

  // Constructors
  Vector3() : x(0), y(0), z(0) {}
  Vector3(T x, T y, T z) : x(x), y(y), z(z) {}

  // Common vectors
  static Vector3 Zero() { return Vector3(0, 0, 0); }
  static Vector3 UnitX() { return Vector3(1, 0, 0); }
  static Vector3 UnitY() { return Vector3(0, 1, 0); }
  static Vector3 UnitZ() { return Vector3(0, 0, 1); }

  static Vector3 fromSensorData(sensors_vec_t d) { return Vector3(d.x, d.y, d.z); }

  // Magnitude and normalization
  T length() const { return sqrt(x * x + y * y + z * z); }
  T lengthSquared() const { return x * x + y * y + z * z; }
  Vector3 normalized() const
  {
    T len = length();
    if (len == 0)
      return Vector3(0, 0, 0);
    return Vector3(x / len, y / len, z / len);
  }

  // Operator overloads for vector arithmetic
  Vector3 operator+(const Vector3 &v) const { return Vector3(x + v.x, y + v.y, z + v.z); }
  Vector3 operator-(const Vector3 &v) const { return Vector3(x - v.x, y - v.y, z - v.z); }
  Vector3 operator-() const { return Vector3(-x, -y, -z); }
  Vector3 operator*(T scalar) const { return Vector3(x * scalar, y * scalar, z * scalar); }
  Vector3 operator/(T scalar) const { return Vector3(x / scalar, y / scalar, z / scalar); }

  Vector3 &operator+=(const Vector3 &v)
  {
    x += v.x;
    y += v.y;
    z += v.z;
    return *this;
  }
  Vector3 &operator-=(const Vector3 &v)
  {
    x -= v.x;
    y -= v.y;
    z -= v.z;
    return *this;
  }
  Vector3 &operator*=(T scalar)
  {
    x *= scalar;
    y *= scalar;
    z *= scalar;
    return *this;
  }
  Vector3 &operator/=(T scalar)
  {
    x /= scalar;
    y /= scalar;
    z /= scalar;
    return *this;
  }

  // Dot product
  T dot(const Vector3 &v) const { return x * v.x + y * v.y + z * v.z; }

  // Cross product
  Vector3 cross(const Vector3 &v) const
  {
    return Vector3(
        y * v.z - z * v.y,
        z * v.x - x * v.z,
        x * v.y - y * v.x);
  }

  const char* toString() {
    static char buffer[64];
    sprintf(buffer, "%.3f,%.3f,%.3f", x, y, z);
    return buffer;
  }
};

// Convenient typedefs for float and double versions
typedef Vector3<float> Vector3f;
typedef Vector3<double> Vector3d;

/*
   Vector2 Class
   -------------
   A 2-dimensional vector with common operations.
*/
template <typename T>
class Vector2
{
public:
  T x, y;

  // Constructors
  Vector2() : x(0), y(0) {}
  Vector2(T x, T y) : x(x), y(y) {}

  // Common vectors
  static Vector2 Zero() { return Vector2(0, 0); }
  static Vector2 UnitX() { return Vector2(1, 0); }
  static Vector2 UnitY() { return Vector2(0, 1); }

  // Magnitude and normalization
  T length() const { return sqrt(x * x + y * y); }
  T lengthSquared() const { return x * x + y * y; }
  Vector2 normalized() const
  {
    T len = length();
    if (len == 0)
      return Vector2(0, 0);
    return Vector2(x / len, y / len);
  }

  // Operator overloads for vector arithmetic
  Vector2 operator+(const Vector2 &v) const { return Vector2(x + v.x, y + v.y); }
  Vector2 operator-(const Vector2 &v) const { return Vector2(x - v.x, y - v.y); }
  Vector2 operator-() const { return Vector2(-x, -y); }
  Vector2 operator*(T scalar) const { return Vector2(x * scalar, y * scalar); }
  Vector2 operator/(T scalar) const { return Vector2(x / scalar, y / scalar); }

  Vector2 &operator+=(const Vector2 &v)
  {
    x += v.x;
    y += v.y;
    return *this;
  }
  Vector2 &operator-=(const Vector2 &v)
  {
    x -= v.x;
    y -= v.y;
    return *this;
  }
  Vector2 &operator*=(T scalar)
  {
    x *= scalar;
    y *= scalar;
    return *this;
  }
  Vector2 &operator/=(T scalar)
  {
    x /= scalar;
    y /= scalar;
    return *this;
  }

  // Dot product
  T dot(const Vector2 &v) const { return x * v.x + y * v.y; }

  const char* toString() {
    static char buffer[64];
    sprintf(buffer, "%.3f,%.3f", x, y);
    return buffer;
  }
};

// Convenient typedefs for float and double versions
typedef Vector2<float> Vector2f;
typedef Vector2<double> Vector2d;

/*
   Quaternion Class
   ----------------
   A quaternion class for representing rotations.
*/
template <typename T>
class Quaternion
{
public:
  T w, x, y, z;

  // Constructors
  Quaternion() : w(1), x(0), y(0), z(0) {} // Identity quaternion
  Quaternion(T w, T x, T y, T z) : w(w), x(x), y(y), z(z) {}

  // Create a quaternion from an axis-angle representation (angle in radians)
  static Quaternion fromAxisAngle(const Vector3<T> &axis, T angle)
  {
    T halfAngle = angle / 2;
    T s = sin(halfAngle);
    return Quaternion(cos(halfAngle), axis.x * s, axis.y * s, axis.z * s);
  }

  // Create a quaternion from Euler angles (roll, pitch, yaw in radians)
  static Quaternion fromEuler(T roll, T pitch, T yaw)
  {
    T cy = cos(yaw * 0.5);
    T sy = sin(yaw * 0.5);
    T cp = cos(pitch * 0.5);
    T sp = sin(pitch * 0.5);
    T cr = cos(roll * 0.5);
    T sr = sin(roll * 0.5);

    T w = cr * cp * cy + sr * sp * sy;
    T x = sr * cp * cy - cr * sp * sy;
    T y = cr * sp * cy + sr * cp * sy;
    T z = cr * cp * sy - sr * sp * cy;

    return Quaternion(w, x, y, z);
  }

  static Quaternion fromVector3d(Vector3d v)
  {
    return fromEuler(v.x, v.y, v.z);
  }

  // Identity quaternion
  static Quaternion Identity() { return Quaternion(1, 0, 0, 0); }

  // Norm and normalization
  T norm() const { return sqrt(w * w + x * x + y * y + z * z); }
  T normSquared() const { return w * w + x * x + y * y + z * z; }
  Quaternion normalized() const
  {
    T n = norm();
    if (n == 0)
      return Quaternion(1, 0, 0, 0);
    return Quaternion(w / n, x / n, y / n, z / n);
  }

  // Conjugate and inverse
  Quaternion conjugate() const { return Quaternion(w, -x, -y, -z); }
  Quaternion inverse() const
  {
    T n2 = normSquared();
    if (n2 == 0)
      return Quaternion(1, 0, 0, 0);
    return conjugate() / n2;
  }

  // Quaternion multiplication (Hamilton product)
  Quaternion operator*(const Quaternion &q) const
  {
    return Quaternion(
        w * q.w - x * q.x - y * q.y - z * q.z,
        w * q.x + x * q.w + y * q.z - z * q.y,
        w * q.y - x * q.z + y * q.w + z * q.x,
        w * q.z + x * q.y - y * q.x + z * q.w);
  }

  // Scalar multiplication and division
  Quaternion operator*(T scalar) const { return Quaternion(w * scalar, x * scalar, y * scalar, z * scalar); }
  Quaternion operator/(T scalar) const { return Quaternion(w / scalar, x / scalar, y / scalar, z / scalar); }

  // Addition and subtraction
  Quaternion operator+(const Quaternion &q) const { return Quaternion(w + q.w, x + q.x, y + q.y, z + q.z); }
  Quaternion operator-(const Quaternion &q) const { return Quaternion(w - q.w, x - q.x, y - q.y, z - q.z); }

  Quaternion &operator*=(const Quaternion &q)
  {
    *this = *this * q;
    return *this;
  }
  Quaternion &operator*=(T scalar)
  {
    w *= scalar;
    x *= scalar;
    y *= scalar;
    z *= scalar;
    return *this;
  }

  // Rotate a vector by this quaternion
  Vector3<T> rotate(const Vector3<T> &v) const
  {
    Quaternion qv(0, v.x, v.y, v.z);
    Quaternion result = (*this) * qv * this->conjugate();
    return Vector3<T>(result.x, result.y, result.z);
  }

  // Convert the quaternion to Euler angles (roll, pitch, yaw in radians)
  Vector3<T> toEuler() const
  {
    Vector3<T> angles;

    // x-axis rotation
    T sinr_cosp = 2 * (w * x + y * z);
    T cosr_cosp = 1 - 2 * (x * x + y * y);
    angles.x = atan2(sinr_cosp, cosr_cosp);

    // y-axis rotation
    T sinp = 2 * (w * y - z * x);
    if (fabs(sinp) >= 1)
      angles.y = (sinp > 0 ? M_PI / 2 : -M_PI / 2); // use 90 degrees if out of range
    else
      angles.y = asin(sinp);

    // z-axis rotation
    T siny_cosp = 2 * (w * z + x * y);
    T cosy_cosp = 1 - 2 * (y * y + z * z);
    angles.z = atan2(siny_cosp, cosy_cosp);

    return angles;
  }

  const char* toString() {
    static char buffer[64];
    sprintf(buffer, "%.3f,%.3f,%.3f,%.3f", w, x, y, z);
    return buffer;
  }
};

// Convenient typedefs for float and double versions
typedef Quaternion<float> QuaternionF;
typedef Quaternion<double> QuaternionD;

#endif // VECTOR_MATH_H
