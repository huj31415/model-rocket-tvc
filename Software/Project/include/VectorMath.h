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
};

// Convenient typedefs for float and double versions
typedef Vector3<float> Vector3f;
typedef Vector3<double> Vector3d;

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
  static Quaternion fromEulerAngles(T roll, T pitch, T yaw)
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
    return fromEulerAngles(v.x, v.y, v.z);
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
  Vector3<T> toEulerAngles() const
  {
    Vector3<T> angles;

    // Roll (x-axis rotation)
    T sinr_cosp = 2 * (w * x + y * z);
    T cosr_cosp = 1 - 2 * (x * x + y * y);
    angles.x = atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    T sinp = 2 * (w * y - z * x);
    if (fabs(sinp) >= 1)
      angles.y = (sinp > 0 ? M_PI / 2 : -M_PI / 2); // use 90 degrees if out of range
    else
      angles.y = asin(sinp);

    // Yaw (z-axis rotation)
    T siny_cosp = 2 * (w * z + x * y);
    T cosy_cosp = 1 - 2 * (y * y + z * z);
    angles.z = atan2(siny_cosp, cosy_cosp);

    return angles;
  }

  // Spherical linear interpolation between two quaternions
  static Quaternion slerp(const Quaternion &q1, const Quaternion &q2, T t)
  {
    // Compute the cosine of the angle between the quaternions.
    T cosTheta = q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z;

    // If cosTheta < 0, negate one quaternion to take the shorter path.
    Quaternion q2_copy = q2;
    if (cosTheta < 0)
    {
      cosTheta = -cosTheta;
      q2_copy = q2_copy * -1;
    }

    // If the quaternions are very close, use linear interpolation.
    if (cosTheta > 0.9995)
    {
      Quaternion result = q1 * (1 - t) + q2_copy * t;
      return result.normalized();
    }
    else
    {
      T theta = acos(cosTheta);
      T sinTheta = sin(theta);
      T factor1 = sin((1 - t) * theta) / sinTheta;
      T factor2 = sin(t * theta) / sinTheta;
      Quaternion result = q1 * factor1 + q2_copy * factor2;
      return result;
    }
  }
};

// Convenient typedefs for float and double versions
typedef Quaternion<float> QuaternionF;
typedef Quaternion<double> QuaternionD;

#endif // VECTOR_MATH_H
