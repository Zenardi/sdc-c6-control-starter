#pragma once
#include <cmath>
#include <cstdint>

namespace carla { namespace geom {

struct Vector3D {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    Vector3D() = default;
    Vector3D(float x, float y, float z) : x(x), y(y), z(z) {}

    float Length() const { return std::sqrt(x * x + y * y + z * z); }
    float SquaredLength() const { return x * x + y * y + z * z; }

    Vector3D operator-(const Vector3D& rhs) const {
        return Vector3D{x - rhs.x, y - rhs.y, z - rhs.z};
    }
    Vector3D operator+(const Vector3D& rhs) const {
        return Vector3D{x + rhs.x, y + rhs.y, z + rhs.z};
    }
};

}} // namespace carla::geom
