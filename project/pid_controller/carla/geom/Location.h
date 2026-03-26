#pragma once
#include "Vector3D.h"

namespace carla { namespace geom {

struct Location : Vector3D {
    Location() = default;
    Location(float x, float y, float z) : Vector3D(x, y, z) {}
};

}} // namespace carla::geom
