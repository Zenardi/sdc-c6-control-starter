#pragma once
#include "Location.h"
#include "Rotation.h"

namespace carla { namespace geom {

struct Transform {
    Location location;
    Rotation rotation;

    Transform() = default;
    Transform(const Location& loc, const Rotation& rot)
        : location(loc), rotation(rot) {}
};

}} // namespace carla::geom
