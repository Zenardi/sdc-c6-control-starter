#pragma once

namespace carla { namespace geom {

struct Rotation {
    float pitch = 0.0f;
    float yaw   = 0.0f;
    float roll  = 0.0f;

    Rotation() = default;
    Rotation(float pitch, float yaw, float roll)
        : pitch(pitch), yaw(yaw), roll(roll) {}
};

}} // namespace carla::geom
