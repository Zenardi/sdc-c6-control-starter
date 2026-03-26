#pragma once
// carla/client/Actor.h — stub (GetTransform used in vehicle_dynamic_model.cpp which is NOT compiled)
#include "carla/geom/Transform.h"

namespace carla { namespace client {

class Actor {
public:
    carla::geom::Transform GetTransform() const { return {}; }
};

}} // namespace carla::client
