#pragma once
// carla/client/Map.h — pulled in only to satisfy namespace alias cc = carla::client
// and SharedPtr<cc::Map> in function signatures that are never called at runtime.
#include "carla/geom/Location.h"
#include <boost/shared_ptr.hpp>
#include <vector>

namespace carla { namespace client {

class Waypoint;

class Map {
public:
    boost::shared_ptr<Waypoint> GetWaypoint(const carla::geom::Location&,
                                             bool project_to_road = true,
                                             int32_t lane_type = 0) const {
        return {};
    }
};

}} // namespace carla::client
