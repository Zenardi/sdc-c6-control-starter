#pragma once
// carla/client/Waypoint.h — stub for types used in behavior_planner_FSM
#include "carla/geom/Transform.h"
#include "carla/road/RoadTypes.h"
#include <boost/shared_ptr.hpp>
#include <vector>

namespace carla { namespace client {

class Waypoint {
public:
    carla::geom::Transform GetTransform() const { return {}; }

    std::vector<boost::shared_ptr<Waypoint>> GetNext(double distance) const {
        return {};
    }

    bool IsJunction() const { return false; }

    carla::road::JuncId GetJunctionId() const { return -1; }

    double GetLaneWidth() const { return 3.5; }
};

}} // namespace carla::client
