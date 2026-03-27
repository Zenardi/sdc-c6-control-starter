/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 				Aaron Brown
 **********************************************/

/**
 * @file main.cpp
 **/

#include <string>
#include <array>
#include <cfloat>
#include <chrono>
#include <cmath>
#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <vector>
#include <iostream>
#include <fstream>
#include <typeinfo>

#include "json.hpp"
#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Client.h>
#include <carla/client/Map.h>
#include <carla/client/Sensor.h>
#include <carla/client/TimeoutException.h>
#include <carla/client/World.h>
#include <carla/geom/Transform.h>
#include <carla/image/ImageIO.h>
#include <carla/image/ImageView.h>
#include <carla/sensor/data/Image.h>
#include "Eigen/QR"
#include "behavior_planner_FSM.h"
#include "motion_planner.h"
#include "planning_params.h"
#include "utils.h"
#include "pid_controller.h"

#include <limits>
#include <iostream>
#include <fstream>
#include <uWS/uWS.h>
#include <math.h>
#include <vector>
#include <cmath>
#include <time.h>

using namespace std;
using json = nlohmann::json;

#define _USE_MATH_DEFINES

string hasData(string s) {
  auto found_null = s.find("null");
    auto b1 = s.find_first_of("{");
    auto b2 = s.find_first_of("}");
    if (found_null != string::npos) {
      return "";
    }
    else if (b1 != string::npos && b2 != string::npos) {
      return s.substr(b1, b2 - b1 + 1);
    }
    return "";
}


template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

double angle_between_points(double x1, double y1, double x2, double y2){
  return atan2(y2-y1, x2-x1);
}

BehaviorPlannerFSM behavior_planner(
      P_LOOKAHEAD_TIME, P_LOOKAHEAD_MIN, P_LOOKAHEAD_MAX, P_SPEED_LIMIT,
      P_STOP_THRESHOLD_SPEED, P_REQ_STOPPED_TIME, P_REACTION_TIME,
      P_MAX_ACCEL, P_STOP_LINE_BUFFER);

// Decalre and initialized the Motion Planner and all its class requirements
MotionPlanner motion_planner(P_NUM_PATHS, P_GOAL_OFFSET, P_ERR_TOLERANCE);

bool have_obst = false;
vector<State> obstacles;

void path_planner(vector<double>& x_points, vector<double>& y_points, vector<double>& v_points, double yaw, double velocity, State goal, bool is_junction, string tl_state, vector< vector<double> >& spirals_x, vector< vector<double> >& spirals_y, vector< vector<double> >& spirals_v, vector<int>& best_spirals){

  State ego_state;

  // Use trajectory tip as planning origin so spirals are generated from the
  // leading edge of the current plan. This prevents spiral deadlock near parked
  // NPCs: the trajectory has already navigated around the obstacle, so new
  // spirals start from beyond it and do not trigger collision detection on entry.
  ego_state.location.x = x_points[x_points.size()-1];
  ego_state.location.y = y_points[y_points.size()-1];
  ego_state.velocity.x = velocity;

  if( x_points.size() > 1 ){
    ego_state.rotation.yaw = angle_between_points(x_points[x_points.size()-2], y_points[y_points.size()-2], x_points[x_points.size()-1], y_points[y_points.size()-1]);
    ego_state.velocity.x = v_points[v_points.size()-1];
    if(velocity < 0.01)
      ego_state.rotation.yaw = yaw;
  } else {
    ego_state.rotation.yaw = yaw;
  }

  Maneuver behavior = behavior_planner.get_active_maneuver();

  goal = behavior_planner.state_transition(ego_state, goal, is_junction, tl_state);

  if(behavior == STOPPED){

  	int max_points = 20;
  	double point_x = x_points[x_points.size()-1];
  	double point_y = y_points[x_points.size()-1];
  	while( x_points.size() < max_points ){
  	  x_points.push_back(point_x);
  	  y_points.push_back(point_y);
  	  v_points.push_back(0);

  	}
  	return;
  }

  auto goal_set = motion_planner.generate_offset_goals(goal);

  auto spirals = motion_planner.generate_spirals(ego_state, goal_set);

  auto desired_speed = utils::magnitude(goal.velocity);

  State lead_car_state;  // = to the vehicle ahead...

  if(spirals.size() == 0){
  	static int dbg_count = 0;
  	if (dbg_count++ < 3) {
  	  cout << "Error: No spirals generated. ego=(" << ego_state.location.x << "," << ego_state.location.y << ") yaw=" << ego_state.rotation.yaw
  	       << " goal_set.size=" << goal_set.size();
  	  if (!goal_set.empty()) cout << " goal0=(" << goal_set[0].location.x << "," << goal_set[0].location.y << ") g0yaw=" << goal_set[0].rotation.yaw;
  	  cout << endl;
  	} else {
  	  cout << "Error: No spirals generated " << endl;
  	}
  	return;
  }

  for(int i = 0; i < spirals.size(); i++){

    auto trajectory = motion_planner._velocity_profile_generator.generate_trajectory( spirals[i], desired_speed, ego_state,
                                                                                    lead_car_state, behavior);

    vector<double> spiral_x;
    vector<double> spiral_y;
    vector<double> spiral_v;
    for(int j = 0; j < trajectory.size(); j++){
      double point_x = trajectory[j].path_point.x;
      double point_y = trajectory[j].path_point.y;
      double velocity = trajectory[j].v;
      spiral_x.push_back(point_x);
      spiral_y.push_back(point_y);
      spiral_v.push_back(velocity);
    }

    spirals_x.push_back(spiral_x);
    spirals_y.push_back(spiral_y);
    spirals_v.push_back(spiral_v);

  }

  best_spirals = motion_planner.get_best_spiral_idx(spirals, obstacles, goal);
  int best_spiral_idx = -1;

  if(best_spirals.size() > 0)
  	best_spiral_idx = best_spirals[best_spirals.size()-1];

  if(best_spiral_idx < 0 || best_spiral_idx >= (int)spirals_x.size()){
    // No valid spiral found (all paths collide). Keep x_points as-is.
    return;
  }

  int index = 0;
  int max_points = 20;
  int add_points = spirals_x[best_spiral_idx].size();
  while( x_points.size() < max_points && index < add_points ){
    double point_x = spirals_x[best_spiral_idx][index];
    double point_y = spirals_y[best_spiral_idx][index];
    double velocity = spirals_v[best_spiral_idx][index];
    index++;
    x_points.push_back(point_x);
    y_points.push_back(point_y);
    v_points.push_back(velocity);
  }


}

void set_obst(vector<double> x_points, vector<double> y_points, vector<double> yaw_points, vector<State>& obstacles, bool& obst_flag){

	for( int i = 0; i < x_points.size(); i++){
		State obstacle;
		obstacle.location.x = x_points[i];
		obstacle.location.y = y_points[i];
		// Use the actual NPC spawn yaw so collision circles are placed correctly
		// along the NPC's longitudinal axis regardless of map/road orientation.
		// For west-facing NPCs (Town06) yaw=π; for east-facing (Town10HD) yaw=0, etc.
		obstacle.rotation.yaw = (i < (int)yaw_points.size()) ? yaw_points[i] : M_PI;
		obstacles.push_back(obstacle);
	}
	obst_flag = true;
}

int main ()
{
  cout << "starting server" << endl;
  uWS::Hub h;

  double new_delta_time;
  int i = 0;
  double prev_sim_time = -1.0;  // tracks CARLA simulation time for sub-second dt

  // Open data files once; kept open for the run duration (lambda appends each frame).
  // Using ofstream::app avoids the fragile seekg/ignore line-rewrite pattern that fails
  // on an empty file opened with default fstream mode.
  ofstream file_steer("steer_pid_data.txt", ofstream::out | ofstream::trunc);
  ofstream file_throttle("throttle_pid_data.txt", ofstream::out | ofstream::trunc);

  // initialize pid steer
  // Steer: Kp=0.05, Ki=0.0, Kd=0.05. With trajectory-tip ego state the CTE is
  // naturally small (trajectory already avoids obstacles), so light Kd=0.05 damps
  // heading oscillation without stalling lateral convergence.
  PID pid_steer = PID();
  pid_steer.Init(0.05, 0.0, 0.05, 0.3, -0.3);

  // Throttle: Kp=0.10, Ki=0.001, Kd=0.05, limits [-1.0, 1.0].
  // In CARLA sync mode (20 Hz), each C++ update covers ~5 physics steps (1 s).
  // Kp=0.10: at error=-3, throttle=0.30 → ~0.9 m/s gained per update → no overshoot.
  // Kd=0.05 damps rapid changes; Ki=0.001 eliminates steady-state offset.
  PID pid_throttle = PID();
  pid_throttle.Init(0.10, 0.001, 0.05, 1.0, -1.0);

  h.onMessage([&pid_steer, &pid_throttle, &new_delta_time, &prev_sim_time, &i, &file_steer, &file_throttle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode)
  {
        auto s = hasData(data);

        if (s != "") {
          try {

          auto data = json::parse(s);

          vector<double> x_points = data["traj_x"];
          vector<double> y_points = data["traj_y"];
          vector<double> v_points = data["traj_v"];
          double yaw = data["yaw"];
          double velocity = data["velocity"];
          double sim_time = data["time"];
          double waypoint_x = data["waypoint_x"];
          double waypoint_y = data["waypoint_y"];
          double waypoint_t = data["waypoint_t"];
          bool is_junction = data["waypoint_j"];
          string tl_state = data["tl_state"];

          double x_position = data["location_x"];
          double y_position = data["location_y"];
          double z_position = data["location_z"];

          cout << "[FRAME " << i << "] npts=" << x_points.size()
               << " veh=(" << x_position << "," << y_position << ")"
               << " yaw=" << yaw << " vel=" << velocity
               << " jct=" << is_junction
               << " goal=(" << waypoint_x << "," << waypoint_y << ")"
               << endl;
          cout.flush();

          if(!have_obst){
          	vector<double> x_obst = data["obst_x"];
          	vector<double> y_obst = data["obst_y"];
          	// Safe parse: iterate array elements to avoid nlohmann 2.x value() template issues
          	vector<double> yaw_obst;
          	if (data.count("obst_yaw")) {
          	    for (auto& v : data["obst_yaw"]) yaw_obst.push_back(v.get<double>());
          	}
          	set_obst(x_obst, y_obst, yaw_obst, obstacles, have_obst);
          }

          State goal;
          goal.location.x = waypoint_x;
          goal.location.y = waypoint_y;
          goal.rotation.yaw = waypoint_t;

          vector< vector<double> > spirals_x;
          vector< vector<double> > spirals_y;
          vector< vector<double> > spirals_v;
          vector<int> best_spirals;

          // Pass vehicle's actual position as ego so each spiral starts from
          // the real vehicle location rather than the stale trajectory tip.
          // x_points/y_points are NOT reset here — x_points[0] remains the
          // nearest planned waypoint used to compute the cross-track error.
          cout << "[FRAME " << i << "] calling path_planner, behavior_before=" << behavior_planner.get_active_maneuver() << endl;
          cout.flush();
          path_planner(x_points, y_points, v_points, yaw, velocity, goal, is_junction, tl_state, spirals_x, spirals_y, spirals_v, best_spirals);
          cout << "[FRAME " << i << "] path_planner done, npts_out=" << x_points.size() << " behavior_after=" << behavior_planner.get_active_maneuver() << endl;
          cout.flush();

          // Compute delta time from CARLA simulation clock (sub-second resolution).
          // Using sim_time from the payload gives proper time scaling for PID integration.
          if (prev_sim_time < 0.0) {
            new_delta_time = 0.05;  // reasonable default for first frame
          } else {
            new_delta_time = sim_time - prev_sim_time;
            if (new_delta_time <= 0.0) new_delta_time = 0.05;
          }
          prev_sim_time = sim_time;

          ////////////////////////////////////////
          // Steering control
          ////////////////////////////////////////

          // Update the delta time with the previous command
          pid_steer.UpdateDeltaTime(new_delta_time);

          // Cross-Track Error in vehicle frame:
          //   dx, dy = vector from nearest waypoint to vehicle (world frame).
          //   Rotating by -yaw projects onto the vehicle's lateral axis:
          //     lateral_offset = -sin(yaw)*dx + cos(yaw)*dy
          // Positive → vehicle left of path → steer right (negative output from TotalError).
          // Guard against empty trajectory (path_planner returned early with no spirals):
          // hold zero steer error so the integral doesn't wind up during recovery.
          double error_steer = 0.0;
          if (!x_points.empty()) {
            double dx = x_position - x_points[0];
            double dy = y_position - y_points[0];
            error_steer = -sin(yaw) * dx + cos(yaw) * dy;
            // Debug: log if CTE exceeds 5m (sanity check)
            if (fabs(error_steer) > 5.0) {
              cout << "[DBG-CTE] i=" << i << " err=" << error_steer
                   << " veh=(" << x_position << "," << y_position << ")"
                   << " wp0=(" << x_points[0] << "," << y_points[0] << ")"
                   << " yaw=" << yaw << " npts=" << x_points.size() << endl;
            }
          }

          double steer_output;

          // Always update with the real error — never pass 0.0 at low speed.
          // Passing 0.0 when velocity < threshold caused a large derivative spike
          // ((0 - prev_error)/dt) that saturated the steer in the wrong direction
          // and spun the vehicle off course.
          pid_steer.UpdateError(error_steer);
          steer_output = pid_steer.TotalError();

          // Save data (append one line per frame)
          file_steer << i << " " << error_steer << " " << steer_output << "\n";
          file_steer.flush();

          ////////////////////////////////////////
          // Throttle control
          ////////////////////////////////////////

          // Update the delta time with the previous command
          pid_throttle.UpdateDeltaTime(new_delta_time);

          // Use v_points[0] (immediate next waypoint speed) rather than the far-horizon last point.
          // Guard against empty v_points (early return from path_planner): brake gently.
          double error_throttle = 0.0;
          if (!v_points.empty()) {
            error_throttle = velocity - v_points[0];
          } else {
            // No trajectory: brake until planner recovers.
            error_throttle = velocity;
          }

          double throttle_output;
          double brake_output;

          // Compute control to apply
          pid_throttle.UpdateError(error_throttle);
          double throttle = pid_throttle.TotalError();

          // Split PID output: positive → throttle, negative → brake (mutually exclusive)
          if (throttle > 0.0) {
            throttle_output = throttle;
            brake_output = 0;
          } else {
            throttle_output = 0;
            brake_output = -throttle;
          }

          // Save data (append one line per frame)
          file_throttle << i << " " << error_throttle << " " << throttle_output << " " << brake_output << "\n";
          file_throttle.flush();


          // Send control
          json msgJson;
          msgJson["brake"] = brake_output;
          msgJson["throttle"] = throttle_output;
          msgJson["steer"] = steer_output;

          msgJson["trajectory_x"] = x_points;
          msgJson["trajectory_y"] = y_points;
          msgJson["trajectory_v"] = v_points;
          msgJson["spirals_x"] = spirals_x;
          msgJson["spirals_y"] = spirals_y;
          msgJson["spirals_v"] = spirals_v;
          msgJson["spiral_idx"] = best_spirals;
          msgJson["active_maneuver"] = behavior_planner.get_active_maneuver();

          //  min point threshold before doing the update — matches reference implementation
          msgJson["update_point_thresh"] = 16;

          auto msg = msgJson.dump();

          i = i + 1;

      ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      cout << "[FRAME " << (i-1) << "] response sent" << endl;
      cout.flush();

          } catch (const std::exception& e) {
            cout << "[EXCEPTION frame " << i << "] " << e.what() << endl;
            cout.flush();
          } catch (...) {
            cout << "[EXCEPTION frame " << i << "] unknown exception" << endl;
            cout.flush();
          }
    }


  });


  h.onConnection([](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
  {
      cout << "Connected!!!" << endl;
    });


  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length)
    {
      // Do NOT call ws.close() here — socket is already closing; double-close segfaults uWS
      cout << "Disconnected" << endl;
    });

  int port = 4567;
  if (h.listen("0.0.0.0", port))
    {
      cout << "Listening to port " << port << endl;
      h.run();
    }
  else
    {
      cerr << "Failed to listen to port" << endl;
      return -1;
    }


}
