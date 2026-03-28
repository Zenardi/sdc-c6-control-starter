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

void path_planner(vector<double>& x_points, vector<double>& y_points, vector<double>& v_points, double yaw, double velocity, double x_ego, double y_ego, State goal, bool is_junction, string tl_state, vector< vector<double> >& spirals_x, vector< vector<double> >& spirals_y, vector< vector<double> >& spirals_v, vector<int>& best_spirals){

  State ego_state;

  // Use trajectory-tip (last planned point) as planning origin — same as the
  // reference implementation. This ensures each new spiral smoothly continues
  // from the end of the previous one, giving stable CTE tracking.
  // If trajectory is empty fall back to actual vehicle position.
  if (x_points.size() > 0) {
    ego_state.location.x = x_points[x_points.size() - 1];
    ego_state.location.y = y_points[y_points.size() - 1];
  } else {
    ego_state.location.x = x_ego;
    ego_state.location.y = y_ego;
  }
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

void set_obst(vector<double> x_points, vector<double> y_points, vector<State>& obstacles, bool& obst_flag){

	for( int i = 0; i < x_points.size(); i++){
		State obstacle;
		obstacle.location.x = x_points[i];
		obstacle.location.y = y_points[i];
		// NPCs face west (yaw = π). Default yaw=0 (east) would misplace collision
		// circles 3m EAST of the NPC, causing false collisions 3m too early.
		// Correct west-facing yaw places circles in the NPC's actual footprint.
		obstacle.rotation.yaw = M_PI;
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

  fstream file_steer;
  file_steer.open("steer_pid_data.txt", std::ofstream::out | std::ofstream::trunc);
  file_steer.close();
  fstream file_throttle;
  file_throttle.open("throttle_pid_data.txt", std::ofstream::out | std::ofstream::trunc);
  file_throttle.close();

  // initialize pid steer
  // Kp=0.05, Ki=0.0, Kd=0.05: reference used Kd=0.25 but that was tuned for a ~10 Hz VM.
  // Our CARLA 0.9.16 runs at ~20 Hz (dt≈0.05 s). Effective Kd/dt gain = 0.25/0.05 = 5.0 →
  // even a 0.06 m CTE change per frame saturates the ±0.3 output. Scaling Kd by dt ratio:
  // Kd = 0.25 × (0.05/0.1) ≈ 0.06. Use 0.05 to be safe.
  PID pid_steer = PID();
  pid_steer.Init(0.05, 0.0, 0.05, 0.3, -0.3);

  // Throttle: Kp=0.3, Ki=0.05, Kd=0.0, limits [-1.0, 1.0].
  // Higher Ki (0.05) eliminates persistent speed offset over time.
  // Kd=0 avoids amplifying noisy velocity signal.
  PID pid_throttle = PID();
  pid_throttle.Init(0.3, 0.05, 0.0, 1.0, -1.0);

  h.onMessage([&pid_steer, &pid_throttle, &new_delta_time, &prev_sim_time, &i](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode)
  {
        auto s = hasData(data);

        if (s != "") {

          auto data = json::parse(s);

          // create file to save values
          fstream file_steer;
          file_steer.open("steer_pid_data.txt");
          fstream file_throttle;
          file_throttle.open("throttle_pid_data.txt");

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

          if(!have_obst){
          	vector<double> x_obst = data["obst_x"];
          	vector<double> y_obst = data["obst_y"];
          	set_obst(x_obst, y_obst, obstacles, have_obst);
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
          path_planner(x_points, y_points, v_points, yaw, velocity, x_position, y_position, goal, is_junction, tl_state, spirals_x, spirals_y, spirals_v, best_spirals);

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

          // Compute steer error
          double error_steer;

          // Cross-Track Error in vehicle frame:
          //   dx, dy = vector from nearest waypoint to vehicle (world frame).
          //   Rotating by -yaw projects onto the vehicle's lateral axis:
          //     lateral_offset = -sin(yaw)*dx + cos(yaw)*dy
          // Positive → vehicle left of path → steer right (negative output from TotalError).
          double dx = x_position - x_points[0];
          double dy = y_position - y_points[0];
          error_steer = -sin(yaw) * dx + cos(yaw) * dy;

          double steer_output;

          // Always update with the real error — never pass 0.0 at low speed.
          // Passing 0.0 when velocity < threshold caused a large derivative spike
          // ((0 - prev_error)/dt) that saturated the steer in the wrong direction
          // and spun the vehicle off course.
          pid_steer.UpdateError(error_steer);
          steer_output = pid_steer.TotalError();

          // Save data
          file_steer.seekg(std::ios::beg);
          for(int j=0; j < i - 1; ++j) {
              file_steer.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
          }
          file_steer  << i ;
          file_steer  << " " << error_steer;
          file_steer  << " " << steer_output << endl;

          ////////////////////////////////////////
          // Throttle control
          ////////////////////////////////////////

          // Update the delta time with the previous command
          pid_throttle.UpdateDeltaTime(new_delta_time);

          // Compute error of speed
          double error_throttle;
          // Use v_points[0] (immediate next waypoint speed) rather than the far-horizon last point.
          // This ensures the controller responds to planner deceleration commands near obstacles:
          // when the planner slows to stop before a parked NPC, v_points[0] drops → positive error
          // → brake output. Using v_points.back() (far horizon) would ignore the deceleration.
          // With TotalError()'s negative sign: -Kp*(actual-desired) → throttle when too slow, brake when too fast.
          error_throttle = velocity - v_points[0];

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

          // Save data
          file_throttle.seekg(std::ios::beg);
          for(int j=0; j < i - 1; ++j){
              file_throttle.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
          }
          file_throttle  << i ;
          file_throttle  << " " << error_throttle;
          file_throttle  << " " << throttle_output;
          file_throttle  << " " << brake_output << endl;


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

          //  min point threshold before doing the update
          // 4 gives ~4x more frequent updates than 16, tightening the feedback loop
          msgJson["update_point_thresh"] = 4;

          auto msg = msgJson.dump();

          i = i + 1;
          file_steer.close();
          file_throttle.close();

      ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

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
