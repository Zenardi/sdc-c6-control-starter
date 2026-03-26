project(PidController)

cmake_minimum_required (VERSION 3.10)

# -g allows for gdb debugging
# turn on -O3 for best performance
add_definitions(-std=c++17 -O3)

set(CXX_FLAGS "-pthread -fPIC -DNDEBUG -Wall -Wextra")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

set(sources main.cpp behavior_planner_FSM.cpp motion_planner.cpp cubic_spiral.cpp spiral_base.cpp integral.cpp spiral_equations.cpp cost_functions.cpp utils.cpp velocity_profile_generator.cpp pid_controller.cpp)

# ---------------------------------------------------------------------------
# Local CARLA stub headers (replaces /opt/carla-source which is not present).
# project/CARLA is a binary-only 0.9.16 install; the C++ client SDK is not
# included.  We provide minimal stubs for the types actually used at compile
# time.  The runtime communication is WebSocket (uWS) via simulatorAPI.py.
# ---------------------------------------------------------------------------
set(LOCAL_CARLA_STUBS_DIR ${CMAKE_CURRENT_SOURCE_DIR})  # carla/ lives here

# rpc library bundled with the local CARLA binary install
set(CARLA_RPC_LIB
    ${CMAKE_CURRENT_SOURCE_DIR}/../CARLA/CarlaUE4/Plugins/Carla/CarlaDependencies/lib/librpc.a)

# System Boost 1.88 (installed via apt libboost-all-dev)
set(BOOST_INCLUDE_DIR /usr/include)
set(BOOST_LIB_DIR     /usr/lib/x86_64-linux-gnu)

# uWS is installed system-wide at /usr/include/uWS and /usr/lib/libuWS.so
include_directories(/usr/include)
link_directories(/usr/lib)

include_directories(eigen-3.3.7)
include_directories(${LOCAL_CARLA_STUBS_DIR})
include_directories(${BOOST_INCLUDE_DIR})

link_directories(${BOOST_LIB_DIR})

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

add_executable(pid_controller ${sources})

target_link_libraries(pid_controller
    z ssl uWS
    ${CARLA_RPC_LIB}
    gtest glog
    boost_filesystem boost_system boost_program_options
    lapack
)

