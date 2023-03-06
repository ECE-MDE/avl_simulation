#===============================================================================
#  Autonomous Vehicle Library
#
#  Description: CMake file for the avl_simulation ROS package.
#===============================================================================

# Required CMake version
cmake_minimum_required(VERSION 2.8.3)

# Package name
project(avl_simulation)

# Compile as C++14, supported in ROS Melodic and newer
add_compile_options(-std=c++14)
add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter -Wno-psabi)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  message("${PROJECT_NAME}: no build type specified, choosing Release")
  set(CMAKE_BUILD_TYPE Release)
endif()

#===============================================================================
#                              FIND DEPENDENCIES
#===============================================================================

# Find other Catkin packages required for build
find_package(catkin REQUIRED COMPONENTS
    roscpp
    message_generation
    std_msgs
    std_srvs
    geometry_msgs
    sensor_msgs
    avl_core
    avl_msgs
    avl_devices
    avl_navigation
)

# Find the Eigen3 package required for build
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

# Specify include directories
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

#===============================================================================
#                              CONFIGURE PACKAGE
#===============================================================================

# Specify catkin-specific information to the build system
catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
    CATKIN_DEPENDS
        roscpp
        message_runtime
        std_msgs
        std_srvs
        geometry_msgs
        sensor_msgs
        avl_core
        avl_msgs
        avl_devices
        avl_navigation
    DEPENDS
    CFG_EXTRAS
)

# Create a variable for all exported targets for dependencies
set(EXPORTED_TARGETS ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#===============================================================================
#                                BUILD LIBRARIES
#===============================================================================

# Simulation utilities library
add_library(sim src/sensor_model.cpp)
add_dependencies(sim ${EXPORTED_TARGETS})

#===============================================================================
#                               BUILD EXECUTABLES
#===============================================================================

# Dynamics Nodes ---------------------------------------------------------------

# AUV Dynamics Node
add_executable(auv_dynamics_node src/node/dynamics/auv_dynamics_node.cpp)
add_dependencies(auv_dynamics_node ${EXPORTED_TARGETS})
target_link_libraries(auv_dynamics_node node ${catkin_LIBRARIES})

# Sensor Nodes -----------------------------------------------------------------

# AHRS Simulation Node
add_executable(ahrs_sim_node src/node/sensor/ahrs_sim_node.cpp)
add_dependencies(ahrs_sim_node ${EXPORTED_TARGETS})
target_link_libraries(ahrs_sim_node node sim ${catkin_LIBRARIES})

# Depth Sensor Simulation Node
add_executable(depth_sim_node src/node/sensor/depth_sim_node.cpp)
add_dependencies(depth_sim_node ${EXPORTED_TARGETS})
target_link_libraries(depth_sim_node node sim ${catkin_LIBRARIES})

# DVL Simulation Node
add_executable(dvl_sim_node src/node/sensor/dvl_sim_node.cpp)
add_dependencies(dvl_sim_node ${EXPORTED_TARGETS})
target_link_libraries(dvl_sim_node node sim device ${catkin_LIBRARIES})

# GPS Simulation Node
add_executable(gps_sim_node src/node/sensor/gps_sim_node.cpp)
add_dependencies(gps_sim_node ${EXPORTED_TARGETS})
target_link_libraries(gps_sim_node node sim device ${catkin_LIBRARIES})

# Height Simulation Node
add_executable(height_sim_node src/node/sensor/height_sim_node.cpp)
add_dependencies(height_sim_node ${EXPORTED_TARGETS})
target_link_libraries(height_sim_node node sim device ${catkin_LIBRARIES})

# IMU Simulation Node
add_executable(imu_sim_node src/node/sensor/imu_sim_node.cpp)
add_dependencies(imu_sim_node ${EXPORTED_TARGETS})
target_link_libraries(imu_sim_node node sim ${catkin_LIBRARIES})

# LBL Range Simulation Node
add_executable(lbl_range_sim_node src/node/sensor/lbl_range_sim_node.cpp)
add_dependencies(lbl_range_sim_node ${EXPORTED_TARGETS})
target_link_libraries(lbl_range_sim_node node sim nav ${catkin_LIBRARIES})

# Multibeam Sonar Simulation Node
add_executable(multibeam_sonar_sim_node src/node/sensor/multibeam_sonar_sim_node.cpp)
add_dependencies(multibeam_sonar_sim_node ${EXPORTED_TARGETS})
target_link_libraries(multibeam_sonar_sim_node node sim nav ${catkin_LIBRARIES})

# PingDSP Sonar Simulation Node
add_executable(pingdsp_sonar_sim_node src/node/sensor/pingdsp_sonar_sim_node.cpp)
add_dependencies(pingdsp_sonar_sim_node ${EXPORTED_TARGETS})
target_link_libraries(pingdsp_sonar_sim_node node ${catkin_LIBRARIES})

# Rogue Beacon Simulation Node
add_executable(rogue_beacon_sim_node src/node/sensor/rogue_beacon_sim_node.cpp)
add_dependencies(rogue_beacon_sim_node ${EXPORTED_TARGETS})
target_link_libraries(rogue_beacon_sim_node node sim nav ${catkin_LIBRARIES})

# RPM Sensor Simulation Node
add_executable(rpm_sim_node src/node/sensor/rpm_sim_node.cpp)
add_dependencies(rpm_sim_node ${EXPORTED_TARGETS})
target_link_libraries(rpm_sim_node node sim ${catkin_LIBRARIES})

# WHOI Micromodem Simulation Node
add_executable(whoi_micromodem_sim_node src/node/sensor/whoi_micromodem_sim_node.cpp)
add_dependencies(whoi_micromodem_sim_node ${EXPORTED_TARGETS})
target_link_libraries(whoi_micromodem_sim_node node asio device ${catkin_LIBRARIES})

# Nodes ------------------------------------------------------------------------

# Dynamics Manager Node
add_executable(dynamics_manager_node src/node/dynamics_manager_node.cpp)
add_dependencies(dynamics_manager_node ${EXPORTED_TARGETS})
target_link_libraries(dynamics_manager_node node nav ${catkin_LIBRARIES})

# Simulation Time Node
add_executable(simulation_time_node src/node/simulation_time_node.cpp)
add_dependencies(simulation_time_node ${EXPORTED_TARGETS})
target_link_libraries(simulation_time_node node comms ${catkin_LIBRARIES})

#===============================================================================
#                                  BUILD TESTS
#===============================================================================

#===============================================================================
#                                    INSTALL
#===============================================================================

# Mark executables and/or libraries for installation
install(TARGETS sim
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

# Mark header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
)
