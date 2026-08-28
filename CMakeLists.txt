cmake_minimum_required(VERSION 2.8.3)
project(groundgrid)

# use c++17
SET(CMAKE_CXX_STANDARD 17)

######################################
## Find catkin macros and libraries ##
######################################

find_package(catkin REQUIRED COMPONENTS
    dynamic_reconfigure
    nodelet
    roscpp
    geometry_msgs
    pcl_ros
    pcl_conversions    
    grid_map_core
    grid_map_ros
    grid_map_cv
    grid_map_msgs
    cv_bridge
    image_transport
    std_msgs
    sensor_msgs
    nav_msgs
    tf2
    tf2_ros
    tf2_msgs
    tf2_geometry_msgs
    roslib
    message_generation
)

################################
## include common CMake setup ##
################################
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(Eigen3 REQUIRED)

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest(test/lunar_pipeline.test)
    catkin_add_nosetests(test/test_skidsteer_identification.py)
endif()

#######################################
## dynamic_reconfigure configuration ##
#######################################

generate_dynamic_reconfigure_options(
    cfg/GroundGrid.cfg
)

################################################
## Declare ROS messages, services and actions ##
################################################
add_message_files(
    FILES
    LunarTrajectory.msg
)

add_service_files(
    FILES
    NextCloud.srv
)


## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    geometry_msgs
    nav_msgs
    std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}_lib
    CATKIN_DEPENDS
        dynamic_reconfigure
        geometry_msgs
        message_runtime
        nav_msgs
        nodelet
        roscpp
        std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    include
)


#####################################################
## Build library, node and nodelet of this package ##
#####################################################

## Declare a cpp library
add_library(${PROJECT_NAME}_lib
    src/GroundGrid.cpp
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(${PROJECT_NAME}_lib
    ${PROJECT_NAME}_gencfg
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_lib
    ${catkin_LIBRARIES}
)

## Declare a cpp library
add_library(${PROJECT_NAME}_groundsegmentation_lib
	src/GroundSegmentation.cpp
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(${PROJECT_NAME}_groundsegmentation_lib
    ${PROJECT_NAME}_gencfg
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_groundsegmentation_lib
    ${catkin_LIBRARIES}
)

## Pure C++ skid-steer dynamics + motion primitive library.
## No ROS/grid_map deps on purpose, so it can be extracted for the Atlas 200i target.
add_library(${PROJECT_NAME}_skidsteer_lib
    src/SkidSteerModel.cpp
    src/MotionPrimitiveLibrary.cpp
)
target_include_directories(${PROJECT_NAME}_skidsteer_lib PUBLIC include)
set_target_properties(${PROJECT_NAME}_skidsteer_lib PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)

## determines target name as lib[name].so, here: libfub_dummy_nodelet.so
## the library path in nodelet_plugins.xml refers to this file
add_library(${PROJECT_NAME}_nodelet
    src/GroundGridNodelet.cpp
)

add_dependencies(${PROJECT_NAME}_nodelet
    ${PROJECT_NAME}_gencfg
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_lib
    ${PROJECT_NAME}_groundsegmentation_lib
)

target_link_libraries(${PROJECT_NAME}_nodelet
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${PROJECT_NAME}_lib
    ${PROJECT_NAME}_groundsegmentation_lib
)

## Declare a cpp executable
add_executable(${PROJECT_NAME}_node
    src/GroundGridNode.cpp
)

add_dependencies(${PROJECT_NAME}_node
    ${PROJECT_NAME}_nodelet
)

target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
)

add_executable(lunar_traversability_node src/LunarTraversabilityNode.cpp)
add_executable(state_lattice_planner_node src/StateLatticePlannerNode.cpp)
add_executable(lunar_path_follower_node src/LunarPathFollowerNode.cpp)

add_dependencies(lunar_traversability_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(state_lattice_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(lunar_path_follower_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(lunar_traversability_node ${catkin_LIBRARIES})
target_link_libraries(state_lattice_planner_node ${catkin_LIBRARIES} ${PROJECT_NAME}_skidsteer_lib)
target_link_libraries(lunar_path_follower_node ${catkin_LIBRARIES} ${PROJECT_NAME}_skidsteer_lib)

## Offline tools built on the pure C++ library (no ROS runtime needed).
add_executable(generate_motion_primitives src/generate_motion_primitives.cpp)
target_link_libraries(generate_motion_primitives ${PROJECT_NAME}_skidsteer_lib)

add_executable(skidsteer_selfcheck src/skidsteer_selfcheck.cpp)
target_link_libraries(skidsteer_selfcheck ${PROJECT_NAME}_skidsteer_lib)
if(CATKIN_ENABLE_TESTING)
    add_test(NAME skidsteer_selfcheck COMMAND skidsteer_selfcheck)
endif()

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(
    TARGETS
        ${PROJECT_NAME}_lib
        ${PROJECT_NAME}_groundsegmentation_lib
        ${PROJECT_NAME}_skidsteer_lib
        ${PROJECT_NAME}_node
        ${PROJECT_NAME}_nodelet
        lunar_traversability_node
        state_lattice_planner_node
        lunar_path_follower_node
        generate_motion_primitives
        skidsteer_selfcheck
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
)


## Mark other files for installation (e.g. launch and bag files, etc.)
install(
    DIRECTORY launch config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Install Python scripts (slope raster saver + offline computer)
catkin_install_python(PROGRAMS
    scripts/save_slope_raster.py
    scripts/compute_slope_offline.py
    scripts/kitti_data_publisher.py
    scripts/eval_groundpoint_classifier.py
    scripts/lunar_surface_sim.py
    scripts/identify_skidsteer.py
    scripts/lunar_terrain.py
    scripts/skidsteer_identification_core.py
    scripts/run_planner_experiments.py
    scripts/summarize_planner_metrics.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    FILES nodelet_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
