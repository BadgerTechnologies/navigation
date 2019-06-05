cmake_minimum_required(VERSION 2.8.3)
project(costmap_3d)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  dynamic_reconfigure
  octomap_server
  pluginlib
  roscpp
  geometry_msgs
  moveit_core
  octomap_msgs
)

find_package(OCTOMAP REQUIRED)

include_directories(
  include
  ${OCTOMAP_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

generate_dynamic_reconfigure_options(
  cfg/Costmap3D.cfg
  cfg/GenericPlugin.cfg
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    costmap_3d
    layers_3d
  CATKIN_DEPENDS
    costmap_2d
    dynamic_reconfigure
    octomap_server
    pluginlib
    roscpp
    geometry_msgs
    moveit_core
    octomap_msgs
  DEPENDS
    OCTOMAP
)

add_library(costmap_3d
  src/costmap_3d.cpp
  src/layer_3d.cpp
  src/layered_costmap_3d.cpp
  src/costmap_3d_ros.cpp
  src/costmap_3d_publisher.cpp
  src/costmap_layer_3d.cpp
)

target_link_libraries(costmap_3d
  ${OCTOMAP_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_library(layers_3d
  plugins/octomap_server_layer_3d.cpp
  plugins/costmap_3d_to_2d_layer.cpp
  plugins/costmap_3d_to_2d_layer_3d.cpp
)

target_link_libraries(layers_3d
  costmap_3d
)

add_dependencies(costmap_3d costmap_3d_gencfg)
add_dependencies(layers_3d costmap_3d_gencfg)

#add_executable(costmap_3d_node src/costmap_3d_node.cpp)
#target_link_libraries(costmap_3d_node
#    costmap_3d
#    )

## TODO: Configure Tests

#install(TARGETS
#    costmap_3d_node
#    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

install(TARGETS
    costmap_3d
    layers_3d
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES costmap_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
