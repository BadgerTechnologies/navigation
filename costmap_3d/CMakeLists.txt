cmake_minimum_required(VERSION 2.8.3)
project(costmap_3d)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  dynamic_reconfigure
  actionlib_msgs
  pluginlib
  roscpp
  geometry_msgs
  octomap_msgs
  pcl_ros
  pcl_msgs
  pcl_conversions
  tf_conversions
  message_generation
)

find_package(OCTOMAP REQUIRED)
find_package(PCL 1.7 REQUIRED)

# Find FCL (flexible collision library)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBFCL_PC REQUIRED fcl)
# find *absolute* paths to LIBFCL_INCLUDE_DIRS and LIBFCL_LIBRARIES
find_path(LIBFCL_INCLUDE_DIRS fcl/config.h HINTS ${LIBFCL_PC_INCLUDE_DIR} ${LIBFCL_PC_INCLUDE_DIRS})
find_library(LIBFCL_LIBRARIES fcl HINTS ${LIBFCL_PC_LIBRARY_DIRS})

include_directories(
  include
  ${OCTOMAP_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${LIBFCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

generate_dynamic_reconfigure_options(
  cfg/Costmap3D.cfg
  cfg/GenericPlugin.cfg
)

add_action_files(
  FILES
  GetPlanCost3D.action
)

add_service_files(
  FILES
  GetPlanCost3DService.srv
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
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
    pluginlib
    roscpp
    geometry_msgs
    octomap_msgs
    pcl_ros
    pcl_msgs
    pcl_conversions
    tf_conversions
  DEPENDS
    OCTOMAP
    PCL
    LIBFCL
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
  ${PCL_LIBRARIES}
  ${LIBFCL_LIBRARIES}
  ${catkin_LIBRARIES}
)
target_compile_options(costmap_3d PRIVATE ${EXTRA_COMPILE_OPTIONS})

add_library(layers_3d
  plugins/octomap_server_layer_3d.cpp
  plugins/point_cloud_layer_3d.cpp
  plugins/costmap_3d_to_2d_layer.cpp
  plugins/costmap_3d_to_2d_layer_3d.cpp
)

target_link_libraries(layers_3d
  costmap_3d
)
target_compile_options(layers_3d PRIVATE ${EXTRA_COMPILE_OPTIONS})

add_dependencies(costmap_3d ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(layers_3d ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(costmap_3d_node src/costmap_3d_node.cpp)
target_link_libraries(costmap_3d_node
    costmap_3d
    )
target_compile_options(costmap_3d_node PRIVATE ${EXTRA_COMPILE_OPTIONS})

## TODO: Configure Tests

install(TARGETS
    costmap_3d_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
    costmap_3d
    layers_3d
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES
  costmap_2d_plugins.xml
  costmap_3d_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
