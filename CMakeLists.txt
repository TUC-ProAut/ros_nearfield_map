cmake_minimum_required(VERSION 3.0.2)
project(nearfield_map)

find_package(catkin REQUIRED COMPONENTS
  roscpp

  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_srvs
  message_generation

  tf

  parameter_pa
  octomap_pa
  pcdfilter_pa
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_service_files(
  FILES
    NearfieldMapFileName.srv
    NearfieldMapGetSize.srv
    NearfieldMapChangeSettings.srv
    NearfieldMapRequest.srv
)
generate_messages(
  DEPENDENCIES
    std_msgs
    sensor_msgs
    std_srvs
)

catkin_package(
  DEPENDS

  INCLUDE_DIRS
    include

  CATKIN_DEPENDS
    roscpp

    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    std_srvs
    message_runtime

    tf

    parameter_pa
    octomap_pa
    pcdfilter_pa

  LIBRARIES
)

# simple node
add_executable(${PROJECT_NAME}_node
  src/nearfield_map_pa_node.cpp
  src/nearfield_map_pa_node_parameter.cpp
)
add_dependencies(${PROJECT_NAME}_node
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  ${PROJECT_NAME}_gencpp
)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

# degrading node - pa
add_executable(${PROJECT_NAME}_degrading_pa_node
  src/nearfield_map_degrading_pa_node.cpp
  src/nearfield_map_pa_node_parameter.cpp
)
add_dependencies(${PROJECT_NAME}_degrading_pa_node
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  ${PROJECT_NAME}_gencpp
)
target_link_libraries(${PROJECT_NAME}_degrading_pa_node
  ${catkin_LIBRARIES}
)

# degrading node - native
add_executable(${PROJECT_NAME}_degrading_native_node
  src/nearfield_map_degrading_native_node.cpp
  src/nearfield_map_pa_node_parameter.cpp
)
add_dependencies(${PROJECT_NAME}_degrading_native_node
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  ${PROJECT_NAME}_gencpp
)
target_link_libraries(${PROJECT_NAME}_degrading_native_node
  ${catkin_LIBRARIES}
)
