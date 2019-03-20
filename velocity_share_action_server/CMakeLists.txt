cmake_minimum_required(VERSION 2.8.3)
project(velocity_share_action_server)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  move_base_msgs
  costmap_2d
  geometry_msgs
  roscpp
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES velocity_share_action_server
#  CATKIN_DEPENDS move_base_msgs roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(velocity_share_action_server
  src/velocity_share_action_server.cpp
)

add_dependencies(velocity_share_action_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(velocity_share_action_server
    ${catkin_LIBRARIES}
    )

install(
    TARGETS
        velocity_share_action_server
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
