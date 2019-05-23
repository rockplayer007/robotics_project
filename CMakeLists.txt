cmake_minimum_required(VERSION 2.8.3)
project(project)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation roscpp std_msgs message_filters dynamic_reconfigure tf)

generate_dynamic_reconfigure_options(
  cfg/configs.cfg
  
)
  
add_message_files(FILES
  floatStamped.msg
)

generate_messages(DEPENDENCIES
  std_msgs
)

## Declare a catkin package
catkin_package( CATKIN_DEPENDS message_filters message_runtime)


## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(odometry
  src/odometry.cpp
)
add_dependencies(odometry ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(odometry ${PROJECT_NAME}_gencfg)
target_link_libraries(odometry
  ${catkin_LIBRARIES}
)



