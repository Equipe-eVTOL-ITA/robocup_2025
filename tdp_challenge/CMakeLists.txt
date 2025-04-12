cmake_minimum_required(VERSION 3.8)
project(tdp_challenge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#find dependencies
find_package(ament_cmake REQUIRED)
find_package(fsm REQUIRED)
find_package(robocup_drone_lib REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(vision_msgs REQUIRED)

include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})

#position

add_executable(position
  src/position.cpp
)

ament_target_dependencies(position
  rclcpp
  fsm
  robocup_drone_lib
  Eigen3
  vision_msgs
)

target_link_libraries(position ${OpenCV_LIBS})


#position_dummy

add_executable(position_dummy
  src/position_dummy.cpp
)

ament_target_dependencies(position_dummy
  rclcpp
  fsm
  robocup_drone_lib
  Eigen3
  vision_msgs
)

target_link_libraries(position ${OpenCV_LIBS})

#transverse

add_executable(transverse
  src/transverse.cpp
)

ament_target_dependencies(transverse
  rclcpp
  fsm
  robocup_drone_lib
  Eigen3
  vision_msgs
)

target_link_libraries(transverse ${OpenCV_LIBS})

#Install

install(
  TARGETS
    position
    position_dummy
    transverse
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
