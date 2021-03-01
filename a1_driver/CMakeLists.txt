cmake_minimum_required(VERSION 3.5)
project(a1_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

find_package(rclcpp REQUIRED)
find_package(a1_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
    include
    ${Boost_INCLUDE_DIR}
    ${colcon_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    $ENV{UNITREE_LEGGED_SDK_PATH}/include
)
#include_directories(include ${colcon_INCLUDE_DIRS})
link_directories($ENV{UNITREE_LEGGED_SDK_PATH}/lib)
string(CONCAT LEGGED_SDK_NAME libunitree_legged_sdk_$ENV{UNITREE_PLATFORM}.so)
set(EXTRA_LIBS ${LEGGED_SDK_NAME} lcm)

set(CMAKE_CXX_FLAGS "-O3")

find_package(a1_msgs REQUIRED)
add_executable(pub_mode src/msg_pub.cpp)
target_link_libraries(pub_mode ${EXTRA_LIBS} ${catkin_LIBRARIES})
ament_target_dependencies(pub_mode rclcpp ${${PEOJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} a1_msgs)

add_executable(sub_mode src/a1_ros.cpp src/msg_comm.cpp)
target_link_libraries(sub_mode ${EXTRA_LIBS} ${catkin_LIBRARIES})
ament_target_dependencies(sub_mode rclcpp ${${PEOJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} a1_msgs)

add_executable(pub_vel src/msg_pub.cpp)
target_link_libraries(pub_mode ${EXTRA_LIBS} ${catkin_LIBRARIES})
ament_target_dependencies(pub_vel rclcpp ${${PEOJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} a1_msgs)

add_executable(sub_vel src/a1_ros.cpp src/msg_comm.cpp)
target_link_libraries(sub_mode ${EXTRA_LIBS} ${catkin_LIBRARIES})
ament_target_dependencies(sub_vel rclcpp ${${PEOJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} a1_msgs)

add_executable(send src/a1_wrapper.cpp src/msg_comm.cpp)
target_link_libraries(send ${EXTRA_LIBS} ${colcon_LIBRARIES})
ament_target_dependencies(send rclcpp ${${PEOJECT_NAME}_EXPORTED_TARGETS} ${colcon_EXPORTED_TARGETS})

install(TARGETS
  pub_mode
  sub_mode
  pub_vel
  sub_vel
  send
  DESTINATION lib/${PROJECT_NAME})



ament_package()
