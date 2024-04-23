cmake_minimum_required(VERSION 3.8)
project(kangsan_gripper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(include)

add_executable(kangsan_gripper src/epos4_force_test.cpp)
ament_target_dependencies(kangsan_gripper rclcpp std_msgs std_srvs)
target_link_libraries(kangsan_gripper -lEposCmd)



install(TARGETS
  kangsan_gripper
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
