cmake_minimum_required(VERSION 2.8.3)

project(intraprocess_eval)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

macro(targets)
  if(NOT "${target_suffix} " STREQUAL " ")
    get_rclcpp_information("${rmw_implementation}" "rclcpp${target_suffix}")
  endif()

  # Simple example of using unique_ptr to minimize copies.
  add_executable(chat_intraprocess${target_suffix}
    src/chat_intraprocess.cpp)
  ament_target_dependencies(chat_intraprocess${target_suffix}
    "rclcpp${target_suffix}"
    "std_msgs")

  install(TARGETS
    chat_intraprocess${target_suffix}
    DESTINATION bin)

endmacro()

call_for_each_rmw_implementation(targets GENERATE_DEFAULT)

ament_package()
