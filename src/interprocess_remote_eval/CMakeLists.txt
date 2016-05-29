cmake_minimum_required(VERSION 2.8.3)

project(interprocess_remote_eval)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(std_msgs REQUIRED)

function(custom_executable target)
  add_executable(${target}${target_suffix}
    src/${target}.cpp)
  ament_target_dependencies(${target}${target_suffix}
    "rclcpp"
    "std_msgs"
	"rt")
  install(TARGETS ${target}${target_suffix}
    DESTINATION bin)
endfunction()

macro(targets)
  if(NOT "${target_suffix} " STREQUAL " ")
    get_rclcpp_information("${rmw_implementation}" "rclcpp${target_suffix}")
  endif()

  # Examples of Publish/Subscribe with Topics
  custom_executable(talker_client_interprocess)
  custom_executable(listener_server_interprocess)
endmacro()

call_for_each_rmw_implementation(targets GENERATE_DEFAULT)

ament_package()
