cmake_minimum_required(VERSION 2.8.3)

project(interprocess_eval)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()

#find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(std_msgs REQUIRED)

#catkin_package()

function(custom_executable target)
  add_executable(${target}${target_suffix}
    src/${target}.cpp)

  #include_directories(${catkin_INCLUDE_DIRS})
  #target_link_libraries(talker ${catkin_LIBRARIES})
  ament_target_dependencies(${target}${target_suffix}
    "rclcpp"
    "std_msgs"
    "rt")

  #install(TARGETS talker
  #  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  install(TARGETS ${target}${target_suffix}
    DESTINATION bin)

endfunction()

macro(targets)
  if(NOT "${target_suffix} " STREQUAL " ")
    get_rclcpp_information("${rmw_implementation}" "rclcpp${target_suffix}")
  endif()

  custom_executable(talker_interprocess)
  custom_executable(listener_interprocess)
  custom_executable(talker_interprocess_single)
  custom_executable(listener_interprocess_single)

endmacro()

ament_package()
