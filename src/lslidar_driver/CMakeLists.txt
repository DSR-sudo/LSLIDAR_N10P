cmake_minimum_required(VERSION 3.5)
project(lslidar_driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(libpcap_LIBRARIES -lpcap)

#set(FastRTPS_INCLUDE_DIR /opt/ros/foxy/include)
#set(FastRTPS_LIBRARY_RELEASE /opt/ros/foxy/lib/libfastrtps.so)

#find_package(Boost REQUIRED COMPONENTS )
find_package(Boost REQUIRED thread)
find_package(rclcpp REQUIRED)
find_package(PCL REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(lslidar_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclpy REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)
#find_package(PCL REQUIRED COMPONENTS common io)											   

include_directories(
		include
    ${PCL_INCLUDE_DIRS}
    ${PCL_COMMON_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

# 目录约定：app 是进程入口，driver 是 ROS 驱动编排，input/serial 是传输层，
# lcp 是纯几何状态机，n10p 是无 ROS 的 N10P 协议模块。
add_library(lcp_core
  src/lcp/lcp_core_state.cc
  src/lcp/lcp_core_fit.cc
  src/lcp/lcp_core_map.cc)
target_include_directories(lcp_core PUBLIC include)

# N10P 协议模块不依赖 ROS，可单独做纯 C++ 回归测试。
add_library(n10p_module src/n10p/n10p_module.cc)
target_include_directories(n10p_module PUBLIC include)

# Node
add_executable(lslidar_driver_node
  src/app/lslidar_driver_node.cc
  src/driver/lslidar_driver.cc
  src/driver/lslidar_driver_parameters.cc
  src/driver/lslidar_driver_io.cc
  src/driver/lslidar_driver_commands.cc
  src/driver/lslidar_driver_processing.cc
  src/driver/lslidar_driver_lcp.cc
  src/driver/lslidar_driver_scans.cc
  src/driver/lslidar_driver_polling.cc
  src/input/input_base.cc
  src/input/input_socket.cc
  src/input/input_pcap.cc
  src/serial/lsiosr_core.cpp
  src/serial/lsiosr_io.cpp)
target_link_libraries(lslidar_driver_node lcp_core n10p_module ${rclcpp_LIBRARIES} ${libpcap_LIBRARIES} ${Boost_LIBRARIES} Boost::thread)
ament_target_dependencies(lslidar_driver_node rclcpp std_msgs std_srvs nav_msgs mavros_msgs lslidar_msgs sensor_msgs diagnostic_updater pcl_conversions)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_lcp_core test/lcp_core_test.cc)
  if(TARGET test_lcp_core)
    target_link_libraries(test_lcp_core lcp_core)
  endif()
  ament_add_gtest(test_lcp_live_capture_replay test/lcp_live_capture_replay_test.cc)
  if(TARGET test_lcp_live_capture_replay)
    target_link_libraries(test_lcp_live_capture_replay lcp_core)
    target_compile_definitions(test_lcp_live_capture_replay PRIVATE
      LCP_LIVE_CAPTURE_REPLAY_FILE="${CMAKE_CURRENT_SOURCE_DIR}/test/data/live_capture_20260725_scans.csv")
  endif()
  ament_add_gtest(test_n10p_module test/n10p_module_test.cc)
  if(TARGET test_n10p_module)
    target_link_libraries(test_n10p_module n10p_module)
  endif()
endif()


install(DIRECTORY launch params rviz docs
		DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/
        DESTINATION include)

install(TARGETS
  lcp_core
  n10p_module
  lslidar_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rclcpp pluginlib lslidar_msgs sensor_msgs pcl_conversions mavros_msgs nav_msgs std_srvs)
ament_export_include_directories(include ${PCL_COMMON_INCLUDE_DIRS})

ament_package()
