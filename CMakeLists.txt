cmake_minimum_required(VERSION 3.8)
project(uwb_ros_bridge_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(jsoncpp CONFIG REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(uwb_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(gz-transport12 QUIET REQUIRED OPTIONAL_COMPONENTS log)

add_executable(bridge
    src/bridge.cpp
    src/tinyxml2.cpp
)

target_include_directories(bridge PUBLIC include/)
target_link_libraries(bridge JsonCpp::JsonCpp)

set(GZ_TRANSPORT_VER ${gz-transport12_VERSION_MAJOR})
target_link_libraries(bridge gz-transport${GZ_TRANSPORT_VER}::core)

ament_target_dependencies(bridge rclcpp nav_msgs geometry_msgs uwb_interfaces)

install(TARGETS bridge DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
