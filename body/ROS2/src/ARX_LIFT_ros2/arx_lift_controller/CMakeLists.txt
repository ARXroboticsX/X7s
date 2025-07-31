cmake_minimum_required(VERSION 3.8)
project(arx_lift_controller)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(arm_control REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)

link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/arx_hardware_interface ${CMAKE_CURRENT_SOURCE_DIR}/lib/arx_lift_src)
add_executable(lift_controller src/lift_controller.cpp)
target_include_directories(lift_controller PUBLIC
        lib/arx_hardware_interface/include lib/arx_lift_src/include)
ament_target_dependencies(lift_controller rclcpp arm_control sensor_msgs tf2)

install(TARGETS
        lift_controller
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch
        FILES_MATCHING PATTERN "*.launch.py")

install(FILES lib/arx_lift_src/libarx_lift_src.so
        DESTINATION lib/
)
target_link_libraries(lift_controller libarx_lift_src.so)
ament_package()
