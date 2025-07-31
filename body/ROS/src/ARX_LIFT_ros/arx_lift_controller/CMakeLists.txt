cmake_minimum_required(VERSION 3.8)
project(arx_lift_controller)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        arm_control
        sensor_msgs
        realtime_tools
)

catkin_package()

include_directories(
        ${catkin_INCLUDE_DIRS}
        lib/arx_lift_src/include
        lib/arx_hardware_interface/include
)

link_directories(lib/arx_hardware_interface lib/arx_lift_src)

add_executable(lift_controller src/lift_controller.cpp)
target_link_libraries(lift_controller
        ${catkin_LIBRARIES}
        libarx_lift_src.so
)
add_dependencies(lift_controller arm_control_generate_messages_cpp)
