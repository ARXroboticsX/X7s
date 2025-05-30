#!/bin/bash

workspace=$(pwd)
source ~/.bashrc

# CAN
gnome-terminal -t "can1" -x sudo bash -c "cd ${workspace};cd ../.. ; cd ARX_CAN/arx_can; ./arx_can0.sh; exec bash;"
sleep 0.1
gnome-terminal -t "can3" -x sudo bash -c "cd ${workspace};cd ../.. ; cd ARX_CAN/arx_can; ./arx_can1.sh; exec bash;"
sleep 0.1
gnome-terminal -t "can5" -x sudo bash -c "cd ${workspace};cd ../.. ; cd ARX_CAN/arx_can; ./arx_can5.sh; exec bash;"
sleep 1

#body
gnome-terminal -t "body" -x  bash -c "cd ${workspace}; cd ../..; cd body/ROS2; source install/setup.bash && ros2 launch arx_lift_controller x7s.launch.py ; exec bash;"
#x7s
sleep 3
gnome-terminal -t "L" -x  bash -c "cd ${workspace}; cd ../..; cd x7s/ROS2/x7s_ws; source install/setup.bash && ros2 launch arx_x7_controller left_arm.launch.py; exec bash;"
sleep 0.5
gnome-terminal -t "R" -x  bash -c "cd ${workspace}; cd ../..; cd x7s/ROS2/x7s_ws; source install/setup.bash && ros2 launch arx_x7_controller right_arm.launch.py; exec bash;"

# gnome-terminal -t "pub" -x bash -c "source ~/.bashrc;rosrun joy joy_node;exec bash;"
#VR
gnome-terminal -t "unity_tcp" -x  bash -c "cd ${workspace}; cd ../..; cd ARX_VR_SDK/ROS2; source install/setup.bash && ros2 run serial_port serial_port_node;exec bash;"
sleep 0.1
gnome-terminal -t "arx5_pos_cmd" -x  bash -c "cd ${workspace}; cd ../..; cd ARX_VR_SDK/ROS2; source install/setup.bash && ros2 topic echo /ARX_VR_L;exec bash;"
