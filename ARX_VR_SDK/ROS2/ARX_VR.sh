#!/bin/bash
source ~/.bashrc

workspace=$(pwd)

gnome-terminal -t "unity_tcp" -x bash -c "cd ${workspace};source install/setup.bash;ros2 run serial_port serial_port_node;exec bash;"

sleep 0.1
gnome-terminal -t "arx5_pos_cmd" -x bash -c "cd ${workspace};source install/setup.bash;ros2 topic hz /ARX_VR_R;exec bash;"

