#!/bin/bash

workspace=$(pwd)
source ~/.bashrc

# CAN
gnome-terminal -t "can1" -x sudo bash -c "cd ${workspace}; cd ../..; cd ARX_CAN/arx_can; ./arx_can0.sh; exec bash;"
sleep 0.1
gnome-terminal -t "can3" -x sudo bash -c "cd ${workspace}; cd ../..; cd ARX_CAN/arx_can; ./arx_can1.sh; exec bash;"
sleep 0.1
gnome-terminal -t "can5" -x sudo bash -c "cd ${workspace}; cd ../..; cd ARX_CAN/arx_can; ./arx_can5.sh; exec bash;"
sleep 1

#body
gnome-terminal -t "body" -x  bash -c "cd ${workspace}; cd ../.. ; cd body/ROS; source devel/setup.bash && roslaunch arx_lift_controller x7s.launch; exec bash;"
sleep 3

gnome-terminal -t "L" -x  bash -c "cd ${workspace}; cd ../..; cd x7s/ROS/x7s_ws; source devel/setup.bash && roslaunch arx_x7_controller left_arm_inference.launch; exec bash;"
sleep 0.5
gnome-terminal -t "R" -x  bash -c "cd ${workspace}; cd ../..; cd x7s/ROS/x7s_ws; source devel/setup.bash && roslaunch arx_x7_controller right_arm_inference.launch; exec bash;"