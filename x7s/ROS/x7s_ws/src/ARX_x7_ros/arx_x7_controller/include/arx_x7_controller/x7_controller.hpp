#pragma once

#include "arx5_arm_msg/RobotCmd.h"
#include "arx5_arm_msg/RobotStatus.h"
#include "arx_x7_src/interfaces/InterfacesThread.hpp"
#include <ros/ros.h>

#include "arm_control/JointControl.h"
#include "arm_control/JointInformation.h"
#include "arm_control/PosCmd.h"

#include <chrono>
#include <memory>

namespace arx::x7 {
class X7Controller {
public:
  X7Controller(ros::NodeHandle nh);

  ~X7Controller();
  // sub
  void CmdCallbackV1(const arm_control::PosCmd::ConstPtr &msg);
  void CmdCallbackV2(const arm_control::JointControl::ConstPtr &msg);
  void robotCmdCallback(const arx5_arm_msg::RobotCmd::ConstPtr &msg);

  // pub
  void PubState(const ros::TimerEvent &);
  // 发布old_joint_state_publisher_和old_ee_pos_publisher_信息
  void pubArmStatus(std::vector<double> joint_pos_vector,
                    std::vector<double> joint_velocities_vector,
                    std::vector<double> joint_current_vector,
                    std::vector<double> xyzrpy);

private:
  std::shared_ptr<InterfacesThread> interfaces_ptr_;

  ros::Publisher joint_state_publisher_;
  ros::Publisher ee_pos_publisher_;
  ros::Subscriber vr_cmd_subscriber_;
  ros::Subscriber normal_cmd_subscriber_;
  ros::Timer timer_;
};
} // namespace arx::x7
