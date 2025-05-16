#include "arx_x7_controller/x7_controller.hpp"
#include <ros/package.h>

#include "ros/ros.h"
#include <chrono>
#include <thread>
// using namespace std::chrono_literals;

namespace arx::x7 {
X7Controller::X7Controller(ros::NodeHandle nh) {
  ROS_INFO("机械臂开始初始化...");
  int arm_type_ = nh.param("arm_type", 0);

  std::string package_path = ros::package::getPath("arx_x7_controller");
  std::string urdf_path;
  std::string vr_topic_name;
  bool vr = nh.param("vr", true);
  bool keyboard = nh.param("keyboard", false);
  if (arm_type_ == 0) {
    urdf_path = package_path + "/X7Sleft1.urdf";
    if (vr)
      vr_topic_name = "/ARX_VR_L";
    else
      vr_topic_name = "/joint_control";
  } else {
    urdf_path = package_path + "/x7sRIGHT.urdf";
    if (vr)
      vr_topic_name = "/ARX_VR_R";
    else
      vr_topic_name = "/joint_control2";
  }
  interfaces_ptr_ = std::make_shared<InterfacesThread>(
      urdf_path, nh.param("arm_can_id", std::string("can0")), arm_type_);
  interfaces_ptr_->arx_x(500, 2000, 10);
  // sub
  if (keyboard)
    normal_cmd_subscriber_ = nh.subscribe<arx5_arm_msg::RobotCmd>(
        "/arm_cmd", 10, &X7Controller::robotCmdCallback, this);
  else if (vr)
    vr_cmd_subscriber_ = nh.subscribe<arm_control::PosCmd>(
        vr_topic_name, 10, &X7Controller::CmdCallbackV1, this);
  else
    vr_cmd_subscriber_ = nh.subscribe<arm_control::JointControl>(
        vr_topic_name, 10, &X7Controller::CmdCallbackV2, this);
  // pub
  ee_pos_publisher_ =
      nh.advertise<arm_control::PosCmd>("/follow1_pos_back", 10);
  joint_state_publisher_ =
      nh.advertise<arm_control::JointInformation>("/joint_information", 10);

  timer_ = nh.createTimer(ros::Duration(0.01), &X7Controller::PubState, this);
}

X7Controller::~X7Controller() {
  interfaces_ptr_->setArmStatus(InterfacesThread::state::PROTECT);
}

void X7Controller::CmdCallbackV1(const arm_control::PosCmd::ConstPtr &msg) {
  double input[6] = {msg->x, msg->y, msg->z, msg->roll, msg->pitch, msg->yaw};
  Eigen::Isometry3d transform = solve::Xyzrpy2Isometry(input);
  interfaces_ptr_->setEndPose(transform);
  interfaces_ptr_->setArmStatus(InterfacesThread::state::END_CONTROL);
  interfaces_ptr_->setCatch(msg->gripper);
}

void X7Controller::CmdCallbackV2(
    const arm_control::JointControl::ConstPtr &msg) {
  std::vector<double> joint_positions = {0, 0, 0, 0, 0, 0, 0};
  for (int i = 0; i < 7; i++)
    joint_positions[i] = msg->joint_pos[i];

  interfaces_ptr_->setJointPositions(joint_positions);
  interfaces_ptr_->setArmStatus(InterfacesThread::state::POSITION_CONTROL);
  interfaces_ptr_->setCatch(msg->joint_pos[7]);
}

void X7Controller::robotCmdCallback(
    const arx5_arm_msg::RobotCmd::ConstPtr &msg) {
  double end_pos[6] = {msg->end_pos[0], msg->end_pos[1], msg->end_pos[2],
                       msg->end_pos[3], msg->end_pos[4], msg->end_pos[5]};

  Eigen::Isometry3d transform = solve::Xyzrpy2Isometry(end_pos);

  interfaces_ptr_->setEndPose(transform);

  std::vector<double> joint_positions = {0, 0, 0, 0, 0, 0};

  for (int i = 0; i < 6; i++) {
    joint_positions[i] = msg->joint_pos[i];
  }

  interfaces_ptr_->setJointPositions(joint_positions);

  interfaces_ptr_->setArmStatus(msg->mode);

  interfaces_ptr_->setCatch(msg->gripper);
}
// Publisher
void X7Controller::PubState(const ros::TimerEvent &) {

  Eigen::Isometry3d transform = interfaces_ptr_->getEndPose();

  // 填充vector

  std::vector<double> xyzrpy = solve::Isometry2Xyzrpy(transform);

  std::vector<double> joint_pos_vector = interfaces_ptr_->getJointPositons();

  std::vector<double> joint_velocities_vector =
      interfaces_ptr_->getJointVelocities();

  std::vector<double> joint_current_vector = interfaces_ptr_->getJointCurrent();

  // 发布消息
  ROS_INFO("Publishing RobotStatus message");

  pubArmStatus(joint_pos_vector, joint_velocities_vector, joint_current_vector,
               xyzrpy);
}

void X7Controller::pubArmStatus(std::vector<double> joint_pos_vector,
                                std::vector<double> joint_velocities_vector,
                                std::vector<double> joint_current_vector,
                                std::vector<double> xyzrpy) {
  arm_control::JointInformation msg;

  for (int i = 0; i < 8; i++) {
    msg.joint_pos[i] = joint_pos_vector[i];
  }
  for (int i = 0; i < 8; i++) {
    msg.joint_vel[i] = joint_velocities_vector[i];
  }
  for (int i = 0; i < 8; i++) {
    msg.joint_cur[i] = joint_current_vector[i];
  }

  joint_state_publisher_.publish(msg);

  arm_control::PosCmd msg_pos_cmd;

  msg_pos_cmd.x = xyzrpy[0];
  msg_pos_cmd.y = xyzrpy[1];
  msg_pos_cmd.z = xyzrpy[2];
  msg_pos_cmd.roll = xyzrpy[3];
  msg_pos_cmd.pitch = xyzrpy[4];
  msg_pos_cmd.yaw = xyzrpy[5];

  msg_pos_cmd.gripper = joint_pos_vector[6];

  ee_pos_publisher_.publish(msg_pos_cmd);
}
} // namespace arx::x7

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "x7_controller");
  ros::NodeHandle nh = ros::NodeHandle("~");
  arx::x7::X7Controller controller(nh);
  ros::spin();
  return 0;
}