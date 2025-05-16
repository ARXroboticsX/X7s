#include "arx_x7_controller/x7_controller.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <csignal>

// using namespace std::chrono_literals;

namespace arx::x7 {
X7Controller::X7Controller() : Node("x7_controller_node") {
  RCLCPP_INFO(this->get_logger(), "机械臂开始初始化...");
  rclcpp::on_shutdown(std::bind(&X7Controller::cleanup, this));
  std::string package_name = "arx_x7_controller";
  std::string package_share_dir = ament_index_cpp::get_package_share_directory(package_name);
  std::string urdf_name = this->declare_parameter("urdf_name", "X7Sleft1.urdf");
  int type = this->declare_parameter("arm_type", 0);
  bool vr = this->declare_parameter("vr", true);
  std::string urdf_path = package_share_dir + "/" + urdf_name;

  interfaces_ptr_ = std::make_shared<InterfacesThread>(urdf_path, this->declare_parameter("arm_can_id", "can0"),
                                                       type);
  interfaces_ptr_->arx_x(500,2000,10);
  std::string vr_sub_topic;
  std::string cmd_sub_topic;
  std::string status_pub_topic;
  std::string pose_pub_topic;
  if (type == 0) {
    vr_sub_topic = "ARX_VR_L";
    cmd_sub_topic = "/joint_control";
    status_pub_topic = "joint_information";
    pose_pub_topic = "follow1_pos_back";
  }
  else {
    vr_sub_topic = "ARX_VR_R";
    cmd_sub_topic = "/joint_control2";
    status_pub_topic = "joint_information2";
    pose_pub_topic = "follow2_pos_back";
  }
  // 创建发布器
  joint_state_publisher_ = this->create_publisher<arx5_arm_msg::msg::RobotStatus>(status_pub_topic, 1);
  vr_joint_state_publisher_ = this->create_publisher<arm_control::msg::PosCmd>(pose_pub_topic, 10);
  // 创建订阅器

  if (vr)
    vr_joint_state_subscriber_ = this->create_subscription<arm_control::msg::PosCmd>(
        vr_sub_topic, 10,
        std::bind(&X7Controller::VrCmdCallback, this, std::placeholders::_1));
  else
    joint_state_subscriber_ = this->create_subscription<arm_control::msg::JointControl>(
        cmd_sub_topic, 10,
        std::bind(&X7Controller::CmdCallback, this, std::placeholders::_1));
  // 定时器，用于发布关节信息
  timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&X7Controller::VrPubState, this));
}

void X7Controller::CmdCallback(const arm_control::msg::JointControl::SharedPtr msg) {

  std::vector<double> joint_positions = {0, 0, 0, 0, 0, 0, 0};

  for (int i = 0; i < 7; i++) {
    joint_positions[i] = msg->joint_pos[i];
  }

  interfaces_ptr_->setJointPositions(joint_positions);

  interfaces_ptr_->setArmStatus(5);

  interfaces_ptr_->setCatch(msg->joint_pos[7]);
}

void X7Controller::PubState() {
  auto message = arx5_arm_msg::msg::RobotStatus();
  message.header.stamp = this->get_clock()->now();

  Eigen::Isometry3d transform = interfaces_ptr_->getEndPose();

  // 创建长度为6的vector
  std::array<double, 6> result;

  std::vector<double> xyzrpy = {0, 0, 0, 0, 0, 0};
  xyzrpy = solve::Isometry2Xyzrpy(transform);

  // 填充vector
  result[0] = xyzrpy[0];
  result[1] = xyzrpy[1];
  result[2] = xyzrpy[2];
  result[3] = xyzrpy[3];
  result[4] = xyzrpy[4];
  result[5] = xyzrpy[5];

  message.end_pos = result;

  std::vector<double> joint_pos_vector = interfaces_ptr_->getJointPositons();
  for (int i = 0; i <= 7; i++) {
    message.joint_pos[i] = joint_pos_vector[i];
  }

  std::vector<double> joint_velocities_vector = interfaces_ptr_->getJointVelocities();
  for (int i = 0; i <= 7; i++) {
    message.joint_vel[i] = joint_velocities_vector[i];
  }

  std::vector<double> joint_current_vector = interfaces_ptr_->getJointCurrent();
  for (int i = 0; i < 7; i++) {
    message.joint_cur[i] = joint_current_vector[i];
  }
  // 发布消息
  joint_state_publisher_->publish(message);
}

void X7Controller::VrCmdCallback(const arm_control::msg::PosCmd::SharedPtr msg) {
  // RCLCPP_INFO(this->get_logger(), "接收到数据");
  double input[6] = {msg->x, msg->y, msg->z, msg->roll, msg->pitch, msg->yaw};
  Eigen::Isometry3d transform = solve::Xyzrpy2Isometry(input);

  interfaces_ptr_->setEndPose(transform);

  interfaces_ptr_->setArmStatus(InterfacesThread::state::END_CONTROL);

  interfaces_ptr_->setCatch(msg->gripper);
}

void X7Controller::VrPubState() {
  // RCLCPP_INFO(this->get_logger(), "发布数据");
  auto message = arm_control::msg::PosCmd();
  // message.header.stamp = this->get_clock()->now();

  Eigen::Isometry3d transform = interfaces_ptr_->getEndPose();

  // 提取四元数和位移
  Eigen::Quaterniond quat(transform.rotation());
  Eigen::Vector3d translation = transform.translation();

  std::vector<double> xyzrpy = solve::Isometry2Xyzrpy(transform);

  // 填充vector

  message.x = xyzrpy[0];
  message.y = xyzrpy[1];
  message.z = xyzrpy[2];
  message.roll = xyzrpy[3];
  message.pitch = xyzrpy[4];
  message.yaw = xyzrpy[5];
  message.quater_x = quat.x();
  message.quater_y = quat.y();
  message.quater_z = quat.z();
  message.quater_w = quat.w();

  std::vector<double> joint_pos_vector = interfaces_ptr_->getJointPositons();
  std::vector<double> joint_velocities_vector = interfaces_ptr_->getJointVelocities();
  std::vector<double> joint_current_vector = interfaces_ptr_->getJointCurrent();

  message.gripper = joint_pos_vector[6];

  // 发布消息
  vr_joint_state_publisher_->publish(message);

  //==========================================================
  auto msg = arx5_arm_msg::msg::RobotStatus();
  msg.header.stamp = this->get_clock()->now();

  // 创建长度为6的vector
  std::array<double, 6> result;

  // 填充vector
  result[0] = xyzrpy[0];
  result[1] = xyzrpy[1];
  result[2] = xyzrpy[2];
  result[3] = xyzrpy[3];
  result[4] = xyzrpy[4];
  result[5] = xyzrpy[5];

  msg.end_pos = result;

  for (int i = 0; i < 8; i++) {
    msg.joint_pos.push_back(joint_pos_vector[i]);
  }

  for (int i = 0; i < 8; i++) {
    msg.joint_vel.push_back(joint_velocities_vector[i]);
  }

  for (int i = 0; i < 8; i++) {
    msg.joint_cur.push_back(joint_current_vector[i]);
  }
  // 发布消息
  joint_state_publisher_->publish(msg);
}
}

void signalHandler(int signum) {
  rclcpp::shutdown();
}

int main(int argc, char *argv[]) {
  std::signal(SIGHUP, signalHandler);
  std::signal(SIGTERM, signalHandler);
  std::signal(SIGINT, signalHandler);

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<arx::x7::X7Controller>());
  rclcpp::shutdown();
  return 0;
}
