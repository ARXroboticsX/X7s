//
// Created by nuc-ros2 on 24-12-6.
//

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <arx_lift_src/lift_head_control_loop.h>
#include <arm_control/msg/pos_cmd.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <csignal>

std::shared_ptr<LiftHeadControlLoop> control_loop;
void signalHandler(int signum) {
  control_loop.reset();
  rclcpp::shutdown();
}

int main(int argc, char **argv) {
  std::signal(SIGHUP, signalHandler);
  std::signal(SIGTERM, signalHandler);
  std::signal(SIGINT, signalHandler);

  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("lift_controller");
  int robot_type = node->declare_parameter("robot_type", 0);
  control_loop = std::make_shared<LiftHeadControlLoop>("can5", static_cast<LiftHeadControlLoop::RobotType>(robot_type));
  int running_state = 2;
  double lift_height = 0;
  bool collect = false;
  auto pub = node->create_publisher<arm_control::msg::PosCmd>("/body_information", 1);
  auto imu_pub = node->create_publisher<sensor_msgs::msg::Imu>("/arx_imu",1);
  auto sub = node->create_subscription<arm_control::msg::PosCmd>("/ARX_VR_L", 1,
                                                                 [&](const arm_control::msg::PosCmd &msg) {
                                                                   collect = true;
                                                                   control_loop->setHeight(msg.height);
                                                                   control_loop->setWaistPos(msg.temp_float_data[0]);
                                                                   control_loop->setHeadYaw(msg.head_yaw);
                                                                   control_loop->setHeadPitch(-msg.head_pit);
                                                                   if (robot_type == 0)
                                                                     control_loop->setChassisCmd(msg.chx / 2.5,
                                                                                                 -msg.chy / 2.5,
                                                                                                 msg.chz / 2.5,
                                                                                                 msg.mode1);
                                                                   else
                                                                     control_loop->setChassisCmd(msg.chx / 5,
                                                                                                 -msg.chy / 5,
                                                                                                 msg.chz / 5,
                                                                                                 msg.mode1);

                                                                 });
  rclcpp::Time last_cb_time = rclcpp::Clock().now();
  auto body_control_sub = node->create_subscription<arm_control::msg::PosCmd>("/body_control",1,[&](const arm_control::msg::PosCmd &msg){
    collect = false;
    last_cb_time = rclcpp::Clock().now();
    control_loop->setHeight(msg.height);
    control_loop->setWaistPos(msg.temp_float_data[0]);
    control_loop->setHeadYaw(msg.head_yaw);
    control_loop->setHeadPitch(-msg.head_pit);
    control_loop->setWheelVel(msg.temp_float_data[1], msg.temp_float_data[2],
                             msg.temp_float_data[3], msg.temp_float_data[4]);
    control_loop->setChassisCmd(0, 0, 0, msg.mode1);
  });
  rclcpp::Time last_callback_time = rclcpp::Clock().now();
  auto joy_sub = node->create_subscription<sensor_msgs::msg::Joy>("/joy", 1, [&](const sensor_msgs::msg::Joy &msg) {
    double duration = (rclcpp::Clock().now() - last_callback_time).seconds();
    if (duration > 1)
      duration = 0;
    lift_height +=
        msg.axes[1] * duration;
    if (lift_height > 0.48)
      lift_height = 0.48;
    else if (lift_height < 0.)
      lift_height = 0.;
    last_callback_time = rclcpp::Clock().now();
    if (msg.buttons[0] == 1)
      running_state = 1;
    if (msg.buttons[1] == 1)
      running_state = 2;
    control_loop->setHeight(lift_height);
    control_loop->setChassisCmd(msg.axes[4] * 2, msg.axes[3] * 2,
                                msg.axes[0] * 4, running_state);
  });
  rclcpp::Rate loop_rate(400);
  while (rclcpp::ok()) {
    control_loop->loop();
    if ((rclcpp::Clock().now() - last_cb_time).seconds() > 0.3 && !collect) {
      control_loop->setChassisCmd(0, 0, 0, 2);
      control_loop->setWheelVel(0, 0, 0, 0);
    }
    arm_control::msg::PosCmd msg;
    msg.head_yaw = control_loop->getHeadYaw();
    msg.head_pit = control_loop->getHeadPitch();
    msg.height = control_loop->getHeight();
    msg.temp_float_data[0] = control_loop->getWaistPos();
    double wheel_vel[4];
    control_loop->getWheelVel(wheel_vel);
    for(int i = 0;i < 4; i++)
      msg.temp_float_data[i + 1] = wheel_vel[i];
    pub->publish(msg);
    sensor_msgs::msg::Imu imu_msg;
    double orientation[3],angular_vel[3],accel[3];
    control_loop->getOrientation(orientation);
    control_loop->getAngularVel(angular_vel);
    control_loop->getAccel(accel);
    imu_msg.header.stamp = rclcpp::Clock().now();
    tf2::Quaternion q;
    q.setRPY(orientation[0],orientation[1],orientation[2]);
    imu_msg.orientation.x = q.x();
    imu_msg.orientation.y = q.y();
    imu_msg.orientation.z = q.z();
    imu_msg.orientation.w = q.w();
    imu_msg.linear_acceleration.x = accel[0];
    imu_msg.linear_acceleration.y = accel[1];
    imu_msg.linear_acceleration.z = accel[2];
    imu_msg.angular_velocity.x = angular_vel[0];
    imu_msg.angular_velocity.y = angular_vel[1];
    imu_msg.angular_velocity.z = angular_vel[2];
    imu_pub->publish(imu_msg);
    spin_some(node);
    loop_rate.sleep();
  }
  return 0;
}
