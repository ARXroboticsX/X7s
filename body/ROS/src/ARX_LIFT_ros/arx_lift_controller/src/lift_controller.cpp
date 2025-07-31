//
// Created by yezi on 24-12-6.
//

#include <arm_control/ArxImu.h>
#include <arm_control/PosCmd.h>
#include <arx_lift_src/lift_head_control_loop.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

void solveHeadPos(double *yaw, double *pitch, double x, double y, double z) {
  *yaw = std::atan2(y, x) / 2;
  double distance_xy = std::sqrt(pow(x, 2) + pow(y, 2));
  double distance = std::sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
  double radius = 0.06;
  double A = asin(radius / distance);
  double B = atan2(z, distance_xy);
  *pitch = A - B;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "lift_controller");
  ros::NodeHandle nh("~");
  ros::Rate loop_rate(500);
  int type = nh.param("robot_type", 0);
  if (type == 0)
    ROS_INFO("robot_type: lift");
  else
    ROS_INFO("robot_type: x7s");
  LiftHeadControlLoop control_loop(
      "can5", static_cast<LiftHeadControlLoop::RobotType>(type));
  int running_state = 2;
  double lift_height = 0;
  std::shared_ptr<realtime_tools::RealtimePublisher<arm_control::PosCmd>> pub =
      std::make_shared<realtime_tools::RealtimePublisher<arm_control::PosCmd>>(
          nh, "/body_information", 1);
  std::shared_ptr<realtime_tools::RealtimePublisher<arm_control::ArxImu>>
      imu_pub = std::make_shared<
          realtime_tools::RealtimePublisher<arm_control::ArxImu>>(
          nh, "/arx_imu", 1);
  bool collect = false;
  double right_arm_xyz[3];
  double init_pos[3]{0.1, 0, 0.1635};
  ros::Subscriber sub = nh.subscribe<arm_control::PosCmd>(
      "/ARX_VR_L", 1, [&](const arm_control::PosCmd::ConstPtr &msg) {
        collect = true;
        control_loop.setHeight(msg->height);
        control_loop.setWaistPos(msg->tempFloatData[0]);
        double yaw, pitch;
        double left_arm_distance_square =
            pow(msg->x, 2) + pow(msg->y, 2) + pow(msg->z, 2);
        double right_arm_distance_square = pow(right_arm_xyz[0], 2) +
                                           pow(right_arm_xyz[1], 2) +
                                           pow(right_arm_xyz[2], 2);
        if (left_arm_distance_square < 0.01 &&
            right_arm_distance_square < 0.01) {
          yaw = 0;
          pitch = 0.3;
        } else if (left_arm_distance_square > right_arm_distance_square)
          solveHeadPos(&yaw, &pitch, msg->x + init_pos[0] - 0.07,
                       msg->y + init_pos[1] + 0.25,
                       msg->z + init_pos[2] - 0.22);
        else
          solveHeadPos(&yaw, &pitch, right_arm_xyz[0] + init_pos[0] - 0.07,
                       right_arm_xyz[1] + init_pos[1] - 0.25,
                       right_arm_xyz[2] + init_pos[2] - 0.22);
        control_loop.setHeadYaw(yaw);
        control_loop.setHeadPitch(pitch);
        if (type == 0)
          control_loop.setChassisCmd(msg->chx / 2.5, -msg->chy / 2.5,
                                     msg->chz / 2.5, msg->mode1);
        else
          control_loop.setChassisCmd(msg->chx / 3, -msg->chy / 3, msg->chz / 3,
                                     msg->mode1);
      });
  ros::Subscriber right_arm_sub = nh.subscribe<arm_control::PosCmd>(
      "/ARX_VR_R", 1, [&](const arm_control::PosCmd::ConstPtr &msg) {
        right_arm_xyz[0] = msg->x;
        right_arm_xyz[1] = msg->y;
        right_arm_xyz[2] = msg->z;
      });
  ros::Time last_cb_time = ros::Time::now();
  ros::Subscriber body_control_sub = nh.subscribe<arm_control::PosCmd>(
      "/body_control", 1, [&](const arm_control::PosCmd::ConstPtr &msg) {
        collect = false;
        last_cb_time = ros::Time::now();
        control_loop.setHeight(msg->height);
        control_loop.setWaistPos(msg->tempFloatData[0]);
        control_loop.setHeadYaw(msg->head_yaw);
        control_loop.setHeadPitch(-msg->head_pit);
        control_loop.setWheelVel(msg->tempFloatData[1], msg->tempFloatData[2],
                                 msg->tempFloatData[3], msg->tempFloatData[4]);
        control_loop.setChassisCmd(0, 0, 0, msg->mode1);
      });
  ros::Time last_callback_time = ros::Time::now();
  ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy>(
      "/joy", 1, [&](const sensor_msgs::Joy::ConstPtr &msg) {
        lift_height +=
            msg->axes[1] * (ros::Time::now() - last_callback_time).toSec();
        last_callback_time = ros::Time::now();
        if (msg->buttons[0] == 1)
          running_state = 1;
        if (msg->buttons[1] == 1)
          running_state = 2;
        control_loop.setHeight(lift_height);
        control_loop.setChassisCmd(msg->axes[4] * 2, msg->axes[3] * 2,
                                   msg->axes[0] * 4, running_state);
      });
  ros::Time last_pub = ros::Time::now();
  while (ros::ok()) {
    control_loop.loop();
    if ((ros::Time::now() - last_cb_time).toSec() > 0.3 && !collect) {
      control_loop.setChassisCmd(0, 0, 0, 2);
      control_loop.setWheelVel(0, 0, 0, 0);
    }
    ros::Time now = ros::Time::now();
    if (last_pub + ros::Duration(1. / 100) < now) {
      if (pub->trylock()) {
        pub->msg_.head_yaw = control_loop.getHeadYaw();
        pub->msg_.head_pit = control_loop.getHeadPitch();
        pub->msg_.height = control_loop.getHeight();
        pub->msg_.tempFloatData[0] = control_loop.getWaistPos();
        double wheel_vel[4];
        control_loop.getWheelVel(wheel_vel);
        for (int i = 0; i < 4; i++)
          pub->msg_.tempFloatData[i + 1] = wheel_vel[i];
        pub->unlockAndPublish();
      }
      if (imu_pub->trylock()) {
        imu_pub->msg_.stamp = now;
        double orientation[3], angular_vel[3];
        control_loop.getOrientation(orientation);
        control_loop.getAngularVel(angular_vel);
        imu_pub->msg_.orientation.x = orientation[0];
        imu_pub->msg_.orientation.y = orientation[1];
        imu_pub->msg_.orientation.z = orientation[2];
        imu_pub->msg_.angular_velocity.x = angular_vel[0];
        imu_pub->msg_.angular_velocity.y = angular_vel[1];
        imu_pub->msg_.angular_velocity.z = angular_vel[2];
        imu_pub->unlockAndPublish();
      }
      last_pub = now;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}
