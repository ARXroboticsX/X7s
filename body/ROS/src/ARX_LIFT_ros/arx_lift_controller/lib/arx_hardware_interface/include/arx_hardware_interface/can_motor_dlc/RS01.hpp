//
// Created by yezi on 2025/4/14.
//

#pragma once

#include "arx_hardware_interface/can_motor_dlc/MotorDlcBase.hpp"
#include "arx_hardware_interface/canbase/CanBaseDef.hpp"
#include "arx_hardware_interface/typedef/HybridJointTypeDef.hpp"
#include <chrono>
#include <cmath>
#include <stdio.h>

namespace arx
{
  namespace hw_interface
  {
    class RS01 : public MotorDlcBase
    {
    public:
      RS01(int motor_id) : motor_id_(motor_id)
      {
        last_update_time_exchange_ = std::chrono::system_clock::now();
      };
      CanFrame packMotorMsg(arx::hw_interface::HybridJointCmd *command) override;
      CanFrame packMotorMsg(double k_p, double k_d, double position,
                            double velocity, double torque) override;
      void CanAnalyze(arx::hw_interface::CanFrame *frame) override;
      CanFrame packEnableMotor() override;
      CanFrame packDisableMotor() override;
      HybridJointStatus GetMotorMsg() override;
      bool online() override;
      void ExchangeMotorMsg() override;

    private:
      double position_ = 0;
      double velocity_ = 0;
      double current_ = 0;
      std::chrono::system_clock::time_point last_update_time_;
      double position_exchange_;
      double velocity_exchange_;
      double current_exchange_;
      std::chrono::system_clock::time_point last_update_time_exchange_;
      int motor_id_;
    };
  } // namespace hw_interface
} // namespace arx