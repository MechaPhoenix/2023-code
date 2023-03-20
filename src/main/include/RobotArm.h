// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "mapping.h"

#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>

class RobotArm {
 public:
  RobotArm();
  void ArmPeriodic();

  private:
    ctre::phoenix::motorcontrol::can::TalonSRX m_lowerArmMotor{ARM_CAN_LOW_NUM};
    ctre::phoenix::motorcontrol::can::TalonSRX m_upperArmMotor{ARM_CAN_HIGH_NUM};
};
