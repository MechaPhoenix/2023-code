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
  void LoadParameters();
  void ArmPeriodic();
  double GetLowerArmAngle();
  double GetHigherArmAngle();
  void SetLowerArmAngle(double angle);
  void SetHigherArmAngle(double angle);
  void ZeroArm();
  void ResetArms();
  void setArmPosition(std::pair<double, double> angles);

  private:
    ctre::phoenix::motorcontrol::can::TalonSRX m_lowerArmMotorController{ARM_CAN_LOW_NUM};
    ctre::phoenix::motorcontrol::can::TalonSRX m_higherArmMotorController{ARM_CAN_HIGH_NUM};
};
