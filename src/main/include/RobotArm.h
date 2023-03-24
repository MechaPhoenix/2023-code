// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "mapping.h"
#include "armAngles.h"

#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <iostream>

class RobotArm {
 public:
  RobotArm();
  void LoadParameters();
  void ArmPeriodic();
  double GetLowerArmAngle();
  double GetHigherArmAngle();
  void SetLowerArmAngle(double angle);
  void SetHigherArmAngle(double angle);
  void ResetArms();
  void armPositioning();
  bool inRange(double low, double high, double x);
  int armState;
  int oldArmState;
  double angles[4][2] = {{HOME_ANGLE_LOWER, HOME_ANGLE_HIGHER},
  {HUMAN_PLAYER_LOWER, HUMAN_PLAYER_HIGHER},
  {MID_SCORE_LOWER, MID_SCORE_HIGHER},
	{PICKUP_ANGLE_LOWER, PICKUP_ANGLE_HIGHER}};

  private:
    ctre::phoenix::motorcontrol::can::TalonSRX m_lowerArmMotorController{ARM_CAN_LOW_NUM};
    ctre::phoenix::motorcontrol::can::TalonSRX m_higherArmMotorController{ARM_CAN_HIGH_NUM};
};
