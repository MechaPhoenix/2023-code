// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "mapping.h"
#include "armAngles.h"

#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <frc/trajectory/TrapezoidProfile.h>
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
  double FeedForwardCalc();
  bool inRange(double low, double high, double x);
  int armState;
  double feedForward;
  frc::TrapezoidProfile<units::degrees>::Constraints constraints{(units::degree_t)(1), 1};
  frc::TrapezoidProfile<units::degrees>::State homeState{HOME_ANGLE_HIGHER_degrees, 0};
  frc::TrapezoidProfile<units::degrees>::State humanState{HUMAN_PLAYER_HIGHER_degrees, 0};
  frc::TrapezoidProfile<units::degrees>::State scoreState{MID_SCORE_HIGHER_degrees, 0};
  frc::TrapezoidProfile<units::degrees>::
  
  double angles[3][2] = {{HOME_ANGLE_LOWER, HOME_ANGLE_HIGHER},
  {HUMAN_PLAYER_LOWER, HUMAN_PLAYER_HIGHER},
  {MID_SCORE_LOWER, MID_SCORE_HIGHER}};



  private:
    ctre::phoenix::motorcontrol::can::TalonSRX m_lowerArmMotorController{ARM_CAN_LOW_NUM};
    ctre::phoenix::motorcontrol::can::TalonSRX m_higherArmMotorController{ARM_CAN_HIGH_NUM};
};
