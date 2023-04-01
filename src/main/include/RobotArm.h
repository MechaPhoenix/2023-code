// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "mapping.h"
#include "armAngles.h"

#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
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
  double FeedForwardCalc();
  bool inRange(double low, double high, double x);
  void setNewArmPos(int stateUpdate);
  int armState;
  double feedForward;
  frc::TrapezoidProfile<units::degrees>::Constraints constraints{units::degrees_per_second_t(105),//
  units::degrees_per_second_squared_t(90)};//acceleration
  frc::TrapezoidProfile<units::degrees>::State currentState{units::degree_t(0), units::degrees_per_second_t(0)};

  frc::TrapezoidProfile<units::degrees>::Constraints lowerConstraints{units::degrees_per_second_t(80),//
  units::degrees_per_second_squared_t(85)};
  frc::TrapezoidProfile<units::degrees>::State currentLowerState{units::degree_t(0), units::degrees_per_second_t(0)};

  
  double angles[4][2] = {{HOME_ANGLE_LOWER, HOME_ANGLE_HIGHER},
  {HUMAN_PLAYER_LOWER, HUMAN_PLAYER_HIGHER},
  {MID_SCORE_LOWER, MID_SCORE_HIGHER},
  {LOW_SCORE_LOWER, LOW_SCORE_HIGHER}};



  private:
    ctre::phoenix::motorcontrol::can::TalonSRX m_lowerArmMotorController{ARM_CAN_LOW_NUM};
    ctre::phoenix::motorcontrol::can::TalonSRX m_higherArmMotorController{ARM_CAN_HIGH_NUM};
};
