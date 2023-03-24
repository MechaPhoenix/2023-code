// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

using namespace std;
#include "Robot.h"

void Robot::setDrive(double left, double right)
{
  std::cout << left*1.11 << ", " << right << std::endl;
  m_leftMotor.Set(ctre::phoenix::motorcontrol::VictorSPXControlMode::PercentOutput, -left*1.11);
  m_rightMotor.Set(ctre::phoenix::motorcontrol::VictorSPXControlMode::PercentOutput, -right);
}

void Robot::RobotInit()
{
  frc::SmartDashboard::PutBoolean("Do Full Auto", mAutoBalance.doingBalance);
  frc::SmartDashboard::PutBoolean("Do Any Auto", mAutoBalance.doAnyAuto);
  frc::SmartDashboard::PutBoolean("Do Taxi Balance", mAutoBalance.taxiBalance);
  // We need to invert one side of the drivetrain so that positive voltages
  // result in both sides moving forward. Depending on how your robot's
  // gearbox is constructed, you might have to invert the left side instead.
  // m_rightMotor.SetInverted(true);
  // m_rightMotor2.SetInverted(true);

  // Set some follow stuff
  m_rightMotor2.Follow(m_rightMotor);
  m_leftMotor2.Follow(m_leftMotor);

  m_rightMotor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  m_rightMotor2.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  m_leftMotor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  m_leftMotor2.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);

  pcmCompressor.EnableDigital();

  g.InitGyro();
  g.Calibrate();
  g.SetDeadband(0.05);
  // Disable Compressor
  // Optional
  // pcmCompressor.Disable();

  // Prints
  std::cout << "Compressor Enabled"
            << "\n";
  std::cout << "Check if Running"
            << "\n";
  std::cout << "RLS On and Flashing"
            << "\n";

  // Solenoid
  gripperSolenoid.Set(frc::DoubleSolenoid::Value::kOff);
  std::cout << "Solenoid Off"
            << "\n";
}

void Robot::RobotPeriodic() {
  frc::SmartDashboard::PutNumber("Auto State", mAutoBalance.getState());
  frc::SmartDashboard::PutNumber("Lower Encoder Value", m_arm.GetLowerArmAngle());
  frc::SmartDashboard::PutNumber("Higher Encoder Value", m_arm.GetHigherArmAngle());
  frc::SmartDashboard::PutNumber("Gyro Delta", round(-g.GetRate()*100)/100);
  frc::SmartDashboard::PutNumber("Gyro Angle", -g.GetAngle());
  if (m_stick.GetRawButtonPressed(7)){mAutoBalance.taxiBalance = !mAutoBalance.taxiBalance;};
  if (m_stick.GetRawButtonPressed(8)){mAutoBalance.doingBalance = !mAutoBalance.doingBalance;};
  if (m_stick.GetRawButtonPressed(9)){mAutoBalance.doAnyAuto = !mAutoBalance.doAnyAuto;};
}

void Robot::AutonomousInit()
{
  std::cout << "Entering autonomous mode" << std::endl;
  std::cout << "Ready to Go" << std::endl;
}

void Robot::AutonomousPeriodic()
{
  double speed = mAutoBalance.autoBalanceRoutine(&g);
  setDrive(speed, speed);
  frc::SmartDashboard::PutNumber("Speed", speed);
  m_arm.SetHigherArmAngle(0);
  m_arm.SetLowerArmAngle(0);
}

void Robot::TeleopPeriodic()
{
  if (m_stick.GetRawButtonPressed(11)){currentJoySens = DEFENCE_JOYSTICK_SENSITIVITY;};
  if (m_stick.GetRawButtonPressed(12)){currentJoySens = BALANCE_JOYSTICK_SENSITIVITY;};
  // Drive with arcade style
  // This is where all our fun stuff goes :)
  // Read the joystick, calculate the drive stuff
  double x = m_stick.GetX()*JOYSTICK_ROT_SENS; // In terms of arcade drive, this is speed
  double y = m_stick.GetY()*currentJoySens; // In terms of arcade drive, this is turn

  // Drive
  double leftPower = (y - x) / 2;
  double rightPower = (y + x) / 2;

  // Update the LEFT drive
  if (leftPower != lastDriveLeft)
  {
    // Update the LEFT drive
    m_leftMotor.Set(ctre::phoenix::motorcontrol::VictorSPXControlMode::PercentOutput, leftPower * S_LEFT_DRIVE);
    lastDriveLeft = leftPower;
  }

  // Update the RIGHT drive
  if (rightPower != lastDriveRight)
  {
    // Update the RIGHT drive
    m_rightMotor.Set(ctre::phoenix::motorcontrol::VictorSPXControlMode::PercentOutput, rightPower * S_RIGHT_DRIVE);
    lastDriveRight = rightPower;
  }

  // Update the Trigger and stuff
  bool engageButton = m_stick.GetRawButton(FEED_BUTTON);
  bool releaseButton = m_stick.GetRawButton(2);

  if (engageButton) {
    if (!lastEngage) {
      gripperSolenoid.Toggle();
      gripperSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
    }
    lastEngage = true;
    lastRelease = false;
  } else if (releaseButton) {
    if (!lastRelease) {
      gripperSolenoid.Toggle();
      gripperSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
    }
    lastRelease = true;
    lastEngage = false;
  }

  // // Arm Trigger
  // if (thumbButton != lastTrigger)
  // {
  //   lastTrigger = thumbButton;
  //   // Checks if trigger is pressed
  //   if (thumbButton)
  //   {
      
  //     // Prints Out
  //     std::cout << "Solenoid Out!" << "\n";
  //     // Disables Solenoid and sets trigger to false
  //   }
  //   else
  //   {
  //     // Sends Back
  //     gripperSolenoid.Toggle();
  //     gripperSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
  //     std::cout << "Solenoid Back" << "\n";
  //   }
  //}

  if (m_stick.GetRawButtonPressed(3)){m_arm.oldArmState = m_arm.armState; m_arm.armState = 0;};
  if (m_stick.GetRawButtonPressed(4)){m_arm.oldArmState = m_arm.armState; m_arm.armState = 1;};
  if (m_stick.GetRawButtonPressed(5)){m_arm.oldArmState = m_arm.armState; m_arm.armState = 2;};
  if (m_stick.GetRawButtonPressed(6)){m_arm.oldArmState = m_arm.armState; m_arm.armState = 3;};

  m_arm.ArmPeriodic();
}

// Drive Doubles
double lastDriveRight;
double lastDriveLeft;

// Runs the Robots Code
#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif