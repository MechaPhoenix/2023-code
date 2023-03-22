// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

using namespace std;
#include "Robot.h"

  void Robot::setDrive(double left, double right)
  {
    m_leftMotor.Set(ctre::phoenix::motorcontrol::VictorSPXControlMode::PercentOutput, -left);
    m_rightMotor.Set(ctre::phoenix::motorcontrol::VictorSPXControlMode::PercentOutput, -right);
  }

  void Robot::RobotInit()
  {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    // m_rightMotor.SetInverted(true);
    // m_rightMotor2.SetInverted(true);
  
    // Set some follow stuff
    m_rightMotor2.Follow(m_rightMotor);
    m_leftMotor2.Follow(m_leftMotor);

    pcmCompressor.EnableDigital();
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
    frc::SmartDashboard::PutNumber("Tilt", mAutoBalance.getTilt());
    frc::SmartDashboard::PutNumber("Roll_Tele", mAutoBalance.getRoll());
    frc::SmartDashboard::PutNumber("Pitch_Tele", mAutoBalance.getPitch());
    frc::SmartDashboard::PutString("Auto State", mAutoBalance.getState());
    frc::SmartDashboard::PutNumber("Lower Encoder Value", m_arm.GetLowerArmAngle());
    frc::SmartDashboard::PutNumber("Higher Encoder Value", m_arm.GetHigherArmAngle());
  }

  void Robot::AutonomousInit()
  {
    std::cout << "Entering autonomous mode" << std::endl;
    std::cout << "Ready to Go" << std::endl;
  }

  void Robot::AutonomousPeriodic()
  {
    double speed = mAutoBalance.scoreAndBalance();
    setDrive(speed, speed);
    frc::SmartDashboard::PutNumber("Speed", speed);
  }

  void Robot::TeleopPeriodic()
  {
    // Drive with arcade style
    // This is where all our fun stuff goes :)
    // Read the joystick, calculate the drive stuff
    double x = m_stick.GetX()*JOYSTICK_ROT_SENS; // In terms of arcade drive, this is speed
    double y = m_stick.GetY()*JOYSTICK_SENSITIVITY; // In terms of arcade drive, this is turn
    // double x = ControllerP.GetLeftY(); // In terms of arcade drive, this is speed
     //double y = ControllerP.GetRightX(); // In terms of arcade drive, this is turn

    // Drive Powers
    // Note if Wheels Invert
    // Flip the y - x and the y + x around

    // drive

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
    // Arm Lower Motor Xbox Controller Handler
    // bool lastBumper;
    // double lastBumperRB;
    // bool c_stick_rb_press = m_stick.GetRawButtonPressed(11);
    // // bool c_stick_rb_press = ControllerX.GetRightBumper();
    // if (c_stick_rb_press != lastBumper)
    // {
    //   lastBumper = c_stick_rb_press;
    //   if (c_stick_rb_press)
    //   {
    //     a_lowMotor.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, lastBumper * ARM_LOW_DRIVE);
    //   }
    //   else
    //   {
    //     std::cout << "Bumper off"
    //               << "\n";
    //   }
    // }

    // Update the Trigger and stuff
    bool thumbButton = m_stick.GetRawButton(FEED_BUTTON);
    // Arm Trigger
    if (thumbButton != lastTrigger)
    {
      lastTrigger = thumbButton;
      // Checks if trigger is pressed
      if (thumbButton)
      {
        gripperSolenoid.Toggle();
        gripperSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
        // Prints Out
        std::cout << "Solenoid Out!" << "\n";
        // Disables Solenoid and sets trigger to false
      }
      else
      {
        // Sends Back
        gripperSolenoid.Toggle();
        gripperSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
        std::cout << "Solenoid Back" << "\n";
      }
    }

     if (m_stick.GetRawButton(8))
    {
      m_arm.SetLowerArmAngle(30);
    }

    if(m_stick.GetRawButtonPressed(9)){
      m_arm.ResetArms();
    }

    if (m_stick.GetRawButtonPressed(7)){
      m_arm.LoadParameters();
    }

    m_arm.ArmPeriodic();
  }



// Code Above checks if Trigger is pressed and fires thr Solenoid from the Pneumatic Pistons

// Not Needed
// bool lastShooterState;
// bool lastFeederState;

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