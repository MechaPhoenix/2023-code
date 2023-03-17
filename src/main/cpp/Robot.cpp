// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <iostream>
#include "mapping.h"
#include "autoBalance.h"
using namespace std;
#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/Compressor.h>
#include <frc/Solenoid.h>
#include <frc/DoubleSolenoid.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/XboxController.h>
#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <frc/Encoder.h>

class Robot : public frc::TimedRobot
{
  // Motors
  ctre::phoenix::motorcontrol::can::VictorSPX m_leftMotor{15};
  ctre::phoenix::motorcontrol::can::VictorSPX m_rightMotor{14};
  ctre::phoenix::motorcontrol::can::VictorSPX m_leftMotor2{17};
  ctre::phoenix::motorcontrol::can::TalonSRX a_lowMotor{ARM_CAN_LOW_NUM};
  ctre::phoenix::motorcontrol::can::TalonSRX a_highMotor{ARM_CAN_HIGH_NUM};
  ctre::phoenix::motorcontrol::can::VictorSPX m_rightMotor2{16};
  frc::Encoder armEncoder{0, 1};
  // JoyStick
  frc::Joystick m_stick{0};
  // Xbox Controller
  frc::XboxController ControllerX{2};
  // Motor Doubles
  double lastDriveRight{0};
  double lastDriveLeft{0};
  // Compressor
  frc::Compressor pcmCompressor{0, frc::PneumaticsModuleType::CTREPCM};
  // Solenoid
  frc::DoubleSolenoid gripperSolenoid{frc::PneumaticsModuleType::CTREPCM, 0, 1};
  // Sol Bools
  bool lastTrigger = false;

public:
  autoBalance mAutoBalance;

  void setDrive(double left, double right)
  {
    m_leftMotor.Set(ctre::phoenix::motorcontrol::VictorSPXControlMode::PercentOutput, left);
    m_rightMotor.Set(ctre::phoenix::motorcontrol::VictorSPXControlMode::PercentOutput, right);
  }
  void RobotInit() override
  {
    // logs
    std::cout << "Robot Started."
              << "\n";
    sleep(5);
    std::cout << "Ready to Move!"
              << "\n";
    sleep(5);
    std::cout << "Debug Loaded."
              << "\n";
    std::cout << "No Error Found."
              << "\n";
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

  void AutonomousInit() override
  {
    std::cout << "Entering autonomous mode" << std::endl;
  }

  void AutonomousPeriodic() override
  {
    double speed = mAutoBalance.scoreAndBalance();
    setDrive(speed, speed);
    std::cout << "Score And Balance Running" << "\n";
  }

  void TeleopPeriodic() override
  {
    // Drive with arcade style
    // This is where all our fun stuff goes :)
    // Read the joystick, calculate the drive stuff
    double x = m_stick.GetX(); // In terms of arcade drive, this is speed
    double y = m_stick.GetY(); // In terms of arcade drive, this is turn

    // Drive Powers
    // Note if Wheels Invert
    // Flip the y - x and the y + x around

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
    bool lastHit;
    double lastHitS;
    bool s_hit = ControllerX.GetLeftBumper();

    if (s_hit != lastHitS)

    bool lastBumper;
    double lastBumperRB;
    bool c_stick_rb_press = m_stick.GetRawButtonPressed(11);
    // bool c_stick_rb_press = ControllerX.GetRightBumper();
    if (c_stick_rb_press != lastBumper)
    {
      lastBumper = c_stick_rb_press;
      if (c_stick_rb_press)
      {
        a_lowMotor.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, lastBumper * ARM_LOW_DRIVE);
      }
      else
      {
        std::cout << "Bumper off"
                  << "\n";
      }
    }

    bool lastright;
    double lastBumperR;
    bool b_press = m_stick.GetRawButtonPressed(ARM_HIGH_TEST_BTN);

    if (b_press != lastBumper)
    {
      lastBumperR = b_press;
      if (b_press)
      {
        a_highMotor.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, lastright * ARM_HIGH_DRIVE);
      } else {
        std::cout << "High Bumper Test off" << std::endl;
      }
    }

    // Update the Trigger and stuff
    bool trigger = m_stick.GetTrigger();
    bool thumbButton = m_stick.GetRawButton(FEED_BUTTON);
    // Arm Trigger
    bool comp = m_stick.GetRawButton(12);
    if (trigger != lastTrigger)
    {
      lastTrigger = trigger;
      // Checks if trigger is pressed
      if (trigger)
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
  }
};

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
