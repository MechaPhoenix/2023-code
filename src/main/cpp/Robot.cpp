// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <iostream>
#include "mapping.h"
#include "autoBalance.h"
using namespace std;
#include <frc/Joystick.h>
#include <frc/PS4Controller.h>
#include <frc/GenericHID.h>
#include <frc/TimedRobot.h>
#include <frc/Compressor.h>
#include <frc/Solenoid.h>
#include <frc/DoubleSolenoid.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/XboxController.h>
#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <frc/Encoder.h>
#include <cameraserver/CameraServer.h>
#include <cstdio>
#include <thread>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class Robot : public frc::TimedRobot
{
  private:
    static void VisionThread() {
      cs::UsbCamera camera = frc::CameraServer::StartAutomaticCapture();

      camera.SetResolution(250, 250);
      camera.SetFPS(10);

      cs::CvSink cvSink = frc::CameraServer::GetVideo();

      cs::CvSource outputStream =
        frc::CameraServer::PutVideo("Rectangle", 640, 480);

        cv::Mat mat;

        while (true) {

          if (cvSink.GrabFrame(mat) == 0) {

            outputStream.NotifyError(cvSink.GetError());

            continue;

          }

          rectangle(mat, cv::Point(100, 100), cv::Point(400, 400),
            cv::Scalar(255, 255, 255), 5);
          outputStream.PutFrame(mat);
        }
    }
  // Motors
  ctre::phoenix::motorcontrol::can::VictorSPX m_leftMotor{15};
  ctre::phoenix::motorcontrol::can::VictorSPX m_rightMotor{14};
  ctre::phoenix::motorcontrol::can::VictorSPX m_leftMotor2{17};
  ctre::phoenix::motorcontrol::can::TalonSRX a_lowMotor{ARM_CAN_LOW_NUM};
  ctre::phoenix::motorcontrol::can::TalonSRX a_highMotor{ARM_CAN_HIGH_NUM};
  ctre::phoenix::motorcontrol::can::VictorSPX m_rightMotor2{16};
  frc::Encoder armEncoder{0, 1};
  
  // JoyStick
  frc::Joystick m_stick{1};
  // Xbox Controller
  frc::XboxController ControllerX{2};
  frc::PS4Controller ControllerP{0};
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
    sleep(5);
    m_leftMotor.Set(ctre::phoenix::motorcontrol::VictorSPXControlMode::Disabled, left);
    m_rightMotor.Set(ctre::phoenix::motorcontrol::VictorSPXControlMode::Disabled, right);
    sleep(2);
    m_leftMotor.Set(ctre::phoenix::motorcontrol::VictorSPXControlMode::PercentOutput, -left);
    m_rightMotor.Set(ctre::phoenix::motorcontrol::VictorSPXControlMode::PercentOutput, -right);
    sleep(1);
    m_leftMotor.Set(ctre::phoenix::motorcontrol::VictorSPXControlMode::Disabled, left);
    m_rightMotor.Set(ctre::phoenix::motorcontrol::VictorSPXControlMode::Disabled, right);
    std::cout << m_leftMotor.GetMotorOutputPercent();
    std::cout << m_rightMotor.GetMotorOutputPercent();
    std::cout << m_leftMotor.GetTemperature();
    std::cout << m_rightMotor.GetTemperature();
  }
  void RobotInit() override
  {



    std::thread visionThread(VisionThread);
    visionThread.detach();

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
    std::cout << "Ready to Go" << std::endl;
  }

  void AutonomousPeriodic() override
  {
    double speed = mAutoBalance.scoreAndBalance();
    setDrive(speed, speed);
    std::cout << "Score And Balance Running" << "\n";
    std::cout << armEncoder.GetDistance();
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
    

    const auto j1 = m_stick.GetRawButtonPressed(1);
    const auto j2 = m_stick.GetRawButtonPressed(2);
    const auto j3 = m_stick.GetRawButtonPressed(3);
    const auto j4 = m_stick.GetRawButtonPressed(4);
    const auto j5 = m_stick.GetRawButtonPressed(5);
    const auto j6 = m_stick.GetRawButtonPressed(6);
    const auto j7 = m_stick.GetRawButtonPressed(7);
    const auto j8 = m_stick.GetRawButtonPressed(8);
    const auto j9 = m_stick.GetRawButtonPressed(9);
    const auto j10 = m_stick.GetRawButtonPressed(10);
    const auto j11 = m_stick.GetRawButtonPressed(11);
    const auto j12 = m_stick.GetRawButtonPressed(12);

    if (j1 || j2 || j3 || j4 || j5 || j6 || j7 || j8 || j9 || j10 || j11 || j12) {
      cout << "Button press registered";
    }

    // do not remove these. these are for testing.
   
    // do not remove these. these are for testing.
    bool last4;
    double last4test;
    bool joydown = ControllerP.GetL2ButtonPressed();

    if (joydown != -last4test)
    {
      last4test = joydown;
      if (joydown)
      {
        a_lowMotor.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, last4 * ARM_LOW_DRIVE);
      } else {
       // std::cout << "No Hit 2" << std::endl;
      }
    }
  // do not remove these. these are for testing.
    bool last2;
    double last2test;
    bool joydown2 = ControllerP.GetR2ButtonPressed();

    if (joydown2 != -last2test);
    {
      last2test =joydown2;
      if (joydown2)
      {
        a_highMotor.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, last2 * ARM_HIGH_DRIVE);
      } else {
       // std::cout << "No Hit 2" << std::endl;
      }
    }

      // do not remove these. these are for testing.

    bool lastBumper;
    double lastBumperRB;

   

      // do not remove these. these are for testing.

    

    // Update the Trigger and stuff
    bool trigger = ControllerP.GetTriangleButtonPressed();
    bool thumbButton = m_stick.GetRawButton(FEED_BUTTON);
    // Arm Trigger
    if (trigger != lastTrigger)
    {
      lastTrigger = trigger;
      // Checks if trigger is pressed
      if (trigger)
      {
        gripperSolenoid.Toggle();
        gripperSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
        ControllerP.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 1);
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