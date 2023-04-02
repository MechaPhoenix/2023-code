// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

using namespace std;
#include "Robot.h"
#include "cameraserver/CameraServer.h"

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
  
  // Creates UsbCamera and MjpegServer [1] and connects them
  cs::UsbCamera armCam = frc::CameraServer::StartAutomaticCapture("arm", 0);
  cs::UsbCamera chassisCam = frc::CameraServer::StartAutomaticCapture("chassis", 1);

  // Creates the CvSink and connects it to the UsbCamera
  cs::CvSink cvSinkArm = frc::CameraServer::GetVideo("arm");
  cs::CvSink cvSinkChassis = frc::CameraServer::GetVideo("chassis");

  // Creates the CvSource and MjpegServer [2] and connects them
  cs::CvSource outputStreamArm = frc::CameraServer::PutVideo("Arm", 1280, 720);
  cs::CvSource outputStreamChassis = frc::CameraServer::PutVideo("Chassis", 1280, 720);

  // Set some follow stuff
  m_rightMotor2.Follow(m_rightMotor);
  m_leftMotor2.Follow(m_leftMotor);

  m_rightMotor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  m_rightMotor2.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  m_leftMotor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  m_leftMotor2.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);

  pcmCompressor.EnableDigital();
  pcmCompressor.Disable();

  ahrs = new AHRS(frc::SPI::Port::kMXP);
  ahrs->Calibrate();

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
  frc::SmartDashboard::PutNumber("State", mAutoBalance.getState());
  frc::SmartDashboard::PutNumber("Lower EV", m_arm.GetLowerArmAngle());
  frc::SmartDashboard::PutNumber("Higher EV", m_arm.GetHigherArmAngle());
  frc::SmartDashboard::PutNumber("Pitch", ahrs->GetPitch());
  if (m_stick.GetRawButtonPressed(7)){mAutoBalance.autoScore = !mAutoBalance.autoScore;};
  if (m_stick.GetRawButtonPressed(8)){mAutoBalance.autoTaxi = !mAutoBalance.autoTaxi;};
  if (m_stick.GetRawButtonPressed(9)){mAutoBalance.autoBalancing = !mAutoBalance.autoBalancing;};
  frc::SmartDashboard::PutBoolean("Score", mAutoBalance.autoScore);
  frc::SmartDashboard::PutBoolean("Taxi", mAutoBalance.autoTaxi);
  frc::SmartDashboard::PutBoolean("Balance", mAutoBalance.autoBalancing);
	frc::SmartDashboard::PutNumber("Command Position", m_arm.angles[m_arm.armState][1]);
}

void Robot::AutonomousInit()
{
  m_arm.armState = 0;
  if (mAutoBalance.autoScore) {
        mAutoBalance.state = 5;
    }else if (mAutoBalance.autoTaxi) {
        mAutoBalance.state = 4;
    }else if (mAutoBalance.autoBalancing) {
        mAutoBalance.state = 0;
    }else {
        mAutoBalance.state = 6;
    }
}

void Robot::AutonomousPeriodic()
{
  double speed = mAutoBalance.autoBalanceRoutine(ahrs);
  mAutoBalance.currentSpeed = speed;
  setDrive(speed, speed);
  m_arm.SetHigherArmAngle(0);
  m_arm.SetLowerArmAngle(0);
  m_arm.feedForward = 0.1;
  m_arm.ArmPeriodic();
}

void Robot::TeleopPeriodic()
{
  pov = m_stick.GetPOV();
  if(pov == 180){
    boost = BOOST_POWER;
  }else if (pov == 0){
    boost = -BOOST_POWER;
  }else{
    boost = 0;
  }

  // Drive with arcade style
  // This is where all our fun stuff goes :)
  // Read the joystick, calculate the drive stuff
  double x = m_stick.GetX()*JOYSTICK_ROT_SENS; // In terms of arcade drive, this is speed
  double y = m_stick.GetY()*DEFENCE_JOYSTICK_SENSITIVITY; // In terms of arcade drive, this is turn

  // Drive
  double leftPower = (y - x) / 2;
  double rightPower = (y + x) / 2;

  // Update the LEFT drive
  if (leftPower != lastDriveLeft)
  {
    // Update the LEFT drive
    m_leftMotor.Set(ctre::phoenix::motorcontrol::VictorSPXControlMode::PercentOutput, leftPower * S_LEFT_DRIVE + boost);
    lastDriveLeft = leftPower;
  }

  // Update the RIGHT drive
  if (rightPower != lastDriveRight)
  {
    // Update the RIGHT drive
    m_rightMotor.Set(ctre::phoenix::motorcontrol::VictorSPXControlMode::PercentOutput, rightPower * S_RIGHT_DRIVE + boost);
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

  if (m_stick.GetRawButtonPressed(4)){
    m_arm.setNewArmPos(0);
  }
  if (m_stick.GetRawButtonPressed(6)){
    m_arm.setNewArmPos(1);
  }
  if (m_stick.GetRawButtonPressed(5)){
    m_arm.setNewArmPos(2);
  }
  if (m_stick.GetRawButtonPressed(3)){
    m_arm.setNewArmPos(3);
  }

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