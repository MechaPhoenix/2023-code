#include <iostream>
#include <frc/Joystick.h>
#include <frc/AnalogGyro.h>
#include <frc/GenericHID.h>
#include <frc/TimedRobot.h>
#include <frc/Compressor.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Solenoid.h>
#include <frc/DoubleSolenoid.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/filter/SlewRateLimiter.h>
#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>

#include "RobotArm.h"
#include "mapping.h"
#include "autoBalance.h"
#include "armAngles.h"

#include "AHRS.h"

class Robot : public frc::TimedRobot
{

  private:

    void RobotInit() override;
    void RobotPeriodic() override;
    void AutonomousInit() override;
    void AutonomousPeriodic() override;
    void TeleopPeriodic() override;

  // JoyStick
  frc::Joystick m_stick{0};
  // Motor Doubles
  double lastDriveRight{0};
  double lastDriveLeft{0};
  // Compressor
  frc::Compressor pcmCompressor{0, frc::PneumaticsModuleType::CTREPCM};
  // Solenoid
  frc::DoubleSolenoid gripperSolenoid{frc::PneumaticsModuleType::CTREPCM, 0, 1};
  // Sol Bools
  bool lastEngage = false;
  bool lastRelease = false;
  // Gyroscope
  frc::AnalogGyro g{0};
  AHRS *ahrs; 
  //AHRS gyro = new AHRS(SerialPort.Port.kMXP);

  double currentJoySens = DEFENCE_JOYSTICK_SENSITIVITY;
  double boost = 0;
  int pov;

  double r_speed = 0.0;
  double l_speed = 0.0;
  frc::SlewRateLimiter<units::scalar> left_sl{1.0 / 0.35_s};
  frc::SlewRateLimiter<units::scalar> right_sl{1.0 / 0.35_s};

  // Drive Motors
  ctre::phoenix::motorcontrol::can::VictorSPX m_leftMotor{15};
  ctre::phoenix::motorcontrol::can::VictorSPX m_rightMotor{14};
  ctre::phoenix::motorcontrol::can::VictorSPX m_leftMotor2{17};
  ctre::phoenix::motorcontrol::can::VictorSPX m_rightMotor2{16};

  // Arm subsystem
  RobotArm m_arm;

  // Auto Mode
  autoBalance mAutoBalance;

  // Robot methods
  void setDrive(double left, double right);

};