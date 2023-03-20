// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotArm.h"
#include <frc/smartdashboard/SmartDashboard.h>

constexpr int kPIDLoopIdx = 0;
constexpr int kTimeoutMs = 30;

 RobotArm::RobotArm()
  {
    frc::SmartDashboard::PutNumber("Lower PID/P", 0);
    frc::SmartDashboard::PutNumber("Lower PID/I", 0);
    frc::SmartDashboard::PutNumber("Lower PID/D", 0);
    frc::SmartDashboard::PutNumber("Higher PID/P", 0);
    frc::SmartDashboard::PutNumber("Higher PID/I", 0);
    frc::SmartDashboard::PutNumber("Higher PID/D", 0);

	LoadParameters();
  }

void RobotArm::LoadParameters() {
    m_lowerArmMotorController.ConfigFactoryDefault();

		/**
		 * Grab the 360 degree position of the MagEncoder's absolute
		 * position, and intitally set the relative sensor to match.
		 */
		int absolutePosition = m_lowerArmMotorController.GetSensorCollection().GetPulseWidthPosition();
		/* use the low level API to set the quad encoder signal */
		m_lowerArmMotorController.SetSelectedSensorPosition(absolutePosition, kPIDLoopIdx,
				kTimeoutMs);

		/* choose the sensor and sensor direction */
		m_lowerArmMotorController.ConfigSelectedFeedbackSensor(
				FeedbackDevice::CTRE_MagEncoder_Relative, kPIDLoopIdx,
				kTimeoutMs);
		m_lowerArmMotorController.SetSensorPhase(true);

		/* set the peak and nominal outputs, 12V means full */
		m_lowerArmMotorController.ConfigNominalOutputForward(0, kTimeoutMs);
		m_lowerArmMotorController.ConfigNominalOutputReverse(0, kTimeoutMs);
		m_lowerArmMotorController.ConfigPeakOutputForward(1, kTimeoutMs);
		m_lowerArmMotorController.ConfigPeakOutputReverse(-1, kTimeoutMs);

		/* set closed loop gains in slot0 */
		//m_lowerArmMotorController.Config_kF(kPIDLoopIdx, 0.0, kTimeoutMs);
		m_lowerArmMotorController.Config_kP(kPIDLoopIdx, frc::SmartDashboard::GetNumber("Lower PID/P", 0.0), kTimeoutMs);
		m_lowerArmMotorController.Config_kI(kPIDLoopIdx, frc::SmartDashboard::GetNumber("Lower PID/I", 0.0), kTimeoutMs);
		m_lowerArmMotorController.Config_kD(kPIDLoopIdx, frc::SmartDashboard::GetNumber("Lower PID/D", 0.0), kTimeoutMs);
}
  

  void RobotArm::ArmPeriodic() {
    frc::SmartDashboard::PutNumber("Lower Encoder Value", GetLowerArmAngle());
    frc::SmartDashboard::PutNumber("Higher Encoder Value", GetHigherArmAngle());
  }

  double RobotArm::GetLowerArmAngle() {
    return m_lowerArmMotorController.GetSensorCollection().GetQuadraturePosition();
  }

   double RobotArm::GetHigherArmAngle() {
    return m_higherArmMotorController.GetSensorCollection().GetQuadraturePosition();
  }

  void SetLowerArmAngle(){

  }