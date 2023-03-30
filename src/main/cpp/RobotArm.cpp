// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotArm.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <ctre/Phoenix.h>

constexpr int kPIDLoopIdx = 0;
constexpr int kTimeoutMs = 30;
constexpr int kCountsPerDegree = 4096 / 360;
 RobotArm::RobotArm()
  {
    //for the love of god dont set Lower PID/P over 2
    frc::SmartDashboard::PutNumber("Lower PID/P", 0.75);
    frc::SmartDashboard::PutNumber("Lower PID/I", 0.000);
    frc::SmartDashboard::PutNumber("Lower PID/D", 0);
    frc::SmartDashboard::PutNumber("Higher PID/P", 1.35);
    frc::SmartDashboard::PutNumber("Higher PID/I", 0.000);
    frc::SmartDashboard::PutNumber("Higher PID/D", 0.0);

	armState = 0;
	oldArmState = 0;

	LoadParameters();
  }

void RobotArm::LoadParameters() {
    m_lowerArmMotorController.ConfigFactoryDefault();

		/**
		 * Grab the 360 degree position of the MagEncoder's absolute
		 * position, and intitally set the relative sensor to match.
		 */
		//int lowerAbsolutePosition = m_lowerArmMotorController.GetSensorCollection().GetPulseWidthPosition();
		/* use the low level API to set the quad encoder signal */
		//m_lowerArmMotorController.SetSelectedSensorPosition(lowerAbsolutePosition, kPIDLoopIdx,
				//kTimeoutMs);

		/* choose the sensor and sensor direction */
		m_lowerArmMotorController.ConfigSelectedFeedbackSensor(
				FeedbackDevice::CTRE_MagEncoder_Relative, kPIDLoopIdx,
				kTimeoutMs);
		m_lowerArmMotorController.SetSensorPhase(false);

		/* set the peak and nominal outputs, 12V means full */
		m_lowerArmMotorController.ConfigNominalOutputForward(0, kTimeoutMs);
		m_lowerArmMotorController.ConfigNominalOutputReverse(0, kTimeoutMs);
		m_lowerArmMotorController.ConfigPeakOutputForward(PEAK_ARM_MOTOR_OUTPUT, kTimeoutMs);
		m_lowerArmMotorController.ConfigPeakOutputReverse(-PEAK_ARM_MOTOR_OUTPUT, kTimeoutMs);

		/* set closed loop gains in slot0 */
		//m_lowerArmMotorController.Config_kF(kPIDLoopIdx, 0.0, kTimeoutMs);
		m_lowerArmMotorController.Config_kP(kPIDLoopIdx, frc::SmartDashboard::GetNumber("Lower PID/P", 0.0), kTimeoutMs);
		m_lowerArmMotorController.Config_kI(kPIDLoopIdx, frc::SmartDashboard::GetNumber("Lower PID/I", 0.0), kTimeoutMs);
		m_lowerArmMotorController.Config_kD(kPIDLoopIdx, frc::SmartDashboard::GetNumber("Lower PID/D", 0.0), kTimeoutMs);
		m_lowerArmMotorController.Config_kF(kPIDLoopIdx, -0.025, kTimeoutMs);

        m_higherArmMotorController.ConfigFactoryDefault();

		/**
		 * Grab the 360 degree position of the MagEncoder's absolute
		 * position, and intitally set the relative sensor to match.
		 */
		//int higherAbsolutePosition = m_higherArmMotorController.GetSensorCollection().GetPulseWidthPosition();
		/* use the low level API to set the quad encoder signal */
		//m_higherArmMotorController.SetSelectedSensorPosition(higherAbsolutePosition, kPIDLoopIdx,
				//kTimeoutMs);
        
        m_higherArmMotorController.SetInverted(InvertType::InvertMotorOutput);

		/* choose the sensor and sensor direction */
		m_higherArmMotorController.ConfigSelectedFeedbackSensor(
				FeedbackDevice::CTRE_MagEncoder_Relative, kPIDLoopIdx,
				kTimeoutMs);
		m_higherArmMotorController.SetSensorPhase(true);

		/* set the peak and nominal outputs, 12V means full */
		m_higherArmMotorController.ConfigNominalOutputForward(0, kTimeoutMs);
		m_higherArmMotorController.ConfigNominalOutputReverse(0, kTimeoutMs);
		m_higherArmMotorController.ConfigPeakOutputForward(PEAK_ARM_MOTOR_OUTPUT, kTimeoutMs);
		m_higherArmMotorController.ConfigPeakOutputReverse(-PEAK_ARM_MOTOR_OUTPUT, kTimeoutMs);

		/* set closed loop gains in slot0 */
		//m_lowerArmMotorController.Config_kF(kPIDLoopIdx, 0.0, kTimeoutMs);
		m_higherArmMotorController.Config_kP(kPIDLoopIdx, frc::SmartDashboard::GetNumber("Higher PID/P", 0.0), kTimeoutMs);
		m_higherArmMotorController.Config_kI(kPIDLoopIdx, frc::SmartDashboard::GetNumber("Higher PID/I", 0.0), kTimeoutMs);
		m_higherArmMotorController.Config_kD(kPIDLoopIdx, frc::SmartDashboard::GetNumber("Higher PID/D", 0.0), kTimeoutMs);
		m_higherArmMotorController.Config_kF(kPIDLoopIdx, 0.025, kTimeoutMs);
	}

  void RobotArm::ArmPeriodic() {
    armPositioning();
  }
  
  void RobotArm::armPositioning(){
	// if(armState == 0){
	// 	m_lowerArmMotorController.Config_kP(kPIDLoopIdx, 0.5, kTimeoutMs);
	// }else{
	// 	m_lowerArmMotorController.Config_kP(kPIDLoopIdx, frc::SmartDashboard::GetNumber("Lower PID/P", 0.0), kTimeoutMs);
	// }

	if(armState<oldArmState){
		if (!(inRange(angles[armState][0]-2, angles[armState][0]+2 ,GetLowerArmAngle()))){
			SetLowerArmAngle(angles[armState][0]);
		}else if (!(inRange(angles[armState][1]-2, angles[armState][1]+2 ,GetHigherArmAngle()))){
			SetHigherArmAngle(angles[armState][1]);
		}
	}else{
		if (!(inRange(angles[armState][1]-2, angles[armState][1]+2 ,GetHigherArmAngle()))){
			SetHigherArmAngle(angles[armState][1]);
		}else if (!(inRange(angles[armState][0]-2, angles[armState][0]+2 ,GetLowerArmAngle()))){
			SetLowerArmAngle(angles[armState][0]);
		}
	}
  }

  bool RobotArm::inRange(double low, double high, double x){
	return (low<=x && x<=high);
  }

  double RobotArm::GetLowerArmAngle() {
    return m_lowerArmMotorController.GetSensorCollection().GetQuadraturePosition() / kCountsPerDegree;
  }

   double RobotArm::GetHigherArmAngle() {
    return m_higherArmMotorController.GetSensorCollection().GetQuadraturePosition() / kCountsPerDegree;
  }

  void RobotArm::SetLowerArmAngle(double angle){
    angle = std::max(0.0, std::min(121.0, angle));
	m_lowerArmMotorController.Set(ControlMode::Position, angle*kCountsPerDegree);
  }

  void RobotArm::SetHigherArmAngle(double angle){
    angle = std::max(-125.0, std::min(0.0, angle));
    m_higherArmMotorController.Set(ControlMode::Position, angle*kCountsPerDegree);
  }

  void RobotArm::ResetArms(){
    m_lowerArmMotorController.Config_kP(kPIDLoopIdx, 0.5, kTimeoutMs);
    m_lowerArmMotorController.Set(ControlMode::Position, 0);
    m_lowerArmMotorController.Config_kP(kPIDLoopIdx, frc::SmartDashboard::GetNumber("Lower PID/P", 0.0), kTimeoutMs);
  }


