// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotArm.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <ctre/Phoenix.h>

constexpr int kPIDLoopIdx = 0;
constexpr int kTimeoutMs = 30;
constexpr double kCountsPerDegree = 4096.0 / 360;
 RobotArm::RobotArm()
  {
    //for the love of god dont set Lower PID/P over 2
    frc::SmartDashboard::PutNumber("Lower PID/P", 0.7);
    frc::SmartDashboard::PutNumber("Lower PID/I", 0.000);
    frc::SmartDashboard::PutNumber("Lower PID/D", 0);
    frc::SmartDashboard::PutNumber("Higher PID/P", 1.2/*was 1.5 and working*/); // speed of upper arm
    frc::SmartDashboard::PutNumber("Higher PID/I", 0.000);
    frc::SmartDashboard::PutNumber("Higher PID/D", 0.0);

	  std::cout << "PIDs Posted to Shuffle";


	armState = 0;

	LoadParameters();

	// m_lowerArmMotorController.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
	// m_higherArmMotorController.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
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
		m_lowerArmMotorController.Config_kP(kPIDLoopIdx, frc::SmartDashboard::GetNumber("Lower PID/P", 0.0), kTimeoutMs);
		m_lowerArmMotorController.Config_kI(kPIDLoopIdx, frc::SmartDashboard::GetNumber("Lower PID/I", 0.0), kTimeoutMs);
		m_lowerArmMotorController.Config_kD(kPIDLoopIdx, frc::SmartDashboard::GetNumber("Lower PID/D", 0.0), kTimeoutMs);

		m_higherArmMotorController.Config_kP(kPIDLoopIdx, frc::SmartDashboard::GetNumber("Higher PID/P", 0.0), kTimeoutMs);
		m_higherArmMotorController.Config_kI(kPIDLoopIdx, frc::SmartDashboard::GetNumber("Higher PID/I", 0.0), kTimeoutMs);
		m_higherArmMotorController.Config_kD(kPIDLoopIdx, frc::SmartDashboard::GetNumber("Higher PID/D", 0.0), kTimeoutMs);

	}

  void RobotArm::ArmPeriodic() {
	frc::TrapezoidProfile<units::degree> highArmProfile{
		constraints,
		frc::TrapezoidProfile<units::degree>::State{
			units::degree_t(angles[armState][1]),
			units::degrees_per_second_t(0)
		},
		currentState
	};

	currentState = highArmProfile.Calculate(20_ms);

	SetHigherArmAngle(currentState.position.value());

	frc::TrapezoidProfile<units::degree> lowArmProfile{
		lowerConstraints,
		frc::TrapezoidProfile<units::degree>::State{
			units::degree_t(angles[armState][0]),
			units::degrees_per_second_t(0)
		},
		currentLowerState
	};

	currentLowerState = lowArmProfile.Calculate(20_ms);

	SetLowerArmAngle(currentLowerState.position.value());

  }
  
  void RobotArm::setNewArmPos(int stateUpdate){
	armState = stateUpdate;
	feedForward = FeedForwardCalc();
    currentState = frc::TrapezoidProfile<units::degree>::State {
      units::degree_t(GetHigherArmAngle()),
      units::degrees_per_second_t(0)
    };

	currentLowerState = frc::TrapezoidProfile<units::degree>::State {
      units::degree_t(GetLowerArmAngle()),
      units::degrees_per_second_t(0)
    };
  }

  double RobotArm::FeedForwardCalc(){
	return FEED_FORWARD_CONSTANT*std::cos(((angles[armState][1])-HIGH_ARM_LEVEL_OFFSET)*(M_PI/180));
  }

  double RobotArm::GetLowerArmAngle() {
    return m_lowerArmMotorController.GetSensorCollection().GetQuadraturePosition() / kCountsPerDegree;
  }

	double RobotArm::GetHigherArmAngle() {
    return m_higherArmMotorController.GetSensorCollection().GetQuadraturePosition() / kCountsPerDegree;
  }

  void RobotArm::SetLowerArmAngle(double angle){
    angle = (std::max(0.0, std::min(LOW_ARM_FULL_DEPLOY, angle)))*kCountsPerDegree;
	m_lowerArmMotorController.Set(ControlMode::Position, angle);
  }

  void RobotArm::SetHigherArmAngle(double angle){
    angle = (std::max(-180.0, std::min(0.0, angle)))*kCountsPerDegree;
	std::cout<<"angle: "<<angle<<std::endl;
    m_higherArmMotorController.Set(ControlMode::Position, angle, DemandType_ArbitraryFeedForward, feedForward);
  }

  bool RobotArm::inRange(double low, double high, double x){
	return (low<=x && x<=high);
  }

  bool RobotArm::armAngleCheck(int deviation){
	if (inRange(angles[armState][0]-deviation, angles[armState][0]+deviation, GetLowerArmAngle())&&inRange(angles[armState][1]-deviation+15, angles[armState][1]+deviation+15, GetHigherArmAngle())){
		return true;
	}
	return false;
  }