/*
 * SwerveModuleV2.cpp
 *
 *  Created on: Jul 17, 2017
 *      Author: Team2481
 */

#include <SwerveModuleV2.h>
#include <Components\SwerveModuleV2Constants.h>
#include <math.h>
#include <sstream>
#include <CTREMagEncoder.h>
#include <GreyhillEncoder.h>
//#include <RoboUtils.h>

SwerveModuleV2::SwerveModuleV2(uint32_t driveID, uint32_t steerID,
		const std::string name) {
	m_name = name;
	std::stringstream ss;
	ss << name << "_STEER_ENCODER";
	m_steerMotor = new CANTalon(steerID);
	m_steerEncoder = new CTREMagEncoder(m_steerMotor, ss.str());
	ss.str("");
	ss << name << "_DRIVE_ENCODER";
	m_driveMotor = new CANTalon(driveID);
	m_driveEncoder = new GreyhillEncoder(m_driveMotor, ss.str(),
			SwerveModuleV2Constants::k_ticksPerRev,
			SwerveModuleV2Constants::k_inchesPerRev);

	m_isCloseLoopControl = false;
	m_angleOptimized = false;
	m_optimizationEnabled = true;
	m_isMoving = false;
	m_motionMagic = false;

	m_driveMotor->SelectProfileSlot(0);
	m_driveMotor->SetControlMode(CANTalon::kPercentVbus);
	m_driveMotor->SetPID(SwerveModuleV2Constants::k_speedP,
			SwerveModuleV2Constants::k_speedI,
			SwerveModuleV2Constants::k_speedD);
	m_driveMotor->SetF(0.1722);
	m_driveMotor->SetIzone(200);
	m_driveMotor->SetSensorDirection(true);
	m_driveMotor->SetClosedLoopOutputDirection(true);

	m_driveMotor->ConfigNominalOutputVoltage(0.0,0.0);
	m_driveMotor->ConfigPeakOutputVoltage(12.0,-12.0);
//	m_driveMotor->SetMotionMagicAcceleration(m_accel);
//	m_driveMotor->SetMotionMagicCruiseVelocity(m_velocity);

	m_driveMotor->ConfigNeutralMode(CANTalon::kNeutralMode_Brake);

	m_steerMotor->SelectProfileSlot(0); //Profile 1 PIDf are P = 0.2 f = 1.1
	m_steerMotor->ConfigNominalOutputVoltage(0,0);
	m_steerMotor->SetControlMode(CANTalon::kPosition);
	m_steerMotor->ConfigNeutralMode(CANTalon::kNeutralMode_Brake);
	m_steerMotor->SetPID(SwerveModuleV2Constants::k_steerP,
			SwerveModuleV2Constants::k_steerI,
			SwerveModuleV2Constants::k_steerD);
	m_steerMotor->SetSensorDirection(true);
	m_steerMotor->SetClosedLoopOutputDirection(false);
	m_steerMotor->SetPulseWidthPosition(m_steerMotor->GetPulseWidthPosition() & 0xFFF);
	m_steerMotor->Enable();
	m_steerMotor->SetAllowableClosedLoopErr(40);
	m_steerMotor->SetStatusFrameRateMs(CANTalon::StatusFrameRateFeedback, 10);
//	m_steerMotor->SetStatusFrameRateMs(CANTalon::StatusFrameRateGeneral, 10);
}

SwerveModuleV2::~SwerveModuleV2() {
	// TODO Auto-generated destructor stub
}

Rotation2D SwerveModuleV2::GetAngle() const {
	return m_steerEncoder->GetAngle();
}

void SwerveModuleV2::SetOptimized(bool isOptimized) {
	m_optimizationEnabled = isOptimized;
}

void SwerveModuleV2::SetAngle(Rotation2D angle, bool force) {
	if(m_isMoving || force) {
		Rotation2D currentAngle = m_steerEncoder->GetAngle();
		Rotation2D deltaAngle = currentAngle.rotateBy(angle.inverse());
		if(m_optimizationEnabled &&
		   deltaAngle.getRadians() > M_PI_2 &&
		   deltaAngle.getRadians() < 3 * M_PI_2) {
			angle = angle.rotateBy(Rotation2D::fromRadians(M_PI));
			m_angleOptimized = true;
		}
		else {
			m_angleOptimized = false;
		}
		int setpoint = m_steerEncoder->ConvertAngleToSetpoint(angle);
		m_steerMotor->Set(setpoint / 4096.0);

		SmartDashboard::PutNumber("CurrentAngle", currentAngle.getDegrees());
		SmartDashboard::PutNumber("DeltaAngle", deltaAngle.getDegrees());
		SmartDashboard::PutNumber("Setpoint", setpoint);
	}
	SmartDashboard::PutNumber("ActualAngle", GetAngle().getDegrees());
	SmartDashboard::PutNumber("SetAngle", angle.getDegrees());
	SmartDashboard::PutNumber("EncoderTicks", m_steerEncoder->GetEncoderTicks(true));


}

bool SwerveModuleV2::IsSteerOnTarget() const {
	return fabs(m_steerMotor->GetClosedLoopError()) <= 20;
}

void SwerveModuleV2::SetOpenLoopSpeed(double speed) {
	if(m_driveMotor->GetTalonControlMode() != CANTalon::kThrottleMode){
		m_driveMotor->SetTalonControlMode(CANTalon::kThrottleMode);
	}
	if(m_angleOptimized) {
		speed *= -1;
	}
	m_driveMotor->Set(speed);
	m_isMoving = fabs(speed) > .05;
	m_isCloseLoopControl = false;
}

double SwerveModuleV2::GetSpeed()const {
	return m_driveEncoder->GetSpeed();
}

void SwerveModuleV2::SetCloseLoopDriveDistance(Translation2D distance) {
	if(m_driveMotor->GetTalonControlMode() != CANTalon::kMotionMagicMode){
		m_driveMotor->SetTalonControlMode(CANTalon::kMotionMagicMode);
	}
	double distInches = distance.getX();
	if(m_angleOptimized) {
		distInches *= -1;
	}
	m_driveMotor->Set(distInches);
	m_isMoving = true;
	m_isCloseLoopControl = true;
}

void SwerveModuleV2::DisableCloseLoopDrive() {
	SetOpenLoopSpeed(0);
}

Translation2D SwerveModuleV2::GetDistance() const {
	return m_driveEncoder->GetDistance();
}

void SwerveModuleV2::ZeroDriveDistance() {
	m_driveEncoder->Reset();
}

double SwerveModuleV2::GetDistanceError() const {
	return m_driveMotor->GetClosedLoopError();
}

bool SwerveModuleV2::IsDriveOnTarget() const {
	return GetDistanceError() < 4; //absolute value?
}

void SwerveModuleV2::Set(double speed, Rotation2D angle) {
	if(m_isCloseLoopControl){
		SetAngle(angle, true);
	}
	else {
		SetOpenLoopSpeed(speed);
		SetAngle(angle, false);
	}
}

void SwerveModuleV2::SetBrake(bool brake) {
	m_driveMotor->ConfigNeutralMode(brake ? CANTalon::kNeutralMode_Brake : CANTalon::kNeutralMode_Coast);
}

void SwerveModuleV2::SetMagicAccel(double accel) {
	m_driveMotor->SetMotionMagicAcceleration(accel);
}
