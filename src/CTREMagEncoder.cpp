/*
 * CTREMagEncoder.cpp
 *
 *  Created on: Jul 3, 2017
 *      Author: Team2481
 */

#include <sstream>
#include <CTREMagEncoder.h>
#include <WPILib.h>

CTREMagEncoder::CTREMagEncoder(CANTalon* talon, const std::string& name)
	: m_talon(talon), m_name(name) {

	std::stringstream ss;
	ss << "ENCODER_OFFSET_" << name;
	m_calibrationKey = ss.str();

	m_offset = Rotation2D::fromDegrees(Preferences::GetInstance()->GetDouble(m_calibrationKey));

	m_talon->SetFeedbackDevice(CANTalon::CtreMagEncoder_Absolute);
	m_talon->SetSensorDirection(true);
	m_talon->SetPulseWidthPosition(m_talon->GetPulseWidthPosition() & 0xFFF);
	m_talon->SetStatusFrameRateMs(CANTalon::StatusFrameRateFeedback, 10);
}

CTREMagEncoder::~CTREMagEncoder() {
	// TODO Auto-generated destructor stub
}

Rotation2D CTREMagEncoder::GetRawAngle() const {
	return Rotation2D::fromRadians(GetEncoderTicks() / 4096.0 * 2 * M_PI);

}

Rotation2D CTREMagEncoder::GetAngle() const {
	return m_offset.rotateBy(GetRawAngle());
}

int CTREMagEncoder::GetRotations() const {
	SmartDashboard::PutNumber("Rotations", GetEncoderTicks(true)/4096);
	SmartDashboard::PutNumber("dummy", -16/4096);
	return GetEncoderTicks(true) / 4096;
}

int CTREMagEncoder::GetEncoderTicks(bool overflow) const {
	int ticks = m_talon->GetPulseWidthPosition();
	ticks *= -1; //negative b/c Pulse Width Position doesn't
				 //take sensor direction into account
	if (!overflow) {
		ticks &= 0xFFF;
	}
	return ticks;
}

void CTREMagEncoder::Calibrate() {
	m_offset = GetRawAngle().inverse();
	Preferences::GetInstance()->PutDouble(m_calibrationKey, m_offset.getDegrees());
}

int CTREMagEncoder::ConvertAngleToSetpoint(Rotation2D targetAngle) {
	Rotation2D angle = targetAngle.rotateBy(m_offset);
	SmartDashboard::PutNumber("Convert Angle To Setpoint Angle", angle.getDegrees());
	int ticks = ConvertAngleToEncoderTicks(angle);
	SmartDashboard::PutNumber("Convert Angle To Setpoint Ticks", ticks);
	//ticks += GetRotations() * 4096;
	int encoderTicks = GetEncoderTicks(true);
	ticks += (encoderTicks/4096)*4096;
	int error = encoderTicks - ticks;
	SmartDashboard::PutNumber("Convert Angle To Setpoint Error", error);
	if (error < -2048) {
		ticks -= 4096;
	}
	else if (error > 2048) {
		ticks += 4096;
	}
	SmartDashboard::PutNumber("Convert Angle To Setpoint Error2", GetEncoderTicks(true) - ticks);
	return ticks;
}

int CTREMagEncoder::ConvertAngleToEncoderTicks(Rotation2D angle) {
	double degrees = angle.getDegrees();
	return degrees / 360.0 * 4096;
}

void CTREMagEncoder::SetEncoderRaw(int ticks) {
	m_talon->SetPulseWidthPosition(ticks);
}
