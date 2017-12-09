/*
 * GreyhillEncoder.cpp
 *
 *  Created on: Jul 10, 2017
 *      Author: Team2481
 */

#include <GreyhillEncoder.h>
#include <sstream>
#include <WPILib.h>

GreyhillEncoder::GreyhillEncoder(CANTalon* talon, const std::string& name, int ticksPerRev, int inchesPerRev)
	: m_talon(talon), m_name(name), m_ticksPerRev(ticksPerRev), m_inchesPerRev(inchesPerRev) {

	m_talon->ConfigEncoderCodesPerRev(ticksPerRev);
	m_talon->SetFeedbackDevice(CANTalon::QuadEncoder);
	m_talon->SetStatusFrameRateMs(CANTalon::StatusFrameRateFeedback, 10);
}

GreyhillEncoder::~GreyhillEncoder() {
	// TODO Auto-generated destructor stub
}

Translation2D GreyhillEncoder::GetRawDistance() const {
	return Translation2D(ConvertRotationsToInches(GetPosition()), 0);
}

Translation2D GreyhillEncoder::GetDistance() const {
	return GetRawDistance().translateBy(m_offset.inverse());
}

int GreyhillEncoder::GetPosition() const {
	return m_talon->GetPosition();
}

double GreyhillEncoder::ConvertRotationsToInches(int rotations) const {
	return rotations * m_inchesPerRev;
}

int GreyhillEncoder::ConvertInchesToRotations(const Translation2D& inches) {
	return inches.getX() * m_ticksPerRev;
}

void GreyhillEncoder::SetEncoderRaw(int ticks) {
	m_talon->SetPosition(ticks);
}

double GreyhillEncoder::GetSpeed() const {
	return m_talon->GetSpeed();
}

void GreyhillEncoder::Reset() {
	m_offset = GetRawDistance();
}
