/*
 * GreyhillEncoder.h
 *
 *  Created on: Jul 10, 2017
 *      Author: Team2481
 */

#ifndef SRC_GREYHILLENCODER_H_
#define SRC_GREYHILLENCODER_H_

#include "CANTalon.h"
#include "utils/Translation2D.h"

class GreyhillEncoder {
	private:
	CANTalon* m_talon;
	std::string m_name;
	std::string m_calibrationKey;
	Translation2D m_offset;
	int m_ticksPerRev;
	int m_inchesPerRev;

public:
	GreyhillEncoder(CANTalon* talon, const std::string& name, int ticksPerRev, int inchesPerRev);
	virtual ~GreyhillEncoder();
	Translation2D GetRawDistance() const;
	Translation2D GetDistance() const;
	double GetSpeed() const;
	int GetRotations() const;
	int GetPosition() const;
	double ConvertRotationsToInches(int rotations) const;
	int ConvertInchesToRotations(const Translation2D& inches);
	void SetEncoderRaw(int ticks);
	void Reset();
};

#endif /* SRC_GREYHILLENCODER_H_ */
