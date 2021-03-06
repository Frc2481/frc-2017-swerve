/*
 * SwerveModuleV2.h
 *
 *  Created on: Jul 17, 2017
 *      Author: Team2481
 */

#ifndef SRC_SWERVEMODULEV2_H_
#define SRC_SWERVEMODULEV2_H_

#include <WPILib.h>
#include "ctre/Phoenix.h"
//#include "RobotParameters.h"
#include "utils\Translation2D.h"

class CTREMagEncoder;
class GreyhillEncoder;

class SwerveModuleV2 {
private:
	TalonSRX *m_steerMotor;
	TalonSRX *m_driveMotor;
	CTREMagEncoder *m_steerEncoder;
	GreyhillEncoder *m_driveEncoder;
	bool m_optimizationEnabled;
	bool m_angleOptimized;
	bool m_isMoving;
	bool m_isCloseLoopControl;
	bool m_motionMagic;
	std::string m_name;

public:
	SwerveModuleV2(uint32_t driveID, uint32_t steerID, const std::string name);
	virtual ~SwerveModuleV2();

	Rotation2D GetAngle() const;
	void SetOptimized(bool isOptimized);
	void SetAngle(Rotation2D angle, bool force = false);
	bool IsSteerOnTarget() const;

	void SetOpenLoopSpeed(double speed);
	double GetSpeed() const;
	void SetCloseLoopDriveDistance(Translation2D distance);
	void DisableCloseLoopDrive();
	Translation2D GetDistance() const;
	void ZeroDriveDistance();
	double GetDistanceError() const;
	bool IsDriveOnTarget() const;

	void Set(double speed, Rotation2D angle);

	void SetBrake(bool brake);

	void SetMagicAccel(double accel);
};

#endif /* SRC_SWERVEMODULEV2_H_ */
