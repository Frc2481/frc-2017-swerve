
#include "OI.h"
#include <WPILib.h>
#include "Components/Joystick2481.h"

OI::OI() {
	// Process operator interface input here.
	m_driverStick = new Joystick2481(0);
	m_operatorStick = new Joystick2481(1);
}

Joystick2481* OI::GetDriverStick() {
	return m_driverStick;
}

Joystick2481* OI::GetOperatorStick() {
	return m_operatorStick;
}
