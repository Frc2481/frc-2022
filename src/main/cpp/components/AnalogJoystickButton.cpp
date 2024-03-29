#include "Components/AnalogJoystickButton.h"
#include <frc/smartdashboard/SmartDashboard.h>

AnalogJoystickButton::AnalogJoystickButton(frc::GenericHID *joystick, uint32_t axisNumber, double threshold)
	: m_threshold(threshold),
	m_pJoystick(joystick),
	m_axisNumber(axisNumber) {
}

AnalogJoystickButton::~AnalogJoystickButton() {
}

bool AnalogJoystickButton::Get() {
	frc::SmartDashboard::PutNumber("trigger info", m_threshold);
	if(m_threshold < 0) {
		return m_pJoystick->GetRawAxis(m_axisNumber) < m_threshold;
	}
	else if(m_threshold > 0) {
		return m_pJoystick->GetRawAxis(m_axisNumber) > m_threshold;
	}
	
	return false;
}
