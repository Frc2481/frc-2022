// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ClimberSubsystem.h"
#include "Constants.h"

ClimberSubsystem::ClimberSubsystem():

m_leftWheelsSpeed(0),
m_rightWheelsSpeed(0),
m_areLeftWheelsExtended(false),
m_areRightWheelsExtended(false),
m_isJavelinExtended(false)
// m_leftSolenoid(frc::PneumaticsModuleType::CTREPCM, SolenoidPorts::kLeftClimberSolenoidPort,SolenoidPorts::kLeftClimberSolenoidReversePort),
// m_rightSolenoid(frc::PneumaticsModuleType::CTREPCM, SolenoidPorts::kRightClimberSolenoidPort,SolenoidPorts::kRightClimberSolenoidReversePort),
// m_JavelinSolenoid(frc::PneumaticsModuleType::CTREPCM, SolenoidPorts::kJavelinSolenoidPort)
{
    m_leftMotor = new TalonFXMotorController(FalconIDs::kLeftClimberMotorID, "leftClimberMotor");
    m_rightMotor = new TalonFXMotorController(FalconIDs::kRightClimberMotorID, "rightClimberMotorMotor");
}

void ClimberSubsystem::extendLeftWheels(){
    m_leftSolenoid->Set(m_leftSolenoid->kForward);
    m_areLeftWheelsExtended = true;
}
void ClimberSubsystem::extendRightWheels(){

}
void ClimberSubsystem::retractLeftWheels(){

}
void ClimberSubsystem::retractRightWheels(){

}
void ClimberSubsystem::setLeftWheelsSpeed(double speed){

}
void ClimberSubsystem::setRightWheelsSpeed(double speed){

}
double ClimberSubsystem::getRightWheelSpeed(){
  return 0;
}
double ClimberSubsystem::getLeftWheelSpeed(){
  return 0;
}
void ClimberSubsystem::fireJavelin(){

}
void ClimberSubsystem::retractJavelin(){

}
bool ClimberSubsystem::areLeftWheelsExtended(){
  return false;
}
bool ClimberSubsystem::areRightWheelsExtended(){
  return false;
}
bool ClimberSubsystem::isJavelinExtended(){
  return false;
}

// This method will be called once per scheduler run
void ClimberSubsystem::Periodic() {}
