// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ClimberSubsystem.h"
#include "Constants.h"

ClimberSubsystem::ClimberSubsystem() :
m_leftWheelsSpeed(0),
m_rightWheelsSpeed(0),
m_areLeftWheelsExtended(false),
m_areRightWheelsExtended(false),
m_isJavelinExtended(false),
m_leftSolenoid(frc::PneumaticsModuleType::CTREPCM, SolenoidPorts::kLeftClimberSolenoidPort,SolenoidPorts::kLeftClimberSolenoidReversePort),
m_rightSolenoid(frc::PneumaticsModuleType::CTREPCM, SolenoidPorts::kRightClimberSolenoidPort,SolenoidPorts::kRightClimberSolenoidReversePort),
m_javelinSolenoid(frc::PneumaticsModuleType::CTREPCM, SolenoidPorts::kJavelinSolenoidPort)
{
    m_leftMotor = new TalonFXMotorController(FalconIDs::kLeftClimberMotorID, "leftClimberMotor");
    m_rightMotor = new TalonFXMotorController(FalconIDs::kRightClimberMotorID, "rightClimberMotorMotor");
}

void ClimberSubsystem::extendLeftWheels(){
    m_leftSolenoid.Set(m_leftSolenoid.kForward);
    m_areLeftWheelsExtended = true;
}
void ClimberSubsystem::extendRightWheels(){
    m_rightSolenoid.Set(m_rightSolenoid.kForward);
    m_areRightWheelsExtended = true;

}
void ClimberSubsystem::retractLeftWheels(){
    m_leftSolenoid.Set(m_leftSolenoid.kReverse);
    m_areLeftWheelsExtended = false;
}
void ClimberSubsystem::retractRightWheels(){
    m_rightSolenoid.Set(m_rightSolenoid.kReverse);
    m_areRightWheelsExtended = true;
}
void ClimberSubsystem::setLeftWheelsSpeed(double speed){
  m_leftWheelsSpeed = speed;
  m_leftMotor->Set(speed);
}
void ClimberSubsystem::setRightWheelsSpeed(double speed){
  m_rightWheelsSpeed = speed;
  m_rightMotor->Set(speed);
}
double ClimberSubsystem::getRightWheelSpeed(){
  return m_rightWheelsSpeed;
}
double ClimberSubsystem::getLeftWheelSpeed(){
  return m_leftWheelsSpeed;
}
void ClimberSubsystem::fireJavelin(){
  m_javelinSolenoid.Set(true);
  m_isJavelinExtended = true;
}
void ClimberSubsystem::retractJavelin(){
  m_javelinSolenoid.Set(false);
  m_isJavelinExtended = false;
}
bool ClimberSubsystem::areLeftWheelsExtended(){
  return m_areLeftWheelsExtended;
}
bool ClimberSubsystem::areRightWheelsExtended(){
  return m_areRightWheelsExtended;
}
bool ClimberSubsystem::isJavelinExtended(){
  return m_isJavelinExtended;
}

// This method will be called once per scheduler run
void ClimberSubsystem::Periodic() {}
