// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ClimberSubsystem.h"
#include "RobotParameters.h"

ClimberSubsystem::ClimberSubsystem() :
m_leftWheelsSpeed(0),
m_rightWheelsSpeed(0),
m_areLeftWheelsExtended(false),
m_areRightWheelsExtended(false),
m_isJavelinExtended(false),
m_pLeftSolenoid(frc::PneumaticsModuleType::REVPH, SolenoidPorts::kLeftClimberSolenoidPort,SolenoidPorts::kLeftClimberSolenoidReversePort),
m_pRightSolenoid(frc::PneumaticsModuleType::REVPH, SolenoidPorts::kRightClimberSolenoidPort,SolenoidPorts::kRightClimberSolenoidReversePort),
m_pJavelinSolenoid(frc::PneumaticsModuleType::REVPH, SolenoidPorts::kJavelinSolenoidPort)
{
    m_pLeftMotor = new TalonFXMotorController(FalconIDs::kLeftClimberMotorID, "leftClimberMotor");
    m_pRightMotor = new TalonFXMotorController(FalconIDs::kRightClimberMotorID, "rightClimberMotorMotor");
}

void ClimberSubsystem::extendLeftWheels(){
    m_pLeftSolenoid.Set(m_pLeftSolenoid.kForward);
    m_areLeftWheelsExtended = true;
}
void ClimberSubsystem::extendRightWheels(){
    m_pRightSolenoid.Set(m_pRightSolenoid.kForward);
    m_areRightWheelsExtended = true;

}
void ClimberSubsystem::retractLeftWheels(){
    m_pLeftSolenoid.Set(m_pLeftSolenoid.kReverse);
    m_areLeftWheelsExtended = false;
}
void ClimberSubsystem::retractRightWheels(){
    m_pRightSolenoid.Set(m_pRightSolenoid.kReverse);
    m_areRightWheelsExtended = true;
}
void ClimberSubsystem::setLeftWheelsSpeed(double speed){
  m_leftWheelsSpeed = speed;
  m_pLeftMotor->Set(speed);
}
void ClimberSubsystem::setRightWheelsSpeed(double speed){
  m_rightWheelsSpeed = speed;
  m_pRightMotor->Set(speed);
}
double ClimberSubsystem::getRightWheelSpeed(){
  return m_rightWheelsSpeed;
}
double ClimberSubsystem::getLeftWheelSpeed(){
  return m_leftWheelsSpeed;
}
void ClimberSubsystem::fireJavelin(){
  m_pJavelinSolenoid.Set(true);
  m_isJavelinExtended = true;
}
void ClimberSubsystem::retractJavelin(){
  m_pJavelinSolenoid.Set(false);
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
