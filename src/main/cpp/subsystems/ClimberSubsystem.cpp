// Copytruss (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ClimberSubsystem.h"
#include "RobotParameters.h"

ClimberSubsystem::ClimberSubsystem() :
m_floorWheelsSpeed(0),
m_trussWheelsSpeed(0),
m_areFloorWheelsExtended(false),
m_areTrussWheelsExtended(false),
m_isJavelinExtended(false),
m_pFloorSolenoid(frc::PneumaticsModuleType::REVPH, SolenoidPorts::kFloorClimberSolenoidPort,SolenoidPorts::kFloorClimberSolenoidReversePort),
m_pTrussSolenoid(frc::PneumaticsModuleType::REVPH, SolenoidPorts::kTrussClimberSolenoidPort,SolenoidPorts::kTrussClimberSolenoidReversePort),
m_pJavelinSolenoid(frc::PneumaticsModuleType::REVPH, SolenoidPorts::kJavelinSolenoidPort)
{
    m_pFloorMotor = new TalonFXMotorController(FalconIDs::kFloorClimberMotorID, "floorClimberMotor");
    m_pTrussMotor = new TalonFXMotorController(FalconIDs::kTrussClimberMotorID, "trussClimberMotorMotor");
}

void ClimberSubsystem::extendFloorWheels(){
    m_pFloorSolenoid.Set(m_pFloorSolenoid.kForward);
    m_areFloorWheelsExtended = true;
}
void ClimberSubsystem::extendTrussWheels(){
    m_pTrussSolenoid.Set(m_pTrussSolenoid.kForward);
    m_areTrussWheelsExtended = true;

}
void ClimberSubsystem::retractFloorWheels(){
    m_pFloorSolenoid.Set(m_pFloorSolenoid.kReverse);
    m_areFloorWheelsExtended = false;
}
void ClimberSubsystem::retractTrussWheels(){
    m_pTrussSolenoid.Set(m_pTrussSolenoid.kReverse);
    m_areTrussWheelsExtended = true;
}
void ClimberSubsystem::setFloorWheelsSpeed(double speed){
  m_floorWheelsSpeed = speed;
  m_pFloorMotor->Set(speed);
}
void ClimberSubsystem::setTrussWheelsSpeed(double speed){
  m_trussWheelsSpeed = speed;
  m_pTrussMotor->Set(speed);
}
double ClimberSubsystem::getTrussWheelSpeed(){
  return m_trussWheelsSpeed;
}
double ClimberSubsystem::getFloorWheelSpeed(){
  return m_floorWheelsSpeed;
}
void ClimberSubsystem::fireJavelin(){
  m_pJavelinSolenoid.Set(true);
  m_isJavelinExtended = true;
}
void ClimberSubsystem::retractJavelin(){
  m_pJavelinSolenoid.Set(false);
  m_isJavelinExtended = false;
}
bool ClimberSubsystem::areFloorWheelsExtended(){
  return m_areFloorWheelsExtended;
}
bool ClimberSubsystem::areTrussWheelsExtended(){
  return m_areTrussWheelsExtended;
}
bool ClimberSubsystem::isJavelinExtended(){
  return m_isJavelinExtended;
}

// This method will be called once per scheduler run
void ClimberSubsystem::Periodic() {}
