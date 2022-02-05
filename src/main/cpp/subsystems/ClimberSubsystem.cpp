// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ClimberSubsystem.h"

ClimberSubsystem::ClimberSubsystem():

m_frontWheelsSpeed(0),
m_backWheelsSpeed(0),
m_areFrontWheelsExtended(false),
m_areBackWheelsExtended(false),
m_isClimberExtended(false)
{
    m_frontWheelsMotor = new TalonFXMotorController(FalconIDs::kFrontClimberMotorID, "FrontClimberMotor");
    m_backWheelsMotor = new TalonFXMotorController(FalconIDs::kBackClimberMotorID, "BackClimberMotorMotor");
}

void extendFrontWheels();
  void extendBackWheels();
  void retractFrontWheels();
  void retractBackWheels();
  void setFrontWheelsSpeed(double speed);
  void setBackWheelsSpeed(double speed);
  double getBackWheelSpeed();
  double getFrontWheelSpeed();
  void extendClimber();
  void retractClimber();
  bool areFrontWheelsExtended();
  bool areBackWheelsExtended();
  bool isClimberExtended();

// This method will be called once per scheduler run
void ClimberSubsystem::Periodic() {}
