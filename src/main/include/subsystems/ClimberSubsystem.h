// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "components/TalonFXMotorController.h"
#include <frc/doubleSolenoid.h>

class ClimberSubsystem : public frc2::SubsystemBase {
 public:
  ClimberSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

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

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  double m_frontWheelsSpeed;
  double m_backWheelsSpeed;
  bool m_areFrontWheelsExtended;
  bool m_areBackWheelsExtended;
  bool m_isClimberExtended;
  TalonFXMotorController* m_frontWheelsMotor;
  TalonFXMotorController* m_backWheelsMotor;
  frc::DoubleSolenoid* m_frontSolenoid;
  frc::DoubleSolenoid* m_backSolenoid;
  frc::DoubleSolenoid* m_climberSolenoid;
};