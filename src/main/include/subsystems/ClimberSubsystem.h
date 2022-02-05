// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "components/TalonFXMotorController.h"
#include <frc/DoubleSolenoid.h>
#include <frc/Solenoid.h>

class ClimberSubsystem : public frc2::SubsystemBase {
 public:
  ClimberSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void extendLeftWheels();
  void extendRightWheels();
  void retractLeftWheels();
  void retractRightWheels();
  void setLeftWheelsSpeed(double speed);
  void setRightWheelsSpeed(double speed);
  double getRightWheelSpeed();
  double getLeftWheelSpeed();
  void fireJavelin();
  void retractJavelin();
  bool areLeftWheelsExtended();
  bool areRightWheelsExtended();
  bool isJavelinExtended();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  double m_leftWheelsSpeed;
  double m_rightWheelsSpeed;
  bool m_areLeftWheelsExtended;
  bool m_areRightWheelsExtended;
  bool m_isJavelinExtended;
  TalonFXMotorController* m_leftMotor;
  TalonFXMotorController* m_rightMotor;
  frc::DoubleSolenoid* m_leftSolenoid;
  frc::DoubleSolenoid* m_rightSolenoid;
  frc::Solenoid* m_JavelinSolenoid;
};