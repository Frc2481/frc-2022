// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "components/VictorMotorController.h"
#include <frc/doubleSolenoid.h>

class IntakeSubsystem : public frc2::SubsystemBase {
 public:
  IntakeSubsystem();
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void extendIntake();
  void retractIntake();
  bool isRollerOn();
  bool isIntakeExtended();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  bool m_isRollerOn;
  bool m_isIntakeExtended;
  int m_rollerSpeed;
  frc::DoubleSolenoid m_intakeSolenoid;
  VictorMotorController* m_rollerMotor;
};