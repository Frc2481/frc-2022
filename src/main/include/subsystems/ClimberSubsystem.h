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

  void extendFloorWheels();
  void retractFloorWheels();
  void extendTrussWheels();
  void retractTrussWheels();
  void setFloorWheelsSpeed(double speed);
  void setTrussWheelsSpeed(double speed);
  double getTrussWheelSpeed();
  double getFloorWheelSpeed();
  void fireJavelin();
  void retractJavelin();
  bool areFloorWheelsExtended();
  bool areTrussWheelsExtended();
  bool isJavelinExtended();
 

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  double m_floorWheelsSpeed;
  double m_trussWheelsSpeed;
  bool m_areFloorWheelsExtended;
  bool m_areTrussWheelsExtended;
  bool m_isJavelinExtended;
  TalonFXMotorController* m_pFloorMotor;
  TalonFXMotorController* m_pTrussMotor;
  frc::DoubleSolenoid m_pFloorSolenoid;
  frc::DoubleSolenoid m_pTrussSolenoid;
  frc::DoubleSolenoid m_pJavelinSolenoid;
};