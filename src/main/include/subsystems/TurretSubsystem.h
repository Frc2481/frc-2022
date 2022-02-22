// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "components/TalonFXMotorController.h"
#include <frc/DigitalInput.h>
#include "networktables/NetworkTableInstance.h"
#include <frc/AnalogInput.h>

class TurretSubsystem : public frc2::SubsystemBase {
 public:
  TurretSubsystem();

  bool isTurretRunning();
  bool isOnTarget();
  double getAngleToTarget();
  double getTurretAngle();
  double getDistance();
  void rotateTurret(double angle); //degrees
  bool isTargetVisible();
  double getTurretAbsoluteAngle();
  double getTurretRelativeAngle();
  void zeroTurret();
  
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  bool m_isTurretRunning;
  bool m_isOnTarget;
  double m_angle_ticks;
  double m_angleOffsetTicks;
  double m_distance;
  

  frc::AnalogInput* m_ABSPositionSensor;
  

  TalonFXMotorController* m_pTurretMotor;
};
