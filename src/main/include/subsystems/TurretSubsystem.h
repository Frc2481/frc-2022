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
  double getTurretAngleTicks();
  double getDistance();
  void rotateTurret(double angle); //degrees
  double getError();
  bool isTargetVisible();
  double getTurretAbsoluteAngle();
  double getTurretCalibratedAngle();
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
  double m_rangeMaxTicks;
  double m_rangeMinTicks;
  bool m_limitAccel;
  double m_angle_to_target;
  double m_vert_angle_to_target;
  int m_target_visible;
  double m_setpointTicks;

  frc::AnalogInput* m_ABSPositionSensor;
  

  TalonFXMotorController* m_pTurretMotor;
};
