// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "components/VictorMotorController.h"

class FeederSubsystem : public frc2::SubsystemBase {
 public:
  FeederSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */

  bool isFeederRunning();
  bool isBallReadyIndexer();
  void primeShooter();
  bool isShooterReady();
  bool isTurretReady();
  void shootBall();
  void setFeederSpeed();
  void stopShooter();
  
  //TODO Function to shoot ball


  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  bool m_isFeederRunning;
  bool m_isBallReadyIndexer; 
  bool m_isShooterReady;
  bool m_isTurretReady;
  double m_feederSpeed;
  VictorMotorController* m_feederMotor;

  //TODO variable that tells you to shoot

};
