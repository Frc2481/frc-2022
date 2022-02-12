// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "components/VictorMotorController.h"
#include <frc/DigitalInput.h>

class FeederSubsystem : public frc2::SubsystemBase {
 public:
  FeederSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */

  bool isFeederRunning();
  bool isIndexerRunning ();
  void setFeederSpeed(double speed);
  void setIndexerSpeed(double speed);
  bool getIndexerBeamBreak ();
  bool getFeederBeamBreak ();
  
  
  //TODO Function to shoot ball


  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  bool m_isFeederRunning;
  bool m_isIndexerRunning;
  VictorMotorController* m_feederMotor;
  VictorMotorController* m_indexerMotor;
  frc::DigitalInput m_feederBeamBreak;
  frc::DigitalInput m_indexerBeamBreak;


  //TODO variable that tells you to shoot

};
