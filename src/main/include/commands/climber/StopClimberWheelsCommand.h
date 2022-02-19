// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/InstantCommand.h>
#include "subsystems/ClimberSubsystem.h"

class StopClimberWheelsCommand
    : public frc2::CommandHelper<frc2::InstantCommand,
                                 StopClimberWheelsCommand> {
 private:
 ClimberSubsystem* m_pClimber;                                  
 public:
  StopClimberWheelsCommand(ClimberSubsystem* climber){
    m_pClimber = climber;
  }
  void Initialize() override{
    m_pClimber->setFloorWheelsSpeed(0.0);
    m_pClimber->setTrussWheelsSpeed(0.0);
  }
};
