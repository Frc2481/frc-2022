// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/InstantCommand.h>
#include "subsystems/ClimberSubsystem.h"
#include "RobotParameters.h"

class StartClimberWheelsCommand
    : public frc2::CommandHelper<frc2::InstantCommand,
                                 StartClimberWheelsCommand> {
 private:
 ClimberSubsystem* m_pClimber;                                  
 public:
  StartClimberWheelsCommand(ClimberSubsystem* climber){
    m_pClimber = climber;
  }

  void Initialize() override{
    m_pClimber->setLeftWheelsSpeed(ClimberConstants::kLeftWheelSpeed);
    m_pClimber->setTrussWheelsSpeed(ClimberConstants::kRightWheelSpeed);
  }
};
