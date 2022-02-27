// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/InstantCommand.h>
#include "subsystems/ClimberSubsystem.h"

class RetractJavelinCommand
    : public frc2::CommandHelper<frc2::InstantCommand,
                                 RetractJavelinCommand> {
     private:                           
        ClimberSubsystem* m_pClimber;                      
 public:
  RetractJavelinCommand(ClimberSubsystem* climber){
    m_pClimber = climber;
    AddRequirements(m_pClimber);
  }

  void Initialize() override{
    m_pClimber->retractJavelin();
  }
};
