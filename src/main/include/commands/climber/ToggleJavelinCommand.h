// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/InstantCommand.h>
#include "subsystems/ClimberSubsystem.h"

class ToggleJavelinCommand
    : public frc2::CommandHelper<frc2::InstantCommand,
                                 ToggleJavelinCommand> {
private:
  ClimberSubsystem* m_pClimber;
  bool isJavelinExtended;
 public:
  ToggleJavelinCommand(ClimberSubsystem* climber){
    m_pClimber = climber;
    isJavelinExtended = false;
  }
  void Initialize() override{
    if(false == isJavelinExtended){
      m_pClimber->fireJavelin();
      isJavelinExtended = true;
    }else{
      m_pClimber->retractJavelin();
      isJavelinExtended = false;
    }
  }
};
