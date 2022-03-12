// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/InstantCommand.h>
#include "subsystems/ClimberSubsystem.h"

class SetTrussWheelsCommand
    : public frc2::CommandHelper<frc2::InstantCommand,
                                 SetTrussWheelsCommand> {
    private:
    ClimberSubsystem* m_pClimber;
    double m_speed;
 public:
  SetTrussWheelsCommand(ClimberSubsystem* climber, double speed){
    m_pClimber = climber;
    m_speed = speed;
  }
  void Initialize() override{
    m_pClimber->setTrussWheelsSpeed(m_speed);
  }
};
