// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include "subsystems/ClimberSubsystem.h"
#include "subsystems/TurretSubsystem.h"
#include "RobotParameters.h"
#include <frc2/command/InstantCommand.h>
#include "commands/climber/ExtendTrussCommand.h"
#include "commands/climber/RetractTrussClimberWheelsCommand.h"
#include "commands/climber/SetTrussWheelsCommand.h"
#include "commands/Turret/GoToAngleCommand.h"

class ExtendTrussClimberWheelsCommand
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 ExtendTrussClimberWheelsCommand> {
  private:
  ClimberSubsystem* m_pClimber;  
  TurretSubsystem* m_pTurret;
 public:
  ExtendTrussClimberWheelsCommand(ClimberSubsystem* climber, TurretSubsystem* turret){
    m_pClimber = climber;
    m_pTurret = turret;
    AddCommands(
      ExtendTrussCommand(m_pClimber),
      GoToAngleCommand(m_pTurret, 0),
      SetTrussWheelsCommand(m_pClimber, -ClimberConstants::kTrussWheelSpeed),
      frc2::WaitCommand(1_s),
      SetTrussWheelsCommand(m_pClimber, 0)
    );
  }
};