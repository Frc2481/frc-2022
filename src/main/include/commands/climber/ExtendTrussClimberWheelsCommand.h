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

class ExtendTrussClimberWheelsCommand
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 ExtendTrussClimberWheelsCommand> {
  private:
  ClimberSubsystem* m_pClimber;
 public:
  ExtendTrussClimberWheelsCommand(ClimberSubsystem* climber){
    m_pClimber = climber;
    AddCommands(
      frc2::InstantCommand([climber]{climber->extendTrussWheels();}, {climber}),
      frc2::InstantCommand([climber]{climber->setTrussWheelsSpeed(ClimberConstants::kTrussWheelSpeed);},{climber}),
      frc2::WaitCommand(1_s),
      frc2::InstantCommand([climber]{climber->setTrussWheelsSpeed(0);},{climber})
    );
  }
};