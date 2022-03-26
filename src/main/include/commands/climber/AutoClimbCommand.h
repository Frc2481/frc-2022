// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/InstantCommand.h>
#include "subsystems/ClimberSubsystem.h"
#include "subsystems/DriveSubsystem.h"
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>

#include "commands/climber/ExtendTrussClimberWheelsCommand.h"
#include "commands/drive/DriveOpenLoopCommand.h"
#include "commands/Drive/WaitForRoll.h"
#include "RobotParameters.h"

class AutoClimbCommand
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 AutoClimbCommand> {
  private: 
  DriveSubsystem* m_pDrive;
  ClimberSubsystem* m_pClimber;                                 
 public:
  AutoClimbCommand(ClimberSubsystem* climber, DriveSubsystem* drive){
    m_pClimber = climber;
    m_pDrive = drive;
 
    AddCommands(
      frc2::SequentialCommandGroup{
        ExtendTrussClimberWheelsCommand(m_pClimber),
        frc2::InstantCommand([this]{m_pClimber->extendFloorWheels();},{m_pClimber}),
        frc2::ParallelRaceGroup{
          frc2::InstantCommand([this]{m_pClimber->setFloorWheelsSpeed(ClimberConstants::kFloorWheelSpeed); 
          m_pClimber->setTrussWheelsSpeed(ClimberConstants::kTrussWheelSpeed);},{m_pClimber}),
          DriveOpenLoopCommand(m_pDrive, DriveConstants::kDriveClimbSpeed, 0_mps, 0_rad_per_s, false),
        },
        WaitForRoll(m_pDrive, ClimberConstants::kJavelinDeployRoll),
        frc2::InstantCommand([this]{m_pClimber->fireJavelin();},{m_pClimber}),
        frc2::ParallelRaceGroup{
          frc2::InstantCommand([this]{m_pClimber->setFloorWheelsSpeed(-ClimberConstants::kFloorWheelSpeed);
          m_pClimber->setTrussWheelsSpeed(-ClimberConstants::kTrussWheelSpeed);},{m_pClimber}),
          DriveOpenLoopCommand(m_pDrive, -DriveConstants::kDriveClimbSpeed, 0_mps, 0_rad_per_s, false),
        },
        frc2::WaitCommand(2_s),
        frc2::ParallelRaceGroup{
          frc2::InstantCommand([this]{m_pClimber->setFloorWheelsSpeed(0.0);
          m_pClimber->setTrussWheelsSpeed(0.0);},{m_pClimber}),
          DriveOpenLoopCommand(m_pDrive, 0_mps, 0_mps, 0_rad_per_s, false),
        },
        frc2::InstantCommand([this]{m_pClimber->retractFloorWheels();},{m_pClimber})
      }
    );
  }

};
