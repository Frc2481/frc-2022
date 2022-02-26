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

#include "commands/climber/ExtendFloorTrussClimberWheelsCommand.h"
#include "commands/climber/RetractFloorTrussClimberWheelsCommand.h"
#include "commands/climber/FireJavelinCommand.h"
#include "commands/climber/StartClimberWheelsCommand.h"
#include "commands/climber/StartClimberWheelsReverseCommand.h"
#include "commands/climber/StopClimberWheelsCommand.h"
#include "commands/drive/DriveOpenLoopCommand.h"
#include "commands/Drive/WaitForRoll.h"
#include "commands/climber/FireJavelinCommand.h"
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
        ExtendClimberWheelsCommand(m_pClimber),
        frc2::ParallelRaceGroup{
          StartClimberWheelsCommand(m_pClimber),
          DriveOpenLoopCommand(m_pDrive, DriveConstants::kDriveClimbSpeed, 0_mps, 0_rad_per_s, false),
        },
        WaitForRoll(m_pDrive, ClimberConstants::kJavelinDeployRoll),
        FireJavelinCommand(m_pClimber),
        frc2::ParallelRaceGroup{
          StartClimberWheelsReverseCommand(m_pClimber),
          DriveOpenLoopCommand(m_pDrive, -DriveConstants::kDriveClimbSpeed, 0_mps, 0_rad_per_s, false),
        },
        frc2::WaitCommand(2_s),
        frc2::ParallelRaceGroup{
          StopClimberWheelsCommand(m_pClimber),
          DriveOpenLoopCommand(m_pDrive, 0_mps, 0_mps, 0_rad_per_s, false),
        },
        RetractFloorClimberWheelsCommand(m_pClimber)
      }
    );
  }

};
