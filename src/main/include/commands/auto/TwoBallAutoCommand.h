// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/FeederSubsystem.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/TurretSubsystem.h"
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/ScheduleCommand.h>
#include "commands/intake/WaitForBallAtIntakeRollerCommand.h"
#include "commands/shooter/ShootCommand.h"

#include "commands/drive/DriveOpenLoopCommand.h"
#include "commands/shooter/AutoAdjustShooterSpeedCommand.h"
#include "commands/intake/ExtendIntakeCommand.h"
#include "commands/Drive/WaitForRoll.h"
#include "RobotParameters.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class TwoBallAutoCommand
    : public frc2::CommandHelper<frc2::SequentialCommandGroup, 
    TwoBallAutoCommand> {
      private: 
      DriveSubsystem* m_pDrive;
      FeederSubsystem* m_pFeeder;
      IntakeSubsystem* m_pIntake;
      ShooterSubsystem* m_pShooter;
      TurretSubsystem* m_pTurret;
 public:
  TwoBallAutoCommand(DriveSubsystem* drive, FeederSubsystem* feeder, IntakeSubsystem* intake, ShooterSubsystem* shooter, TurretSubsystem* turret){
    m_pDrive = drive;
    m_pFeeder = feeder;
    m_pIntake = intake;
    m_pShooter = shooter;
    m_pTurret = turret;

    AddCommands(
      frc2::ParallelCommandGroup{
        AutoAdjustShooterSpeedCommand(m_pShooter, m_pTurret),

        //FeederDefaultCommand(m_pFeeder),
        ExtendIntakeCommand(m_pIntake),
        DriveOpenLoopCommand(m_pDrive, DriveConstants::kAutoDriveSpeed, 0_mps, 0_rad_per_s, false), 
          frc2::ParallelCommandGroup{
            WaitForBallAtIntakeRollerCommand(m_pIntake).WithTimeout(3_s),
            DriveOpenLoopCommand(m_pDrive, 0_mps, 0_mps, 0_rad_per_s, false),
            ShootCommand(m_pFeeder).WithTimeout(3_s)
          }
        }
      );
    }
  };