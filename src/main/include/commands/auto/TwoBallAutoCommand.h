// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/ScheduleCommand.h>
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
#include <frc2/command/InstantCommand.h>
#include "commands/intake/WaitForBallAtIntakeRollerCommand.h"
#include "commands/shooter/ShootCommand.h"
#include "commands/WaitForTwoBallsInFeeder.h"
#include "commands/FeederDefaultCommand.h"
#include "commands/Turret/StayOnTargetCommand.h"

#include "commands/drive/DriveOpenLoopCommand.h"
#include "commands/shooter/AutoAdjustShooterSpeedCommand.h"
#include "commands/shooter/StartShooterCommand.h"
#include "commands/intake/ExtendIntakeCommand.h"
#include "commands/intake/RetractIntakeCommand.h"
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
        StayOnTargetCommand(m_pTurret),

        frc2::ScheduleCommand(new FeederDefaultCommand(m_pFeeder, m_pIntake)),

        //FeederDefaultCommand(m_pFeeder),
        
          frc2::SequentialCommandGroup{
            frc2::InstantCommand([this]{m_pTurret->zeroTurret();},{}),
            frc2::InstantCommand([this]{m_pDrive->setGyroLock(true);},{m_pDrive}),
              frc2::InstantCommand([this]{
                              m_pDrive->ResetOdometry(frc::Pose2d(
                                                                m_pDrive->GetPose().Translation().X(), 
                                                                m_pDrive->GetPose().Translation().Y(),
                                                                frc::Rotation2d(units::degree_t(0))));
                              },{}),
            ExtendIntakeCommand(m_pIntake),
            frc2::WaitCommand(6.5_s),
            DriveOpenLoopCommand(m_pDrive, 0_mps, DriveConstants::kAutoDriveSpeed, 0_rad_per_s, false),
            frc2::WaitCommand(1_s), //give intake roller time to start before checking for ball
            WaitForBallAtIntakeRollerCommand(m_pIntake).WithTimeout(1_s),
            DriveOpenLoopCommand(m_pDrive, 0_mps, 0_mps, 0_rad_per_s, false),
            WaitForTwoBallsInFeederCommand(m_pFeeder).WithTimeout(3_s),
            RetractIntakeCommand(m_pIntake),
            ShootCommand(m_pFeeder, m_pShooter).WithTimeout(3_s)
          }
        }
      );
    }
  };
