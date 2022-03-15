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
#include "commands/intake/WaitForBallAtIntakeRollerCommand.h"
#include "commands/shooter/ShootCommand.h"
#include "commands/WaitForTwoBallsInFeeder.h"
#include "commands/WaitForBallInFeeder.h"
#include "commands/FeederDefaultCommand.h"
#include <frc2/command/InstantCommand.h>
#include "commands/Turret/StayOnTargetCommand.h"

#include "commands/drive/DriveOpenLoopCommand.h"
#include "commands/shooter/AutoAdjustShooterSpeedCommand.h"
#include "commands/shooter/StartShooterCommand.h"
#include "commands/intake/ExtendIntakeCommand.h"
#include "commands/intake/RetractIntakeCommand.h"
#include "commands/Drive/WaitForRoll.h"
#include "RobotParameters.h"

class FourBallAutoCommand
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 FourBallAutoCommand> {
  private: 
  DriveSubsystem* m_pDrive;
  FeederSubsystem* m_pFeeder;
  IntakeSubsystem* m_pIntake;
  ShooterSubsystem* m_pShooter;
  TurretSubsystem* m_pTurret;                          
 public:
  FourBallAutoCommand(DriveSubsystem* drive, FeederSubsystem* feeder, IntakeSubsystem* intake, ShooterSubsystem* shooter, TurretSubsystem* turret){
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

            //acquire ball 2
            ExtendIntakeCommand(m_pIntake),
            DriveOpenLoopCommand(m_pDrive, 0_mps, DriveConstants::kAutoDriveSpeed, 0_rad_per_s, false), //TODO ajust speeds off of robot starting angle sence it was once in robot frame
            frc2::WaitCommand(1_s), //give intake roller time to start before checking for ball
            WaitForBallAtIntakeRollerCommand(m_pIntake).WithTimeout(1_s),
            DriveOpenLoopCommand(m_pDrive, 0_mps, 0_mps, 0_rad_per_s, false), //TODO ajust speeds off of robot starting angle sence it was once in robot frame
            WaitForTwoBallsInFeederCommand(m_pFeeder),
            RetractIntakeCommand(m_pIntake),
            //shoot balls 1 and 2
            ShootCommand(m_pFeeder, m_pShooter).WithTimeout(1.5_s), 

            //acquire ball 3
            ExtendIntakeCommand(m_pIntake),
            DriveOpenLoopCommand(m_pDrive, 0_mps, DriveConstants::kAutoDriveSpeed, 0_rad_per_s, false), //TODO ajust speeds off of robot starting angle sence it was once in robot frame
            frc2::WaitCommand(1_s), //give intake roller time to start before checking for ball
            WaitForBallAtIntakeRollerCommand(m_pIntake).WithTimeout(1.5_s),
            DriveOpenLoopCommand(m_pDrive, 0_mps, 0_mps, 0_rad_per_s, false), //TODO ajust speeds off of robot starting angle sence it was once in robot frame
            WaitForTwoBallsInFeederCommand(m_pFeeder).WithTimeout(3_s),
            WaitForBallInFeederCommand(m_pFeeder),
            DriveOpenLoopCommand(m_pDrive, 0_mps, -DriveConstants::kAutoDriveSpeed, 0_rad_per_s, false), //TODO ajust speeds off of robot starting angle sence it was once in robot frame
            frc2::WaitCommand(2_s),
            DriveOpenLoopCommand(m_pDrive, 0_mps, 0_mps, 0_rad_per_s, false), //TODO ajust speeds off of robot starting angle sence it was once in robot frame
            
            RetractIntakeCommand(m_pIntake),
            //shoot balls 3 and 4
            ShootCommand(m_pFeeder, m_pShooter).WithTimeout(1.5_s), 
             
          }
        }
    );
        
  }

};
