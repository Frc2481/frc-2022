// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/ShooterSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class StartShooterCommand
    : public frc2::CommandHelper<frc2::CommandBase, StartShooterCommand> {
      private:
        ShooterSubsystem* m_pShooter;
 public:
  StartShooterCommand(ShooterSubsystem* shooter){
        m_pShooter = shooter;
      AddRequirements(m_pShooter);
  }

  void Execute() override{
    m_pShooter->topMotorSetSpeed(frc::SmartDashboard::GetNumber("Top Motor Speed", ShooterConstants::kTopShooterSpeed));
    m_pShooter->bottomMotorSetSpeed(frc::SmartDashboard::GetNumber("Bottom Motor Speed", ShooterConstants::kBottomShooterSpeed));
    frc::SmartDashboard::PutBoolean("We are shooting", true);
  }
  bool IsFinished(){
    return false;
  }

  
};
