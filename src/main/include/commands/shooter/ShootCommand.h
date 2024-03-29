// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/FeederSubsystem.h"
#include "RobotParameters.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class ShootCommand
    : public frc2::CommandHelper<frc2::CommandBase, ShootCommand> {
 private:
 FeederSubsystem* m_pFeeder;
 ShooterSubsystem* m_pShooter;

 public:
  ShootCommand(FeederSubsystem* feeder, ShooterSubsystem* shooter){ //, ShooterSubsystem* shooter
    m_pFeeder = feeder;
    m_pShooter = shooter;
    AddRequirements(m_pFeeder);
    // AddRequirements(m_pShooter);
  }

  void Initialize() override{
    m_pFeeder->setFeederSpeed(FeederConstants::kShootingSpeed);
    m_pFeeder->setIndexerSpeed(FeederConstants::kShootingIndexerSpeed);
    m_pShooter->disableAutoSpeed();
    // m_pShooter->bottomMotorSetSpeed(ShooterConstants::kBottomShooterSpeed);
    // m_pShooter->topMotorSetSpeed(ShooterConstants::kTopShooterSpeed);
  }

  void Execute() override{}

  void End(bool interrupted) override{
    m_pFeeder->setFeederSpeed(0);
    m_pFeeder->setIndexerSpeed(0);
    m_pShooter->enableAutoSpeed();
  }

  bool IsFinished() override{
    return false;
  }
};
