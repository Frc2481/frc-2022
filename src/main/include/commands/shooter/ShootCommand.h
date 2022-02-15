// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/FeederSubsystem.h"
#include "Constants.h"

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

 public:
  ShootCommand(FeederSubsystem* feeder){
    m_pFeeder = feeder;
    AddRequirements(m_pFeeder);
  }

  void Initialize() override{
    m_pFeeder->setFeederSpeed(FeederConstants::kShootingSpeed);
    m_pFeeder->setIndexerSpeed(FeederConstants::kShootingIndexerSpeed);
  }

  void Execute() override{}

  void End(bool interrupted) override{
    m_pFeeder->setFeederSpeed(0);
    m_pFeeder->setIndexerSpeed(0);
  }

  bool IsFinished() override{
    return false;
  }
};
