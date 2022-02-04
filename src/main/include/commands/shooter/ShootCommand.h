// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/FeederSubsystem.h"

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
 FeederSubsystem* m_feeder;

 public:
  ShootCommand(FeederSubsystem* feeder){
    m_feeder = feeder;
    AddRequirements(m_feeder);
  }

  void Initialize() override{
    m_feeder->shootBall();
  }

  void Execute() override{}

  void End(bool interrupted) override{
    m_feeder->stopShooter();
  }

  bool IsFinished() override{
    return false;
  }
};
