// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/FeederSubsystem.h"
#include "Constants.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class ExtendIntakeCommand
    : public frc2::CommandHelper<frc2::CommandBase, ExtendIntakeCommand> {
 private:
 IntakeSubsystem* m_intake;
 FeederSubsystem* m_feeder;

 public:
  ExtendIntakeCommand(IntakeSubsystem* intake, FeederSubsystem* feeder){
    m_intake = intake;
    m_feeder = feeder;
    AddRequirements(m_intake);
    AddRequirements(m_feeder);
  }

  void Initialize() override{
    m_intake->extendIntake();
    m_intake->setRollerSpeed(IntakeConstants::kDefaultIntakeRollerSpeed);
  }

  void Execute() override{}

  void End(bool interrupted) override{
    m_feeder->setFeederSpeed(0);
  }

  bool IsFinished() override{
    return m_feeder->isShooterPrimed();
  }
};
