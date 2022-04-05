// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/IntakeSubsystem.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class WaitForBallAtIntakeRollerCommand
    : public frc2::CommandHelper<frc2::CommandBase, WaitForBallAtIntakeRollerCommand> {
      private: 
      IntakeSubsystem* m_pIntake;
 public:
  WaitForBallAtIntakeRollerCommand(IntakeSubsystem* intake){
    m_pIntake = intake;
  }

  bool IsFinished() override {
    return m_pIntake->getIntakeBeamBreak();
  }
};
