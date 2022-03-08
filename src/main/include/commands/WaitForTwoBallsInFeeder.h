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
class WaitForTwoBallsInFeederCommand
    : public frc2::CommandHelper<frc2::CommandBase, WaitForTwoBallsInFeederCommand> {
      private: 
      FeederSubsystem* m_pFeeder;
 public:
  WaitForTwoBallsInFeederCommand(FeederSubsystem* feeder){
    m_pFeeder = feeder;
  }

  bool IsFinished() override {
    return m_pFeeder->getFeederBeamBreak() && m_pFeeder->getIndexerBeamBreak();
  }
};
