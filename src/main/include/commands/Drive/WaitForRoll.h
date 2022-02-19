// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/DriveSubsystem.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class WaitForRoll
    : public frc2::CommandHelper<frc2::CommandBase, WaitForRoll> {

      private: 
       DriveSubsystem* m_pDrive;
       double m_rollTarget;

 public:
  WaitForRoll(DriveSubsystem* pDrive, double rollTarget) {
    m_pDrive = pDrive;
    m_rollTarget = rollTarget;
  }

  bool IsFinished() override {
    return m_pDrive->GetRoll() > m_rollTarget;
  }
};
