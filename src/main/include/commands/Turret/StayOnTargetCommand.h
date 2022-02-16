// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "Subsystems/TurretSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class StayOnTargetCommand
    : public frc2::CommandHelper<frc2::CommandBase, StayOnTargetCommand> {
 private:
  TurretSubsystem* m_pTurret;
 public:

  StayOnTargetCommand(TurretSubsystem* turret)
  {
    m_pTurret = turret;
    AddRequirements(m_pTurret);
  }

  void Initialize() override
  {

  }

  void Execute() override
  {
    frc::SmartDashboard::PutNumber("Distance to Target", m_pTurret->getDistance());
    frc::SmartDashboard::PutNumber("Angle to Target", m_pTurret->getAngleToTarget());
    frc::SmartDashboard::PutNumber("Current Angle", m_pTurret->getTurretAngle());
    
    // m_pTurret->rotateTurret(frc::SmartDashboard::GetNumber("Set Target Angle", 0));//m_pTurret->getAngleToTarget());
  }

  void End(bool interrupted) override
  {

  }

  bool IsFinished() override
  {
    return false;
  }
};
