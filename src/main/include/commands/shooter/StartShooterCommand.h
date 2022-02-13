// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/TurretSubsystem.h"

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
 TurretSubsystem* m_pTurret;

 public:
  StartShooterCommand(ShooterSubsystem* shooter, TurretSubsystem* turret){
    m_pShooter = shooter;
    m_pTurret = turret;
    AddRequirements(m_pShooter);
    AddRequirements(m_pTurret);
  }

  void Initialize() override{}

  void Execute() override{
        m_pShooter->startShooter(m_pTurret->getDistance());
  }

  void End(bool interrupted) override{}

  bool IsFinished() override{
    return false;
  }
};
