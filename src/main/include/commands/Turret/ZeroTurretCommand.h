// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/TurretSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/InstantCommand.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class ZeroTurretCommand
    : public frc2::CommandHelper<frc2::CommandBase, ZeroTurretCommand> {
      private: 
      TurretSubsystem* m_pTurret;
 public:
  ZeroTurretCommand(TurretSubsystem* turret){
    m_pTurret = turret;
    AddRequirements(m_pTurret);
    
  }

  void Initialize() override{
      m_pTurret->zeroTurret();
  }

};
