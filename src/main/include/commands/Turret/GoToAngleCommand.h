// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/InstantCommand.h>
#include "subsystems/TurretSubsystem.h"

class GoToAngleCommand
    : public frc2::CommandHelper<frc2::InstantCommand,
                                 GoToAngleCommand> {
      private: 
      TurretSubsystem* m_pTurret;
      double m_angle;
 public:
  GoToAngleCommand(TurretSubsystem* turret, double angle){
    m_pTurret = turret;
    m_angle = angle;
    AddRequirements(m_pTurret);
  }

  void Initialize() override{
    m_pTurret->rotateTurret(m_angle);
  }
};
