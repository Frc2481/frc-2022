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
  double angle;
  bool forward = false;
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
    if(m_pTurret->isTargetVisible()){
      m_pTurret->rotateTurret(m_pTurret->getTurretCalibratedAngle() - m_pTurret->getAngleToTarget());
      angle = m_pTurret->getTurretCalibratedAngle();
    }else{
      angle = forward ? angle + 1.5 : angle - 1.5;
      m_pTurret->rotateTurret(angle);
      if(angle >= 90){
        forward = false;
      }else if(angle <= -90){
        forward = true;
      }
    }
    frc::SmartDashboard::PutNumber("Turret Setpoint", angle);
  }

  void End(bool interrupted) override
  {

  }

  bool IsFinished() override
  {
    return false;
  }
};
