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
  bool m_prevVisible;
 public:

  StayOnTargetCommand(TurretSubsystem* turret)
  {
    m_pTurret = turret;
    m_prevVisible = true;
    AddRequirements(m_pTurret);
  }

  void Initialize() override
  {
    // m_prevVisible = true;
  }

  void Execute() override
  {
    bool visible = m_pTurret->isTargetVisible();
    if(visible){
      m_pTurret->rotateTurret(m_pTurret->getTurretCalibratedAngle() - m_pTurret->getAngleToTarget());
      angle = m_pTurret->getTurretCalibratedAngle();
    }else{
      angle = m_pTurret->getTurretCalibratedAngle();
      // angle = forward ? angle + 4.5 : angle - 4.5;
      // m_pTurret->rotateTurret(angle);
      if(angle >= RobotParameters::k_maxTurretSearchDegrees - 5){
        // forward = false;
        m_pTurret->rotateTurret(RobotParameters::k_minTurretSearchDegrees);
      }else if(angle <= RobotParameters::k_minTurretSearchDegrees + 5){
        // forward = true;
        m_pTurret->rotateTurret(RobotParameters::k_maxTurretSearchDegrees);
      }else if (m_prevVisible != visible){
        m_pTurret->rotateTurret(RobotParameters::k_minTurretSearchDegrees);
      }
    }
    m_prevVisible = visible;

    frc::SmartDashboard::PutNumber("Turret Setpoint 2", angle);
  }

  void End(bool interrupted) override
  {

  }

  bool IsFinished() override
  {
    return false;
  }

  virtual bool RunsWhenDisabled() const override {
    return true;
  }
};
