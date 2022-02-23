// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/TurretSubsystem.h"
#include "components/Joystick2481.h"
#include "components/XboxController2481.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class MoveTurretWithJoystickCommand
    : public frc2::CommandHelper<frc2::CommandBase, MoveTurretWithJoystickCommand> {
      private:
      TurretSubsystem* m_pTurret;
      Joystick2481* m_pAuxController;
 public:
  MoveTurretWithJoystickCommand(TurretSubsystem* turret, Joystick2481* auxController){
      m_pTurret = turret;
      m_pAuxController = auxController;
    AddRequirements(m_pTurret);
  }

  void Initialize() override{
    
  }

  void Execute() override{
    double targetAngle = atan2(m_pAuxController->GetRawAxis(XBOX_LEFT_Y_AXIS), m_pAuxController->GetRawAxis(XBOX_LEFT_X_AXIS));
    double magnitudeJoystick = hypot(m_pAuxController->GetRawAxis(XBOX_LEFT_Y_AXIS), m_pAuxController->GetRawAxis(XBOX_LEFT_X_AXIS));

    if (magnitudeJoystick > 1) {//joystick needs to be pressed almost to edge
        m_pTurret->rotateTurret(targetAngle*(180/wpi::numbers::pi));
    }
  }

  void End(bool interrupted) override{

  }

  bool IsFinished() override{
    return false;
  }
};
