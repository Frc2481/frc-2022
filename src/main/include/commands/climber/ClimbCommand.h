// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/ClimberSubsystem.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/TurretSubsystem.h"
#include "components/Joystick2481.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class ClimbCommand
    : public frc2::CommandHelper<frc2::CommandBase, ClimbCommand> {
 private:
  ClimberSubsystem* m_pClimber;
  DriveSubsystem* m_pDriveTrain;
  TurretSubsystem* m_pTurret;
  Joystick2481* m_pController;

 public:
  ClimbCommand(ClimberSubsystem* climber, DriveSubsystem* drive, TurretSubsystem* turret, Joystick2481* controller){
    m_pClimber = climber;
    m_pDriveTrain = drive;
    m_pTurret = turret;
    m_pController = controller;
    AddRequirements(m_pClimber);
    AddRequirements(m_pDriveTrain);
    AddRequirements(m_pTurret);
  }
  
  void Initialize() override{
    m_pClimber->extendFloorWheels();
    m_pTurret->rotateTurret(0);
  }

  void Execute() override{
    double joystickValue = m_pController->GetRawAxis(XBOX_LEFT_Y_AXIS);
    m_pClimber->setFloorWheelsSpeed(joystickValue*frc::SmartDashboard::GetNumber("Floor Wheel Constant", ClimberConstants::kFloorWheelSpeed));
    m_pClimber->setTrussWheelsSpeed(joystickValue*frc::SmartDashboard::GetNumber("Truss Wheel Constant", ClimberConstants::kTrussWheelSpeed));
    m_pDriveTrain->Drive(units::meters_per_second_t(-m_pController->GetRawAxis(XBOX_LEFT_X_AXIS)),
     -units::meters_per_second_t (joystickValue*frc::SmartDashboard::GetNumber("Drive Train Constant", ClimberConstants::kDriveTrainSpeedConstant)),
      units::radians_per_second_t(m_pController->GetRawAxis(XBOX_RIGHT_X_AXIS)*2), false);   
  }

  void End(bool interrupted) override{
    m_pClimber->setFloorWheelsSpeed(0.2);
    m_pClimber->setTrussWheelsSpeed(0);
    m_pDriveTrain->stop();
    m_pClimber->retractFloorWheels();
  }

  bool IsFinished() override{
    return false;
  }
};
