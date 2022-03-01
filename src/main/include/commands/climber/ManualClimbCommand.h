// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/ClimberSubsystem.h"
#include "subsystems/DriveSubsystem.h"
#include "components/Joystick2481.h"
#include <frc/smartdashboard/Smartdashboard.h>
#include "components/XboxController2481.h"
#include <units/velocity.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class ManualClimbCommand
    : public frc2::CommandHelper<frc2::CommandBase, ManualClimbCommand> {
      private: 
        ClimberSubsystem* m_pClimber;
        DriveSubsystem* m_pDriveTrain;
        Joystick2481* m_pController;
 public:
 

  ManualClimbCommand(ClimberSubsystem* climber, DriveSubsystem* driveTrain, Joystick2481* controller){
    m_pClimber = climber;
    m_pDriveTrain = driveTrain;
    m_pController = controller;
    AddRequirements(m_pClimber);
    AddRequirements(m_pDriveTrain);
  }

  void Initialize() override{
    m_pClimber->extendFloorTrussWheels();
  }

  void Execute() override{
    double joystickValue = m_pController->GetRawAxis(XBOX_LEFT_Y_AXIS);
    m_pClimber->setFloorWheelsSpeed(joystickValue*frc::SmartDashboard::GetNumber("Floor Wheel Constant", ClimberConstants::kFloorWheelSpeed));
    m_pClimber->setTrussWheelsSpeed(joystickValue*frc::SmartDashboard::GetNumber("Truss Wheel Constant", ClimberConstants::kTrussWheelSpeed));
     m_pDriveTrain->Drive(0_mps, -units::meters_per_second_t (joystickValue*frc::SmartDashboard::GetNumber("Drive Train Constant", ClimberConstants::kDriveTrainSpeedConstant)), 0_rpm, false);                  
  }

  void End(bool interrupted) override{
    m_pClimber->setFloorWheelsSpeed(0);
    m_pClimber->setTrussWheelsSpeed(0);
    m_pDriveTrain->stop();
  }

  bool IsFinished() override{
    return false;
  }
};
