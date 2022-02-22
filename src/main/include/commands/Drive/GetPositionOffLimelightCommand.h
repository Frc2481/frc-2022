// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/InstantCommand.h>
#include "Subsystems/TurretSubsystem.h"
#include "Subsystems/DriveSubsystem.h"

class GetPositionOffLimelightCommand
    : public frc2::CommandHelper<frc2::InstantCommand,
                                 GetPositionOffLimelightCommand> {
      private:
      double m_theta;
      TurretSubsystem* m_pTurret;
      DriveSubsystem* m_pDrive;
      double m_robotX;
      double m_robotY;

 public:
  GetPositionOffLimelightCommand(DriveSubsystem* driveTrain, TurretSubsystem* turret){
    m_pDrive = driveTrain;
    m_pTurret = turret;
    AddRequirements(m_pDrive);
    AddRequirements (m_pTurret);
  }

  void Initialize() override{
    double distance = m_pTurret->getDistance();
    double m_theta = (90 - (m_pTurret->getAngleToTarget() + m_pTurret->getTurretAngle() + m_pDrive->GetHeading()))*(wpi::numbers::pi/180);
    double m_robotX = distance*cos(m_theta);
    double m_robotY = distance*sin(m_theta);
  }
};
