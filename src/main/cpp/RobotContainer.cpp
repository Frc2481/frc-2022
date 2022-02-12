// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "RobotParameters.h"
#include "commands/ControlMotorWithJoystickCommand.h"
#include <frc2/command/InstantCommand.h>
RobotContainer::RobotContainer(): m_driverController(0) {
  
  // Initialize all of your commands and subsystems here
    // m_turret.SetDefaultCommand(StayOnTargetCommand(&m_turret));
  // Configure the button bindings



  ConfigureButtonBindings();
  frc::SmartDashboard::PutNumber("test",342);
  frc::SmartDashboard::PutNumber("TurretP", RobotParameters::k_shooterP);
  frc::SmartDashboard::PutNumber("TurretI", RobotParameters::k_shooterI);
  frc::SmartDashboard::PutNumber("TurretD", RobotParameters::k_shooterD);
  frc::SmartDashboard::PutNumber("TurretF", RobotParameters::k_shooterF);
  frc::SmartDashboard::PutNumber("Set Target Angle", 0);

}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here
  //TODO Add fireshooter button
  frc2::InstantCommand m_zeroTurret{[this] {m_turretSubsystem.zeroTurret(); }, {&m_turretSubsystem}};
  aButton.WhenPressed(m_zeroTurret);
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return nullptr;
}
