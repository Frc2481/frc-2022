// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "RobotParameters.h"
#include "commands/ControlMotorWithJoystickCommand.h"
#include <frc2/command/InstantCommand.h>
#include "commands/Drive/DriveWithJoystickCommand.h"
#include "commands/intake/ExtendIntakeCommand.h"
#include "commands/intake/RetractIntakeCommand.h"
#include "commands/FeederDefaultCommand.h"
#include "commands/shooter/AutoAdjustShooterSpeedCommand.h"
#include "commands/shooter/StopShooterCommand.h"
RobotContainer::RobotContainer(): m_driverController(0), m_auxController(1)
 {
  
  // Initialize all of your commands and subsystems here
  // Configure the button bindings



  ConfigureButtonBindings();
  frc::SmartDashboard::PutNumber("test",342);
  frc::SmartDashboard::PutNumber("TurretP", RobotParameters::k_shooterP);
  frc::SmartDashboard::PutNumber("TurretI", RobotParameters::k_shooterI);
  frc::SmartDashboard::PutNumber("TurretD", RobotParameters::k_shooterD);
  frc::SmartDashboard::PutNumber("TurretF", RobotParameters::k_shooterF);
  frc::SmartDashboard::PutNumber("Set Target Angle", 0);
  
  frc::SmartDashboard::PutBoolean("Feeder Beam Break", false);
  frc::SmartDashboard::PutBoolean("Indexer Beam Break", false);
}

class InstantDisabledCommand : public frc2::InstantCommand {
public:

  InstantDisabledCommand(std::function<void()> toRun,
                 std::initializer_list<frc2::Subsystem*> requirements = {}) : frc2::InstantCommand(toRun, requirements) {} 

  virtual bool RunsWhenDisabled() const override {
    return true;
  }
};

void RobotContainer::ConfigureButtonBindings() {
  frc::SmartDashboard::PutData("Zero Steer Encoders", new InstantDisabledCommand([this](){
    m_driveSubsystem.ResetEncoders();
  }));

  frc::SmartDashboard::PutData("Reset Odometry", new InstantDisabledCommand([this](){
    m_driveSubsystem.ResetOdometry(frc::Pose2d());
  }));
  // Configure your button bindings here
  //TODO Add fireshooter button

  // ClimberSubsystem Commands

  // DriveSubsystem Commands

  // FeederSubsystem Commands
  m_feederSubsystem.SetDefaultCommand(FeederDefaultCommand(&m_feederSubsystem, &m_intakeSubsystem)); 
  
  // IntakeSubsystem Commands
  m_bButtonDriver.WhenPressed(ExtendIntakeCommand(&m_intakeSubsystem));
  m_bButtonDriver.WhenReleased(RetractIntakeCommand(&m_intakeSubsystem));
  
  // ShooterSubsystem Commands
  m_aButtonAux.WhenPressed(AutoAdjustShooterSpeedCommand(&m_shooterSubsystem, &m_turretSubsystem));
  m_bButtonAux.WhenPressed(StopShooterCommand(&m_shooterSubsystem));

  // TurretSubsystem Commands
  m_turretSubsystem.SetDefaultCommand(StayOnTargetCommand(&m_turretSubsystem));
  frc2::InstantCommand m_zeroTurret{[this] {m_turretSubsystem.zeroTurret(); }, {&m_turretSubsystem}};
  m_aButtonDriver.WhenPressed(m_zeroTurret);

  
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return nullptr;
}
