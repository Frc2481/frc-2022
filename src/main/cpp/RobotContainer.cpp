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
  
  frc::SmartDashboard::PutBoolean("Feeder Beam Break", false);
  frc::SmartDashboard::PutBoolean("Indexer Beam Break", false);

  m_driveSubsystem.SetDefaultCommand(DriveWithJoystickCommand(&m_driveSubsystem, &m_driverController)); 
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
  // Configure your button bindings here
  //TODO Add fireshooter button

  // ClimberSubsystem Commands

  // DriveSubsystem Commands
  frc::SmartDashboard::PutData("Zero Steer Encoders", new InstantDisabledCommand([this](){
    m_driveSubsystem.ResetEncoders();
  }));

  frc::SmartDashboard::PutData("Reset Odometry", new InstantDisabledCommand([this](){
    m_driveSubsystem.ResetOdometry(frc::Pose2d());
  }));
  m_startDriver.WhenPressed(new frc2::InstantCommand([this]{
                              m_driveSubsystem.ResetOdometry(frc::Pose2d(
                                                                m_driveSubsystem.GetPose().Translation().X(), 
                                                                m_driveSubsystem.GetPose().Translation().Y(),
                                                                frc::Rotation2d(units::degree_t(0))));
                              },{&m_driveSubsystem}));

  // FeederSubsystem Commands
  m_feederSubsystem.SetDefaultCommand(FeederDefaultCommand(&m_feederSubsystem, &m_intakeSubsystem)); 
  m_bButtonDriver.WhenPressed(ExtendIntakeCommand(&m_intakeSubsystem));
  m_bButtonDriver.WhenReleased(RetractIntakeCommand(&m_intakeSubsystem));
  
  // IntakeSubsystem Commands
  m_bButtonDriver.WhenPressed(ExtendIntakeCommand(&m_intakeSubsystem));
  m_bButtonDriver.WhenReleased(RetractIntakeCommand(&m_intakeSubsystem));
  
  // ShooterSubsystem Commands
  
  // TurretSubsystem Commands
  frc2::InstantCommand m_zeroTurret{[this] {m_turretSubsystem.zeroTurret(); }, {&m_turretSubsystem}};
  m_aButtonDriver.WhenPressed(m_zeroTurret);

  
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return nullptr;
}
