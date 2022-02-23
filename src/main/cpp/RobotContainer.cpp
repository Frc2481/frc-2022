// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "RobotParameters.h"
#include "commands/ControlMotorWithJoystickCommand.h"
#include <frc2/command/InstantCommand.h>
// #include "commands/Drive/DriveWithJoystickCommand.h"
#include "commands/intake/ExtendIntakeCommand.h"
#include "commands/intake/RetractIntakeCommand.h"
#include "commands/FeederDefaultCommand.h"
#include "commands/shooter/AutoAdjustShooterSpeedCommand.h"
#include "commands/shooter/StopShooterCommand.h"
// #include "commands/turret/StayOnTargetCommand.h"
#include "commands/climber/AutoClimbCommand.h"
#include "commands/shooter/ShootCommand.h"
#include "commands/ManualStartIntakeFeederCommand.h"
#include "commands/ManualStopIntakeFeederCommand.h"
#include "commands/shooter/StartShooterCommand.h"
#include "commands/turret/MoveTurretWithJoystickCommand.h"
#include "components/Joystick2481.h"

RobotContainer::RobotContainer(): m_driverController(0), m_auxController(1)
 {
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

  frc::SmartDashboard::PutNumber("Bottom Motor Speed", 0);
  frc::SmartDashboard::PutNumber("Top Motor Speed", 0);

  // m_driveSubsystem.SetDefaultCommand(DriveWithJoystickCommand(&m_driveSubsystem, &m_driverController)); 
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
    // m_rBumperAux.WhenPressed(new frc2::InstantCommand([this](){
    //   if(m_auxController.GetRawButton(XBOX_LEFT_BUMPER)){
    //     new AutoClimbCommand(&m_climberSubsystem, &m_driveSubsystem);
    //     frc::SmartDashboard::PutBoolean("Button Works", true);
    //     // new ExtendIntakeCommand(&m_intakeSubsystem);
    // }
    // else{
    //   frc::SmartDashboard::PutBoolean("Button Works", false);
    // }
    // },{&m_climberSubsystem, &m_driveSubsystem}));

  m_xButtonAux.WhenPressed(ManualStartIntakeFeederCommand(&m_feederSubsystem, &m_intakeSubsystem));
  m_yButtonAux.WhenPressed(ManualStopIntakeFeederCommand(&m_feederSubsystem, &m_intakeSubsystem));
  // DriveSubsystem Commands
  // frc::SmartDashboard::PutData("Zero Steer Encoders", new InstantDisabledCommand([this](){
  //   m_driveSubsystem.ResetEncoders();
  // }));

  // frc::SmartDashboard::PutData("Reset Odometry", new InstantDisabledCommand([this](){
  //   m_driveSubsystem.ResetOdometry(frc::Pose2d());
  // }));
  // m_startDriver.WhenPressed(new frc2::InstantCommand([this]{
  //                             m_driveSubsystem.ResetOdometry(frc::Pose2d(
  //                                                               m_driveSubsystem.GetPose().Translation().X(), 
  //                                                               m_driveSubsystem.GetPose().Translation().Y(),
  //                                                               frc::Rotation2d(units::degree_t(0))));
  //                             },{&m_driveSubsystem}));

  // FeederSubsystem Commands
  m_feederSubsystem.SetDefaultCommand(FeederDefaultCommand(&m_feederSubsystem, &m_intakeSubsystem));
  m_turretSubsystem.SetDefaultCommand(MoveTurretWithJoystickCommand(&m_turretSubsystem, &m_auxController));
  
  // IntakeSubsystem Commands
  m_rTriggerDriver.WhenPressed(ExtendIntakeCommand(&m_intakeSubsystem));
  m_rTriggerDriver.WhenReleased(RetractIntakeCommand(&m_intakeSubsystem));
  
  // ShooterSubsystem Commands
  m_aButtonLeftBumpAux.WhenPressed(StartShooterCommand(&m_shooterSubsystem));
  m_aButtonAux.WhenPressed(AutoAdjustShooterSpeedCommand(&m_shooterSubsystem, &m_turretSubsystem));
  m_bButtonAux.WhenPressed(StopShooterCommand(&m_shooterSubsystem));
  m_rTriggerAux.WhileHeld(ShootCommand(&m_feederSubsystem));

  //ClimberSubsystem commands
  m_startBackAux.WhenPressed(AutoClimbCommand(&m_climberSubsystem, &m_driveSubsystem));

  //DriveSubsystem commands
  m_lBumperDriver.WhenPressed(new frc2::InstantCommand([this]{m_driveSubsystem.toggleFieldCentricForJoystick();},{&m_driveSubsystem}));

  // TurretSubsystem Commands
  // m_turretSubsystem.SetDefaultCommand(StayOnTargetCommand(&m_turretSubsystem));
  // frc2::InstantCommand m_zeroTurret{[this] {m_turretSubsystem.zeroTurret(); }, {&m_turretSubsystem}};
  // m_aButtonDriver.WhenPressed(m_zeroTurret);

  
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return nullptr;
}
