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
#include "commands/turret/StayOnTargetCommand.h"
#include "commands/climber/AutoClimbCommand.h"
#include "commands/shooter/ShootCommand.h"
#include "commands/ManualStartIntakeFeederCommand.h"
#include "commands/ManualStopIntakeFeederCommand.h"
#include "commands/shooter/StartShooterCommand.h"
#include "commands/climber/RetractFloorTrussClimberWheelsCommand.h"
#include "commands/climber/ToggleJavelinCommand.h"
#include "commands/turret/MoveTurretWithJoystickCommand.h"
#include "commands/BarfCommand.h"
#include "commands/climber/ManualClimbCommand.h"
#include "commands/climber/RetractJavelinCommand.h"
#include "components/Joystick2481.h"
#include "commands/auto/TwoBallAutoCommand.h"
#include "commands/Turret/ZeroTurretCommand.h"
#include "cameraserver/CameraServer.h"
#include "commands/auto/TwoBallAutoCommand.h"
#include "commands/auto/FourBallAutoCommand.h"

RobotContainer::RobotContainer(): m_driverController(0), m_auxController(1)
 {
  // Initialize all of your commands and subsystems here
    // m_turretSubsystem.SetDefaultCommand(StayOnTargetCommand(&m_turretSubsystem));
  // Configure the button bindings

  // m_pcm.set
  frc::CameraServer::StartAutomaticCapture();
  ConfigureButtonBindings();
  // frc::SmartDashboard::PutNumber("test",342);
  // frc::SmartDashboard::PutNumber("TurretP", RobotParameters::k_shooterP);
  // frc::SmartDashboard::PutNumber("TurretI", RobotParameters::k_shooterI);
  // frc::SmartDashboard::PutNumber("TurretD", RobotParameters::k_shooterD);
  // frc::SmartDashboard::PutNumber("TurretF", RobotParameters::k_shooterF);
  // frc::SmartDashboard::PutNumber("Set Target Angle", 0);
  
  frc::SmartDashboard::PutBoolean("Feeder Beam Break", false);
  frc::SmartDashboard::PutBoolean("Indexer Beam Break", false);

  // frc::SmartDashboard::PutNumber("Bottom Motor Speed", 0);
  // frc::SmartDashboard::PutNumber("Top Motor Speed", 0);

  frc::SmartDashboard::PutNumber("Drive Train Constant", ClimberConstants::kDriveTrainSpeedConstant);
  frc::SmartDashboard::PutNumber("Truss Wheel Constant", ClimberConstants::kTrussWheelSpeed);
  frc::SmartDashboard::PutNumber("Floor Wheel Constant", ClimberConstants::kFloorWheelSpeed);

  frc::SmartDashboard::PutNumber("Top Motor Speed", ShooterConstants::kTopShooterSpeed);
  frc::SmartDashboard::PutNumber("Bottom Motor Speed", ShooterConstants::kBottomShooterSpeed);

  frc::SmartDashboard::PutData("Two Ball Auto", new TwoBallAutoCommand(&m_driveSubsystem, &m_feederSubsystem, &m_intakeSubsystem, &m_shooterSubsystem,  &m_turretSubsystem));
  frc::SmartDashboard::PutData("Four Ball Auto", new FourBallAutoCommand(&m_driveSubsystem, &m_feederSubsystem, &m_intakeSubsystem, &m_shooterSubsystem,  &m_turretSubsystem));

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
  frc::SmartDashboard::PutData("Zero Steer Encoders", new InstantDisabledCommand([this](){
    m_driveSubsystem.ResetEncoders();
  }));
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





    // DEFAULT COMMANDS
    m_feederSubsystem.SetDefaultCommand(FeederDefaultCommand(&m_feederSubsystem, &m_intakeSubsystem));
    m_driveSubsystem.SetDefaultCommand(DriveWithJoystickCommand(&m_driveSubsystem, &m_driverController));

    // DRIVER BUTTONS

    // Driver Climber Subsystem
    
    // Driver Drive Subsystem
    m_lBumperDriver.WhenPressed(new frc2::InstantCommand([this]{m_driveSubsystem.toggleFieldCentricForJoystick();},{&m_driveSubsystem}));
    m_startDriver.WhenPressed(new frc2::InstantCommand([this]{
                              m_driveSubsystem.ResetOdometry(frc::Pose2d(
                                                                m_driveSubsystem.GetPose().Translation().X(), 
                                                                m_driveSubsystem.GetPose().Translation().Y(),
                                                                frc::Rotation2d(units::degree_t(0))));
                              },{&m_driveSubsystem}));

    // Driver Feeder Subsystem

    // Driver Intake Subsystem
    m_rTriggerDriver.WhenPressed(ExtendIntakeCommand(&m_intakeSubsystem));
    m_rTriggerDriver.WhenReleased(RetractIntakeCommand(&m_intakeSubsystem));

    // Driver Shooter Subsystem

    // Driver Turret Subsystem


    // OPERATOR BUTTONS

    // Operator Climber Subsystem
     m_startBackAux.WhenPressed(AutoClimbCommand(&m_climberSubsystem, &m_driveSubsystem));
     m_rBumperAux.WhenPressed(ManualClimbCommand(&m_climberSubsystem, &m_driveSubsystem, &m_auxController));
     m_rBumperAux.WhenReleased(RetractFloorClimberWheelsCommand(&m_climberSubsystem));
     m_xButtonAux.WhenPressed(ToggleJavelinCommand(&m_climberSubsystem));
    //  m_lBumperAux.WhenPressed(FireJavelinCommand(&m_climberSubsystem));
    //  m_lBumperAux.WhenReleased(RetractJavelinCommand(&m_climberSubsystem));

    // Operator Drive Subsystem

    // Operator Feeder Subsystem

    // Operator Intake Subsystem
    // m_xButtonAux.WhenPressed(ManualStartIntakeFeederCommand(&m_feederSubsystem, &m_intakeSubsystem));
    // m_yButtonAux.WhenPressed(ManualStopIntakeFeederCommand(&m_feederSubsystem, &m_intakeSubsystem));

    // Operator Shooter Subsystem
     m_aButtonLeftBumpAux
     .WhenPressed(StartShooterCommand(&m_shooterSubsystem));
     m_rTriggerAux.WhileHeld(ShootCommand(&m_feederSubsystem, &m_shooterSubsystem));
     m_lTriggerAux.WhileHeld(AutoAdjustShooterSpeedCommand(&m_shooterSubsystem, &m_turretSubsystem));

    // Operator Turret Subsystem


    //=================================================================================================================

  
  // frc::SmartDashboard::PutData("Zero Steer Encoders", new InstantDisabledCommand([this](){
  //   m_driveSubsystem.ResetEncoders();
  // }));

  frc::SmartDashboard::PutData("Reset Odometry", new InstantDisabledCommand([this](){
    m_driveSubsystem.ResetOdometry(frc::Pose2d());
  }));

  frc::SmartDashboard::PutData("Zero Turret", new InstantDisabledCommand([this](){
    m_turretSubsystem.zeroTurret();
  }));
  
  m_lTriggerDriver.WhileHeld(BarfCommand(&m_intakeSubsystem, &m_feederSubsystem));
  m_aButtonAux.WhenPressed(AutoAdjustShooterSpeedCommand(&m_shooterSubsystem, &m_turretSubsystem));
  m_bButtonAux.WhenPressed(StopShooterCommand(&m_shooterSubsystem));
  

  
  m_turretSubsystem.SetDefaultCommand(StayOnTargetCommand(&m_turretSubsystem));
  frc2::InstantCommand m_zeroTurret{[this] {m_turretSubsystem.zeroTurret(); }, {&m_turretSubsystem}};
  m_aButtonDriver.WhenPressed(m_zeroTurret);

  
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return nullptr;
  // return new TwoBallAutoCommand(&m_driveSubsystem, &m_feederSubsystem, &m_intakeSubsystem, &m_shooterSubsystem, &m_turretSubsystem);
}
