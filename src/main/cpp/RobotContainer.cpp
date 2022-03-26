// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "RobotParameters.h"
#include "commands/ControlMotorWithJoystickCommand.h"
#include <frc2/command/InstantCommand.h>
#include "cameraserver/CameraServer.h"
#include <frc2/command/FunctionalCommand.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/DataLogManager.h>

//Auto
#include "commands/auto/TwoBallAutoCommand.h"
#include "commands/auto/FourBallAutoCommand.h"
#include "commands/auto/TwoBallAutoCommand.h"

//Turret
#include "commands/turret/GoToAngleCommand.h"

//Shooter
#include "commands/shooter/AutoAdjustShooterSpeedCommand.h"
#include "commands/turret/StayOnTargetCommand.h"
#include "commands/shooter/ShootCommand.h"
#include "commands/shooter/StartShooterCommand.h"

//Feeder
#include "commands/ManualStartIntakeFeederCommand.h"
#include "commands/FeederDefaultCommand.h"

//Intake
#include "commands/intake/ExtendIntakeCommand.h"
#include "commands/intake/RetractIntakeCommand.h"

//Drive
#include "commands/Drive/DriveWithJoystickCommand.h"
#include "components/Joystick2481.h"

//Climber
#include "commands/climber/ExtendTrussClimberWheelsCommand.h"
#include "commands/climber/ClimbCommand.h"
#include "commands/climber/ToggleJavelinCommand.h"
#include "commands/drive/AlignToTrussCommandGroup.h"
#include "commands/climber/AutoClimbCommand.h"

RobotContainer::RobotContainer(): m_driverController(0), m_auxController(1),
                                  m_tDpadAux(&m_auxController, XBOX_DPAD_TOP),
                                  m_bDpadAux(&m_auxController, XBOX_DPAD_BOTTOM),
                                  m_lDpadAux(&m_auxController, XBOX_DPAD_LEFT),
                                  m_rDpadAux(&m_auxController, XBOX_DPAD_RIGHT)
 {
  // Initialize all of your commands and subsystems here
  
  // Configure the button bindings

  // m_pcm.set
  auto cam = frc::CameraServer::StartAutomaticCapture();
  cam.SetFPS(12);
  cam.SetResolution(160, 90);
  frc::DataLogManager::Start();
  frc::LiveWindow::DisableAllTelemetry();

  ConfigureButtonBindings();
  m_chooser.SetDefaultOption("Two Ball", new TwoBallAutoCommand(&m_driveSubsystem, &m_feederSubsystem, &m_intakeSubsystem, &m_shooterSubsystem,  &m_turretSubsystem));
  m_chooser.AddOption("Four Ball", new FourBallAutoCommand(&m_driveSubsystem, &m_feederSubsystem, &m_intakeSubsystem, &m_shooterSubsystem,  &m_turretSubsystem));
  frc::SmartDashboard::PutData(&m_chooser);
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
    m_backDriver.WhenPressed(ExtendTrussClimberWheelsCommand(&m_climberSubsystem));
    m_rBumperDriver.ToggleWhenPressed(ClimbCommand(&m_climberSubsystem, &m_driveSubsystem, &m_turretSubsystem, &m_driverController));
    m_xButtonDriver.WhenHeld(AlignToTrussCommandGroup(&m_driveSubsystem));

    // Driver Drive Subsystem
    m_lBumperDriver.WhenPressed(new frc2::InstantCommand([this]{m_driveSubsystem.toggleFieldCentricForJoystick();},{&m_driveSubsystem}));
    m_startDriver.WhenPressed(new frc2::InstantCommand([this]{
                              m_driveSubsystem.ResetOdometry(frc::Pose2d(
                                                                m_driveSubsystem.GetPose().Translation().X(), 
                                                                m_driveSubsystem.GetPose().Translation().Y(),
                                                                frc::Rotation2d(units::degree_t(0))));
                              },{&m_driveSubsystem}));

    // Driver Feeder Subsystem
    m_lTriggerDriver.WhenPressed(new frc2::InstantCommand([this]{
    m_intakeSubsystem.extendIntake();
    m_intakeSubsystem.setRollerSpeed(-IntakeConstants::kDefaultIntakeRollerSpeed);
    m_feederSubsystem.setIndexerSpeed(-FeederConstants::kIndexerSpeed);
    m_feederSubsystem.setFeederSpeed(-FeederConstants::kDefaultFeederSpeed);},{&m_intakeSubsystem, &m_feederSubsystem}));

    m_lTriggerDriver.WhenReleased(new frc2::InstantCommand([this]{  
    m_intakeSubsystem.retractIntake();
    m_intakeSubsystem.setRollerSpeed(0);
    m_feederSubsystem.setIndexerSpeed(0);
    m_feederSubsystem.setFeederSpeed(0);},{&m_intakeSubsystem, &m_feederSubsystem}));

    // Driver Intake Subsystem
    m_rTriggerDriver.WhenPressed(ExtendIntakeCommand(&m_intakeSubsystem));
    m_rTriggerDriver.WhenReleased(RetractIntakeCommand(&m_intakeSubsystem));

    // Driver Shooter Subsystem

    // Driver Turret Subsystem


    // OPERATOR BUTTONS

    // Operator Climber Subsystem
     m_backAux.WhenPressed(ToggleJavelinCommand(&m_climberSubsystem));

    // Operator Drive Subsystem

    // Operator Feeder Subsystem

    // Operator Intake Subsystem
    // m_xButtonAux.WhenPressed(ManualStartIntakeFeederCommand(&m_feederSubsystem, &m_intakeSubsystem));
    m_yButtonAux.WhenPressed(frc2::InstantCommand(
        [this]{
          m_turretSubsystem.rotateTurret(0);
          m_shooterSubsystem.startShooter(84);
        },{&m_shooterSubsystem, &m_turretSubsystem}));

    // Operator Shooter Subsystem
     m_aButtonLeftBumpAux
     .WhenPressed(StartShooterCommand(&m_shooterSubsystem));
     m_rTriggerAux.WhileHeld(ShootCommand(&m_feederSubsystem, &m_shooterSubsystem));
     m_lTriggerAux.WhileHeld(AutoAdjustShooterSpeedCommand(&m_shooterSubsystem, &m_turretSubsystem));

    // Operator Turret Subsystem
    m_tDpadAux.WhenPressed(new GoToAngleCommand(&m_turretSubsystem, -90));
    m_rDpadAux.WhenPressed(new GoToAngleCommand(&m_turretSubsystem, 0));
    m_lDpadAux.WhenPressed(new GoToAngleCommand(&m_turretSubsystem, 90));
    m_bDpadAux.WhenPressed(new StayOnTargetCommand(&m_turretSubsystem));

    //=================================================================================================================

  
  // frc::SmartDashboard::PutData("Zero Steer Encoders", new InstantDisabledCommand([this](){
  //   m_driveSubsystem.ResetEncoders();
  // }));
  frc::SmartDashboard::PutNumber("Limelight Angle", 18);

  frc::SmartDashboard::PutData("Reset Odometry", new InstantDisabledCommand([this](){
    m_driveSubsystem.ResetOdometry(frc::Pose2d());
  }));

  frc::SmartDashboard::PutData("Zero Turret", new InstantDisabledCommand([this](){
    m_turretSubsystem.zeroTurret();
  }));
  
  m_aButtonAux.WhenPressed(AutoAdjustShooterSpeedCommand(&m_shooterSubsystem, &m_turretSubsystem));
  m_bButtonAux.WhenPressed(new frc2::InstantCommand([this]{m_shooterSubsystem.stopShooter();},{&m_shooterSubsystem}));
  

  

  frc2::InstantCommand m_zeroTurret{[this] {m_turretSubsystem.zeroTurret(); }, {&m_turretSubsystem}};
  // m_aButtonDriver.WhenPressed(m_zeroTurret);

  
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return m_chooser.GetSelected();
  // return new TwoBallAutoCommand(&m_driveSubsystem, &m_feederSubsystem, &m_intakeSubsystem, &m_shooterSubsystem, &m_turretSubsystem);
}
