// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include "subsystems/TurretSubsystem.h"
#include "subsystems/ExampleSubsystem.h"
#include "components/Joystick2481.h"
#include <frc2/command/button/Button.h>
#include "components/XboxController2481.h"
#include "subsystems/FeederSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/DriveSubsystem.h"
/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();

 private:
  // The robot's subsystems and commands are defined here...
  Joystick2481 m_driverController;
  
 public:
  DriveSubsystem m_driveSubsystem;
  TurretSubsystem m_turretSubsystem;
  FeederSubsystem m_feederSubsystem;
  IntakeSubsystem m_intakeSubsystem;
  //driver
  frc2::Button m_aButtonDriver{[&] { return m_driverController.GetRawButton(XBOX_A_BUTTON); }};
  frc2::Button m_bButtonDriver{[&] { return m_driverController.GetRawButton(XBOX_B_BUTTON); }};
  frc2::Button m_yButtonDriver{[&] { return m_driverController.GetRawButton(XBOX_Y_BUTTON); }};
  frc2::Button m_xButtonDriver{[&] { return m_driverController.GetRawButton(XBOX_X_BUTTON); }};

  frc2::Button m_rBumperDriver{[&] { return m_driverController.GetRawButton(XBOX_RIGHT_BUMPER); }};//
  frc2::Button m_lBumperDriver{[&] { return m_driverController.GetRawButton(XBOX_LEFT_BUMPER); }};//
  frc2::Button m_rTriggerDriver{[&] { return m_driverController.GetAxis(XBOX_RIGHT_TRIGGER, .5); }};//
  frc2::Button m_lTriggerDriver{[&] { return m_driverController.GetAxis(XBOX_LEFT_TRIGGER, .5); }};//

  //operator
  frc2::Button m_startAux{[&] { return m_auxController.GetRawButton(XBOX_START_BUTTON); }};//
  frc2::Button m_backAux{[&] { return m_auxController.GetRawButton(XBOX_BACK_BUTTON); }};//

  frc2::Button m_aButtonAux{[&] { return m_auxController.GetRawButton(XBOX_A_BUTTON); }};//

  frc2::Button m_bButtonAux{[&] { return m_auxController.GetRawButton(XBOX_B_BUTTON); }};//
  frc2::Button m_yButtonAux{[&] { return m_auxController.GetRawButton(XBOX_Y_BUTTON); }};//
  frc2::Button m_xButtonAux{[&] { return m_auxController.GetRawButton(XBOX_X_BUTTON); }};//
  
  frc2::Button m_rBumperAux{[&] { return m_auxController.GetRawButton(XBOX_RIGHT_BUMPER); }};//
  frc2::Button m_lBumperAux{[&] { return m_auxController.GetRawButton(XBOX_LEFT_BUMPER); }};//
  frc2::Button m_rTriggerAux{[&] { return m_auxController.GetAxis(XBOX_RIGHT_TRIGGER, .5); }};//
  frc2::Button m_lTriggerAux{[&] { return m_auxController.GetAxis(XBOX_LEFT_TRIGGER, .5); }};//
  // TurretSubsystem m_turret;
  

  void ConfigureButtonBindings();
};
