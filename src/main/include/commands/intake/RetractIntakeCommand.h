// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/FeederSubsystem.h"
#include "RobotParameters.h"
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class RetractIntakeCommand
    : public frc2::CommandHelper<frc2::CommandBase, RetractIntakeCommand> {
 private:
  IntakeSubsystem* m_pIntake;
  frc::Timer m_timer;
 public:
  RetractIntakeCommand(IntakeSubsystem* intake){
    m_pIntake = intake;
    
    // AddRequirements(m_pIntake);
  }

  void Initialize() override{
    m_timer.Reset();
    m_timer.Start();
    m_pIntake->retractIntake();
  }

  void Execute() override{}

  void End(bool interrupted) override{
    m_pIntake->setRollerSpeed(0);
    m_timer.Stop();
    frc::SmartDashboard::PutBoolean("Intake Extended", false);
  }

  bool IsFinished() override{
    return (int)m_timer.Get() >= 1;
  }
};
