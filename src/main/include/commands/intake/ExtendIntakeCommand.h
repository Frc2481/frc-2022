// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/InstantCommand.h>
#include "subsystems/IntakeSubsystem.h"
#include "constants.h"
#include <frc/smartdashboard/SmartDashboard.h>

class ExtendIntakeCommand
    : public frc2::CommandHelper<frc2::InstantCommand,
                                 ExtendIntakeCommand> {
 private:
 IntakeSubsystem* m_pIntake;
 public:
  ExtendIntakeCommand(IntakeSubsystem* intake){
    m_pIntake = intake;
    AddRequirements(m_pIntake);
  }
  void Initialize() override{
    m_pIntake->extendIntake();
    m_pIntake->setRollerSpeed(IntakeConstants::kDefaultIntakeRollerSpeed);
    frc::SmartDashboard::PutBoolean("Intake Extended", true);
  }
};
