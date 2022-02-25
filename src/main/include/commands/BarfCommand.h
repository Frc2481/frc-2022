// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/FeederSubsystem.h"
#include "RobotParameters.h"
#include <frc/smartdashboard/SmartDashboard.h>

class BarfCommand
    : public frc2::CommandHelper<frc2::CommandBase,
                                 BarfCommand> {
 private:
 IntakeSubsystem* m_pIntake;
 FeederSubsystem* m_pFeeder;
 public:
  BarfCommand(IntakeSubsystem* intake, FeederSubsystem* feeder){
    m_pIntake = intake;
    m_pFeeder = feeder;
    AddRequirements(m_pIntake);
    AddRequirements(m_pFeeder);
  }
  void Initialize() override{
    m_pIntake->extendIntake();
    m_pIntake->setRollerSpeed(-IntakeConstants::kDefaultIntakeRollerSpeed);
    m_pFeeder->setIndexerSpeed(-FeederConstants::kIndexerSpeed);
    m_pFeeder->setFeederSpeed(-FeederConstants::kDefaultFeederSpeed);
  
  }
};
