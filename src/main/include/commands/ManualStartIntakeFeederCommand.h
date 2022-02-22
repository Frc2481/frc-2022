// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/FeederSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "RobotParameters.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class ManualStartIntakeFeederCommand
    : public frc2::CommandHelper<frc2::CommandBase, ManualStartIntakeFeederCommand> {
      private:
      FeederSubsystem* m_pFeeder;
      IntakeSubsystem* m_pIntake;
 public:
  ManualStartIntakeFeederCommand(FeederSubsystem* feeder, IntakeSubsystem* intake){
      m_pFeeder = feeder;
      m_pIntake = intake;
      AddRequirements(m_pIntake);
      AddRequirements(m_pFeeder);
  }
  void Initialize() override{
      // m_pFeeder->setFeederSpeed(FeederConstants::kDefaultFeederSpeed);
      m_pFeeder->setIndexerSpeed(FeederConstants::kIndexerSpeed);
      m_pIntake->setRollerSpeed(IntakeConstants::kDefaultIntakeRollerSpeed);

      m_pFeeder->setFeederSpeed((frc::SmartDashboard::GetNumber("Feeder Speed", 0)/60.0/10.0)*2048.0);
      m_pFeeder->setFeederSpeed((frc::SmartDashboard::PutNumber("Feeder Speed", 0)));
  }

};
