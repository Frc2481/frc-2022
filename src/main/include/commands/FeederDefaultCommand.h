// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/FeederSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
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
class FeederDefaultCommand
    : public frc2::CommandHelper<frc2::CommandBase, FeederDefaultCommand> {
      private:
      FeederSubsystem* m_pFeeder;
      IntakeSubsystem* m_pIntake;
      bool m_isIntakeOn;
      bool m_ballInIndexer;
      frc::Timer m_timeout;
      
 public:
  FeederDefaultCommand(FeederSubsystem* feeder, IntakeSubsystem* intake){
    m_pFeeder = feeder;
    m_pIntake = intake;
    m_isIntakeOn = false;
    AddRequirements (m_pFeeder);
  }

  void Initialize() override{
    frc::SmartDashboard::PutBoolean("I exist my boi", true);

  }

  void Execute() override{
    if (m_isIntakeOn){
      m_pFeeder->setIndexerSpeed(FeederConstants::kIndexerSpeed);
      frc::SmartDashboard::PutBoolean("Indexer Running", true);
      if (m_ballInIndexer){
        if (m_pFeeder->getFeederBeamBreak()){
         m_pFeeder->setIndexerSpeed(0.0);
         m_pFeeder->setFeederSpeed(0.0);
         m_isIntakeOn = false;
         m_timeout.Stop();
         m_ballInIndexer = false;
         frc::SmartDashboard::PutBoolean("Feeder Running", false);
         frc::SmartDashboard::PutBoolean("Indexer Running", false);
        }
        else {
          m_pFeeder->setFeederSpeed(FeederConstants::kDefaultFeederSpeed);
          frc::SmartDashboard::PutBoolean("Feeder Running", true);
        }
      }
      else if ((int)m_timeout.Get() >= 3){//in seconds
          m_pFeeder->setIndexerSpeed(0.0);
          m_pFeeder->setFeederSpeed(0.0);
          m_isIntakeOn = false;
          m_timeout.Stop();
          frc::SmartDashboard::PutBoolean("Feeder Running", false);
          frc::SmartDashboard::PutBoolean("Indexer Running", false);
      }
     else {
       m_ballInIndexer = m_pFeeder->getIndexerBeamBreak();
     } 
    }
    else {
      m_isIntakeOn = m_pIntake->isIntakeExtended();
      if (m_isIntakeOn){
        m_timeout.Reset();
        m_timeout.Start();
      }
    }

  }

  void End(bool interrupted) override{
    m_pFeeder->setIndexerSpeed(0.0);
    m_pFeeder->setFeederSpeed(0.0);
    m_isIntakeOn = false;
    m_timeout.Stop();
          frc::SmartDashboard::PutBoolean("Feeder Running", false);
         frc::SmartDashboard::PutBoolean("Indexer Running", false);
             frc::SmartDashboard::PutBoolean("I exist my boi", false);

  }

  bool IsFinished() override{
    return false;
  }
};
