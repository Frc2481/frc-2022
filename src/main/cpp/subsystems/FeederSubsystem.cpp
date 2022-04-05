// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/FeederSubsystem.h"
#include "RobotParameters.h"
#include <frc/smartdashboard/SmartDashboard.h>

FeederSubsystem::FeederSubsystem() :
    m_isFeederRunning (false),
    m_isIndexerRunning (false),
    m_feederBeamBreak(DigitalInputs::kFeederBeamBreakPort),
    m_indexerBeamBreak(DigitalInputs::kIndexerBeamBreakPort)
   {
       m_pFeederMotor = new VictorMotorController(VictorIDs::kFeederMotorID, "feederMotor");
       m_pFeederMotor->ConfigFactoryDefault();
       m_pFeederMotor->SetNeutralMode(ctre::phoenix::motorcontrol::Brake);
    //    m_pFeederMotor->SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_1_General, 100, 0); //TODO check to see if proper delay
       m_pIndexerMotor = new VictorMotorController(VictorIDs::kIndexerMotorID, "indexerMotor");
       m_pIndexerMotor->ConfigFactoryDefault();
       m_pIndexerMotor->SetNeutralMode(ctre::phoenix::motorcontrol::Brake);
    //    m_pIndexerMotor->SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_1_General, 100, 0); //TODO check to see if proper delay

       
       
   }


   bool FeederSubsystem::isFeederRunning(){
       return m_isFeederRunning;
   }
   bool FeederSubsystem::isIndexerRunning(){
       return m_isIndexerRunning;
   }
   void FeederSubsystem::setFeederSpeed(double speed){
        // m_pFeederMotor->Set(speed);
        m_pFeederMotor->Set(speed);
        m_isFeederRunning = speed;
   }
   void FeederSubsystem::setIndexerSpeed(double speed){
        m_pIndexerMotor->Set(speed);
        m_isIndexerRunning = speed;
   }
   bool FeederSubsystem::getFeederBeamBreak(){
    //    return frc::SmartDashboard::GetBoolean("Feeder Beam Break", false);
      return !m_feederBeamBreak.Get();
   }
   bool FeederSubsystem::getIndexerBeamBreak(){
    //    return frc::SmartDashboard::GetBoolean("Indexer Beam Break", false);
      return !m_indexerBeamBreak.Get();
   }




// This method will be called once per scheduler run
void FeederSubsystem::Periodic() {
    frc::SmartDashboard::PutBoolean("Feeder Beam Break", m_feederBeamBreak.Get());
    frc::SmartDashboard::PutBoolean("Indexer Beam Break", m_indexerBeamBreak.Get());
}




    