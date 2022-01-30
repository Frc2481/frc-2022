// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/FeederSubsystem.h"
#include "Constants.h"
#include "RobotParameters.h"

FeederSubsystem::FeederSubsystem() :

   m_isBallReadyIndexer(false),
   m_feederSpeed(0.0),
   m_isShooterReady(false),
   m_isTurretReady(false)
   {
       m_feederMotor = new VictorMotorController(VictorIDs::kFeederMotorID, "feederMotor");
       m_feederMotor->ConfigFactoryDefault();
   }

   bool FeederSubsystem::isBallReadyIndexer(){
       return m_isBallReadyIndexer;
   } 
   bool FeederSubsystem::isShooterReady(){
       return m_isShooterReady;
   }
   bool FeederSubsystem::isTurretReady(){
       return m_isTurretReady;
   }
   




// This method will be called once per scheduler run
void FeederSubsystem::Periodic() {}




    // void ShooterSubsystem::shoot(){
    //    if (m_isShooterOn == true && m_isOnTarget == true)
    //    {

    //    }