// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/FeederSubsystem.h"
#include "Constants.h"
#include "RobotParameters.h"

FeederSubsystem::FeederSubsystem() :

   
   m_isShooterReady(false),
   m_isTurretReady(false),
   m_feederBeamBreak(DigitalInputs::kFeederBeamBreakPort)
   {
       m_feederMotor = new VictorMotorController(VictorIDs::kFeederMotorID, "feederMotor");
       m_feederMotor->ConfigFactoryDefault();
   }

   bool FeederSubsystem::isFeederRunning(){
       return m_isFeederRunning;
   }
   
   bool FeederSubsystem::isShooterPrimed(){
       return !m_feederBeamBreak.Get();
   }
   void FeederSubsystem::primeShooter(){
       m_isFeederRunning = true;
       m_feederMotor->Set(CommonModes::PercentOutput,FeederConstants::kPrimeShooterSpeed);
   }
   bool FeederSubsystem::isShooterReady(){
       return m_isShooterReady;
   }
   bool FeederSubsystem::isTurretReady(){
       return m_isTurretReady;
   }
   void FeederSubsystem::shootBall(){
       m_isFeederRunning = true;
       m_feederMotor->Set(CommonModes::PercentOutput, FeederConstants::kShooterSpeed);
   }

   void FeederSubsystem::setFeederSpeed(double speed){
        m_feederSpeed = speed;
        m_feederMotor->Set(speed);
   }
   void FeederSubsystem::stopShooter(){
       m_isFeederRunning = false;
       m_feederMotor->Set(CommonModes::PercentOutput, 0.0);
   }
   
   
   




// This method will be called once per scheduler run
void FeederSubsystem::Periodic() {}




    