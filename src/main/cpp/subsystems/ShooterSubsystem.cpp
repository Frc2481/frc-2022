// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ShooterSubsystem.h"
#include "Constants.h"
#include "RobotParameters.h"

ShooterSubsystem::ShooterSubsystem() :
   m_setSpeedTopWheel(0.0),
   m_setSpeedBottomWheel(0.0),
   m_isShooterOn(false),
   m_isOnTarget(false),
   m_distanceToTarget(0.0)
   {
       m_topShooterMotor = new TalonFXMotorController(FalconIDs::kTopShooterMotorID, "topShooterMotor");
       m_topShooterMotor->ConfigFactoryDefault();
       m_topShooterMotor->Config_kP(0, RobotParameters::k_shooterP);
       m_topShooterMotor->Config_kI(0,RobotParameters::k_shooterI);
       m_topShooterMotor->Config_kD(0,RobotParameters::k_shooterD);
       m_topShooterMotor->Config_kF(0,RobotParameters::k_shooterF);
       m_topShooterMotor->Config_IntegralZone(0,25); //TODO correct values
    //    m_topShooterSpeedsVect.push_back(0.0,0.0);
    //    m_topShooterSpeedsVect.push_back(0.0,0.0);
    //    m_topShooterSpeedsVect.push_back(0.0,0.0);

       m_bottomShooterMotor = new TalonFXMotorController(FalconIDs::kBottomShooterMotorID, "bottomShooterMotor");
       m_bottomShooterMotor->ConfigFactoryDefault();
       m_bottomShooterMotor->Config_kP(0, RobotParameters::k_shooterP);
       m_bottomShooterMotor->Config_kI(0,RobotParameters::k_shooterI);
       m_bottomShooterMotor->Config_kD(0,RobotParameters::k_shooterD);
       m_bottomShooterMotor->Config_kF(0,RobotParameters::k_shooterF);
       m_bottomShooterMotor->Config_IntegralZone(0,25); //TODO correct values

       

       
   }

   bool ShooterSubsystem::isShooterOn(){
       return m_isShooterOn;
   }

   bool ShooterSubsystem::isShooterOnTarget(){
       return m_isOnTarget;
   }
   double ShooterSubsystem::getTopShooterSpeed(){
       return m_topShooterMotor->GetVelocity();
   }
   double ShooterSubsystem::getBottomShooterSpeed(){
       return m_bottomShooterMotor->GetVelocity();
   }

    void ShooterSubsystem::stopShooter(){
       m_isShooterOn = false;
       m_topShooterMotor->Set(CommonModes::PercentOutput, 0.0); 
       m_bottomShooterMotor->Set(CommonModes::PercentOutput, 0.0);   
   }

   void ShooterSubsystem::startShooter(){
       m_isShooterOn = true;
   }

   
   




// This method will be called once per scheduler run
void ShooterSubsystem::Periodic() {}
