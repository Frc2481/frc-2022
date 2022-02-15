// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ShooterSubsystem.h"
#include "RobotParameters.h"
#include "utils/Interpolate.h"

ShooterSubsystem::ShooterSubsystem() :
   m_isShooterOn(false),
   m_isOnTarget(false),
   m_isInManual(false),
   m_distanceToTarget(0.0)
   {
       m_pTopShooterMotor = new TalonFXMotorController(FalconIDs::kTopShooterMotorID, "topShooterMotor");
       m_pTopShooterMotor->ConfigFactoryDefault();
       m_pTopShooterMotor->Config_kP(0, RobotParameters::k_shooterP);
       m_pTopShooterMotor->Config_kI(0,RobotParameters::k_shooterI);
       m_pTopShooterMotor->Config_kD(0,RobotParameters::k_shooterD);
       m_pTopShooterMotor->Config_kF(0,RobotParameters::k_shooterF);
       m_pTopShooterMotor->Config_IntegralZone(0,25); //TODO correct values
       m_topShooterSpeedsVect.push_back(0.0);
       m_topShooterSpeedsVect.push_back(0.0);
       m_topShooterSpeedsVect.push_back(0.0);

       m_pBottomShooterMotor = new TalonFXMotorController(FalconIDs::kBottomShooterMotorID, "bottomShooterMotor");
       m_pBottomShooterMotor->ConfigFactoryDefault();
       m_pBottomShooterMotor->Config_kP(0, RobotParameters::k_shooterP);
       m_pBottomShooterMotor->Config_kI(0,RobotParameters::k_shooterI);
       m_pBottomShooterMotor->Config_kD(0,RobotParameters::k_shooterD);
       m_pBottomShooterMotor->Config_kF(0,RobotParameters::k_shooterF);
       m_pBottomShooterMotor->Config_IntegralZone(0,25); //TODO correct values

       

       
   }

   bool ShooterSubsystem::isShooterOn(){
       return m_isShooterOn;
   }

   bool ShooterSubsystem::isShooterOnTarget(){
       return m_isOnTarget;
   }
   double ShooterSubsystem::getTopShooterSpeed(){
       return m_pTopShooterMotor->GetVelocity();
   }
   double ShooterSubsystem::getBottomShooterSpeed(){
       return m_pBottomShooterMotor->GetVelocity();
   }

    void ShooterSubsystem::stopShooter(){
       m_isShooterOn = false;
       m_pTopShooterMotor->Set(CommonModes::PercentOutput, 0.0); 
       m_pBottomShooterMotor->Set(CommonModes::PercentOutput, 0.0);   
   }

   void ShooterSubsystem::startShooter(double distance){ 
       m_pBottomShooterMotor->Set(CommonModes::Velocity, interpolate::rangedInterp(m_distancesToTarget, m_bottomShooterSpeedsVect, distance, true, 0, 255));//TODO find min max
       m_pTopShooterMotor->Set(CommonModes::Velocity, interpolate::rangedInterp(m_distancesToTarget, m_topShooterSpeedsVect, distance, true, 0, 255));//TODO find min max
       m_isShooterOn = true;
   }
   void ShooterSubsystem::toggleManualShooter(){
       m_isInManual = !m_isInManual;
      if  (m_isInManual)
      {
       m_pTopShooterMotor->Set(CommonModes::Velocity, m_manualOffsetSpeed + ShooterConstants::kDonutTopWheelSpeed);
       m_pBottomShooterMotor->Set(CommonModes::Velocity, m_manualOffsetSpeed + ShooterConstants::kDonutBottomWheelSpeed);
      }
      else 
      {
          this->stopShooter();
      }

   }
   bool ShooterSubsystem::isInManual(){
       return m_isInManual;
   }
   void ShooterSubsystem::incrementManualSpeed(){
       m_manualOffsetSpeed += 100;
       m_pTopShooterMotor->Set(CommonModes::Velocity, m_manualOffsetSpeed + ShooterConstants::kDonutTopWheelSpeed);
       m_pBottomShooterMotor->Set(CommonModes::Velocity, m_manualOffsetSpeed + ShooterConstants::kDonutBottomWheelSpeed);

       
   }
   void ShooterSubsystem::decrementManualSpeed(){
       m_manualOffsetSpeed -= 100;
       m_pBottomShooterMotor->Set(CommonModes::Velocity, m_manualOffsetSpeed + ShooterConstants::kDonutBottomWheelSpeed);
       m_pTopShooterMotor->Set(CommonModes::Velocity, m_manualOffsetSpeed + ShooterConstants::kDonutTopWheelSpeed);

   }

   
   




// This method will be called once per scheduler run
void ShooterSubsystem::Periodic() {}
