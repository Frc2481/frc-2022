// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ShooterSubsystem.h"
#include "RobotParameters.h"
#include "utils/Interpolate.h"
#include <frc/smartdashboard/SmartDashboard.h>


ShooterSubsystem::ShooterSubsystem() :
   m_isShooterOn(false),
   m_isOnTarget(false),
   m_isInManual(false),
   m_distanceToTarget(0.0), 
   m_bottomShooterSpeedsVect{   2600, 2750, 2900, 3025, 3125, 3225, 3325, 3500, 3600, 3725, 3850, 4000},
   m_topShooterSpeedsVect   {   800, 800},
   m_distancesToTarget      {   64.79, 74.2, 84, 94, 105, 116, 126, 138, 151, 161, 173, 184 }
   {
       m_pTopShooterMotor = new TalonFXMotorController(FalconIDs::kTopShooterMotorID, "topShooterMotor");
       m_pTopShooterMotor->ConfigFactoryDefault();
       m_pTopShooterMotor->Config_kP(0, RobotParameters::k_shooterP);
       m_pTopShooterMotor->Config_kI(0,RobotParameters::k_shooterI);
       m_pTopShooterMotor->Config_kD(0,RobotParameters::k_shooterD);
       m_pTopShooterMotor->Config_kF(0,RobotParameters::k_shooterF);
       m_pTopShooterMotor->Config_IntegralZone(0,25); //TODO correct values
       m_pTopShooterMotor->SetInverted(true);
        

       m_pBottomShooterMotor = new TalonFXMotorController(FalconIDs::kBottomShooterMotorID, "bottomShooterMotor");
       m_pBottomShooterMotor->ConfigFactoryDefault();
       m_pBottomShooterMotor->Config_kP(0, RobotParameters::k_shooterP);
       m_pBottomShooterMotor->Config_kI(0,RobotParameters::k_shooterI);
       m_pBottomShooterMotor->Config_kD(0,RobotParameters::k_shooterD);
       m_pBottomShooterMotor->Config_kF(0,RobotParameters::k_shooterF);
       m_pBottomShooterMotor->Config_IntegralZone(0,25); //TODO correct values
       m_pBottomShooterMotor->SetInverted(false);
       


       m_pBottomShooterMotor->Set(CommonModes::Velocity, frc::SmartDashboard::PutNumber("Bot Shoot Spd", 0));
       m_pTopShooterMotor->Set(CommonModes::Velocity, frc::SmartDashboard::PutNumber("Top Shoot Spd", 0));

       
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
       m_pBottomShooterMotor->Set(CommonModes::Velocity, interpolate::interp(m_distancesToTarget, m_bottomShooterSpeedsVect, distance, true)/60.0/10.0*2048.0);//TODO find min max
       m_pTopShooterMotor->Set(CommonModes::Velocity, interpolate::interp(m_distancesToTarget, m_topShooterSpeedsVect, distance, true)/60.0/10.0*2048.0);//TODO find min max
       m_isShooterOn = true;
       frc::SmartDashboard::PutNumber("Bottom Interpolate", interpolate::interp(m_distancesToTarget, m_bottomShooterSpeedsVect, distance, true)/60.0/10.0*2048.0);
       frc::SmartDashboard::PutNumber("Top Interpolate", interpolate::interp(m_distancesToTarget, m_topShooterSpeedsVect, distance, true)/60.0/10.0*2048.0);
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
   void ShooterSubsystem::topMotorSetSpeed(double speed){
    //    m_pTopShooterMotor->Set(speed);//CommonModes::Velocity, 
    //    m_pTopShooterMotor->Set(CommonModes::Velocity, speed);//, 
        m_pTopShooterMotor->Set(CommonModes::Velocity, speed/60.0/10.0*2048.0);//, 
   }
   void ShooterSubsystem::bottomMotorSetSpeed(double speed){
    //    m_pBottomShooterMotor->Set(speed);//CommonModes::Velocity, 
    //    m_pBottomShooterMotor->Set(CommonModes::Velocity, speed);//, 
       m_pBottomShooterMotor->Set(CommonModes::Velocity, speed/60.0/10.0*2048.0);
   }


   
   




// This method will be called once per scheduler run
void ShooterSubsystem::Periodic() {
    //  m_pBottomShooterMotor->Set(CommonModes::Velocity, (frc::SmartDashboard::GetNumber("Bot Shoot Spd", 0)/60.0/10.0)*2048.0);
    //    m_pTopShooterMotor->Set(CommonModes::Velocity, (frc::SmartDashboard::GetNumber("Top Shoot Spd", 0)/60.0/10.0)*2048.0);

    frc::SmartDashboard::PutNumber("Actual Bot Shoot Spd", (m_pBottomShooterMotor->GetVelocity()/60.0/10.0)*2048.0);
    frc::SmartDashboard::PutNumber("Actual Top Shoot Spd", (m_pTopShooterMotor->GetVelocity()/60.0/10.0)*2048.0);
}
