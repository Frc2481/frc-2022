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
   m_autoSpeed(true),
   m_isInManual(false),
   m_distanceToTarget(0.0), 
   m_topShooterSpeedsVect   { 3100, 3180, 3200, 3250, 3300, 3350, 3450, 3550, 3800}, //  3100, 3200, 3400, 3800 / 3100, 2400, 2400 / 2100, 2100, 2100, 2100, 2100, 2100, 2100, 2100, 2700
   m_bottomShooterSpeedsVect{ 1000, 1750, 1900, 1950, 2075, 2175, 2200, 2300, 2800}, // 1000, 1900, 2200, 2800 / 1000, 3000, 3750 / 1950, 2200, 2450, 2700, 2900, 3300, 3650, 4000, 4000 
   m_distancesToTarget      { 66, 107, 120, 130, 140, 150, 160, 170, 200} // 66, 120, 160, 200 / 66, 144, 180 / 66, 80, 100, 120, 140, 160, 180, 200, 220 
   {
       m_pTopShooterMotor = new TalonFXMotorController(FalconIDs::kTopShooterMotorID, "topShooterMotor");
       m_pTopShooterMotor->ConfigFactoryDefault();
       m_pTopShooterMotor->Config_kP(0, RobotParameters::k_shooterTopP);
       m_pTopShooterMotor->Config_kI(0,RobotParameters::k_shooterTopI);
       m_pTopShooterMotor->Config_kD(0,RobotParameters::k_shooterTopD);
       m_pTopShooterMotor->Config_kF(0,RobotParameters::k_shooterTopF);
       m_pTopShooterMotor->Config_IntegralZone(0,25); //TODO correct values
       m_pTopShooterMotor->SetInverted(true);
       m_pTopShooterMotor->ConfigVoltageCompSaturation(10);
       m_pTopShooterMotor->EnableVoltageCompensation(true);

       m_pBottomShooterMotor = new TalonFXMotorController(FalconIDs::kBottomShooterMotorID, "bottomShooterMotor");
       m_pBottomShooterMotor->ConfigFactoryDefault();
       m_pBottomShooterMotor->Config_kP(0, RobotParameters::k_shooterBotP);
       m_pBottomShooterMotor->Config_kI(0,RobotParameters::k_shooterBotI);
       m_pBottomShooterMotor->Config_kD(0,RobotParameters::k_shooterBotD);
       m_pBottomShooterMotor->Config_kF(0,RobotParameters::k_shooterBotF);
       m_pBottomShooterMotor->Config_IntegralZone(0,25); //TODO correct values
       m_pBottomShooterMotor->SetInverted(false);
       m_pBottomShooterMotor->ConfigVoltageCompSaturation(10);
       m_pBottomShooterMotor->EnableVoltageCompensation(true);


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
        if(m_autoSpeed && (distance > 0)){
            // if(distance){
            m_pBottomShooterMotor->Set(CommonModes::Velocity, interpolate::interp(m_distancesToTarget, m_bottomShooterSpeedsVect, distance, false)/60.0/10.0*2048.0);//TODO find min max
            m_pTopShooterMotor->Set(CommonModes::Velocity, interpolate::interp(m_distancesToTarget, m_topShooterSpeedsVect, distance, false)/60.0/10.0*2048.0);//TODO find min max
        }
        m_isShooterOn = true;
       frc::SmartDashboard::PutNumber("Bottom Interpolate", interpolate::interp(m_distancesToTarget, m_bottomShooterSpeedsVect, distance, true));
       frc::SmartDashboard::PutNumber("Top Interpolate", interpolate::interp(m_distancesToTarget, m_topShooterSpeedsVect, distance, true));
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
        m_pTopShooterMotor->Set(CommonModes::Velocity, speed/60.0/10.0*2048.0);// /60.0/10.0*2048.0
   }
   void ShooterSubsystem::bottomMotorSetSpeed(double speed){
    //    m_pBottomShooterMotor->Set(speed);//CommonModes::Velocity, 
    //    m_pBottomShooterMotor->Set(CommonModes::Velocity, speed);//, 
       m_pBottomShooterMotor->Set(CommonModes::Velocity, speed/60.0/10.0*2048.0);// /60.0/10.0*2048.0
   }


   
   

void ShooterSubsystem::enableAutoSpeed(){
    m_autoSpeed = true;
}
void ShooterSubsystem::disableAutoSpeed(){
    m_autoSpeed = false;
}

// This method will be called once per scheduler run
void ShooterSubsystem::Periodic() {
    //  m_pBottomShooterMotor->Set(CommonModes::Velocity, (frc::SmartDashboard::GetNumber("Bot Shoot Spd", 0)/60.0/10.0)*2048.0);
    //    m_pTopShooterMotor->Set(CommonModes::Velocity, (frc::SmartDashboard::GetNumber("Top Shoot Spd", 0)/60.0/10.0)*2048.0);

    static int counter = 0;
    if (counter++ == 10) {
        frc::SmartDashboard::PutNumber("Actual Bot Shoot Spd", m_pBottomShooterMotor->GetVelocity()*600.0/2048.0);// /60.0/10.0*2048.0
        frc::SmartDashboard::PutNumber("Actual Top Shoot Spd", m_pTopShooterMotor->GetVelocity()*600.0/2048.0);// /60.0/10.0*2048.0
        counter = 0;
    }
}