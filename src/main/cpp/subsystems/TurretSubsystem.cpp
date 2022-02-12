// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/TurretSubsystem.h"
#include "Constants.h"
#include "RobotParameters.h"
#include "Utils/NormalizeToRange.h"
#include <wpi/numbers>
#include <frc/smartdashboard/SmartDashboard.h>



TurretSubsystem::TurretSubsystem() :
    m_isTurretRunning(false),
    m_isOnTarget(false), //true??
    m_angle_ticks(0.0),   //TODO find angle
    m_distance(0.0) //TODO find distance

    {
        m_ABSPositionSensor = new frc::AnalogInput(0);
        m_turretMotor = new TalonFXMotorController(FalconIDs::kturretMotorID, "turretMotor");
        m_turretMotor->ConfigFactoryDefault();
        // m_turretMotor->SetCon
        m_turretMotor->ConfigMotionCruiseVelocity((RobotParameters::k_maxTurretSpeed)/RobotParameters::k_turretEncoderTicksToDPS);  //Degrees per second
        m_turretMotor->ConfigMotionAcceleration(((RobotParameters::k_maxTurretSpeed)/RobotParameters::k_turretEncoderTicksToDPS)*2);
        m_turretMotor->Config_kP(0, RobotParameters::k_shooterP); //do we need PIDF for turret motor?
       m_turretMotor->Config_kI(0,RobotParameters::k_shooterI);
       m_turretMotor->Config_kD(0,RobotParameters::k_shooterD);
       m_turretMotor->Config_kF(0,RobotParameters::k_shooterF);
       m_turretMotor->Config_IntegralZone(0,25); //TODO correct values
       m_turretMotor->SetSensorPhase(true);
       frc::SmartDashboard::PutNumber("Turret ADC", 0);

    //    m_turretMotor->getSensorCollection.setIntegratedSensorPosition(0, 25); //TODO find
    }

    bool TurretSubsystem::isTurretRunning(){
        return m_isTurretRunning;
    }
    bool TurretSubsystem::isOnTarget(){
        return m_isOnTarget;
    }
    double TurretSubsystem::getDistance(){
        if((bool)nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tv",0.0)){
            m_distance = tan((nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx",0.0)-LimelightConstants::kLimelightAngle)*
                (wpi::numbers::pi/180))*(LimelightConstants::kTargetHeight-LimelightConstants::kLimelightHeight);
            return m_distance;
    } else{
        return m_distance;
    }

    }
    double TurretSubsystem::getAngleToTarget(){
        return nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx",0.0);
    }
    double TurretSubsystem::getTurretAngle(){
        return m_angle_ticks - m_angleOffsetTicks;
    }
    void TurretSubsystem::rotateTurret(double angle){
        double diff = normalizeToRange::RangedDifference(angle/RobotParameters::k_turretEncoderTicksToDegrees-m_angle_ticks, -1024,1024);//frc::SmartDashboard::GetNumber("scale motion error", 1)
    
        frc::SmartDashboard::PutNumber("Diff", diff);
     
      m_turretMotor->Set(CommonModes::MotionMagic, angle); //m_angle_ticks + ( diff)/RobotParameters::k_turretEncoderTicksToDegrees);
    }

    bool TurretSubsystem::isTargetVisible(){
        
        return (bool)nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tv",0.0);
    }
    double TurretSubsystem::getTurretAbsoluteAngle(){
        return (frc::SmartDashboard::GetNumber("Turret ADC", 0)/RobotParameters::k_turretADCPerRotation)*RobotParameters::k_turretABSDegreesPerShaftRotation-360;
        return (m_ABSPositionSensor->GetValue()/RobotParameters::k_turretADCPerRotation)*RobotParameters::k_turretABSDegreesPerShaftRotation-360;
    }
    double TurretSubsystem::getTurretRelativeAngle(){
        return (m_angle_ticks/RobotParameters::k_turretTicksPerRotation)*RobotParameters::k_turretABSDegreesPerShaftRotation*RobotParameters::k_turretGearRatio;
    }
    void TurretSubsystem::zeroTurret(){
        m_angleOffsetTicks = ((getTurretRelativeAngle()) - (getTurretAbsoluteAngle())/(RobotParameters::k_turretABSDegreesPerShaftRotation*RobotParameters::k_turretGearRatio))*RobotParameters::k_turretTicksPerRotation;
    }


// This method will be called once per scheduler run
void TurretSubsystem::Periodic() {
    m_angle_ticks = m_turretMotor->GetSelectedSensorPosition(0);
    frc::SmartDashboard::PutNumber("Turret Position Ticks", m_angle_ticks);
    frc::SmartDashboard::PutNumber("Turret Angle Offset", m_angleOffsetTicks);
    frc::SmartDashboard::PutNumber("Turret Absolute Angle",getTurretAbsoluteAngle());
    frc::SmartDashboard::PutNumber("Turret Relative Angle",getTurretRelativeAngle());
    frc::SmartDashboard::PutNumber("Turret Calibrated Angle",getTurretAngle());


    

     

    m_turretMotor->Config_kP(0,frc::SmartDashboard::GetNumber("TurretP", 0)); 
    m_turretMotor->Config_kI(0,frc::SmartDashboard::GetNumber("TurretI", 0));
    m_turretMotor->Config_kD(0,frc::SmartDashboard::GetNumber("TurretD", 0));
    m_turretMotor->Config_kF(0,frc::SmartDashboard::GetNumber("TurretF", 0));
}
