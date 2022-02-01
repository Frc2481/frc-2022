// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/TurretSubsystem.h"
#include "Constants.h"
#include "RobotParameters.h"
#include "Utils/NormalizeToRange.h"
#include <wpi/numbers>


TurretSubsystem::TurretSubsystem() :
    m_isTurretRunning(false),
    m_isOnTarget(false), //true??
    m_angle(0.0),   //TODO find angle
    m_distance(0.0) //TODO find distance

    {
        m_turretMotor = new VictorMotorController(VictorIDs::kFeederMotorID, "turretMotor");
        m_turretMotor->ConfigFactoryDefault();
        m_turretMotor->ConfigMotionCruiseVelocity((RobotParameters::k_maxTurretSpeed)/RobotParameters::k_turretEncoderTicksToMPS);
        m_turretMotor->ConfigMotionAcceleration(((RobotParameters::k_maxTurretSpeed)/RobotParameters::k_turretEncoderTicksToMPS)*2);
        m_turretMotor->Config_kP(0, RobotParameters::k_shooterP); //do we need PIDF for turret motor?
       m_turretMotor->Config_kI(0,RobotParameters::k_shooterI);
       m_turretMotor->Config_kD(0,RobotParameters::k_shooterD);
       m_turretMotor->Config_kF(0,RobotParameters::k_shooterF);
       m_turretMotor->Config_IntegralZone(0,25); //TODO correct values
    }

    bool TurretSubsystem::isTurretRunning(){
        return m_isTurretRunning;
    }
    bool TurretSubsystem::isOnTarget(){
        return m_isOnTarget;
    }
    double TurretSubsystem::getDistance(){
        return tan((nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx",0.0)-LimelightConstants::kLimelightAngle)*
        (wpi::numbers::pi/180))*(LimelightConstants::kTargetHeight-LimelightConstants::kLimelightHeight);

    }
    double TurretSubsystem::getAngle(){
        return nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx",0.0);
    }
    void TurretSubsystem::rotateTurret(double angle){
        double diff = normalizeToRange::RangedDifference(angle-m_angle, -180,180)*.7;//frc::SmartDashboard::GetNumber("scale motion error", 1)
    

     
      m_turretMotor->Set(CommonModes::MotionMagic, m_turretMotor->GetPos() + (RobotParameters::k_turretRadius * diff * MATH_CONSTANTS_PI/180)/RobotParameters::k_driveMotorEncoderTicksToMeters);
    }
    bool TurretSubsystem::isTargetVisible(){
        
        return (bool)nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tv",0.0);
    }


// This method will be called once per scheduler run
void TurretSubsystem::Periodic() {}
