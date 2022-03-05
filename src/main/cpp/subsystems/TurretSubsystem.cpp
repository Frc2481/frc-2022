// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/TurretSubsystem.h"
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
        m_pTurretMotor = new TalonFXMotorController(FalconIDs::kturretMotorID, "turretMotor");
        m_pTurretMotor->ConfigFactoryDefault();
        // m_turretMotor->SetCon
        m_pTurretMotor->ConfigMotionCruiseVelocity(RobotParameters::k_maxTurretSpeed);  //Degrees per second
        m_pTurretMotor->ConfigMotionAcceleration(RobotParameters::k_turretAcceleration);
        m_pTurretMotor->Config_kP(0, RobotParameters::k_turretP, 10); //do we need PIDF for turret motor?
       m_pTurretMotor->Config_kI(0,RobotParameters::k_turretI, 10);
       m_pTurretMotor->Config_kD(0,RobotParameters::k_turretD, 10);
       m_pTurretMotor->Config_kF(0,RobotParameters::k_turretF, 10);
       m_pTurretMotor->Config_IntegralZone(0,25, 10); //TODO correct values
       m_pTurretMotor->SetSensorPhase(true);
       m_pTurretMotor->SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_2_Feedback0, 20, 0);
       m_pTurretMotor->ConfigPeakOutputForward(.15, 10);
       m_pTurretMotor->ConfigPeakOutputReverse(-.15, 10);
       m_pTurretMotor->ConfigNominalOutputForward(0.02, 10);
       m_pTurretMotor->ConfigNominalOutputReverse(-0.02, 10);
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
        double target_angle = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ty",0.0);
        double angle_to_target = target_angle + LimelightConstants::kLimelightAngle;
        double angle_to_target_rad = angle_to_target * (wpi::numbers::pi/180);
        m_distance = (LimelightConstants::kTargetHeight-LimelightConstants::kLimelightHeight)/tan(angle_to_target_rad);  //LimelightConstants::kLimelightAngle)*);
        return m_distance;
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
     
      m_pTurretMotor->Set(CommonModes::MotionMagic, angle); //m_angle_ticks + ( diff)/RobotParameters::k_turretEncoderTicksToDegrees);
    }

    bool TurretSubsystem::isTargetVisible(){
        
        return (bool)nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tv",0.0);
    }
    double TurretSubsystem::getTurretAbsoluteAngle(){
        // return (frc::SmartDashboard::GetNumber("Turret ADC", 0)/RobotParameters::k_turretADCPerRotation)*RobotParameters::k_turretABSDegreesPerShaftRotation-360;
        return (m_ABSPositionSensor->GetValue()/RobotParameters::k_turretADCPerRotation)*RobotParameters::k_turretABSDegreesPerShaftRotation-360;
    }
    double TurretSubsystem::getTurretRelativeAngle(){
        return (m_angle_ticks/RobotParameters::k_turretTicksPerRotation)*RobotParameters::k_turretABSDegreesPerShaftRotation*RobotParameters::k_turretGearRatio;
    }
    void TurretSubsystem::zeroTurret(){
        //this is a very impotant line to make the absolute encoder work
        // m_angleOffsetTicks = ((getTurretRelativeAngle()) - (getTurretAbsoluteAngle())/(RobotParameters::k_turretABSDegreesPerShaftRotation*RobotParameters::k_turretGearRatio))*RobotParameters::k_turretTicksPerRotation;
        m_angleOffsetTicks = m_angle_ticks;
        // Rotate turrent to extremes and then determine distance from 0 point to get these numbers. 
        m_pTurretMotor->ConfigSoftLimits(m_angle_ticks + 6031, m_angle_ticks - 5564); 
    }
    


// This method will be called once per scheduler run
void TurretSubsystem::Periodic() {
    m_angle_ticks = m_pTurretMotor->GetSelectedSensorPosition(0);
    frc::SmartDashboard::PutNumber("Turret Position Ticks", m_angle_ticks);
    frc::SmartDashboard::PutNumber("Turret Angle Offset", m_angleOffsetTicks);
    frc::SmartDashboard::PutNumber("Turret Absolute Angle",getTurretAbsoluteAngle());
    frc::SmartDashboard::PutNumber("Turret Relative Angle",getTurretRelativeAngle());
    frc::SmartDashboard::PutNumber("Turret Calibrated Angle",getTurretAngle());
}
