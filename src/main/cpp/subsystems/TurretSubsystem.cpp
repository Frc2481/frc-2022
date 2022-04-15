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
    m_distance(0.0), //TODO find distance
    m_limitAccel(false)

    {
        m_ABSPositionSensor = new frc::AnalogInput(0);
        m_pTurretMotor = new TalonFXMotorController(FalconIDs::kturretMotorID, "turretMotor");
        m_pTurretMotor->ConfigFactoryDefault();
        // m_turretMotor->SetCon
        m_pTurretMotor->ConfigMotionCruiseVelocity(RobotParameters::k_maxTurretSpeed);  //Degrees per second
        m_pTurretMotor->ConfigMotionAcceleration(RobotParameters::k_turretAcceleration);
        m_pTurretMotor->ConfigMotionSCurveStrength(3);
        m_pTurretMotor->Config_kP(0, RobotParameters::k_turretP, 10); //do we need PIDF for turret motor?
       m_pTurretMotor->Config_kI(0,RobotParameters::k_turretI, 10);
       m_pTurretMotor->Config_kD(0,RobotParameters::k_turretD, 10);
       m_pTurretMotor->Config_kF(0,RobotParameters::k_turretF, 10);
       m_pTurretMotor->GetBase()->ConfigAllowableClosedloopError(0, 135);
       m_pTurretMotor->GetBase()->SetNeutralMode(Coast);
       m_pTurretMotor->Config_IntegralZone(0, 400, 10); //TODO correct values
       m_pTurretMotor->SetSensorPhase(true);
       m_pTurretMotor->SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_2_Feedback0, 20, 0);
       m_pTurretMotor->ConfigPeakOutputForward(1, 10);
       m_pTurretMotor->ConfigPeakOutputReverse(-1, 10);
       m_pTurretMotor->ConfigNominalOutputForward(0, 10);
       m_pTurretMotor->ConfigNominalOutputReverse(0, 10);
        zeroTurret();
    //    m_turretMotor->getSensorCollection.setIntegratedSensorPosition(0, 25); //TODO find
    }

    bool TurretSubsystem::isTurretRunning(){
        return m_isTurretRunning;
    }
    bool TurretSubsystem::isOnTarget(){
        return m_isOnTarget;
    }
    double TurretSubsystem::getDistance(){
        // m_distance = (LimelightConstants::kTargetHeight-LimelightConstants::kLimelightHeight)/tan((nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ty",0.0)+LimelightConstants::kLimelightAngle)*  //LimelightConstants::kLimelightAngle)*
        //     (wpi::numbers::pi/180));
        // return m_distance;
        if(isTargetVisible()){
            double target_angle = m_vert_angle_to_target;
            double angle_to_target = target_angle + LimelightConstants::kLimelightAngle;
            double angle_to_target_rad = angle_to_target * (wpi::numbers::pi/180);
            m_distance = (LimelightConstants::kTargetHeight-LimelightConstants::kLimelightHeight)/tan(angle_to_target_rad);  //LimelightConstants::kLimelightAngle)*);
        }
        return m_distance;
    }

    double TurretSubsystem::getAngleToTarget(){
        if (abs(m_angle_to_target) < 0.25) {
            return 0;
        }
        return m_angle_to_target;
    }
    double TurretSubsystem::getTurretAngleTicks(){

        return m_angle_ticks - m_angleOffsetTicks;
    }
    void TurretSubsystem::rotateTurret(double angle){

        if(angle > RobotParameters::k_maxTurretDegrees){
            angle -= 360;
        }else if(angle < RobotParameters::k_minTurretDegrees){
            angle += 360;
        }

        // double targetTicks = (angle / 360) * RobotParameters::k_turretTicksPerRotation;
        double targetTicks = (angle * RobotParameters::k_turretTicksPerDegree) + m_angleOffsetTicks;
        targetTicks = std::min(targetTicks, m_rangeMaxTicks);
        targetTicks = std::max(targetTicks, m_rangeMinTicks);

         m_setpointTicks = targetTicks;

        // if (isTargetVisible() && !m_limitAccel){
        //    // m_pTurretMotor->ConfigMotionAcceleration(2000);
        //    m_pTurretMotor->ConfigMotionCruiseVelocity(1500);
        //    m_limitAccel = true;
        //}
        //else if(!isTargetVisible() && m_limitAccel){
        //    // m_pTurretMotor->ConfigMotionAcceleration(80000);
        //    m_pTurretMotor->ConfigMotionCruiseVelocity(4000);
        //    m_limitAccel = false;
        //}
        
        m_pTurretMotor->Set(CommonModes::MotionMagic, targetTicks);
    }
    double TurretSubsystem::getError(){
       return (m_setpointTicks - m_angle_ticks)/RobotParameters::k_turretTicksPerDegree;
    }
    bool TurretSubsystem::isTargetVisible(){
        
        return m_target_visible > 0;
    }
    double TurretSubsystem::getTurretAbsoluteAngle(){
        // return (frc::SmartDashboard::GetNumber("Turret ADC", 0)/RobotParameters::k_turretADCPerRotation)*RobotParameters::k_turretABSDegreesPerShaftRotation-360;
        return (m_ABSPositionSensor->GetValue()/RobotParameters::k_turretADCPerRotation)*RobotParameters::k_turretABSDegreesPerShaftRotation-360;
    }
    double TurretSubsystem::getTurretCalibratedAngle(){
        return getTurretAngleTicks()/RobotParameters::k_turretTicksPerDegree;
    //return (getTurretAngleTicks()/RobotParameters::k_turretTicksPerRotation)*RobotParameters::k_turretABSDegreesPerShaftRotation*RobotParameters::k_turretGearRatio;
    }
    double TurretSubsystem::getTurretRelativeAngle(){
        return m_angle_ticks/RobotParameters::k_turretTicksPerDegree;
    }
    void TurretSubsystem::zeroTurret(){
        m_angle_ticks = m_pTurretMotor->GetSelectedSensorPosition(0);
        //this is a very impotant line to make the absolute encoder work
        // m_angleOffsetTicks = ((getTurretRelativeAngle()) - (getTurretAbsoluteAngle())/(RobotParameters::k_turretABSDegreesPerShaftRotation*RobotParameters::k_turretGearRatio))*RobotParameters::k_turretTicksPerRotation;
        m_angleOffsetTicks = m_angle_ticks;
        // Rotate turrent to extremes and then determine distance from 0 point to get these numbers. 
        // m_pTurretMotor->ConfigSoftLimits(m_angle_ticks + 6031, m_angle_ticks - 5564); 

        // m_rangeMaxTicks = m_angle_ticks + 6031 - 1000;
        // m_rangeMinTicks = m_angle_ticks - 5564 + 1000;

        // 225 deg
        // 135 deg

        double maxTicks = (RobotParameters::k_turretTicksPerDegree * RobotParameters::k_maxTurretDegrees);
        double minTicks = (RobotParameters::k_turretTicksPerDegree * RobotParameters::k_minTurretDegrees);

        m_pTurretMotor->ConfigSoftLimits(m_angle_ticks + maxTicks, m_angle_ticks + minTicks); 

        m_rangeMaxTicks = m_angle_ticks + (maxTicks - 100);
        m_rangeMinTicks = m_angle_ticks + (minTicks + 100);
    }
    


// This method will be called once per scheduler run
void TurretSubsystem::Periodic() {
    m_angle_ticks = m_pTurretMotor->GetSelectedSensorPosition(0);
    m_vert_angle_to_target = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ty",0.0);
    if((bool)nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tv",0.0)){
        m_target_visible = 50;
        m_angle_to_target = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx",0.0);
    }
    else{
        m_target_visible--;
        m_target_visible = std::max(0, m_target_visible);

        if (m_target_visible == 0) {
            m_angle_to_target = 0;
        }
    }
    
    static int counter = 0;
    counter++;
    if (counter == 2) {
        frc::SmartDashboard::PutNumber("Turret Position Ticks", m_angle_ticks);
        frc::SmartDashboard::PutNumber("Turret Angle Offset", m_angleOffsetTicks);
        frc::SmartDashboard::PutNumber("Turret Error", getError());
    } else if (counter == 4) {
        frc::SmartDashboard::PutNumber("Turret Absolute Angle",getTurretAbsoluteAngle());
        frc::SmartDashboard::PutNumber("Turret Relative Angle",getTurretRelativeAngle());
        frc::SmartDashboard::PutNumber("Turret Setpoint", m_setpointTicks / RobotParameters::k_turretTicksPerDegree);
    } else if (counter == 6) {
        frc::SmartDashboard::PutNumber("Turret Calibrated Angle",getTurretCalibratedAngle());
        frc::SmartDashboard::PutNumber("Distance to Target", getDistance());
    } else if (counter == 8) {
        frc::SmartDashboard::PutNumber("Angle to Target", getAngleToTarget());
        frc::SmartDashboard::PutNumber("Current Angle", getTurretAngleTicks());
        counter = 0;
    }
}
