#include "Utils/MathConstants.h"
#include <math.h>
#ifndef ROBOT_PARAMETERS_H
#define ROBOT_PARAMETERS_H

//TODO: figure this all out

namespace RobotParameters { 
    // robot main
	static constexpr unsigned k_updateRate = 50; // Hz

    static constexpr double k_wheelBase = 1; // in
    static constexpr double k_wheelTrack = 1; // in
    static constexpr double k_wheelLeverArm = sqrt(std::pow(k_wheelBase/2,2) + std::pow(k_wheelTrack/2,2));
    static constexpr double k_wheelRad = (3.79/2)*.0254*1.03; // in TODO find actual size
    static constexpr double k_maxSpeed = 1; //TODO change also in driveWithJoystickCommand
    static constexpr double k_maxAccel = 1;
    static constexpr double k_maxDeccel = 1;
    static constexpr double k_steerEncoderToWheelGearRatio = 1; // gear ratio from steer encoder to wheel TODO find
    static constexpr double k_driveMotorGearRatio = 1;
    static constexpr double k_ticksPerRev= 2048.0;//ticks per 100ms TODO check
    static constexpr double k_driveMotorEncoderTicksToMPS = (1/k_ticksPerRev)*(1/k_driveMotorGearRatio)*k_wheelRad*3.14159265*2*10;
    static constexpr double k_driveMotorEncoderTicksToMeters = (1/k_ticksPerRev)*(1/k_driveMotorGearRatio)*k_wheelRad*3.14159265*2;
    static constexpr double k_minRobotVelocity = 1;
    static constexpr double k_minRobotYawRate = 1;
    static constexpr double k_driveWheelSlotError = 1;
    static constexpr double k_robotWidth = 1;
    static constexpr double k_robotLength = 1;
    static constexpr double k_maxYawRate = k_maxSpeed / k_wheelLeverArm *180/MATH_CONSTANTS_PI;
    static constexpr double k_maxYawAccel = k_maxAccel / k_wheelLeverArm*180/MATH_CONSTANTS_PI;
    static constexpr double k_maxYawDeccel = k_maxDeccel / k_wheelLeverArm*180/MATH_CONSTANTS_PI;
    static constexpr double k_minYawRate = k_minRobotVelocity / k_wheelLeverArm *180 / MATH_CONSTANTS_PI;
    // static constexpr double k_driveMotorEncoderMPSToRPM  = (RobotParameters::k_driveMo\[]\torGearRatio/(RobotParameters::k_wheelRad*3.14159265*2))*60;

    // TODO check rest

    //pathfollowing 
    static constexpr double  k_maxCentripAccel = 10.0;//10

    // // steer motors
    static constexpr double k_steerMotorControllerKp = 3;
    static constexpr double k_steerMotorControllerKi = 0;
    static constexpr double k_steerMotorControllerKd = 40;
    static constexpr double k_steerMotorControllerKsf = 0;
    static constexpr double k_steerMotorControllerKv = 0;
    static constexpr double k_steerMotorControllerKap = 0;
    static constexpr double k_steerMotorControllerKan = 0;


    // encoders
    static constexpr unsigned k_ctreMagEncoderTicksPerRev = 4096;
    static constexpr unsigned k_grayhillEncoderTicksPerRev = 512;
    static constexpr unsigned k_falconFXEncoderTicksPerRev = 2048;


    //shooter Constants TODO set correct values
    static constexpr double k_shooterP = 0.0;
    static constexpr double k_shooterI = 0.0;
    static constexpr double k_shooterD = 0.0;
    static constexpr double k_shooterF = 0.0;

    //turret Constants TODO set correct values
    static constexpr double k_turretP = 0.0;
    static constexpr double k_turretI = 0.0;
    static constexpr double k_turretD = 0.0;
    static constexpr double k_turretF = 0.0;

    static constexpr double k_maxTurretSpeed = 600.0*360.0;
    static constexpr double k_turretEncoderTicksToDegrees = 360.0/2048.0; //Figure out Gear ratio
    static constexpr double k_turretEncoderTicksToDPS = 12.0*(360.0/2048.0);
    static constexpr double k_turretRadius = 6; 
    static constexpr double k_turretTeeth = 229.0;
    static constexpr double k_turretDriveTeeth = 16.0;
    static constexpr double k_turretGearRatio = k_turretTeeth/k_turretDriveTeeth;
    static constexpr double k_turretABSMaxRotations = k_turretGearRatio*2;
    static constexpr double k_turretADCPerRotation = 4096/k_turretABSMaxRotations;
    static constexpr double k_turretABSDegreesPerShaftRotation = 720.0/k_turretABSMaxRotations;
    static constexpr double k_turretTicksPerRotation = 2048*k_turretGearRatio;
    


    //limelight TODO find corret values
    static constexpr double k_limeLightP = 4.9;
    static constexpr double k_limeLightI = 1;
    static constexpr double k_limeLightD = 0;
    static constexpr double k_limeLightIZone = 10;

    
}


#endif // ROBOT_PARAMETERS_H