/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/trajectory/TrapezoidProfile.h>
//#include <units/units.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <wpi/numbers>

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or bool constants.  This should not be used for any other purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace VictorIDs{
    static constexpr int kIntakeRollerMotorID = 11;
    static constexpr int kIndexerRollerMotorID = 12;
    static constexpr int kFrontRightTurningMotorID = 3;//1
    static constexpr int kFrontLeftTurningMotorID = 1;//4
    static constexpr int kRearRightTurningMotorID = 4;//3
    static constexpr int kRearLeftTurningMotorID = 2;//2
    static constexpr int kRearMiddleTurningMotorID = 2;//TODO figure me out
    static constexpr int kFeederMotorID = 10; //TODO figure me out
}

namespace FalconIDs{
    static constexpr int kFrontRightDriveMotorID = 3;//5
    static constexpr int kFrontLeftDriveMotorID = 4;//6
    static constexpr int kRearRightDriveMotorID = 8;//7
    static constexpr int kRearLeftDriveMotorID = 6;//8
    static constexpr int kRearMiddleDriveMotorID = 99;//TODO figure me out
    static constexpr int kTopShooterMotorID = 25; //TODO figure me out
    static constexpr int kBottomShooterMotorID = 50; //TODO figure me out
    static constexpr int kturretMotorID = 3; 
    
} 

namespace CANCoderIDs
{
    static constexpr int kFrontRightSteerCANCoderID = 1;
    static constexpr int kFrontLeftSteerCANCoderID = 2;
    static constexpr int kRearRightSteerCANCoderID = 3;
    static constexpr int kRearLeftSteerCANCoderID = 4;
    static constexpr int kRearMiddleSteerCANCoderID = 5;
}

namespace SparkMaxIDs{
    static constexpr int kDumbMotorID = 1;
    static constexpr int kSteerMotorID = 2;
}

namespace SolenoidPorts{
    static constexpr int kManipulatorSolenoidPort = 7;
    static constexpr int kManipulatorSolenoidReversePort = 6;
    static constexpr int kIntakeSolenoidPort = 0;
    static constexpr int kIntakeSolenoidReversePort = 1;
    static constexpr int kShooterSolenoidPort = 2;
    static constexpr int kShooterSolenoidReversePort = 3;
}

namespace DriveConstants {

constexpr bool kFrontLeftTurningEncoderReversed = false;
constexpr bool kRearLeftTurningEncoderReversed = false;
constexpr bool kFrontRightTurningEncoderReversed = false;
constexpr bool kRearRightTurningEncoderReversed = false;
constexpr bool kRearMiddleTurningEncoderReversed = false;

constexpr bool kFrontLeftDriveEncoderReversed = true;
constexpr bool kRearLeftDriveEncoderReversed = true;
constexpr bool kFrontRightDriveEncoderReversed = true;
constexpr bool kRearRightDriveEncoderReversed = true;
constexpr bool kRearMiddleDriveEncoderReversed = true;

constexpr bool kGyroReversed = true;

}  // namespace DriveConstants

namespace ModuleConstants {
constexpr int kEncoderCPR = 4096;
constexpr double kWheelDiameterMeters = .15;
constexpr double kDriveEncoderDistancePerPulse =
    // Assumes the encoders are directly mounted on the wheel shafts
    (kWheelDiameterMeters * wpi::numbers::pi) / static_cast<double>(kEncoderCPR);

constexpr double kTurningEncoderDistancePerPulse =
    // Assumes the encoders are directly mounted on the wheel shafts
    (wpi::numbers::pi * 2) / static_cast<double>(kEncoderCPR);

constexpr double kPModuleTurningController = 1;
constexpr double kPModuleDriveController = 1;
}  // namespace ModuleConstants

namespace AutoConstants {
using radians_per_second_squared_t =
    units::compound_unit<units::radians,
                         units::inverse<units::squared<units::second>>>;

constexpr auto kMaxSpeed = units::meters_per_second_t(3);
constexpr auto kMaxAcceleration = units::meters_per_second_squared_t(3);
constexpr auto kMaxAngularSpeed = units::radians_per_second_t(3.142);
constexpr auto kMaxAngularAcceleration =
    units::unit_t<radians_per_second_squared_t>(3.142);

constexpr double kPXController = 0.5;
constexpr double kPYController = 0.5;
constexpr double kPThetaController = 0.5;

extern const frc::TrapezoidProfile<units::radians>::Constraints
    kThetaControllerConstraints;

}  // namespace AutoConstants

namespace OIConstants {
constexpr int kDriverControllerPort = 0;
}  // namespace OIConstants


enum class CommonModes{
    DutyCycle = 0,
    Velocity = 1,
    Voltage = 2,
    Position = 3,
    SmartMotion = 4,
    Current = 5,
    SmartVelocity = 6,
    PercentOutput = 7,
    Follower = 8,
    MotionProfile = 9,
    MotionMagic = 10,
    MotionProfileArc = 11,
    Disabled = 15
};
namespace LimeLightConstants{ // TODO check values
    static constexpr double kLimeLightHeight = 43/12.0;//ft
    static constexpr double kLimeLightAngle = 20;//deg
    static constexpr double kLimeLightHardWarePanAngle = 12;//deg
    static constexpr double kTargetHeight =  8.1875;//ft
}
namespace PathConstants{ // TODO check
    static constexpr double kMinLookAhead = 6*.0254;
    static constexpr double kMaxLookAhead = 24*.0254;


}
namespace IntakeConstants{
    static constexpr double kDefaultIntakeRollerSpeed = 0; //TODO: Find out
}
namespace FeederConstants{ //TODO figure out
    static constexpr double kPrimeShooterSpeed = 0.0;
    static constexpr double kShooterSpeed = 0.0;
}

namespace DigitalInputs{
    static constexpr int kFeederBeamBreakPort = 0;
}

namespace ShooterConstants{ //TODO wheel speeds
    static constexpr double kDonutTopWheelSpeed = 0; 
    static constexpr double kDonutBottomWheelSpeed = 0; 


}

namespace LimelightConstants{
    static constexpr double kTargetHeight = 103; //inches
    static constexpr double kLimelightHeight = 37; //TODO find actual height
    static constexpr double kLimelightAngle = 45; //TODO find angle
    
}

