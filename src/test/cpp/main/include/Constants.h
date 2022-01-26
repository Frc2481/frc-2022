/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/trajectory/TrapezoidProfile.h>
// #include <units/units.h>
#include <wpi/math>

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or bool constants.  This should not be used for any other purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace TalonIDs{
    // static constexpr int kFrontRightTurningMotorID = 3;//1
    // static constexpr int kFrontLeftTurningMotorID = 1;//4
    // static constexpr int kRearRightTurningMotorID = 4;//3
    // static constexpr int kRearLeftTurningMotorID = 2;//2
}

namespace FalconIDs{
    static constexpr int kFrontRightDriveMotorID = 3;//5
    static constexpr int kFrontLeftDriveMotorID = 4;//6
    // static constexpr int kRearRightDriveMotorID = 8;//7
    // static constexpr int kRearLeftDriveMotorID = 6;//8
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

constexpr bool kFrontLeftDriveEncoderReversed = true;
constexpr bool kRearLeftDriveEncoderReversed = true;
constexpr bool kFrontRightDriveEncoderReversed = true;
constexpr bool kRearRightDriveEncoderReversed = true;

constexpr bool kGyroReversed = true;

}  // namespace DriveConstants

namespace ModuleConstants {
constexpr int kEncoderCPR = 4096;
constexpr double kWheelDiameterMeters = .15;
constexpr double kDriveEncoderDistancePerPulse =
    // Assumes the encoders are directly mounted on the wheel shafts
    (kWheelDiameterMeters * wpi::math::pi) / static_cast<double>(kEncoderCPR);

constexpr double kTurningEncoderDistancePerPulse =
    // Assumes the encoders are directly mounted on the wheel shafts
    (wpi::math::pi * 2) / static_cast<double>(kEncoderCPR);

constexpr double kPModuleTurningController = 1;
constexpr double kPModuleDriveController = 1;
}  // namespace ModuleConstants

// namespace AutoConstants {
// using radians_per_second_squared_t =
//     units::compound_unit<units::radians,
//                          units::inverse<units::squared<units::second>>>;

// constexpr auto kMaxSpeed = units::meters_per_second_t(3);
// constexpr auto kMaxAcceleration = units::meters_per_second_squared_t(3);
// constexpr auto kMaxAngularSpeed = units::radians_per_second_t(3.142);
// constexpr auto kMaxAngularAcceleration =
//     units::unit_t<radians_per_second_squared_t>(3.142);

// constexpr double kPXController = 0.5;
// constexpr double kPYController = 0.5;
// constexpr double kPThetaController = 0.5;

// extern const frc::TrapezoidProfile<units::radians>::Constraints
//     kThetaControllerConstraints;

// }  // namespace AutoConstants

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