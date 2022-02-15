/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <ctre/Phoenix.h>
#include <rev/CANSparkMax.h>
#include <frc/Encoder.h>
// #include <frc/Spark.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <wpi/numbers>

#include "RobotParameters.h"
#include "components/CTRECANEncoder.h"
#include "components/MotorPositionController.h"
#include "components/CommonMotorController.h"
#include "components/VictorMotorController.h"
#include "components/TalonFXMotorController.h"
#include "components/SparkMaxMotorController.h"

class SwerveModule {
  using radians_per_second_squared_t =
      units::compound_unit<units::radians,
                           units::inverse<units::squared<units::second>>>;

 public:
  SwerveModule(int driveMotorID, int turningMotorID, int turnEncoderID,
               bool driveEncoderReversed, bool turningEncoderReversed, const std::string &name);

  frc::SwerveModuleState GetState();

  void SetDesiredState(frc::SwerveModuleState& state, bool percentMode);

  void ResetEncoders();
    void updateSteerPID(double p, double i, double d);
    void updateDrivePID(double p, double i, double d, double f);
    void setCoast();
    void setBrake();
    void DriveArc(double arcLength, double wheelAngle);
 private:
  // We have to use meters here instead of radians due to the fact that
  // ProfiledPIDController's constraints only take in meters per second and
  // meters per second squared.

  static constexpr units::radians_per_second_t kModuleMaxAngularVelocity =
      units::radians_per_second_t(3.0 * 2.0*wpi::numbers::pi);  // radians per second
  static constexpr units::unit_t<radians_per_second_squared_t>
      kModuleMaxAngularAcceleration =
          units::unit_t<radians_per_second_squared_t>(
              wpi::numbers::pi * 2.0 * 12.0);  // radians per second squared

  TalonFXMotorController* m_pDriveMotor;
  VictorMotorController* m_pTurningMotor;
  CTRECANEncoder* m_pTurningEncoder;
  MotorPositionController* m_pTurningMotorController;

  bool m_reverseDriveEncoder;
  bool m_reverseTurningEncoder;
  std::string m_name;

  frc2::PIDController m_drivePIDController{
      0.0, 0.0, 0.0};
  frc::ProfiledPIDController<units::radians> m_turningPIDController{
      1.0,
      0.0,
      0.05,
      {kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration}};
      
    
};
