// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "components/TalonFXMotorController.h"
#include "components/VictorMotorController.h"


class ShooterSubsystem : public frc2::SubsystemBase {
 public:
  ShooterSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */


  bool isShooterOn();
  bool isShooterOnTarget();
  double getTopShooterSpeed();
  double getBottomShooterSpeed();
  void stopShooter();
  void startShooter(double speed);
  void toggleManualShooter();
  bool isInManual();
  void incrementManualSpeed();
  void decrementManualSpeed();
  void topMotorSetSpeed(double speed);
  void bottomMotorSetSpeed(double speed);

 void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  struct waypoint_t {
        double pos;
        double speed;
    };

  double m_setSpeedTopWheel;
  double m_setSpeedBottomWheel;
  bool m_isShooterOn;
  bool m_isOnTarget;
  double m_manualOffsetSpeed;
  bool m_isInManual;
   double m_distanceToTarget;
  TalonFXMotorController* m_pTopShooterMotor;
  TalonFXMotorController* m_pBottomShooterMotor;
  
  std::vector<double> m_bottomShooterSpeedsVect;
  std::vector<double> m_topShooterSpeedsVect;
  std::vector<double> m_distancesToTarget;



};
