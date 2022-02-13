/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "Subsystems/DriveSubsystem.h"
#include "RobotParameters.h"
#include "Utils/NormalizeToRange.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/Timer.h>
#include "networktables/NetworkTableInstance.h"
#include "Utils/NormalizeToRange.h"
/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class RotateWithMotionMagic
    : public frc2::CommandHelper<frc2::CommandBase, RotateWithMotionMagic> {
 private:
  DriveSubsystem *m_pSwerveDrive;
  double m_targetAngle;
  double m_turnInput;
  double m_targetZone;
  double m_count;
  double m_targetCounter = 5;
  bool m_limeLight;
  double m_timeOut;
  frc::Timer m_timer;
 public:
  RotateWithMotionMagic(DriveSubsystem* swerveDrive, double targetAngle, double targetZone, bool limeLight = false, double timeOut = -1){
    m_pSwerveDrive = swerveDrive;
    m_targetAngle = targetAngle;
    m_targetZone = targetZone;
    m_limeLight = limeLight;
    m_timeOut = timeOut;
    AddRequirements(m_pSwerveDrive);
    frc::SmartDashboard::PutNumber("scale motion error", 1);
    
  }

  void Initialize() override{
    if(m_limeLight){
      m_targetAngle = 0;
    }
    m_count = 0;
    m_timer.Reset();
    m_timer.Start();
    // m_targetAngle = normalizeToRange::NormalizeToRange(m_targetAngle, -180,180,true);
  };

  void Execute() override{
    if(m_limeLight){
      m_turnInput = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx",0);
    }else{
      m_turnInput = m_pSwerveDrive->GetPose().Rotation().Degrees().to<double>();
    }
    
    double diff = normalizeToRange::RangedDifference(m_targetAngle-m_turnInput, -180,180)*.7;//frc::SmartDashboard::GetNumber("scale motion error", 1)
    m_pSwerveDrive->DriveArc(RobotParameters::k_wheelLeverArm * diff * MATH_CONSTANTS_PI/180);
    if(fabs(diff) < m_targetZone){
      m_count++;
    }else{
      m_count = 0;
    }
    printf("RotateWithMotionMagicTimedOut: %d\n", (m_timer.Get() >= m_timeOut && m_timeOut != -1));
    printf("RotateWithMotionMagicTime: %f\n", m_timer.Get());
  }

  void End(bool interrupted) override{
    printf("RotateWithMotionMagic: %f\n",m_timer.Get());
    
    m_timer.Stop();
    m_pSwerveDrive->Drive(0_mps,0_mps,0_rpm,false);
  }

  bool IsFinished() override{
    return m_targetCounter <= m_count || (m_timer.Get() >= m_timeOut && m_timeOut != -1);
  }
};
