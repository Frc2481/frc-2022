/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#ifndef SRC_SWERVEDRIVETRAINSETOPENLOOP
#define SRC_SWERVEDRIVETRAINSETOPENLOOP

#include <frc/commands/Command.h>
#include "Subsystems/SwerveDrivetrain.h"
#include "CommandBase.h"

class SwerveDrivetrainSetOpenLoop : public frc::Command {
 private:
  double m_desiredTime;
  double m_xVel;
  double m_yVel;
  double m_yawRate;
  bool m_prevFieldCentric;

 public:
  SwerveDrivetrainSetOpenLoop(double desiredTime, double xVel, double yVel, double yawRate) : Command("SwerveDrivetrainSetOpenLoop"){
    Requires(CommandBase::m_pSwerveDrivetrain.get());
    m_desiredTime = desiredTime;
    m_xVel = xVel;
    m_yVel = yVel;
    m_yawRate = yawRate;
  }
  void Initialize() override {
    SetTimeout(m_desiredTime);
    m_prevFieldCentric = CommandBase::m_pSwerveDrivetrain->getFieldFrame();
    CommandBase::m_pSwerveDrivetrain->setIsOpenLoopFieldFrame(false);
  }
  void Execute() override {
    CommandBase::m_pSwerveDrivetrain->driveOpenLoopControl(m_xVel, m_yVel, m_yawRate);
  }
  bool IsFinished() override {
    return IsTimedOut();
  }
  void End() override {
    CommandBase::m_pSwerveDrivetrain->stop();
    CommandBase::m_pSwerveDrivetrain->setIsOpenLoopFieldFrame(m_prevFieldCentric);
  }
  void Interrupted() override {
    End();
  }
};

#endif //SRC_SWERVEDRIVETRAINSETOPENLOOP
