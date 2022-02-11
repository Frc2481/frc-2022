// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "components/Joystick2481.h"
#include "subsystems/TurretSubsystem.h"
#include "components/TalonFXMotorController.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class ControlMotorWithJoystickCommand
    : public frc2::CommandHelper<frc2::CommandBase, ControlMotorWithJoystickCommand> {
      private: 
      Joystick2481* m_controller;
      int m_motorID;
      TalonFXMotorController* m_motor;
 public:
  ControlMotorWithJoystickCommand(Joystick2481* controller, int motorID) //Add whatever subsystem we want to control
  
  {
    m_motorID = motorID;
    m_controller = controller;
    m_motor = new TalonFXMotorController(m_motorID, "Motor");
        m_motor->ConfigFactoryDefault();
        m_motor->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
        // m_turretMotor->SetCon
      //   m_motor->ConfigMotionCruiseVelocity((RobotParameters::k_maxTurretSpeed)/RobotParameters::k_turretEncoderTicksToDPS);  //Degrees per second
      //   m_motor->ConfigMotionAcceleration(((RobotParameters::k_maxTurretSpeed)/RobotParameters::k_turretEncoderTicksToDPS)*2);
      //   m_motor->Config_kP(0, RobotParameters::k_shooterP); //do we need PIDF for turret motor?
      //  m_motor->Config_kI(0,RobotParameters::k_shooterI);
      //  m_motor->Config_kD(0,RobotParameters::k_shooterD);
      //  m_motor->Config_kF(0,RobotParameters::k_shooterF);
      //  m_motor->Config_IntegralZone(0,25); 
      
  }
  void Initialize() override
  {
    
  }

  void Execute() override
  {
    double yleftHand = m_controller->GetRawAxis(1);
        if(fabs(yleftHand) <=0.075){
          yleftHand = 0.0;
        }
        m_motor->Set(yleftHand);

        
  }

  void End(bool interrupted) override
  {
    m_motor->Set(0.0);
  }

  bool IsFinished() override
  {
    return false;
  }
};
