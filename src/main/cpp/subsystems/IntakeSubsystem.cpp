// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeSubsystem.h"
#include "Constants.h"

IntakeSubsystem::IntakeSubsystem() :
m_isRollerOn(false),
m_isIntakeExtended(false),
m_rollerSpeed(0),
m_intakeSolenoid(frc::PneumaticsModuleType::CTREPCM, SolenoidPorts::kIntakeSolenoidPort,SolenoidPorts::kIntakeSolenoidReversePort)
{
m_rollerMotor = new VictorMotorController(VictorIDs::kIntakeRollerMotorID, "RollerMotor");
}

void IntakeSubsystem::extendIntake(){
    m_intakeSolenoid.Set(m_intakeSolenoid.kForward);
    m_isIntakeExtended = true;
}

void IntakeSubsystem::startRoller(){
    m_rollerMotor->Set(m_rollerSpeed);
    m_isRollerOn = true;
}

void IntakeSubsystem::retractIntake(){
    m_intakeSolenoid.Set(m_intakeSolenoid.kReverse);
    m_isIntakeExtended = false;
}

void IntakeSubsystem::stopRoller(){
    m_rollerMotor->Set(0);
    m_isRollerOn = false;
}

bool IntakeSubsystem::isRollerOn(){
    return m_isRollerOn;
}

bool IntakeSubsystem::isIntakeExtended(){
    return m_isIntakeExtended;
}

// This method will be called once per scheduler run
void IntakeSubsystem::Periodic() {}