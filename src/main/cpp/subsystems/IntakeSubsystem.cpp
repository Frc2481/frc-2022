// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeSubsystem.h"
#include "Constants.h"

IntakeSubsystem::IntakeSubsystem() :
m_isIntakeExtended(false),
m_rollerSpeed(0),
m_intakeSolenoid(frc::PneumaticsModuleType::CTREPCM, SolenoidPorts::kIntakeSolenoidPort,SolenoidPorts::kIntakeSolenoidReversePort)
{
    m_rollerMotor = new VictorMotorController(VictorIDs::kIntakeRollerMotorID, "RollerMotor");
}

void IntakeSubsystem::setRollerSpeed(double speed){
        m_rollerSpeed = speed;
        m_rollerMotor->Set(speed);
}

void IntakeSubsystem::extendIntake(){
    m_intakeSolenoid.Set(m_intakeSolenoid.kForward);
    m_isIntakeExtended = true;
}

void IntakeSubsystem::retractIntake(){
    m_rollerMotor->Set(0);
    m_intakeSolenoid.Set(m_intakeSolenoid.kReverse);
    m_isIntakeExtended = false;
}

double IntakeSubsystem::getRollerSpeed(){
    return m_rollerSpeed;
}

bool IntakeSubsystem::isIntakeExtended(){
    return m_isIntakeExtended;
}

// This method will be called once per scheduler run
void IntakeSubsystem::Periodic() {}