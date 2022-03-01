// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeSubsystem.h"
#include "RobotParameters.h"

IntakeSubsystem::IntakeSubsystem() :
m_isIntakeExtended(false),
m_rollerSpeed(0),
 m_intakeSolenoid(0, frc::PneumaticsModuleType::CTREPCM, SolenoidPorts::kIntakeSolenoidPort, SolenoidPorts::kIntakeSolenoidReversePort)
{
    m_pRollerMotor = new TalonSRXMotorController(TalonIDs::kIntakeRollerMotorID, "RollerMotor");
    // m_pRollerMotor->SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_1_General, 200, 0); //TODO check to see if proper delay
}

void IntakeSubsystem::setRollerSpeed(double speed){
        m_rollerSpeed = speed;
        m_pRollerMotor->Set(speed);
}

void IntakeSubsystem::extendIntake(){
    m_intakeSolenoid.Set(m_intakeSolenoid.kForward);
    m_isIntakeExtended = true;
}

void IntakeSubsystem::retractIntake(){
    m_intakeSolenoid.Set(m_intakeSolenoid.kReverse);
    m_isIntakeExtended = false;
}

double IntakeSubsystem::getRollerSpeed(){
    return m_rollerSpeed;
}
double IntakeSubsystem::getCurrent(){
    return m_PDP.GetCurrent(PDPChannels::kIntake);
}
bool IntakeSubsystem::isIntakeExtended(){
    return m_isIntakeExtended;
}

// This method will be called once per scheduler run
void IntakeSubsystem::Periodic() {}