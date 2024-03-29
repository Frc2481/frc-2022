#include "components/MotorVelocityController.h"
// #include "frc/WPILib.h"
#include "RobotParameters.h"
#include "Utils/Sign.h"

MotorVelocityController::MotorVelocityController()
	: m_pDriveMotor(nullptr),
	m_kv(0),
	m_kap(0),
	m_kan(0),
	m_ksf(0),
	m_ticksPerRev(0) {
}

MotorVelocityController::MotorVelocityController(
    CommonMotorController* pController,
    bool phase,
	bool inverted,
    double kp,
    double ki,
    double kd,
    double kv,
    double kap,
	double kan,
	double ksf,
    double iZone,
    double iErrorLim,
    unsigned ticksPerRev)
    
    : m_pDriveMotor(pController),
    m_kv(kv),
    m_kap(kap),
	m_kan(kan),
	m_ksf(ksf),
    m_ticksPerRev(ticksPerRev) {

    m_pDriveMotor->SelectProfileSlot(0, 0);
	m_pDriveMotor->Set(CommonModes::PercentOutput, 0);
	m_pDriveMotor->Config_kP(0, kp, 10);
	m_pDriveMotor->Config_kI(0, ki, 10);
	m_pDriveMotor->Config_kD(0, kd, 10);
	m_pDriveMotor->Config_kF(0, 0, 10);
    m_pDriveMotor->Config_IntegralZone(0, iZone, 10);
    m_pDriveMotor->ConfigMaxIntegralAccumulator (0, iErrorLim, 10);
    m_pDriveMotor->SetNeutralMode(NeutralMode::Brake);
    m_pDriveMotor->EnableVoltageCompensation(true);
    m_pDriveMotor->ConfigVoltageCompSaturation(12, 10);
    m_pDriveMotor->ConfigNeutralDeadband(0.04, 10);
    m_pDriveMotor->ConfigNominalOutputForward(0, 10);
	m_pDriveMotor->ConfigNominalOutputReverse(0, 10);
	m_pDriveMotor->ConfigPeakOutputForward(1, 10);
	m_pDriveMotor->ConfigPeakOutputReverse(-1, 10);
	m_pDriveMotor->SetSensorPhase(phase);
	m_pDriveMotor->SetInverted(inverted);
}

MotorVelocityController::~MotorVelocityController() {
}

void MotorVelocityController::setTicksPerRev(unsigned ticksPerRev) {
	m_ticksPerRev = ticksPerRev;
}

void MotorVelocityController::updateClosedLoopControl(double refV, double refA) {
    refV *= m_ticksPerRev / 360.0 / 10.0; // convert to talon native units
    refA *= m_ticksPerRev / 360.0 / 10.0; // convert to talon native units

    // use different ka if vel and accel have opposite direction
    double ka = m_kap;
	if((refV > 0) != (refA > 0)) {
		ka = m_kan;
	}

    double feedforwardControl = refV * m_kv + refA * ka + Sign::Sign(refV) * m_ksf;

    m_pDriveMotor->Set(CommonModes::Velocity, refV, DemandType::DemandType_ArbitraryFeedForward, feedforwardControl);
}

void MotorVelocityController::updateOpenLoopControl(double refPercent) {
    m_pDriveMotor->Set(CommonModes::PercentOutput, refPercent);
}
