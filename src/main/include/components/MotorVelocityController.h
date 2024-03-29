#ifndef MOTOR_VELOCITY_CONTROLLER_H
#define MOTOR_VELOCITY_CONTROLLER_H

#include "components/CommonMotorController.h"

class MotorVelocityController {
public:
    MotorVelocityController();
    MotorVelocityController(
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
        unsigned ticksPerRev);
    ~MotorVelocityController();

    void setTicksPerRev(unsigned ticksPerRev);

    //////////////////////////////////////////////////////////////////////
    // @brief update reference points of closed loop motor controller
    // @param refV - angular velocity reference point (deg/s)
    // @param refA - angular acceleration reference point (deg/s^2)
    //////////////////////////////////////////////////////////////////////
    void updateClosedLoopControl(double refV, double refA);

    //////////////////////////////////////////////////////////////////////
    // @brief update reference points of open loop motor controller
    // @param refPercent - percent reference point (-1 to 1)
    //////////////////////////////////////////////////////////////////////
    void updateOpenLoopControl(double refPercent);

private:
    CommonMotorController* m_pDriveMotor;
    double m_kv;
    double m_kap;
    double m_kan;
    double m_ksf;
    unsigned m_ticksPerRev;
};

#endif // MOTOR_VELOCITY_CONTROLLER_H
