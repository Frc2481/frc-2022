#ifndef SWERVE_DRIVETRAIN_H
#define SWERVE_DRIVETRAIN_H

#include <frc/WPILib.h>
#include "ctre/Phoenix.h"
#include "AHRS.h"
#include "Components/GrayhillEncoder.h"
#include "Components/CTREMagEncoder.h"
#include "Utils/Pose2D.h"
#include "Utils/PoseDot2D.h"
#include "Utils/SwerveDrivePose.h"
#include "Utils/MotorPositionController.h"
#include "Utils/MotorVelocityController.h"
#include "rev/CANSparkMax.h"

// +x = robot right
// +y = robot forward
// +yaw = CCW, zero is robot forward

class SwerveDrivetrain : public frc::Subsystem {
public:
	SwerveDrivetrain();
    ~SwerveDrivetrain();
    virtual void InitDefaultCommand();
    virtual void Periodic();

    //////////////////////////////////////////////////////////////////////
    // @brief command robot to drive using open loop control
    // @param percentVelX - velocity x percent (-1 to 1)
    // @param percentVelY - velocity y percent (-1 to 1)
    // @param percentYawRate - yaw rate percent (-1 to 1)
    // @param isFieldFrame - field reference frame driving
    //////////////////////////////////////////////////////////////////////
    void driveOpenLoopControl(
        double percentVelX,
        double percentVelY,
        double percentYawRate);

    //////////////////////////////////////////////////////////////////////
    // @brief command robot to drive using closed loop control
    // @param robotVel - forward velocity of robot (in/s)
    // @param robotYawRate - yaw rate of robot (deg/s)
    // @param robotAccel - forward acceleration of robot (in/s^2)
    // @param robotYawAccel - yaw accel of robot (deg/s^2)
    //////////////////////////////////////////////////////////////////////
    void driveClosedLoopControl(
        double robotVelX,
		double robotVelY,
		double robotAccelX,
		double robotAccelY,
        double robotYawRate,
		double robotYawAccel);
    
    void stop();
    void zeroSteerEncoders();
    void zeroGyroYaw();
    void setIsOpenLoopFieldFrame(bool isOpenLoopFieldFrame);
    bool areAllSteerEncodersConnected();
    void setBrakeMode();
    double getGyroYaw();
    bool getFieldFrame();
    void setGyroOffset(double offSet);
    double getGyroOffset();

private:
    rev::CANSparkMax* m_pFRDriveMotor;
    rev::CANSparkMax* m_pBRDriveMotor;
    rev::CANSparkMax* m_pBLDriveMotor;
    rev::CANSparkMax* m_pFLDriveMotor;
    TalonSRX* m_pFRSteerMotor;
	TalonSRX* m_pBRSteerMotor;
	TalonSRX* m_pBLSteerMotor;
	TalonSRX* m_pFLSteerMotor;
	MotorPositionController* m_pFRSteerMotorController;
	MotorPositionController* m_pBRSteerMotorController;
	MotorPositionController* m_pBLSteerMotorController;
	MotorPositionController* m_pFLSteerMotorController;
	CTREMagEncoder* m_pFRSteerEncoder;
	CTREMagEncoder* m_pBRSteerEncoder;
	CTREMagEncoder* m_pBLSteerEncoder;
	CTREMagEncoder* m_pFLSteerEncoder;
    AHRS* m_pChassisIMU;
    SwerveDriveKinematics m_kinematics;
    double m_gyroYaw;
    bool m_isOpenLoopFieldFrame;
    bool m_areAllSteerEncodersConnected;
    double m_gyroOffset;
};

#endif // SWERVE_DRIVETRAIN_H
