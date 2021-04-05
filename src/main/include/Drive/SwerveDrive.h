#pragma once
#include "Drive/DriveConstants.h"
#include <frc/Joystick.h>
#include <ctre/Phoenix.h>

#include "AHRS.h"

struct SwerveModule {
    int yawMotor, driveMotor;
};

struct SwerveModule_Falcon {
    TalonFX *yawMotor, *driveMotor;

    bool isReversed = false;

    SwerveModule_Falcon(SwerveModule module, PIDSettings drive, PIDSettings yaw);

    void Reverse();

    void ConfigPIDYaw(double p = 0, double i = 0, double d = 0);
    void ConfigPIDDrive(double p = 0, double i = 0, double d = 0);
    
};

class SwerveDrive {
private:
    SwerveModule_Falcon m_FrontLeft, m_FrontRight, m_BackLeft, m_BackRight;

    AHRS* m_AHRS;

    double m_CurrentTargetYawPosition = 0;

    bool m_Strafe = true;

public:

    SwerveDrive(SwerveModule frontLeft, SwerveModule frontRight, SwerveModule backLeft, SwerveModule backRight); // SwerveDrive({0, 4}, {1, 5}, {2, 6}, {3, 7});

    void Update(frc::Joystick* joy);

    void StopAll();
    void StopDrive();
    void StopYaw();

    void BasicPower(float drivePower, float yawPower);

    void ZeroEncoders();

    void SetAllTargetAngle(double angle);
    void SetDriveTargetVelocity(double speed);
    void SetDriveTargetDistance(double distance);

    inline double GetCurrentTargetYawPosition() { return m_CurrentTargetYawPosition; };

    void SetMovement(double speed, double movementDirection, double facing);

};