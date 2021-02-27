#include "Drive/SwerveDrive.h"
#include <cmath>

#include <frc/smartdashboard/SmartDashboard.h>

SwerveModule_Falcon::SwerveModule_Falcon(SwerveModule module, PIDSettings drive, PIDSettings yaw) {
    driveMotor = new TalonFX(module.driveMotor);
    driveMotor->Config_kP(0, drive.kP);
    driveMotor->Config_kI(0, drive.kI);
    driveMotor->Config_kD(0, drive.kD);
    // driveMotor->Config_kF(0, 1);

    yawMotor = new TalonFX(module.yawMotor);
    yawMotor->Config_kP(0, yaw.kP);
    yawMotor->Config_kI(0, yaw.kI);
    yawMotor->Config_kD(0, yaw.kD);
}

void SwerveModule_Falcon::Reverse() {
    isReversed = true;

    yawMotor->SetInverted(true);
    driveMotor->SetInverted(true);
}

SwerveDrive::SwerveDrive(SwerveModule frontLeft, SwerveModule frontRight, SwerveModule backLeft, SwerveModule backRight) : 
            m_FrontLeft(frontLeft, SWERVE_CONFIG_PID.kFrontLeftDrive, SWERVE_CONFIG_PID.kFrontLeftYaw),
            m_FrontRight(frontRight, SWERVE_CONFIG_PID.kFrontRightDrive, SWERVE_CONFIG_PID.kFrontRightYaw),
            m_BackLeft(backLeft, SWERVE_CONFIG_PID.kBackLeftDrive, SWERVE_CONFIG_PID.kBackLeftYaw),
            m_BackRight(backRight, SWERVE_CONFIG_PID.kBackRightDrive, SWERVE_CONFIG_PID.kBackRightYaw) {
    // m_FrontLeft.Reverse();


    m_AHRS = new AHRS(frc::SerialPort::kUSB);
}

void SwerveDrive::Update(frc::Joystick* joy) {
    // BasicPower(joy->GetY() * (float)joy->GetThrottle(), joy->GetX() * (float)joy->GetThrottle());
    double x = joy->GetX();
    double y = -joy->GetY();

    double angle = std::atan2(y, x) + PI / 2.0;
    if (angle < -PI / 2.0) {
        angle = PI - angle;
    }

    // angle = PI / 2.0;
    double mag = std::abs(std::sqrt(x * x + y * y));
    SetAllTargetAngle(angle);
    SetDriveTargetVelocity(mag);
    frc::SmartDashboard::PutNumber("Target Strafe Angle", angle);
    frc::SmartDashboard::PutNumber("Target Speed", mag);
    frc::SmartDashboard::PutNumber("Joystick X", x);
    frc::SmartDashboard::PutNumber("Joystick Y", y);

}

void SwerveDrive::StopAll() {
    StopDrive();
    StopYaw();
}

void SwerveDrive::StopYaw() {
    m_FrontLeft.yawMotor->Set(ControlMode::PercentOutput, 0);
    m_FrontRight.yawMotor->Set(ControlMode::PercentOutput, 0);
    m_BackLeft.yawMotor->Set(ControlMode::PercentOutput, 0);
    m_BackRight.yawMotor->Set(ControlMode::PercentOutput, 0);
}

void SwerveDrive::StopDrive() {
    m_FrontLeft.driveMotor->Set(ControlMode::PercentOutput, 0);
    m_FrontRight.driveMotor->Set(ControlMode::PercentOutput, 0);
    m_BackLeft.driveMotor->Set(ControlMode::PercentOutput, 0);
    m_BackRight.driveMotor->Set(ControlMode::PercentOutput, 0);    
}

void SwerveDrive::BasicPower(float drivePower, float yawPower) {
    m_FrontLeft.driveMotor->Set(ControlMode::PercentOutput, drivePower);
    m_FrontRight.driveMotor->Set(ControlMode::PercentOutput, drivePower);
    m_BackLeft.driveMotor->Set(ControlMode::PercentOutput, drivePower);
    m_BackRight.driveMotor->Set(ControlMode::PercentOutput, drivePower);

    m_FrontLeft.yawMotor->Set(ControlMode::PercentOutput, 0);
    m_FrontRight.yawMotor->Set(ControlMode::PercentOutput, 0);
    m_BackLeft.yawMotor->Set(ControlMode::PercentOutput, 0);
    m_BackRight.yawMotor->Set(ControlMode::PercentOutput, 0);
}

void SwerveDrive::ZeroEncoders() {
    m_FrontLeft.driveMotor->SetSelectedSensorPosition(0);
    m_FrontRight.driveMotor->SetSelectedSensorPosition(0);
    m_BackLeft.driveMotor->SetSelectedSensorPosition(0);
    m_BackRight.driveMotor->SetSelectedSensorPosition(0);

    m_FrontLeft.yawMotor->SetSelectedSensorPosition(0);
    m_FrontRight.yawMotor->SetSelectedSensorPosition(0);
    m_BackLeft.yawMotor->SetSelectedSensorPosition(0);
    m_BackRight.yawMotor->SetSelectedSensorPosition(0);
}

void SwerveDrive::SetAllTargetAngle(double angle) {
    double targetPosition = angle * ANGLE_TO_TICKS_SWERVE_YAW;

    m_FrontLeft.yawMotor->Set(ControlMode::Position, targetPosition);
    m_FrontRight.yawMotor->Set(ControlMode::Position, targetPosition);
    m_BackLeft.yawMotor->Set(ControlMode::Position, targetPosition);
    m_BackRight.yawMotor->Set(ControlMode::Position, targetPosition);
}

void SwerveDrive::SetDriveTargetVelocity(double speed) {
    double targetPosition = speed * TICKS_PER_ROTATION_FALCON * FALCON_UPDATE_PER_SECOND;

    m_FrontLeft.driveMotor->Set(ControlMode::Velocity, targetPosition);
    m_FrontRight.driveMotor->Set(ControlMode::Velocity, targetPosition);
    m_BackLeft.driveMotor->Set(ControlMode::Velocity, targetPosition);
    m_BackRight.driveMotor->Set(ControlMode::Velocity, targetPosition);

}