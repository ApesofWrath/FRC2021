#include "Drive/SwerveSubsystem.h"
#include "Drive/SwerveDrive.h"

SwerveSubsystem::SwerveSubsystem(SwerveDrive *swerve, frc::Joystick *joyOp){
    m_swerve = swerve;
    m_joystick = joyOp;
}

void SwerveSubsystem::Periodic() {
    //put update function?
}

void SwerveSubsystem::GoDistance(double distance, double angle){
    m_swerve->SetAllTargetAngle(angle);
}
// void SwerveSubsystem
// 7.39:1 drive 1 rot of wheel
// 4pi inches for one rot of wheel 