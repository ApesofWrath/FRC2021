#include "Drive/SwerveSubsystem.h"
#include "Drive/SwerveDrive.h"

SwerveSubsystem::SwerveSubsystem(SwerveDrive *swerve, frc::Joystick *joyOp, AHRS *ahrs){
    m_swerve = swerve;
    m_joystick = joyOp;
    m_ahrs = ahrs;
    
    m_kinematics = new frc::SwerveDriveKinematics<4>(
        m_front_left_location, m_front_right_location,
        m_back_left_location, m_back_right_location);

    // m_odometry = new frc::SwerveDriveOdometry<4>(m_kinematics, GetHeading(), frc::Pose2d()); //, frc::Pose2d(0_m, 0_m, 0_rad)
    
}

void SwerveSubsystem::Periodic() {
    //put update function?
}

// double SwerveSubsystem::GetHeading() {
//     return std::remainder(-m_ahrs->GetAngle(), 360);
// }

void SwerveSubsystem::GoDistance(double distance, double angle){ // should be in meters
    m_swerve->SetAllTargetAngle(angle);
    m_swerve->SetDriveTargetVelocity(distance);
    
}

std::array<double, 4> SwerveSubsystem::GetCurrentPositions(){
    
    return m_swerve->GetAllDrivePositions();
}

double SwerveSubsystem::GetHeading() {
    // std::cout << "gh\n";
    return std::remainder(-m_ahrs->GetAngle(), 360);
}

double SwerveSubsystem::GetTurnRate() {
    return m_ahrs->GetRate();
}

frc::Pose2d SwerveSubsystem::GetPose() { 
    return m_odometry->GetPose();
}
// void SwerveSubsystem
// 7.39:1 drive 1 rot of wheel
// 4pi inches for one rot of wheel 