#include "Drive/SwerveSubsystem.h"
#include "Drive/SwerveDrive.h"

SwerveSubsystem::SwerveSubsystem(SwerveDrive *swerve, frc::Joystick *joyOp){
    m_swerve = swerve;
    m_joystick = joyOp;
    m_ahrs = new AHRS(frc::SerialPort::kUSB);
    
    // m_kinematics = new frc::SwerveDriveKinematics<4>(
    //     m_front_left_location, m_front_right_location,
    //     m_back_left_location, m_back_right_location);

    frc::Pose2d this_is_pose = frc::Pose2d(0_m, 0_m, 0_rad);
    frc::Rotation2d this_is_rotation = GetHeading(); 

    m_odometry = new frc::SwerveDriveOdometry<4>(m_kinematics, this_is_rotation, this_is_pose); //, frc::Pose2d(0_m, 0_m, 0_rad)
    
}

void SwerveSubsystem::SetModuleStates(std::array<frc::SwerveModuleState, 4> desired_states) {
    
}

void SwerveSubsystem::Periodic() {
    //put update function?
}

frc::Rotation2d SwerveSubsystem::GetHeading() {
    return frc::Rotation2d((units::degree_t)std::remainder(-m_ahrs->GetAngle(), 360));
}

// void SwerveSubsystem::GoDistance(double distance, double angle){ // should be in meters
//     m_swerve->SetAllTargetAngle(angle);
//     m_swerve->SetDriveTargetVelocity(distance);
    
// }

std::array<double, 4> SwerveSubsystem::GetCurrentPositions(){
    
    return m_swerve->GetAllDrivePositions();
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
void SwerveSubsystem::BuildOdometry(frc::Pose2d pose) {
  m_ahrs->Reset();
  m_ahrs->ZeroYaw();
  m_odometry->ResetPosition(pose,
                           GetHeading());
}
