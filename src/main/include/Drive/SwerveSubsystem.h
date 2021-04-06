#include "Drive/SwerveDrive.h"

#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <units/units.h>

class SwerveSubsystem : public frc2::SubsystemBase {
    public: 

    SwerveSubsystem(SwerveDrive *swerve, frc::Joystick *joyOp, AHRS *ahrs);

    void Periodic() override; //runs regardless of state
    
    void GoDistance(double distance, double angle);


    void StopAll();
    void StopDrive();
    void StopYaw();

    void ZeroEncoders();

    void SetAllTargetAngle(double angle);
    void SetDriveTargetVelocity(double speed);

    // use _t after the desired unit or else it wont like you
    const units::meter_t x_offset = 11_in;
    const units::meter_t y_offset = 11.033_in;
    
    frc::Rotation2d GetHeading() {
        return frc::Rotation2d((units::degree_t) (std::remainder(-m_ahrs->GetAngle(), 360)));
    };

    frc::Translation2d m_front_left_location {-x_offset, y_offset}; // (x, y) -> (-11, -11.033)
    frc::Translation2d m_front_right_location {x_offset, y_offset}; 
    frc::Translation2d m_back_left_location {-x_offset, -y_offset}; 
    frc::Translation2d m_back_right_location {x_offset, -y_offset}; 

    frc::SwerveDriveKinematics<4> *m_kinematics;
    frc::SwerveDriveOdometry<4> *m_odometry {m_kinematics, GetHeading(), frc::Pose2d()};
    AHRS *m_ahrs;

    frc::Pose2d m_this_is_pose;

    double GetTurnRate();
    frc::Pose2d GetPose(); 
 
    std::array<double, 4> GetCurrentPositions();

    private:
    
    double m_current_target_yaw_pos = 0;
    
    bool m_strafe = true;

    SwerveDrive *m_swerve;
    frc::Joystick *m_joystick;
};