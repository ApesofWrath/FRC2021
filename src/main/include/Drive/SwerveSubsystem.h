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
    AHRS *m_ahrs;
    frc::SwerveDriveOdometry<4> *m_odometry;

    double x_offset = -11 * INCHES_TO_METER;
    const units::meter x_offset = -11_m * INCHES_TO_METER;

    frc::Translation2d m_frontLeftLocation{-0.2794_m, 0.381_m}; // (x, y) -> (-11, -11.033)

    double GetHeading();
    double GetTurnRate();
    frc::Pose2d GetPose(); 
 
    std::array<double, 4> GetCurrentPositions();

    private:
    
    double m_current_target_yaw_pos = 0;
    
    bool m_strafe = true;

    SwerveDrive *m_swerve;
    frc::Joystick *m_joystick;
};