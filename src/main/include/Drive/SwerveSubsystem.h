#include "Drive/SwerveDrive.h"

#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <units/units.h>
#include <iostream>
#include <frc/Joystick.h>
#include "ctre/Phoenix.h"
#include "AHRS.h"
#include <thread>
#include <chrono>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <cstdlib>
#include <frc/Timer.h>
#include <vector>
#include <list>
#include <frc/PWMTalonFx.h>
#include <frc/SerialPort.h>
#include <frc/kinematics/SwerveModuleState.h>

class SwerveSubsystem : public frc2::SubsystemBase {
    public: 

    SwerveSubsystem(SwerveDrive *swerve, frc::Joystick *joyOp);

    void Periodic() override; //runs regardless of state
    
    // void GoDistance(double distance, double angle);


    void StopAll();
    void StopDrive();
    void StopYaw();

    void SetModuleStates(std::array<frc::SwerveModuleState, 4> desired_states);

    void ZeroEncoders();

    // void SetAllTargetAngle(double angle);
    // void SetDriveTargetVelocity(double speed);

    // use _t after the desired unit or else it wont like you
    const units::meter_t x_offset = 11_in;
    const units::meter_t y_offset = 11.033_in;
    
    frc::Rotation2d GetHeading();

    frc::Translation2d m_front_left_location {-x_offset, y_offset}; // (x, y) -> (-11, -11.033)
    frc::Translation2d m_front_right_location {x_offset, y_offset}; 
    frc::Translation2d m_back_left_location {-x_offset, -y_offset}; 
    frc::Translation2d m_back_right_location {x_offset, -y_offset}; 

    const frc::SwerveDriveKinematics<4> m_kinematics {
        m_front_left_location, m_front_right_location, 
        m_back_left_location, m_back_right_location
    };
    frc::SwerveDriveOdometry<4> *m_odometry;
    AHRS *m_ahrs;

    

    double GetTurnRate();
    frc::Pose2d GetPose();
    void BuildOdometry(frc::Pose2d pose); 
 
    std::array<double, 4> GetCurrentPositions();

    private:
    
    double m_current_target_yaw_pos = 0;
    
    bool m_strafe = true;

    SwerveDrive *m_swerve;
    frc::Joystick *m_joystick;
};