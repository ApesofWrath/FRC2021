#pragma once

#include "AHRS.h"
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/geometry/Pose2d.h>
#include <frc/PowerDistributionPanel.h>
#include <frc/SpeedController.h>
#include "../Drive/DriveBase.h"
#include "../Drive/DriveConstants.h"
#include <ctre/Phoenix.h>
#include <frc2/command/SubsystemBase.h>
#include "../Drive/SwerveDrive.h"

class AutonDrive : public frc2::SubsystemBase {
public:
    AutonDrive(SwerveDrive* swerve, AHRS* ahrs_);

    AHRS* ahrs;
    frc::PowerDistributionPanel *pdp;

    TalonFX *front_left_drive, *front_right_drive, *back_left_drive, *back_right_drive,
        *front_left_yaw, *front_right_yaw, *back_left_yaw, *back_right_yaw;

    // frc::SpeedControllerGroup *m_leftMotors; // do we still need this?
    // frc::SpeedControllerGroup *m_rightMotors;

    frc::DifferentialDriveOdometry *m_odometry;

    double GetHeading();

    void Periodic() override;

    double GetAverageEncoderDistance();

    frc::Pose2d GetPose();
    
    frc::DifferentialDriveWheelSpeeds GetWheelSpeeds();

    void TankDriveVolts(units::volt_t left, units::volt_t right);

    double percentFromVolts(units::volt_t volts);

    frc::DifferentialDrive* m_drive;

    void ResetOdometry(frc::Pose2d pose);
    double GetTurnRate();
    void SetMaxOutput(double maxOutput);
    void ResetEncoders();

};

// extern const frc::DifferentialDriveKinematics K_DRIVE_KINEMATICS;

const double K_RAMSETE_B = 2;
const double K_RAMSETE_ZETA = 0.7;