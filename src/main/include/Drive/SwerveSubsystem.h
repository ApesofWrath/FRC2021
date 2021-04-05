#include "Drive/SwerveDrive.h"

#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>

class SwerveSubsystem : public frc2::SubsystemBase {
    public: 

    SwerveSubsystem(SwerveDrive *swerve, frc::Joystick *joyOp);

    void Periodic() override; //runs regardless of state
    
    void GoDistance(double distance, double angle);


    void StopAll();
    void StopDrive();
    void StopYaw();

    void ZeroEncoders();

    void SetAllTargetAngle(double angle);
    void SetDriveTargetVelocity(double speed);

    inline double GetCurrentTargetYawPosition() {return m_current_target_yaw_pos; };

    private:
    
    double m_current_target_yaw_pos = 0;
    
    bool m_strafe = true;

    SwerveDrive *m_swerve;
    frc::Joystick *m_joystick;
};