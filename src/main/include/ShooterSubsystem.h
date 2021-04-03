#pragma once

#include "Shooter.h"

#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>
#include <rev/CANSparkMax.h>

class ShooterSubsystem : public frc2::SubsystemBase {
    public: 
    
    ShooterSubsystem(Shooter *shooter, frc::Joystick *joyOp);

    void Periodic() override;

    void ShootLow();
    void ShootMedium();
    void ShootHigh();
    void StopMotors();

    Shooter *m_shooter;
    frc::Joystick *m_joystick;

    double top_max_speed = 0;
    double bottom_max_speed = 0;
    double belt_max_speed = 0;
    
    int m_periodic_counter = 0;

    private:
    

};