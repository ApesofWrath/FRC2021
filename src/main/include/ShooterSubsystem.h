#pragma once

#include "Shooter.h"

#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>
#include <rev/CANSparkMax.h>

class ShooterSubsystem : public frc2::SubsystemBase {
    public: 
    
    ShooterSubsystem(Shooter *shooter);

    void ShootLow();
    void ShootMedium();
    void ShootHigh();

    Shooter *m_shooter;

    private:
    

};