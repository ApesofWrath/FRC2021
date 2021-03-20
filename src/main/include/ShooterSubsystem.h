#pragma once

#include "Shooter.h"

#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>
#include <rev/CANSparkMax.h>

class ShooterSubsystem : public frc2::SubsystemBase {
    public: 
    
    ShooterSubsystem();

    void ShootLow();
    void ShootMedium();
    void ShootHigh();

    rev::CANSparkMax *belt_spark, *left_indexer_spark, *right_indexer_spark;
    WPI_TalonFX *top_wheel_talon, *bottom_wheel_talon;


    private:
    

};