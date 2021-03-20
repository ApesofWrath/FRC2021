#include "Shooter.h"
#include "ShooterSubsystem.h"

ShooterSubsystem::ShooterSubsystem(){
    
}

void ShooterSubsystem::ShootLow(){
top_wheel_talon->Set(ControlMode::PercentOutput, 0.5);
}

void ShooterSubsystem::ShootMedium(){

}

void ShooterSubsystem::ShootHigh(){

}