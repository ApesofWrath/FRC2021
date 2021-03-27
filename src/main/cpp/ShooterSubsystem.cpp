#include "Shooter.h"
#include "ShooterSubsystem.h"

ShooterSubsystem::ShooterSubsystem(Shooter *shooter){
    m_shooter = shooter;
}

void ShooterSubsystem::Periodic(){
    
}

void ShooterSubsystem::ShootLow(){


}

void ShooterSubsystem::ShootMedium(){

}


void ShooterSubsystem::ShootHigh(){
    m_shooter->top_wheel_talon->Set(ControlMode::PercentOutput, 0.5);
    frc::SmartDashboard::PutString("COmmand", "WORKINBG");

}