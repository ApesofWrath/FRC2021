#include "Commands/GoDistance.h"
#include <frc/smartdashboard/SmartDashboard.h>

GoDistanceCommand::GoDistanceCommand(SwerveSubsystem *swerve, frc::Joystick *joyOp, double distance, double angle){
    m_swerve_subsystem = swerve;
    m_joystick = joyOp;
    m_distance = distance;
    m_angle = angle;
    
}

void GoDistanceCommand::Initialize(){
    m_start = m_swerve_subsystem->GetCurrentPositions();
    front_left_pos = m_start[0];
    front_right_pos = m_start[1];
    back_left_pos = m_start[2];
     back_right_pos = m_start[3];
}

void GoDistanceCommand::Execute() {
    m_swerve_subsystem->GoDistance(m_distance, m_angle);
}

void GoDistanceCommand::End(bool interrupted) {
    
}

bool GoDistanceCommand::IsFinished() { // lol thats a long conditional
    if((m_distance * (TICKS_PER_ROTATION_FALCON) * (SWERVE_MOD_DRIVE_RATIO) 
        / (METER_TO_INCHES / WHEEL_CIRCUMFERENCE) + front_left_pos 
        < m_swerve_subsystem->GetCurrentPositions()[0] && m_distance > 0) ||
        (m_distance * (TICKS_PER_ROTATION_FALCON) * (SWERVE_MOD_DRIVE_RATIO) 
        / (METER_TO_INCHES / WHEEL_CIRCUMFERENCE) + front_left_pos 
        > m_swerve_subsystem->GetCurrentPositions()[0] && m_distance < 0)){
        return true;
    }
    return false;
}

