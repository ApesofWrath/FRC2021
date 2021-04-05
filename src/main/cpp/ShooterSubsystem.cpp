#include "Shooter.h"
#include "ShooterSubsystem.h"

ShooterSubsystem::ShooterSubsystem(Shooter *shooter, frc::Joystick *joyOp){
    m_shooter = shooter;
    m_joystick = joyOp;

    frc::SmartDashboard::PutNumber("!shooter periodic counter", m_periodic_counter);
    m_shooter->top_wheel_PID->SetReference(0, rev::ControlType::kVelocity);
    m_shooter->top_wheel_spark->Set(0);

    m_shooter->bottom_wheel_PID->SetReference(0, rev::ControlType::kVelocity);
    m_shooter->bottom_wheel_spark->Set(0);

    m_shooter->belt_PID->SetReference(0, rev::ControlType::kVelocity); 
    m_shooter->belt_spark->Set(0);
}

void ShooterSubsystem::Periodic(){
    // if((!(m_joystick->GetRawButton(3)))){ //&& m_joystick->GetRawAxis(3) > .5
    // m_periodic_counter++;
    // frc::SmartDashboard::PutNumber("!shooter periodic counter", m_periodic_counter);
    // m_shooter->top_wheel_PID->SetReference(0, rev::ControlType::kVelocity);
    // m_shooter->top_wheel_spark->Set(0);

    // m_shooter->bottom_wheel_PID->SetReference(0, rev::ControlType::kVelocity);
    // m_shooter->bottom_wheel_spark->Set(0);

    // m_shooter->belt_PID->SetReference(0, rev::ControlType::kVelocity); 
    // m_shooter->belt_spark->Set(0);
    // }
    m_shooter->right_indexer_spark->Set(.5);
    frc::SmartDashboard::PutNumber("!shootertopwwheel ", m_shooter->top_wheel_encoder->GetVelocity());
    frc::SmartDashboard::PutNumber("!shooterbotwheeel ", m_shooter->bottom_wheel_encoder->GetVelocity());
    frc::SmartDashboard::PutNumber("!shooterbelt ", m_shooter->belt_encoder->GetVelocity());

    //max vals
    if(m_shooter->top_wheel_encoder->GetVelocity() < top_max_speed) {
        frc::SmartDashboard::PutNumber("!shootermaxtopwwheel ", m_shooter->top_wheel_encoder->GetVelocity());

    }

    if(m_shooter->bottom_wheel_encoder->GetVelocity() < bottom_max_speed) {
        frc::SmartDashboard::PutNumber("!shootermaxbotwheeel ", m_shooter->bottom_wheel_encoder->GetVelocity());

    }

    if(m_shooter->belt_encoder->GetVelocity() < belt_max_speed) {
        frc::SmartDashboard::PutNumber("!shootermaxbelt ", m_shooter->belt_encoder->GetVelocity());

    }

}

void ShooterSubsystem::ShootLow(){


}

void ShooterSubsystem::ShootMedium(){

}

void ShooterSubsystem::StopMotors(){
    m_shooter->top_wheel_PID->SetReference(0, rev::ControlType::kVelocity);
    m_shooter->top_wheel_spark->Set(0);

    m_shooter->bottom_wheel_PID->SetReference(0, rev::ControlType::kVelocity);
    m_shooter->bottom_wheel_spark->Set(0);

    m_shooter->belt_PID->SetReference(0, rev::ControlType::kVelocity); 
    m_shooter->belt_spark->Set(0);
}


void ShooterSubsystem::ShootHigh(){
        m_shooter->top_wheel_PID->SetReference(-5750, rev::ControlType::kVelocity);
        // m_shooter->top_wheel_spark->Set(-1);
        frc::SmartDashboard::PutNumber("!shootertopwwheel velo ", m_shooter->top_wheel_encoder->GetVelocity());
        frc::SmartDashboard::PutNumber("!shooter top %out ", m_shooter->top_wheel_spark->GetAppliedOutput());


        // m_shooter->bottom_wheel_spark->Set(1);
        m_shooter->bottom_wheel_PID->SetReference(5750, rev::ControlType::kVelocity);
        frc::SmartDashboard::PutNumber("!shooterbotwheeel velo ", m_shooter->bottom_wheel_encoder->GetVelocity());
        frc::SmartDashboard::PutNumber("!shooter bottom %out ", m_shooter->bottom_wheel_spark->GetAppliedOutput());


        // m_shooter->belt_spark->Set(-1);
        m_shooter->belt_PID->SetReference(-5650, rev::ControlType::kVelocity); 
        frc::SmartDashboard::PutNumber("!shooterbelt velo ", m_shooter->belt_encoder->GetVelocity());
        frc::SmartDashboard::PutNumber("!shooter belt %out ", m_shooter->belt_spark->GetAppliedOutput());


        //max vals
    if(m_shooter->top_wheel_encoder->GetVelocity() < top_max_speed) {
        frc::SmartDashboard::PutNumber("!shootermaxtopwwheel ", m_shooter->top_wheel_encoder->GetVelocity());

    }

    if(m_shooter->bottom_wheel_encoder->GetVelocity() > bottom_max_speed) {
        frc::SmartDashboard::PutNumber("!shootermaxbotwheeel ", m_shooter->bottom_wheel_encoder->GetVelocity());

    }

    if(m_shooter->belt_encoder->GetVelocity() < belt_max_speed) {
        frc::SmartDashboard::PutNumber("!shootermaxbelt ", m_shooter->belt_encoder->GetVelocity());

    }
}