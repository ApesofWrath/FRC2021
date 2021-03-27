#include "Commands/ShootLow.h"
#include <frc/smartdashboard/SmartDashboard.h>

    ShootLowCommand::ShootLowCommand(ShooterSubsystem *shooter, frc::Joystick *joyOp){
        m_shooter = shooter;
        m_joystick = joyOp;
    };
    
    void ShootLowCommand::Initialize() {
        
    }

    void ShootLowCommand::Execute() {
        m_shooter->ShootLow();
        frc::SmartDashboard::PutNumber("CounterLow", ++m_counter);
        frc::SmartDashboard::PutString("COmmand 2", "WORKINBG LOW");

    }
    
    void ShootLowCommand::End(bool interrupted) { 
        frc::SmartDashboard::PutString("COmmand", "I AM OFF LOW");
    
    }

    bool ShootLowCommand::IsFinished() {
        return !(m_joystick->GetRawButton(2));
    }

    ShootLowCommand::~ShootLowCommand(){

    }

    // bool ShootLow() {
    //     return true; //NO
    // }