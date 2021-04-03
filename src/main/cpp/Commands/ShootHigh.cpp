#include "Commands/ShootHigh.h"
#include <frc/smartdashboard/SmartDashboard.h>

    ShootHighCommand::ShootHighCommand(ShooterSubsystem *shooter, frc::Joystick *joyOp){
        m_shooter = shooter;
        m_joystick = joyOp;
    };
    
    void ShootHighCommand::Initialize() {
        
    }

    void ShootHighCommand::Execute() {
        m_shooter->ShootHigh();
        frc::SmartDashboard::PutString("COmmand 2", "WORKINBG HIGH");

    }
    
    void ShootHighCommand::End(bool interrupted) { 
        frc::SmartDashboard::PutString("COmmand", "I AM OF highF");
    
    }

    bool ShootHighCommand::IsFinished() {
        if((m_joystick->GetRawButton(3))){
            return false;

        }
        m_counter++;
        frc::SmartDashboard::PutNumber("!shooterfinishedcounter", m_counter);
        //make function to stop motor 
        m_shooter->StopMotors();
        return true;
    }

    ShootHighCommand::~ShootHighCommand(){

    }

    // bool ShootHigh() {
    //     return true; //NO
    // }