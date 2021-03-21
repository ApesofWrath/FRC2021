#include "Commands/ShootHigh.h"
#include <frc/smartdashboard/SmartDashboard.h>

    ShootHighCommand::ShootHighCommand(ShooterSubsystem *shooter){
        m_shooter = shooter;
    };
    
    void ShootHighCommand::Initialize() {
        
    }

    void ShootHighCommand::Execute() {
        m_shooter->ShootHigh();
        frc::SmartDashboard::PutString("COmmand", "WORKINBG");

    }
    
    void ShootHighCommand::End(bool interrupted) { 
    
    }

    bool ShootHighCommand::IsFinished() {
        return true;
    }

    ShootHighCommand::~ShootHighCommand(){

    }

    // bool ShootHigh() {
    //     return true; //NO
    // }