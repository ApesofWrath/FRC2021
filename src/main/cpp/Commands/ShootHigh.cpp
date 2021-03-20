#include "Commands/ShootHigh.h"

    ShootHighCommand::ShootHighCommand(ShooterSubsystem *shooter){
        m_shooter = shooter;
    };
    
    void ShootHighCommand::Initialize() {
        
    }

    void ShootHighCommand::Execute() {

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