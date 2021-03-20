#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "ShooterSubsystem.h"

class ShootHighCommand : public frc2::CommandHelper<frc2::CommandBase, ShootHighCommand> {
    public: 
    explicit ShootHighCommand(ShooterSubsystem *shooter);
    
    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;
     
    ~ShootHighCommand() override; // delete me?
    // bool ShootHigh();
    private: 
    ShooterSubsystem *m_shooter;

};