#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Joystick.h>

#include "ShooterSubsystem.h"

class ShootHighCommand : public frc2::CommandHelper<frc2::CommandBase, ShootHighCommand> {
    public: 
    explicit ShootHighCommand(ShooterSubsystem *shooter, frc::Joystick *joyOp);
    
    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;
     
    ~ShootHighCommand() override; // delete me?
    int m_counter = 0;
    // bool ShootHigh();
    private: 
    ShooterSubsystem *m_shooter;
    frc::Joystick *m_joystick;

};