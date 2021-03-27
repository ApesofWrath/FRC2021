#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "ShooterSubsystem.h"

class ShootLowCommand : public frc2::CommandHelper<frc2::CommandBase, ShootLowCommand> {
    public: 
    explicit ShootLowCommand(ShooterSubsystem *shooter, frc::Joystick *joyOp);
    
    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;
     
    ~ShootLowCommand() override; // delete me?
    int m_counter = 0;

    private: 
    ShooterSubsystem *m_shooter;
    frc::Joystick *m_joystick;

};