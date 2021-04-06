#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Joystick.h>

#include "Drive/SwerveSubsystem.h"

class GoDistanceCommand : public frc2::CommandHelper<frc2::CommandBase, GoDistanceCommand> {
    public: 
    explicit GoDistanceCommand(SwerveSubsystem *swerve, frc::Joystick *joyOp, double distance, double angle); // probably wont be taking joystick input
    
    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;
    
    std::array<double, 4> m_start;

    double front_left_pos;
    double front_right_pos;
    double back_left_pos;
    double back_right_pos;

     double m_distance;
     double m_angle;

    ~GoDistanceCommand() override; // delete me?
    // bool ShootHigh();
    private: 
    SwerveSubsystem *m_swerve_subsystem;
    frc::Joystick *m_joystick;


};