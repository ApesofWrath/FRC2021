#pragma once

#include "Intake.h"
#include "Arm.h"
#include "Shooter.h"
#include "ControlPanel.h"

#include <frc/Joystick.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include <string>

using namespace frc;

namespace ButtonIDs {
    const int WAIT_FOR_BUTTON                    = 5,//13 < change back to 13
              LOWER_INTAKE_BUTTON                = 8,
              INTAKE_OUT_BUTTON                  = 2,
              SHOOT_BUTTON                       = 3,
              SHOOTER_INTAKE_BUTTON              = 9,
              SHOOTER_REVERSE_BUTTON             = 16,
              FULL_RAISE_B1                      = 14,
              FULL_RAISE_B2                      = 12,
              ROTATION_MODE_CONTROL_PANEL_BUTTON = 4,
              POSITION_MODE_CONTROL_PANEL_BUTTON = 1,
              HUMAN_LOAD_BUTTON                  = 15,
              EMERGENCY_BUTTON                   = 99;

}


struct ButtonData {
    bool wait_for_button, 
    lower_intake_button, intake_out_button,
    shoot_button, shooter_intake_button, shooter_reverse_button,
    rotation_mode_control_panel_button, position_mode_control_panel_button,
    full_raise, human_load_button, emergency_button;
};

class TeleopStateMachine
{
private:
    /* data */
public:
    TeleopStateMachine(Shooter* shooter_, Intake* intake_, ControlPanel* control_panel_, Arm* arm_);
    ~TeleopStateMachine();

    void StateMachine(ButtonData data);
    void ProcessButtonData(ButtonData data);

    ButtonData GatherButtonDataFromJoysticks(Joystick* joyThrottle, Joystick* joyWheel, Joystick* joyOp);

    enum States {
        INIT_STATE, WAIT_FOR_BUTTON_STATE,
        INTAKE_SHOOTER_STATE, SHOOT_STATE, SHOOTER_REVERSE_STATE,
        INTAKE_STATE, INTAKE_OUT_STATE, FULL_RAISE_STATE,
        ROTATION_MODE_CONTROL_PANEL_STATE, POSITION_MODE_CONTROL_PANEL_STATE,
        HUMAN_LOAD_STATE,
    };

    static std::string StateName(States state);

    States state;
    States last_state;

    bool state_arm = false;
    bool state_intake = false;
    bool state_shooter = false;
    bool state_control_panel = false;


    Intake *intake;
    Arm *arm;
    Shooter *shooter;
    ControlPanel *control_panel;
};
