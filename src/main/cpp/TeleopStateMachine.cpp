#include "TeleopStateMachine.h"

using namespace frc;

TeleopStateMachine::TeleopStateMachine(Shooter* shooter_, Intake* intake_, ControlPanel* control_panel_, Arm* arm_)
{
    state = WAIT_FOR_BUTTON_STATE;
    last_state = WAIT_FOR_BUTTON_STATE;

    arm = arm_;
    intake = intake_;
    control_panel = control_panel_;
    shooter = shooter_;

}

TeleopStateMachine::~TeleopStateMachine(){
}

ButtonData TeleopStateMachine::GatherButtonDataFromJoysticks(Joystick* joyThrottle, Joystick* joyWheel, Joystick* joyOp) {
    return ButtonData {
        joyOp->GetRawButton(ButtonIDs::WAIT_FOR_BUTTON), //wfb

        joyOp->GetRawButton(ButtonIDs::LOWER_INTAKE_BUTTON), //lib
        joyOp->GetRawButton(ButtonIDs::INTAKE_OUT_BUTTON), // iob

        joyOp->GetRawButton(ButtonIDs::SHOOT_BUTTON), // sb
        joyOp->GetRawButton(ButtonIDs::SHOOTER_INTAKE_BUTTON), // sib
        joyOp->GetRawButton(ButtonIDs::SHOOTER_REVERSE_BUTTON),
        
        joyOp->GetRawButton(ButtonIDs::ROTATION_MODE_CONTROL_PANEL_BUTTON), // rmcpb
        joyOp->GetRawButton(ButtonIDs::POSITION_MODE_CONTROL_PANEL_BUTTON), // pmcpb
        
        joyOp->GetRawButton(ButtonIDs::FULL_RAISE_B1) && joyOp->GetRawButton(ButtonIDs::FULL_RAISE_B2),
        joyOp->GetRawButton(ButtonIDs::HUMAN_LOAD_BUTTON),
        // joyOp->GetRawButton(ButtonIDs::EMERGENCY_BUTTON)
        
        false
     };
}

void TeleopStateMachine::ProcessButtonData(ButtonData data) {
    if (data.lower_intake_button) {
      state = INTAKE_STATE;
    } else if (data.intake_out_button) {
      state = INTAKE_OUT_STATE;
    } else {
      state = WAIT_FOR_BUTTON_STATE;
    } 
    if (data.shoot_button) {
        state = SHOOT_STATE;
    }
    if (data.shooter_reverse_button) {
      state = SHOOTER_REVERSE_STATE;
    }
    if (data.shooter_intake_button) {
      state = INTAKE_SHOOTER_STATE;
    }
    if (data.rotation_mode_control_panel_button) {
        state = ROTATION_MODE_CONTROL_PANEL_STATE;
    }
    if (data.position_mode_control_panel_button) {
        state = POSITION_MODE_CONTROL_PANEL_STATE;
    }
    //arm
    if (data.intake_out_button) {
      state = INTAKE_OUT_STATE;
    } 
    if (data.full_raise) {
      state = FULL_RAISE_STATE;
    }
    if (data.human_load_button){
      state = HUMAN_LOAD_STATE;
    }
    if (data.wait_for_button) {
        state = WAIT_FOR_BUTTON_STATE;
    // } else if (data.intake_button) {
    //     state = INTAKE_STATE;
    }
}

void TeleopStateMachine::StateMachine(ButtonData data) {
    ProcessButtonData(data);

    

    //intake
    // if (data.intake_in) {
    //   state_intake = false;
    //   intake->intake_state = intake->IN_STATE;
    // } else 
    
    //control panel
    // if ()

    switch (state) {


        case INIT_STATE:
          arm->intake_arm_state = arm->REST_STATE;
          intake->intake_state = intake->STOP_STATE;
          shooter->shooter_state = shooter->INIT_STATE;
          control_panel->state = control_panel->IDLE;

          state = WAIT_FOR_BUTTON_STATE;
        break;


        case WAIT_FOR_BUTTON_STATE:
          arm->intake_arm_state = arm->UP_STATE;
          intake->intake_state = intake->STOP_STATE;
          shooter->shooter_state = shooter->STOP_STATE;
          control_panel->state = control_panel->IDLE;
          last_state = WAIT_FOR_BUTTON_STATE;
        break;


        case INTAKE_STATE:
          arm->intake_arm_state = arm->DOWN_STATE;
          intake->intake_state = intake->IN_STATE;
          shooter->shooter_state = shooter->INTAKE_STATE;
          control_panel->state = control_panel->IDLE;
          last_state = INTAKE_STATE;
        break;

        case INTAKE_SHOOTER_STATE: // i hope we arent using this
          shooter->shooter_state = shooter->INTAKE_STATE;
          arm->intake_arm_state = arm->UP_STATE;
          control_panel->state = control_panel->IDLE;
          intake->intake_state = intake->STOP_STATE;
          last_state = INTAKE_SHOOTER_STATE;
        break;

        case SHOOT_STATE:
          shooter->shooter_state = shooter->FAR_SHOOT_STATE;
          arm->intake_arm_state = arm->UP_STATE;
          control_panel->state = control_panel->IDLE;
          intake->intake_state = intake->STOP_STATE;
          last_state = SHOOT_STATE;
        break;

        case ROTATION_MODE_CONTROL_PANEL_STATE:
          control_panel->state = control_panel->ROTATION_MODE;
          arm->intake_arm_state = arm->UP_STATE;
          intake->intake_state = intake->STOP_STATE;
          shooter->shooter_state = shooter->WAITING_STATE;
          last_state = ROTATION_MODE_CONTROL_PANEL_STATE;
        break;

        case FULL_RAISE_STATE:
          arm->intake_arm_state = arm->REST_STATE;
          intake->intake_state = intake->STOP_STATE;
          control_panel->state = control_panel->IDLE;
          shooter->shooter_state = shooter->WAITING_STATE;
          arm->MoveToPosition(arm->armStartPos);

        case POSITION_MODE_CONTROL_PANEL_STATE:
          control_panel->state = control_panel->POSITION_MODE;
          arm->intake_arm_state = arm->UP_STATE;
          intake->intake_state = intake->STOP_STATE;
          shooter->shooter_state = shooter->WAITING_STATE;
          last_state = POSITION_MODE_CONTROL_PANEL_STATE;
        break;

        case INTAKE_OUT_STATE:
          intake->intake_state = intake->OUT_STATE;
          if (arm->intake_arm_state != arm->DOWN_STATE) {
            arm->intake_arm_state = arm->DOWN_STATE;
          }
          control_panel->state = control_panel->IDLE;
          shooter->shooter_state = shooter->WAITING_STATE;
          last_state = INTAKE_OUT_STATE;
        break;

        case SHOOTER_REVERSE_STATE:
          arm->intake_arm_state = arm->UP_STATE;
          intake->intake_state = intake->STOP_STATE;
          control_panel->state = control_panel->IDLE;
          shooter->shooter_state = shooter->REVERSE_STATE;
          last_state = SHOOTER_REVERSE_STATE;
        break;

        case HUMAN_LOAD_STATE:
          control_panel->state = control_panel->HUMAN_LOAD;
          last_state = HUMAN_LOAD_STATE;
    }

    SmartDashboard::PutString("State", TeleopStateMachine::StateName(state));

    last_state = state;

    arm->IntakeArmStateMachine();
    intake->IntakeStateMachine();
    shooter->ShooterStateMachine();
    control_panel->StateMachine();

}

std::string TeleopStateMachine::StateName(TeleopStateMachine::States state) {
    switch (state) {
        case INIT_STATE:
            return "Robot::Init";
        case WAIT_FOR_BUTTON_STATE:
            return "Robot::Wait For Button";
        case INTAKE_STATE:
            return "Robot::Intake";
        case SHOOT_STATE:
            return "Shooter::Shoot";
        case ROTATION_MODE_CONTROL_PANEL_STATE:
            return "Control Panel::Rotation Mode";
        case POSITION_MODE_CONTROL_PANEL_STATE:
            return "Control Panel::Position Mode";
        default:
            return "Robot::Null State";
    }
}
